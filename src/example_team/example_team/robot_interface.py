import asyncio

from enum import Enum

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Duration

from moveit_configs_utils import MoveItConfigsBuilder
from moveit.utils import create_params_file_from_dict

from moveit.planning import (
    MoveItPy,
    PlanningComponent,
    PlanningSceneMonitor,
    TrajectoryExecutionManager,
    PlanRequestParameters
)

from moveit.core.planning_interface import MotionPlanResponse
from moveit.core.controller_manager import ExecutionStatus as MoveitExecutionStatus
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel, JointModelGroup
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory

from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose, PoseStamped

from example_team.utils import AsyncUtils

class ExecutionStatus(Enum):
    UNKNOWN = "UNKNOWN"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    PREEMPTED = "PREEMPTED"
    TIMED_OUT = "TIMED_OUT"
    ABORTED = "ABORTED"
    FAILED = "FAILED"

class Robot(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_interface')

        self.robot_name = robot_name
        self.planning_group = f'{self.robot_name}_arm'

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.moveit_py = MoveItPy(
            node_name=f"{robot_name}_moveit_py", 
            name_space=robot_name, 
            launch_params_filepaths=[get_moveit_params(robot_name)]
        )

        self.planning_component: PlanningComponent = self.moveit_py.get_planning_component(self.planning_group)
        self.planning_scene_monitor: PlanningSceneMonitor = self.moveit_py.get_planning_scene_monitor()
        self.robot_model: RobotModel = self.moveit_py.get_robot_model()
        self.joint_group: JointModelGroup = self.robot_model.get_joint_model_group(self.planning_group)
        self.trajectory_execution_manager: TrajectoryExecutionManager = self.moveit_py.get_trajectory_execution_manager()

    @property
    def current_pose(self) -> Pose:            
        return self._current_state().get_pose(self.joint_group.link_model_names[-1])
    
    @property
    def joint_values(self) -> list[float]:
        return self._current_state().get_joint_group_positions(self.joint_group.name)

    async def ready(self):
        await self._clock_ready()
        await self._state_ready()
        await self._controller_ready()

    async def plan_to_named_configuration(self, configuration: str, asf=1.0, vsf=1.0) -> RobotTrajectory:
        """Plan to a joint state specified in the srdf"""
        
        if not configuration in self.planning_component.named_target_states:
            raise RuntimeError(f'Unable to plan to {configuration}. Not in named targets')
        
        self.planning_component.set_goal_state(configuration_name=configuration)

        return await self._plan(asf=asf, vsf=vsf)
    
    async def plan_to_pose(self, pose: Pose, linear=False, use_ompl=False, asf=1.0, vsf=1.0) -> RobotTrajectory:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'world'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose

        self.planning_component.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=self.joint_group.link_model_names[-1])

        if linear:
            return await self._plan(planner_config="pilz_lin", asf=asf, vsf=vsf)
        if use_ompl:
            return await self._plan(planner_config="ompl_rrts", asf=asf, vsf=vsf)
        else:
            return await self._plan(planner_config="pilz_ptp", asf=asf, vsf=vsf)
    
    async def plan_to_joint_state(self, joint_values: list[float], asf=1.0, vsf=1.0) -> RobotTrajectory:
        goal_state = RobotState(self.robot_model)
        goal_state.set_joint_group_positions(self.planning_group, joint_values)

        self.planning_component.set_goal_state(robot_state=goal_state)

        return await self._plan(asf=asf, vsf=vsf)
    
    async def execute(self, trajectory: RobotTrajectory, timeout: float = 60):
        timeout_task = asyncio.create_task(AsyncUtils.await_for_duration(self.get_clock(), Duration(seconds=timeout)))
        execution_task = self._start_execution(trajectory)
        
        await asyncio.wait([execution_task, timeout_task], return_when=asyncio.FIRST_COMPLETED)

        if timeout_task.done():
            raise RuntimeError("Timeout reached during exection of trajectory")
        
        execption = execution_task.exception()
        
        if execption is not None:
            raise execption
    
    async def _plan(self, planner_config="pilz_ptp", asf=1.0, vsf=1.0) -> RobotTrajectory:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene

            self.planning_component.set_start_state(robot_state=scene.current_state)

        if planner_config not in ["ompl_rrts", "pilz_ptp", "pilz_lin"]:
            raise RuntimeError(f'{planner_config} is not a valid planning config name.')

        single_plan_parameters = PlanRequestParameters(self.moveit_py, planner_config)
        single_plan_parameters.max_acceleration_scaling_factor = asf
        single_plan_parameters.max_velocity_scaling_factor = vsf

        plan_task = asyncio.to_thread(self.planning_component.plan, single_plan_parameters)
        plan: MotionPlanResponse = await asyncio.wait_for(plan_task, timeout=5)

        plan_result: MoveItErrorCodes = plan.error_code

        if plan_result.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'Unable to plan trajectory. Error code: {plan_result.val}')

        return plan.trajectory
    
    def _start_execution(self, trajectory: RobotTrajectory) -> asyncio.Future[ExecutionStatus]:
        loop = asyncio.get_running_loop()
        future: asyncio.Future[ExecutionStatus] = loop.create_future()

        def _on_status(execution: MoveitExecutionStatus):
            status = ExecutionStatus(execution.status)
            if status == ExecutionStatus.RUNNING:
                return
            elif status == ExecutionStatus.SUCCEEDED:
                loop.call_soon_threadsafe(future.set_result, status)
            else:
                exc = RuntimeError(f'Execution failed with status: {status.value.lower()}')
                loop.call_soon_threadsafe(future.set_exception, exc)

        # push and start the motion
        self.trajectory_execution_manager.push(trajectory.get_robot_trajectory_msg())

        self.trajectory_execution_manager.execute(_on_status)

        return future
    
    def _current_state(self) -> RobotState:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene
            return scene.current_state
    
    async def _clock_ready(self):
        start_time = asyncio.get_running_loop().time()
        while self.get_clock().now().nanoseconds == 0:
            if (asyncio.get_running_loop().time() - start_time) > 5.0:
                raise TimeoutError("Timed out waiting for ROS clock to start")
            
            await asyncio.sleep(0.1)

    async def _state_ready(self):
        start_time = self.get_clock().now()
        
        while True:
            if any([v!=0 for v in self.joint_values]):
                break

            if (self.get_clock().now() - start_time) > Duration(seconds=5.0):
                raise TimeoutError("Timed out waiting for state")

            await asyncio.sleep(0.1)
    
    async def _controller_ready(self):
        start_time = self.get_clock().now()
        
        while True:
            if self.trajectory_execution_manager.is_controller_active(controller=f'/{self.robot_name}/joint_trajectory_controller'):
                break

            if (self.get_clock().now() - start_time) > Duration(seconds=5.0):
                raise TimeoutError("Timed out waiting for controllers")

            await asyncio.sleep(0.1)


def get_moveit_params(robot_name):
    moveit_config = (
        MoveItConfigsBuilder(f"{robot_name}")
        .moveit_cpp()
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner", "ompl"])
        .to_moveit_configs()
        .to_dict()
    )

    moveit_config.update({"use_sim_time": True})

    return create_params_file_from_dict(moveit_config, "/**")