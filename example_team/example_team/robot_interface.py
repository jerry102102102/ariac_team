import os

from typing import Literal

import yaml

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
from geometry_msgs.msg import Pose

from ament_index_python.packages import get_package_share_directory

from example_team.utils import AsyncUtils

class ExecutionStatus(Enum):
    UNKNOWN = "UNKNOWN"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    PREEMPTED = "PREEMPTED"
    TIMED_OUT = "TIMED_OUT"
    ABORTED = "ABORTED"
    FAILED = "FAILED"

class RobotInterface(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_interface')

        self.robot_name = robot_name
        self.planning_group = f'{self.robot_name}_arm'

        sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])
        
        self.planner_config_names = ["ompl_rrts", "pilz_ptp", "pilz_lin"]

        params_file = get_moveit_params(self.robot_name)
        print(params_file)

        # Create MoveItPy Node
        self.moveit_py = MoveItPy(
            node_name=f"{self.robot_name}_moveit_py", 
            name_space=self.robot_name, 
            launch_params_filepaths=[params_file]
        )

        self.planning_component: PlanningComponent = self.moveit_py.get_planning_component(self.planning_group)
        self.planning_scene_monitor: PlanningSceneMonitor = self.moveit_py.get_planning_scene_monitor()
        self.robot_model: RobotModel = self.moveit_py.get_robot_model()
        self.joint_group: JointModelGroup = self.robot_model.get_joint_model_group(self.planning_group)
        self.trajectory_execution_manager: TrajectoryExecutionManager = self.moveit_py.get_trajectory_execution_manager()

    @property
    def current_pose(self) -> Pose:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene
            current_state: RobotState = scene.current_state
            
            return current_state.get_pose(self.joint_group.link_model_names[-1])
    
    @property
    def joint_values(self) -> list[float]:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene
            current_state: RobotState = scene.current_state
            
            return current_state.get_joint_group_positions(self.joint_group.name)

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
                raise TimeoutError("Timed out waiting state")

            await asyncio.sleep(0.1)

    async def ready(self):
        await self._clock_ready()
        await self._state_ready()

    async def plan_to_named_configuration(self, configuration: str, asf=1.0, vsf=1.0) -> RobotTrajectory:
        """Plan to a joint state specified in the srdf"""
        
        if not configuration in self.planning_component.named_target_states:
            raise RuntimeError(f'Unable to plan to {configuration}. Not in named targets')
        
        self.planning_component.set_goal_state(configuration_name=configuration)

        plan = await self._plan(asf=asf, vsf=vsf)
        return plan
    
    async def execute(self, trajectory: RobotTrajectory, timeout: float = 60):
        timeout_task = asyncio.create_task(AsyncUtils.await_for_duration(self.get_clock(), Duration(seconds=timeout)))
        execution_task = self._execute(trajectory)
        
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

        if planner_config not in self.planner_config_names:
            raise RuntimeError(f'{planner_config} is not a valid planning config name.')

        single_plan_parameters = PlanRequestParameters(self.moveit_py, planner_config)
        single_plan_parameters.max_acceleration_scaling_factor = asf
        single_plan_parameters.max_velocity_scaling_factor = vsf

        # Generate plan
        plan_task = asyncio.to_thread(self._get_plan, single_plan_parameters)
        plan: MotionPlanResponse = await asyncio.wait_for(plan_task, timeout=5)

        plan_result: MoveItErrorCodes = plan.error_code

        self.get_logger().info(f"Planning result {plan_result}")

        if plan_result.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'Unable to plan trajectory. Error code: {plan_result.val}')

        return plan.trajectory
    
    def _get_plan(self, params: PlanRequestParameters):
        return self.planning_component.plan(single_plan_parameters=params)
    
    def _execute(self, trajectory: RobotTrajectory) -> asyncio.Future[ExecutionStatus]:
        """
        Kick off execution and return a Future which completes when we get
        a SUCCEEDED or terminal failure status.
        """
        loop = asyncio.get_running_loop()
        future: asyncio.Future[ExecutionStatus] = loop.create_future()

        def _on_status(execution: MoveitExecutionStatus):
            status = ExecutionStatus(execution.status)
            if status == ExecutionStatus.SUCCEEDED:
                # tell the future we succeeded
                loop.call_soon_threadsafe(future.set_result, status)
            elif status in (
                ExecutionStatus.FAILED,
                ExecutionStatus.ABORTED,
                ExecutionStatus.TIMED_OUT,
                ExecutionStatus.UNKNOWN,
            ):
                # error the future
                exc = RuntimeError(f'Execution failed with status: {status.value.lower()}')
                loop.call_soon_threadsafe(future.set_exception, exc)

        # push and start the motion
        self.trajectory_execution_manager.push(trajectory.get_robot_trajectory_msg())

        self.trajectory_execution_manager.execute(_on_status)

        return future

def get_moveit_params(robot_name: Literal['inspection_robot_1', 'inspection_robot_2', 'assembly_robot_1', 'assembly_robot_2']):
    robot_config_dir = os.path.join(get_package_share_directory("example_team"), "config", "moveit", f"{robot_name}")

    moveit_config = (
        MoveItConfigsBuilder(f"{robot_name}")
        .robot_description(file_path=os.path.join(get_package_share_directory("ariac_description"), "urdf", f"{robot_name}.urdf.xacro"))
        .robot_description_semantic(file_path=os.path.join(robot_config_dir, "robot.srdf"))
        .robot_description_kinematics(file_path=os.path.join(robot_config_dir, "kinematics.yaml"))
        .moveit_cpp(file_path=os.path.join(robot_config_dir, "moveit_cpp.yaml"))
        .trajectory_execution(file_path=os.path.join(robot_config_dir, "controllers.yaml"))
        .joint_limits(file_path=os.path.join(robot_config_dir, "joint_limits.yaml"))
        .pilz_cartesian_limits(file_path=os.path.join(robot_config_dir, "pilz_cartesian_limits.yaml"))
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner", "ompl"])
        .to_moveit_configs()
        .to_dict()
    )

    # Load planner params
    with open(os.path.join(robot_config_dir, "planner_params.yaml"), "r") as f:
        moveit_config.update(yaml.safe_load(f))

    moveit_config.update({"use_sim_time": True})

    return create_params_file_from_dict(moveit_config, "/**")