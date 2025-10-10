import math 

import tf2_ros

from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import Pose, Point, Vector3, Quaternion

from example_team.robot_interface import Robot
from example_team.gripper_interface import Gripper
from example_team.utils import AsyncUtils, quaternion_from_list

class InspectionRobot2(Robot):
    def __init__(self):
        super().__init__('inspection_robot_2')

        self.tester_locations: dict[int, Vector3] = {}

        self.orientations: dict[str, Quaternion] = {
            'tester': quaternion_from_list(quaternion_from_euler(math.pi, 0.0, 0.0)),
            'recycling': quaternion_from_list(quaternion_from_euler(math.pi, 0.0, math.pi/2))
        }

        self.joint_states = {
            'ready': [-0.52, -1.7, 2.22, -2.08, -1.57, 1.05],
            'recycling': [-5.15, -1.76, 1.822, -1.57, -1.57, 0.0],
        }

        # Initialize TF
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        self.gripper = Gripper(self, 'inspection_robot_2')

    async def ready(self): 
        await super().ready()
        for i in (1, 2):
            t = await AsyncUtils.get_tf_transform(self.tf_buffer, 'world', f'voltage_tester_{i}_frame')
            self.tester_locations[i] = t.transform.translation

        plan = await self.plan_to_joint_state(self.joint_states['ready'])
        await self.execute(plan)

    async def pick_cell_from_tester(self, tester: int):
        self.get_logger().info(f'Picking cell from tester {tester}')

        await self.gripper.open()
        
        p = self.tester_locations[tester]
        above = Point(x=p.x, y=p.y, z=p.z+0.05)
        grasp = Point(x=p.x, y=p.y, z=p.z)

        for i, position in enumerate([above, grasp, above]):
            plan = await self.plan_to_pose(Pose(position=position, orientation=self.orientations['tester']))
            await self.execute(plan)

            if i == 1:
                await self.gripper.close()
                if not self.gripper.is_holding:
                    raise RuntimeError(f"Failed to grasp cell from tester {tester}")
    
    async def recycle_cell(self):
        self.get_logger().info(f'Recycling cell')

        plan = await self.plan_to_joint_state(self.joint_states['recycling'])
        await self.execute(plan)

        position = self.current_pose.position
        position.z -= 0.2

        plan = await self.plan_to_pose(Pose(position=position, orientation=self.orientations['recycling']))
        await self.execute(plan)

        await self.gripper.open()

        plan = await self.plan_to_joint_state(self.joint_states['ready'], vsf=0.5)
        await self.execute(plan)