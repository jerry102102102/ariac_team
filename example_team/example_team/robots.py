from rclpy.time import Duration

from example_team.robot_interface import RobotInterface
from example_team.utils import AsyncUtils

class InspectionRobot1(RobotInterface):
    def __init__(self):
        super().__init__('inspection_robot_1')

    async def startup(self): 
        await AsyncUtils.await_for_duration(self.get_clock(), Duration(seconds=2))

        self.get_logger().info("Inspection robot 1 ready")