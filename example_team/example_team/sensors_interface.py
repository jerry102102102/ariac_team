import asyncio

from dataclasses import dataclass

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time, Duration
from rclpy.qos import qos_profile_sensor_data

import tf2_ros

from geometry_msgs.msg import Point, TransformStamped

from ariac_interfaces.msg import BreakBeamStatus

@dataclass
class BatteryCell:
    detection_time: Time

class Sensors(Node):
    def __init__(self):
        super().__init__(f'sensors_interface')

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.inspection_bb = InspectionBreakBeam(self)

class InspectionBreakBeam:
    def __init__(self, node: Node):
        self._node = node

        self.cell_queue: asyncio.Queue[BatteryCell] = asyncio.Queue()

        self.status_cb = self._node.create_subscription(
            BreakBeamStatus,
            '/inspection_breakbeam/change',
            self._change_cb,
            qos_profile_sensor_data
        )
    
    def _change_cb(self, msg: BreakBeamStatus):
        if msg.object_detected:
            self.cell_queue.put_nowait(BatteryCell(detection_time=Time.from_msg(msg.header.stamp)))
