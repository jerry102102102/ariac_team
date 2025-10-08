"""Light-weight robot handles used by the MVP pipeline.

中文說明：此模組提供檢測區域兩支機械臂的封裝，
透過整合 example_team 的 MoveIt 與感測樣板，
實際執行輸送帶 → 測試器 → AGV 的取放流程。
"""

from __future__ import annotations

import abc
import asyncio
import math
from dataclasses import dataclass, field
from typing import Sequence, Tuple

import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler

from rclpy.node import Node

from example_team.gripper_interface import Gripper
from example_team.robot_interface import Robot
from example_team.robots import InspectionRobot2 as ExampleInspectionRobot2
from example_team.sensors_interface import BatteryCell, Sensors
from example_team.utils import AsyncUtils


def _quaternion_from_euler(r: float, p: float, y: float) -> Quaternion:
    """Helper to convert Euler angles (radians) into a ROS quaternion."""

    qx, qy, qz, qw = quaternion_from_euler(r, p, y)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


@dataclass(slots=True)
class RobotHandle(abc.ABC):
    """Common behaviour for MVP robot placeholders.

    中文說明：封裝共通屬性與 ready 檢查，
    讓各臂別在動作前確認已完成初始化。
    """

    name: str
    description: str
    _ready: bool = False

    def nodes(self) -> Tuple[Node, ...]:
        """Return the ROS 2 nodes that must be spun for this handle."""

        return ()

    def destroy(self) -> None:
        """Tear down any ROS nodes owned by the handle."""

        for node in self.nodes():
            node.destroy_node()

    async def ensure_ready(self) -> None:
        """Ensure the robot handle performed its lazy initialization.

        中文說明：第一次呼叫時完成 MoveIt/夾爪等初始化，
        後續即可直接執行動作而不需重複設定。
        """

        if self._ready:
            return

        await self._initialize()
        self._ready = True

    @abc.abstractmethod
    async def _initialize(self) -> None:
        """Perform the concrete initialization for a robot handle."""


class _InspectionRobot1Controller(Robot):
    """Concrete MoveIt wrapper for inspection robot 1."""

    conveyor_pick_frame_candidates: Tuple[str, ...] = (
        'inspection_conveyor_pick_frame',
        'inspection_conveyor_surface_frame',
        'ariac/inspection_conveyor_pick_frame',
    )
    tester_frame_template: str = 'voltage_tester_{tester}_frame'

    def __init__(self) -> None:
        super().__init__('inspection_robot_1')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.gripper = Gripper(self, 'inspection_robot_1')

        # Offsets tuned for a gentle pick/place sequence (meters).
        self._approach_height = 0.08
        self._grasp_clearance = 0.015

        self._orientations = {
            'downward': _quaternion_from_euler(math.pi, 0.0, 0.0),
        }

    @property
    def approach_height(self) -> float:
        return self._approach_height

    @property
    def grasp_clearance(self) -> float:
        return self._grasp_clearance

    async def lookup_conveyor_pick_pose(self) -> Pose:
        """Resolve the pose used to grasp a cell on the conveyor."""

        transform = await self._lookup_first_available(
            self.conveyor_pick_frame_candidates
        )
        translation = transform.transform.translation
        pose = Pose()
        pose.position = Point(
            x=translation.x,
            y=translation.y,
            z=translation.z + self.grasp_clearance,
        )
        pose.orientation = self._orientations['downward']
        return pose

    async def lookup_tester_pose(self, tester_id: int) -> Pose:
        """Resolve the placement pose for a given tester."""

        frame = self.tester_frame_template.format(tester=tester_id)
        transform = await AsyncUtils.get_tf_transform(
            self.tf_buffer, 'world', frame
        )
        translation = transform.transform.translation
        pose = Pose()
        pose.position = Point(
            x=translation.x,
            y=translation.y,
            z=translation.z + self.grasp_clearance,
        )
        pose.orientation = self._orientations['downward']
        return pose

    async def _lookup_first_available(
        self, frame_candidates: Sequence[str]
    ) -> TransformStamped:
        """Try candidate TF frames until one resolves."""

        last_exception: Exception | None = None
        for frame in frame_candidates:
            try:
                return await AsyncUtils.get_tf_transform(
                    self.tf_buffer, 'world', frame
                )
            except Exception as exc:  # noqa: BLE001 - keep last error for context
                last_exception = exc
        message = 'Unable to resolve conveyor pick frame from candidates'
        if last_exception is not None:
            raise RuntimeError(message) from last_exception
        raise RuntimeError(message)


@dataclass(slots=True)
class InspectionRobot1Handle(RobotHandle):
    """Concrete implementation for the conveyor robot.

    中文說明：代表檢測區域的第一台機器人，負責輸送帶 → 測試器的取放。
    """

    _controller: _InspectionRobot1Controller = field(init=False)
    _sensors: Sensors = field(init=False)

    def __post_init__(self) -> None:
        self._controller = _InspectionRobot1Controller()
        self._sensors = Sensors()

    def nodes(self) -> Tuple[Node, ...]:
        return (self._controller, self._sensors)

    async def _initialize(self) -> None:
        await self._controller.ready()
        plan = await self._controller.plan_to_named_configuration('home', vsf=0.3, asf=0.3)
        await self._controller.execute(plan)
        await self._controller.gripper.open()

    async def place_cell_on_tester(self, tester_id: int) -> None:
        """Pick a cell from the conveyor and place it onto a tester."""

        await self.ensure_ready()

        detection = await self._await_latest_cell_detection()
        self._controller.get_logger().info(
            'Breakbeam detection for conveyor cell at %.3fs',
            detection.detection_time.nanoseconds / 1e9,
        )

        pick_pose = await self._controller.lookup_conveyor_pick_pose()
        approach_pick = Pose()
        approach_pick.position = Point(
            x=pick_pose.position.x,
            y=pick_pose.position.y,
            z=pick_pose.position.z + self._controller.approach_height,
        )
        approach_pick.orientation = pick_pose.orientation

        await self._plan_and_execute_pose(approach_pick)
        await self._plan_and_execute_pose(pick_pose, linear=True, vsf=0.1)
        await self._controller.gripper.close()
        if not self._controller.gripper.is_holding:
            raise RuntimeError('Failed to grasp the cell on the conveyor')

        await self._plan_and_execute_pose(approach_pick, linear=True, vsf=0.2)

        tester_pose = await self._controller.lookup_tester_pose(tester_id)
        tester_approach = Pose()
        tester_approach.position = Point(
            x=tester_pose.position.x,
            y=tester_pose.position.y,
            z=tester_pose.position.z + self._controller.approach_height,
        )
        tester_approach.orientation = tester_pose.orientation

        await self._plan_and_execute_pose(tester_approach)
        await self._plan_and_execute_pose(tester_pose, linear=True, vsf=0.1)
        await self._controller.gripper.open()

        await self._plan_and_execute_pose(tester_approach, linear=True, vsf=0.2)
        plan = await self._controller.plan_to_named_configuration('home', vsf=0.3, asf=0.3)
        await self._controller.execute(plan)

    async def _plan_and_execute_pose(
        self,
        pose: Pose,
        *,
        linear: bool = False,
        vsf: float = 0.5,
        asf: float = 0.5,
    ) -> None:
        plan = await self._controller.plan_to_pose(pose, linear=linear, vsf=vsf, asf=asf)
        await self._controller.execute(plan)

    async def _await_latest_cell_detection(self, timeout: float = 5.0) -> BatteryCell:
        """Fetch the most recent breakbeam detection event."""

        latest: BatteryCell | None = None
        while not self._sensors.inspection_bb.cell_queue.empty():
            latest = self._sensors.inspection_bb.cell_queue.get_nowait()

        if latest is not None:
            return latest

        try:
            return await asyncio.wait_for(
                self._sensors.inspection_bb.cell_queue.get(), timeout=timeout
            )
        except asyncio.TimeoutError as exc:  # pragma: no cover - runtime guard
            raise TimeoutError('Timed out waiting for conveyor cell detection') from exc


class _InspectionRobot2Controller(ExampleInspectionRobot2):
    """Small extension over the example implementation for robot 2."""

    agv_frame_templates: Tuple[str, ...] = (
        'agv{agv}_tray_center_frame',
        'agv{agv}_kit_tray_frame',
        'agv{agv}_tray_frame',
    )

    def __init__(self) -> None:
        super().__init__()
        self.agv_targets: dict[int, Point] = {}
        self.orientations['agv'] = _quaternion_from_euler(math.pi, 0.0, 0.0)
        self._drop_clearance = 0.02
        self._approach_height = 0.12

    async def ready(self) -> None:
        await super().ready()
        for agv_id in (1, 2, 3):
            try:
                await self._resolve_agv_target(agv_id)
            except Exception as exc:  # noqa: BLE001 - cache best-effort frames
                self.get_logger().warn(
                    'Unable to pre-compute AGV %d tray transform: %s', agv_id, exc
                )

    async def place_cell_on_agv(self, agv_id: int) -> None:
        target = await self._resolve_agv_target(agv_id)

        approach = Pose(
            position=Point(
                x=target.x,
                y=target.y,
                z=target.z + self._approach_height,
            ),
            orientation=self.orientations['agv'],
        )
        drop = Pose(
            position=Point(
                x=target.x,
                y=target.y,
                z=target.z + self._drop_clearance,
            ),
            orientation=self.orientations['agv'],
        )

        await self._plan_and_execute_pose(approach)
        await self._plan_and_execute_pose(drop, linear=True, vsf=0.1)
        await self.gripper.open()
        await self._plan_and_execute_pose(approach, linear=True, vsf=0.2)
        plan = await self.plan_to_joint_state(self.joint_states['ready'], vsf=0.4, asf=0.4)
        await self.execute(plan)

    async def _resolve_agv_target(self, agv_id: int) -> Point:
        if agv_id in self.agv_targets:
            return self.agv_targets[agv_id]

        transform = await self._lookup_first_available(agv_id)
        translation = transform.transform.translation
        point = Point(x=translation.x, y=translation.y, z=translation.z)
        self.agv_targets[agv_id] = point
        return point

    async def _lookup_first_available(self, agv_id: int) -> TransformStamped:
        last_exception: Exception | None = None
        for template in self.agv_frame_templates:
            frame = template.format(agv=agv_id)
            try:
                return await AsyncUtils.get_tf_transform(self.tf_buffer, 'world', frame)
            except Exception as exc:  # noqa: BLE001 - keep trying fallbacks
                last_exception = exc
        message = f'Unable to resolve AGV {agv_id} tray frame'
        if last_exception is not None:
            raise RuntimeError(message) from last_exception
        raise RuntimeError(message)

    async def _plan_and_execute_pose(
        self,
        pose: Pose,
        *,
        linear: bool = False,
        vsf: float = 0.5,
        asf: float = 0.5,
    ) -> None:
        plan = await self.plan_to_pose(pose, linear=linear, vsf=vsf, asf=asf)
        await self.execute(plan)


@dataclass(slots=True)
class InspectionRobot2Handle(RobotHandle):
    """Concrete implementation for the AGV loading robot.

    中文說明：代表檢測區域的第二台機器人，負責測試器 → AGV 的轉運。
    """

    _controller: _InspectionRobot2Controller = field(init=False)

    def __post_init__(self) -> None:
        self._controller = _InspectionRobot2Controller()

    def nodes(self) -> Tuple[Node, ...]:
        return (self._controller,)

    async def _initialize(self) -> None:
        await self._controller.ready()
        await self._controller.gripper.open()

    async def load_cell_to_agv(self, *, tester_id: int, agv_id: int) -> None:
        """Transfer the cell from the voltage tester to the requested AGV."""

        await self.ensure_ready()

        await self._controller.pick_cell_from_tester(tester_id)
        await self._controller.place_cell_on_agv(agv_id)

