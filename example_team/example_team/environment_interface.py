from typing import cast, Any

import asyncio

from rclpy.node import Node
from rclpy.task import Future
from rclpy.parameter import Parameter
from rclpy.time import Duration
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from ariac_interfaces.msg import (
    AgvStations,
    CompetitionStates,
    CompetitionStatus,
    CompetitionTime,
    CellTypes,
    ConveyorStatus,
    CellFeederStatus,
    InspectionReport,
    OperationStates
)

from ariac_interfaces.srv import (
    Trigger,
    EndCompetition,
    ControlCellFeeder,
    SubmitInspectionReport
)

from ariac_interfaces.action import MoveAgv

from example_team.utils import AsyncUtils

class Environment(Node):
    def __init__(self):
        super().__init__(f'environment_interface')

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.competition = Competition(self)
        self.inspection_conveyor = InspectionConveyor(self)

        self.agvs: dict[int, AGV] = {i: AGV(self, i) for i in (1,2,3)}

class Competition:
    def __init__(self, node: Node):
        self._node = node
        self._start_srv = self._node.create_client(Trigger, '/start_competition')
        self._end_srv = self._node.create_client(EndCompetition, '/end_competition')
        self._submit_kit_srv = self._node.create_client(Trigger, '/submit_kitting_order')
        
        self._state: int | None = None
        self._time: CompetitionTime | None = None

        self.ready = asyncio.Event()
        self.started = asyncio.Event()

        self._node.create_subscription(CompetitionStatus, '/competition_status', self._status_cb, 10)

    @property
    def state(self) -> int:
        if self._state is None:
            return -1
        
        return self._state
    
    @property
    def time_remaining(self) -> Duration:
        if self._time is None:
            return Duration()
        
        return Duration.from_msg(self._time.remaining)
    
    async def start(self):
        await AsyncUtils.await_service_ready(self._start_srv)

        result = await AsyncUtils.await_service_response(self._start_srv, Trigger.Request())
        
        response = cast(Trigger.Response, result)

        if not response.success:
            raise RuntimeError(response.message)
        
        # Wait for competition status to be STARTED
        await self.started.wait()
        
    async def end(self, shutdown=False):
        await AsyncUtils.await_service_ready(self._end_srv)

        req = EndCompetition.Request()
        req.shutdown_gazebo = shutdown

        result = await AsyncUtils.await_service_response(self._end_srv, req)
        
        response = cast(Trigger.Response, result)

        if not response.success:
            raise RuntimeError(response.message)
        
    async def submit_kit(self) -> bool:
        await AsyncUtils.await_service_ready(self._submit_kit_srv)

        result = await AsyncUtils.await_service_response(self._submit_kit_srv, Trigger.Request())
        
        response = cast(Trigger.Response, result)

        self._node.get_logger().info(f"Kit submission response: {response.message}")

        return response.success
    
    def _status_cb(self, msg: CompetitionStatus):
        self._state = msg.competition_state
        self._time = msg.time

        if self._state == CompetitionStates.READY:
            self.ready.set()
        
        if self._state == CompetitionStates.STARTED:
            self.started.set()

class InspectionConveyor:
    def __init__(self, node: Node):
        self._node = node
        self._speed: float
        self._cell_type: int
        self._feed_rate: float
        self._operating_status: int
        
        self._status_sub = self._node.create_subscription(ConveyorStatus, '/inspection_conveyor/status', self._conveyor_status_cb, 10)
        self._cell_feeder_status_sub = self._node.create_subscription(CellFeederStatus, '/inspection_conveyor/cell_feed/status', self._cell_feeder_status_cb, 10)

        self._control_srv = self._node.create_client(ControlCellFeeder, '/inspection_conveyor/cell_feed/control')
        self._submit_report_srv = self._node.create_client(SubmitInspectionReport, '/inspection_conveyor/inspection/submit')
    
    @property
    def speed(self) -> float:
        return self._speed
    
    @property
    def cell_type(self) -> int:
        return self._cell_type
    
    @property
    def feed_rate(self) -> float:
        return self._feed_rate
    
    @property
    def malfunctioning(self) -> bool:
        return self._operating_status == OperationStates.MALFUNCTIONING
    
    async def start_cell_feed(self, cell_type: int):
        await AsyncUtils.await_service_ready(self._control_srv)

        request = ControlCellFeeder.Request()
        request.cell_type = cell_type

        result = await AsyncUtils.await_service_response(self._control_srv, request)
        
        response = cast(ControlCellFeeder.Response, result)

        if not response.success:
            raise RuntimeError(response.message)

    async def stop_cell_feed(self):
        await AsyncUtils.await_service_ready(self._control_srv)
        
        request = ControlCellFeeder.Request()
        request.cell_type = CellTypes.NONE

        result = await AsyncUtils.await_service_response(self._control_srv, request)

        response = cast(ControlCellFeeder.Response, result)

        if not response.success:
            raise RuntimeError(response.message)
        
    async def submit_inspection_report(self, report: InspectionReport):
        await AsyncUtils.await_service_ready(self._submit_report_srv)
        
        request = SubmitInspectionReport.Request()
        request.report = report

        result = await AsyncUtils.await_service_response(self._submit_report_srv, request)
        
        response = cast(SubmitInspectionReport.Response, result)

        if not response.success:
            raise RuntimeError(response.message)

    def _conveyor_status_cb(self, msg: ConveyorStatus):
        self._speed = msg.speed
        self._operating_status = msg.operating_status
    
    def _cell_feeder_status_cb(self, msg: CellFeederStatus):
        self._cell_type = msg.cell_type
        self._feed_rate = msg.feed_rate

class AGV:
    def __init__(self, node: Node, agv: int):
        self.node = node

        action_name = f'agv{agv}/move'
        self.action_client: ActionClient = ActionClient(self.node, MoveAgv, action_name)

        self._done_future: asyncio.Future

        self._station: int = AgvStations.INSPECTION
    
    @property
    def station(self) -> int:
        return self._station

    async def move(self, station: int):
        await AsyncUtils.await_action_ready(self.action_client)

        goal_msg: MoveAgv.Goal = MoveAgv.Goal()
        goal_msg.station_id = station

        self._holding_object = False

        self._done_future = asyncio.get_event_loop().create_future()

        goal_future = self.action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._goal_response_callback)

        await self._done_future

        if (exception := self._done_future.exception()) is not None:
            raise exception

    def _goal_response_callback(self, future: Any) -> None:
        goal_handle = cast(ClientGoalHandle, future.result())

        if not goal_handle.accepted:
            self._done_future.set_exception(RuntimeError('Gripper goal was not accepted.'))
            return

        result_future: Future  = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, result_future: Any) -> None:
        status = cast(int, result_future.result().status)
        result = cast(MoveAgv.Result, result_future.result().result)
        
        self._station = result.status.station_id

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._done_future.set_result(True)
        else:
            self._done_future.set_exception(RuntimeError(f'Agv move failed with status {status}'))
        