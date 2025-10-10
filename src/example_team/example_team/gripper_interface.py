import asyncio
from typing import Any, cast

from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from ariac_interfaces.action import GripperCommand
from example_team.utils import AsyncUtils

class Gripper:
    def __init__(self, node: Node, robot_name: str):
        self.node = node

        action_name = f'{robot_name}/gripper_controller/gripper_command'
        self.action_client: ActionClient = ActionClient(self.node, GripperCommand, action_name)

        self._width = 0
        self._holding_object = False
        
        self._open_width = 0.04
        self._close_width = 0.015

        self._done_future: asyncio.Future

    @property
    def is_holding(self) -> bool:
        return self._holding_object

    @property
    def width(self) -> float:
        return self._width
    
    async def open(self):
        return await self._send_gripper_command(width=self._open_width)

    async def close(self):
        return await self._send_gripper_command(width=self._close_width)

    def _goal_response_callback(self, future: Any) -> None:
        goal_handle = cast(ClientGoalHandle, future.result())

        if not goal_handle.accepted:
            self._done_future.set_exception(RuntimeError('Gripper goal was not accepted.'))
            return

        result_future: Future  = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, result_future: Any) -> None:
        status = cast(int, result_future.result().status)
        result = cast(GripperCommand.Result, result_future.result().result)
        
        self._width = result.width

        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.stalled:
                self._holding_object = True

            self._done_future.set_result(result.width)
        else:
            self._done_future.set_exception(RuntimeError(f'Gripper failed with status {status}'))

    async def _send_gripper_command(self, width: float):
        await AsyncUtils.await_action_ready(self.action_client)

        goal_msg: GripperCommand.Goal = GripperCommand.Goal()
        goal_msg.width = width

        self._holding_object = False

        self._done_future = asyncio.get_event_loop().create_future()

        goal_future = self.action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._goal_response_callback)

        await self._done_future

        if (exception := self._done_future.exception()) is not None:
            raise exception