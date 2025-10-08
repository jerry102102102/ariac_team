"""Coordinator node for the end-to-end MVP smoke test.

中文說明：此節點負責串接整個任務一 MVP 的快樂路徑，
透過 pipeline 階段化的方式調用機器人與環境，
以便快速確認端到端流程是否運作正常。
"""

from __future__ import annotations

import asyncio
from typing import Dict

import rclpy
from rclpy.node import Node

from ariac_interfaces.msg import CellTypes

from ..environment import AriacEnvironment
from ..pipeline.base import Pipeline, StageBookkeeping, StageContext
from ..pipeline import stages
from ..robots import InspectionRobot1Handle, InspectionRobot2Handle


class MvpCoordinator(Node):
    """Orchestrates the happy-path pipeline for task one.

    中文說明：扮演高層協調者，負責初始化環境、機器人，
    並根據設定建立 Stage pipeline 後啟動執行。
    """

    def __init__(self) -> None:
        super().__init__('mvp_coordinator')

        # 中文說明：宣告可由 launch 檔覆寫的參數，
        # 以便在不同 trial 測試時快速切換測試器、AGV 以及 cell 類型。
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('tester_id', 1)
        self.declare_parameter('agv_id', 1)
        self.declare_parameter('cell_type', 'LI_ION')
        self.declare_parameter('feed_wait_s', 2.0)

        self.environment = AriacEnvironment()
        self.robots: Dict[str, InspectionRobot1Handle | InspectionRobot2Handle] = {
            # 中文說明：初始化我們在 MVP 中會操作的兩台檢測機器人，
            # 透過封裝的 Handle 介面進行動作呼叫。
            'inspection_robot_1': InspectionRobot1Handle(
                name='inspection_robot_1',
                description='Inspection Robot 1 (conveyor arm)'
            ),
            'inspection_robot_2': InspectionRobot2Handle(
                name='inspection_robot_2',
                description='Inspection Robot 2 (AGV arm)'
            ),
        }

        # 中文說明：ROS 參數拿到的 cell 型別為字串，需轉換成列舉的整數值。
        cell_type = self._resolve_cell_type(self.get_parameter('cell_type').get_parameter_value().string_value)
        tester_id = int(self.get_parameter('tester_id').value)
        agv_id = int(self.get_parameter('agv_id').value)

        self.pipeline = Pipeline([
            # 中文說明：依照任務流程建立 Stage 清單，之後會線性執行。
            stages.CompetitionBootStage(),
            stages.ConveyorPrimeStage(cell_type=cell_type),
            stages.PlaceCellOnTesterStage(tester_id=tester_id),
            stages.TesterBypassStage(),
            stages.PlaceCellOnAgvStage(tester_id=tester_id, agv_id=agv_id),
            stages.SubmitKitStage(agv_id=agv_id),
            stages.EndCompetitionStage(),
        ])

        self.bookkeeping = StageBookkeeping(self)
        # 中文說明：啟動計時器，等到 ROS 事件循環建立後再開始執行 pipeline，
        # 避免在 __init__ 階段直接跑協程。
        self._startup_timer = self.create_timer(0.1, self._kickoff_pipeline)

    def _resolve_cell_type(self, name: str) -> int:
        """Translate the string parameter into the CellTypes enum value.

        中文說明：確保傳入名稱合法；若不合法，回報所有可用選項以利除錯。
        """

        if not name:
            return CellTypes.LI_ION

        try:
            return getattr(CellTypes, name)
        except AttributeError as exc:  # noqa: B904
            valid = ', '.join([n for n in dir(CellTypes) if n.isupper()])
            raise ValueError(f'Unknown cell type "{name}". Expected one of: {valid}') from exc

    def _kickoff_pipeline(self) -> None:
        """Start the asynchronous pipeline execution once the node is ready.

        中文說明：確保只觸發一次，並在 asyncio 事件圈中建立背景任務。
        """

        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None

        loop = asyncio.get_running_loop()
        loop.create_task(self._run())

    async def _run(self) -> None:
        """Entry point of the sequential MVP pipeline.

        中文說明：建立 StageContext，帶入協調節點、環境與機器人，
        然後依序執行每個 Stage 並記錄結果。
        """

        context = StageContext(
            coordinator=self,
            environment=self.environment,
            robots=self.robots,
            params={'feed_wait_s': float(self.get_parameter('feed_wait_s').value)}
        )

        results = await self.pipeline.run(context)
        for result in results:
            # 中文說明：逐筆寫入 StageBookkeeping，之後可以查詢整體結果。
            await self.bookkeeping.record(result)

        summary = ', '.join(f"{result.status.name}:{result.message}" for result in results)
        self.get_logger().info(f'MVP pipeline finished: {summary}')


def main() -> None:
    """ROS 2 entry point to spin the coordinator node.

    中文說明：建立 AsyncIO 執行器，並同時管理協調節點與環境節點。
    """

    rclpy.init()
    coordinator = MvpCoordinator()
    executor = rclpy.executors.AsyncIOExecutor()

    executor.add_node(coordinator)
    executor.add_node(coordinator.environment)

    try:
        executor.spin()
    finally:
        # 中文說明：確保離開時釋放 ROS 資源，避免下次啟動殘留。
        executor.shutdown()
        coordinator.environment.destroy_node()
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
