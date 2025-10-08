"""Concrete pipeline stages for the MVP inspection task.

中文說明：此檔案列出任務一 MVP 所需的實際 Stage，
每個 Stage 封裝一段可獨立測試的動作，例如啟動競賽、
輸送帶出料、機械臂取放與提交 kit。
"""

from __future__ import annotations

import asyncio
from ariac_interfaces.msg import CellTypes

from .base import StageContext, StageResult, TaskStage


class CompetitionBootStage(TaskStage):
    """Wait for READY and start the competition.

    中文說明：等待官方系統進入 READY，
    並呼叫開始競賽，作為整個流程的第一步。
    """

    def __init__(self) -> None:
        super().__init__('competition_boot')

    async def execute(self, context: StageContext) -> StageResult:
        env = context.environment
        context.coordinator.get_logger().info('Waiting for competition readiness')
        # 中文說明：阻塞直到競賽宣布 READY，避免過早操作。
        await env.wait_for_ready()
        context.coordinator.get_logger().info('Starting competition')
        # 中文說明：READY 後立即啟動，確保後續 Stage 有出料。
        await env.start_competition()
        return StageResult.success('Competition started')


class ConveyorPrimeStage(TaskStage):
    """Request a single cell from the inspection conveyor and pause it.

    中文說明：向檢測輸送帶索取一顆指定類型的電池，
    並在等待一段時間後關閉供料，方便 Robot1 在靜止狀態取貨。
    """

    def __init__(self, *, cell_type: int = CellTypes.LI_ION) -> None:
        super().__init__('conveyor_prime')
        self._cell_type = cell_type

    async def execute(self, context: StageContext) -> StageResult:
        env = context.environment
        conveyor = env.inspection_conveyor

        context.coordinator.get_logger().info(
            'Requesting a single cell on the inspection conveyor')
        # 中文說明：呼叫官方 API 啟動一次性出料。
        await conveyor.start_cell_feed(self._cell_type)
        # 中文說明：MVP 先用簡單 sleep 等待電池到位。
        await asyncio.sleep(context.params.get('feed_wait_s', 2.0))
        # 中文說明：停帶避免機械臂追逐移動目標。
        await conveyor.stop_cell_feed()
        return StageResult.success('Cell feed primed', cell_type=self._cell_type)


class PlaceCellOnTesterStage(TaskStage):
    """Inspection robot 1 grabs the cell and places it onto a tester.

    中文說明：指揮檢測臂 1 從輸送帶抓取電池，
    並放入指定編號的測試儀。
    """

    def __init__(self, tester_id: int) -> None:
        super().__init__('place_cell_on_tester')
        self._tester_id = tester_id

    async def execute(self, context: StageContext) -> StageResult:
        robot = context.robots['inspection_robot_1']
        context.coordinator.get_logger().info(
            f'Commanding {robot.description} to load tester {self._tester_id}')
        await robot.ensure_ready()
        # 中文說明：實際執行取放，細節封裝在 Robot Handle 中。
        await robot.place_cell_on_tester(self._tester_id)
        return StageResult.success('Cell placed onto tester', tester=self._tester_id)


class TesterBypassStage(TaskStage):
    """Bypass the actual voltage inspection while keeping the pipeline intact.

    中文說明：MVP 階段先跳過真實的電壓判定，
    只保留流程節點以後續替換。
    """

    def __init__(self) -> None:
        super().__init__('tester_bypass')

    async def execute(self, context: StageContext) -> StageResult:
        await asyncio.sleep(0.1)
        # 中文說明：保留微小延遲，模擬測試器工作時間。
        return StageResult.success('Voltage inspection bypassed for MVP')


class PlaceCellOnAgvStage(TaskStage):
    """Inspection robot 2 picks the cell from the tester and loads the AGV.

    中文說明：指揮檢測臂 2 從測試器取出電池，
    依照設定放入指定 AGV 托盤。
    """

    def __init__(self, tester_id: int, agv_id: int) -> None:
        super().__init__('place_cell_on_agv')
        self._tester_id = tester_id
        self._agv_id = agv_id

    async def execute(self, context: StageContext) -> StageResult:
        robot = context.robots['inspection_robot_2']
        context.coordinator.get_logger().info(
            f'Commanding {robot.description} to move cell from tester {self._tester_id} '
            f'to AGV {self._agv_id}')
        await robot.ensure_ready()
        # 中文說明：利用 Handle 實際呼叫行為，包含移動與夾持。
        await robot.load_cell_to_agv(tester_id=self._tester_id, agv_id=self._agv_id)
        return StageResult.success('Cell placed on AGV', tester=self._tester_id, agv=self._agv_id)


class SubmitKitStage(TaskStage):
    """Send the AGV to shipping and submit a kit for scoring.

    中文說明：當 AGV 上的 kit 完成後，
    先把 AGV 開到 Shipping，再呼叫提交服務。
    """

    def __init__(self, agv_id: int) -> None:
        super().__init__('submit_kit')
        self._agv_id = agv_id

    async def execute(self, context: StageContext) -> StageResult:
        env = context.environment
        context.coordinator.get_logger().info(
            f'Sending AGV {self._agv_id} to the shipping station')
        await env.send_agv_to_shipping(self._agv_id)

        context.coordinator.get_logger().info('Submitting kit for validation')
        success = await env.competition.submit_kit()
        if not success:
            # 中文說明：若提交失敗（例如 kit 未滿四顆），立即回報失敗方便除錯。
            return StageResult.failure('Kit submission failed')
        return StageResult.success('Kit successfully submitted', agv=self._agv_id)


class EndCompetitionStage(TaskStage):
    """Gracefully stop the competition once the smoke test is finished.

    中文說明：流程結束後停止競賽，確保模擬器回到安全狀態。
    """

    def __init__(self) -> None:
        super().__init__('competition_shutdown')

    async def execute(self, context: StageContext) -> StageResult:
        env = context.environment
        context.coordinator.get_logger().info('Stopping competition after MVP run')
        await env.stop_competition(shutdown=False)
        return StageResult.success('Competition ended')
