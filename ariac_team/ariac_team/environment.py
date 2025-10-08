"""Thin wrapper around the example environment interface.

中文說明：透過繼承官方 example 的 Environment 介面，
提供 MVP 流程需要的精簡便利函式，避免在 Stage 中重複樣板程式。
"""

from __future__ import annotations

from ariac_interfaces.msg import AgvStations

from example_team.environment_interface import Environment as ExampleEnvironment


class AriacEnvironment(ExampleEnvironment):
    """Expose a few convenience helpers used by the MVP pipeline.

    中文說明：包裝我們常用的環境操作，
    例如等待競賽開始、送 AGV 到特定站點等。
    """

    shipping_station: int = AgvStations.SHIPPING

    async def wait_for_ready(self) -> None:
        """Await the competition ready event from the simulator.

        中文說明：利用 example 介面的同步事件，
        阻塞直到官方系統發出 READY 狀態，確保後續動作合法。
        """

        await self.competition.ready.wait()

    async def start_competition(self) -> None:
        """Trigger the official start service.

        中文說明：呼叫環境提供的 start() 協程，
        正式開始計時與出料。
        """

        await self.competition.start()

    async def stop_competition(self, *, shutdown: bool = False) -> None:
        """Stop the competition gracefully.

        中文說明：結束競賽；若 `shutdown=True`，
        會連同模擬器做最終釋放，用於終止流程。
        """

        await self.competition.end(shutdown=shutdown)

    async def send_agv_to_shipping(self, agv_id: int) -> None:
        """Command an AGV to drive to the shipping station.

        中文說明：把指定的 AGV 移動到出貨站，
        作為提交 kit 前的標準流程。
        """

        await self.agvs[agv_id].move(self.shipping_station)
