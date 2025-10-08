"""Light-weight robot handles used by the MVP pipeline.

中文說明：此模組提供簡化的機器人操作封裝，
目前僅以 sleep 模擬動作，未來可逐步替換為真實控制指令。
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass


@dataclass(slots=True)
class RobotHandle:
    """Common behaviour for MVP robot placeholders.

    中文說明：封裝共通屬性與 ready 檢查，
    讓各臂別在動作前確認已完成初始化。
    """

    name: str
    description: str
    _ready: bool = False

    async def ensure_ready(self) -> None:
        """Ensure the robot handle performed its lazy initialization.

        中文說明：第一次呼叫時以 sleep 模擬啟動耗時，
        並將 `_ready` 旗標設為 True，以後就不再等待。
        """

        if not self._ready:
            await asyncio.sleep(0.1)
            self._ready = True


class InspectionRobot1Handle(RobotHandle):
    """Placeholder for the conveyor robot.

    中文說明：代表檢測區域的第一台機器人，負責輸送帶 → 測試器的取放。
    """

    async def place_cell_on_tester(self, tester_id: int) -> None:
        """Simulate placing a cell on the tester.

        中文說明：目前僅用 sleep 模擬動作時間，
        後續會改為實際的 MoveIt/夾爪控制。
        """

        await asyncio.sleep(0.1)


class InspectionRobot2Handle(RobotHandle):
    """Placeholder for the AGV loading robot.

    中文說明：代表檢測區域的第二台機器人，負責測試器 → AGV 的轉運。
    """

    async def load_cell_to_agv(self, *, tester_id: int, agv_id: int) -> None:
        """Simulate transferring the cell from tester to AGV.

        中文說明：同樣先以 sleep 保留流程骨架，
        之後替換為真實的運動與夾具控制。
        """

        await asyncio.sleep(0.1)
