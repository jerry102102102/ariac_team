"""Pipeline primitives for the ARIAC MVP tasks.

中文說明：此模組提供 MVP 階段流程的基礎元件，
協助我們以階段（Stage）方式包裝任務流程，方便除錯與追蹤。
"""

from __future__ import annotations

import abc
import asyncio
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, Iterable, List

from rclpy.node import Node


class StageStatus(Enum):
    """Outcome of an individual stage execution.

    中文說明：描述單一階段執行後的結果狀態，
    方便後續決策是否繼續執行下一階段。
    """

    SUCCESS = auto()
    FAILURE = auto()
    SKIPPED = auto()


@dataclass(slots=True)
class StageResult:
    """Normalized stage execution result.

    中文說明：統一包裝每個階段返回的結果，
    包含狀態、訊息以及額外的資料載荷，
    讓 pipeline 可以一致地處理成功或失敗。
    """

    status: StageStatus
    message: str = ""
    payload: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def success(cls, message: str = "", **payload: Any) -> "StageResult":
        """工廠函式：建立標示成功的結果。

        中文說明：方便每個 Stage 在完成工作時
        快速建立統一格式的成功結果。
        """

        return cls(StageStatus.SUCCESS, message=message, payload=payload)

    @classmethod
    def failure(cls, message: str, **payload: Any) -> "StageResult":
        """工廠函式：建立標示失敗的結果。

        中文說明：失敗時除了記錄訊息，
        也可補充 payload 協助除錯。
        """

        return cls(StageStatus.FAILURE, message=message, payload=payload)

    @classmethod
    def skipped(cls, message: str = "", **payload: Any) -> "StageResult":
        """工廠函式：建立被略過的結果。

        中文說明：當某些階段因條件不成立而略過時，
        使用此方法仍保留紀錄，方便後續追蹤。
        """

        return cls(StageStatus.SKIPPED, message=message, payload=payload)


@dataclass(slots=True)
class StageContext:
    """Shared runtime context passed to each stage.

    中文說明：統一管理執行階段時需要的共用資源，
    像是協調節點、環境句柄、機器人介面與自訂參數。
    """

    coordinator: Node
    environment: Node
    robots: Dict[str, Any]
    params: Dict[str, Any] = field(default_factory=dict)

    def with_updates(self, **kwargs: Any) -> "StageContext":
        """Return a new context with merged parameter overrides.

        中文說明：建立帶有額外參數的 context 副本，
        讓下游 Stage 能閱讀到前面階段產生的狀態或設定。
        """

        updated = dict(self.params)
        updated.update(kwargs)
        return StageContext(
            coordinator=self.coordinator,
            environment=self.environment,
            robots=self.robots,
            params=updated,
        )


class TaskStage(abc.ABC):
    """Base class for all stages in the MVP pipeline.

    中文說明：所有任務階段的共同父類別，
    定義標準生命週期（setup → execute → teardown）。
    """

    def __init__(self, name: str):
        self._name = name

    @property
    def name(self) -> str:
        return self._name

    async def run(self, context: StageContext) -> StageResult:
        """Execute the stage lifecycle.

        中文說明：依序執行前置、主要邏輯與收尾，
        確保資源被妥善開啟與釋放。
        """

        await self.setup(context)
        try:
            result = await self.execute(context)
        finally:
            await self.teardown(context)
        return result

    async def setup(self, context: StageContext) -> None:  # pragma: no cover - hook
        """Lifecycle hook for pre-execution preparation.

        中文說明：必要時可覆寫此方法準備狀態，
        預設為空實作。
        """
        return None

    @abc.abstractmethod
    async def execute(self, context: StageContext) -> StageResult:
        raise NotImplementedError

    async def teardown(self, context: StageContext) -> None:  # pragma: no cover - hook
        """Lifecycle hook for cleanup tasks.

        中文說明：讓子類別在執行後可釋放資源，
        預設為空實作。
        """
        return None


class Pipeline:
    """Sequential orchestrator for the task stages.

    中文說明：將多個 Stage 串成線性流程，
    逐一執行並記錄結果。
    """

    def __init__(self, stages: Iterable[TaskStage]):
        self._stages: List[TaskStage] = list(stages)

    async def run(self, context: StageContext) -> List[StageResult]:
        results: List[StageResult] = []
        for stage in self._stages:
            # 中文說明：逐步執行 Stage，並記錄在 ROS 日誌中，方便觀察流程進度。
            context.coordinator.get_logger().info(
                f"Starting stage: {stage.name}")
            try:
                result = await stage.run(context)
            except Exception as exc:  # noqa: BLE001 - propagate failure details via result
                # 中文說明：捕捉未預期例外，避免整個流程崩潰，
                # 轉換為失敗結果後繼續記錄。
                context.coordinator.get_logger().error(
                    f"Stage '{stage.name}' raised an exception: {exc}")
                result = StageResult.failure(str(exc))
            results.append(result)

            if result.status is StageStatus.FAILURE:
                # 中文說明：若有階段失敗，立即中止後續流程，
                # 避免在不一致狀態下繼續操作硬體。
                context.coordinator.get_logger().error(
                    f"Aborting pipeline after '{stage.name}' failure")
                break
        return results


class StageBookkeeping:
    """Utility to publish the aggregate status of a pipeline run.

    中文說明：集中記錄每個 Stage 的結果，
    方便日後查詢整體流程狀態。
    """

    def __init__(self, node: Node):
        self._node = node
        self._lock = asyncio.Lock()
        self._results: List[StageResult] = []

    async def record(self, result: StageResult) -> None:
        async with self._lock:
            # 中文說明：使用 asyncio.Lock 確保多協程同時寫入時安全。
            self._results.append(result)

    async def snapshot(self) -> List[StageResult]:
        async with self._lock:
            # 中文說明：回傳結果的複本，避免外部修改內部狀態。
            return list(self._results)
