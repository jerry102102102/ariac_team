import asyncio
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import ariac_interfaces  # type: ignore[import]
except ModuleNotFoundError:  # pragma: no cover - test shim
    import types

    ariac_interfaces = types.ModuleType('ariac_interfaces')
    msg_module = types.ModuleType('ariac_interfaces.msg')

    class _StubCellTypes:
        LI_ION = 1
        NONE = 0

    msg_module.CellTypes = _StubCellTypes
    ariac_interfaces.msg = msg_module

    sys.modules['ariac_interfaces'] = ariac_interfaces
    sys.modules['ariac_interfaces.msg'] = msg_module

try:
    import rclpy  # type: ignore[import]
except ModuleNotFoundError:  # pragma: no cover - test shim
    import types

    rclpy = types.ModuleType('rclpy')
    node_module = types.ModuleType('rclpy.node')

    class _StubLogger:
        def info(self, *_args, **_kwargs):
            return None

        def warning(self, *_args, **_kwargs):
            return None

        def error(self, *_args, **_kwargs):
            return None

    class _StubNode:  # minimal stub used only for typing
        def __init__(self) -> None:
            self._logger = _StubLogger()

        def get_logger(self):
            return self._logger

    node_module.Node = _StubNode
    rclpy.node = node_module

    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = node_module

from ariac_team.pipeline import StageContext, StageStatus
from ariac_team.pipeline.stages import SubmitKitStage


class _DummyLogger:
    def info(self, *_args, **_kwargs):
        return None

    def warning(self, *_args, **_kwargs):
        return None

    def error(self, *_args, **_kwargs):
        return None


class _DummyNode:
    def get_logger(self):
        return _DummyLogger()


class _DummyCompetition:
    def __init__(self, *, success: bool = True):
        self.success = success
        self.submit_calls = 0

    async def submit_kit(self) -> bool:
        self.submit_calls += 1
        await asyncio.sleep(0)
        return self.success


class _DummyEnvironment:
    def __init__(self, competition: _DummyCompetition):
        self.competition = competition
        self.shipped = []

    async def send_agv_to_shipping(self, agv_id: int) -> None:
        self.shipped.append(agv_id)
        await asyncio.sleep(0)


def test_submit_kit_stage_requires_full_tray():
    competition = _DummyCompetition()
    environment = _DummyEnvironment(competition)
    context = StageContext(
        coordinator=_DummyNode(),
        environment=environment,
        robots={},
        params={'cells_required': 4, 'cells_loaded': 2},
    )

    stage = SubmitKitStage(agv_id=1)
    result = asyncio.run(stage.run(context))

    assert result.status is StageStatus.FAILURE
    assert competition.submit_calls == 0
    assert environment.shipped == []


def test_submit_kit_stage_submits_when_full():
    competition = _DummyCompetition(success=True)
    environment = _DummyEnvironment(competition)
    context = StageContext(
        coordinator=_DummyNode(),
        environment=environment,
        robots={},
        params={'cells_required': 4, 'cells_loaded': 4},
    )

    stage = SubmitKitStage(agv_id=2)
    result = asyncio.run(stage.run(context))

    assert result.status is StageStatus.SUCCESS
    assert competition.submit_calls == 1
    assert environment.shipped == [2]
    assert result.payload['loaded'] == 4
    assert result.payload['required'] == 4
