from __future__ import annotations

from typing import Protocol

from robotcan.core.types import DetectedObject, Observation, RobotAction, RobotGoal


class Perception(Protocol):
    def detect(self, observation: Observation) -> list[DetectedObject]:
        ...


class Policy(Protocol):
    def reset(self) -> None:
        ...

    def act(self, observation: Observation) -> RobotGoal:
        ...


class Planner(Protocol):
    def plan(self, observation: Observation, goal: RobotGoal) -> RobotAction:
        ...


class Recorder(Protocol):
    def record_step(self, observation: Observation, goal: RobotGoal, action: RobotAction) -> None:
        ...

