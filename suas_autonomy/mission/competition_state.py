"""Pure competition mission state machine.

The module contains no MAVLink calls.  SITL, replay, and eventually the real
aircraft adapter feed it events and execute the returned high-level actions.
This makes lap/drop ordering and abort behaviour deterministic and testable.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class Phase(str, Enum):
    PRECHECK = "precheck"
    TAKEOFF = "takeoff"
    WAYPOINT_LAP = "waypoint_lap"
    TASKING = "tasking"
    DROP_RUN = "drop_run"
    LANDING = "landing"
    COMPLETE = "complete"
    ABORTED = "aborted"


class Event(str, Enum):
    PRECHECK_PASSED = "precheck_passed"
    AIRBORNE = "airborne"
    LAP_COMPLETED = "lap_completed"
    REQUEST_MAPPING = "request_mapping"
    REQUEST_SEARCH = "request_search"
    REQUEST_DROP = "request_drop"
    TASK_COMPLETED = "task_completed"
    DROP_RELEASED = "drop_released"
    REQUEST_LAND = "request_land"
    LANDED = "landed"
    ABORT = "abort"


@dataclass(frozen=True)
class Transition:
    previous: Phase
    current: Phase
    event: Event
    action: str


@dataclass
class CompetitionMission:
    """Enforce a lap before tasking and a new lap after every drop."""

    payload_count: int = 4
    phase: Phase = Phase.PRECHECK
    completed_laps: int = 0
    released_payloads: int = 0
    lap_credit_available: bool = False
    active_task: str | None = None
    history: list[Transition] = field(default_factory=list)

    def dispatch(self, event: Event) -> Transition:
        previous = self.phase

        if event is Event.ABORT:
            self.phase = Phase.ABORTED
            return self._record(previous, event, "command_rtl")
        if self.phase in {Phase.COMPLETE, Phase.ABORTED}:
            raise ValueError(f"mission is terminal ({self.phase.value})")

        action = self._transition(event)
        return self._record(previous, event, action)

    def _transition(self, event: Event) -> str:
        if self.phase is Phase.PRECHECK and event is Event.PRECHECK_PASSED:
            self.phase = Phase.TAKEOFF
            return "command_takeoff"
        if self.phase is Phase.TAKEOFF and event is Event.AIRBORNE:
            self.phase = Phase.WAYPOINT_LAP
            return "fly_waypoint_lap"
        if self.phase is Phase.WAYPOINT_LAP and event is Event.LAP_COMPLETED:
            self.completed_laps += 1
            self.lap_credit_available = True
            self.phase = Phase.TASKING
            return "await_task"
        if self.phase is Phase.TASKING and event in {Event.REQUEST_MAPPING, Event.REQUEST_SEARCH}:
            self.active_task = "mapping" if event is Event.REQUEST_MAPPING else "search"
            return f"start_{self.active_task}"
        if self.phase is Phase.TASKING and event is Event.TASK_COMPLETED:
            if self.active_task is None:
                raise ValueError("no mapping/search task is active")
            completed = self.active_task
            self.active_task = None
            return f"finish_{completed}"
        if self.phase is Phase.TASKING and event is Event.REQUEST_DROP:
            if self.active_task is not None:
                raise ValueError("finish the active task before starting a drop run")
            if not self.lap_credit_available:
                raise ValueError("a complete waypoint lap is required before each drop")
            if self.released_payloads >= self.payload_count:
                raise ValueError("no payloads remain")
            self.phase = Phase.DROP_RUN
            return "compute_release_and_begin_drop_run"
        if self.phase is Phase.DROP_RUN and event is Event.DROP_RELEASED:
            self.released_payloads += 1
            self.lap_credit_available = False
            self.phase = Phase.WAYPOINT_LAP
            return "fly_waypoint_lap"
        if self.phase is Phase.TASKING and event is Event.REQUEST_LAND:
            if self.active_task is not None:
                raise ValueError("finish the active task before landing")
            self.phase = Phase.LANDING
            return "command_land"
        if self.phase is Phase.LANDING and event is Event.LANDED:
            self.phase = Phase.COMPLETE
            return "safe_shutdown"
        raise ValueError(f"event {event.value} is invalid during {self.phase.value}")

    def _record(self, previous: Phase, event: Event, action: str) -> Transition:
        transition = Transition(previous, self.phase, event, action)
        self.history.append(transition)
        return transition

    def snapshot(self) -> dict[str, object]:
        return {
            "phase": self.phase.value,
            "completed_laps": self.completed_laps,
            "released_payloads": self.released_payloads,
            "payloads_remaining": self.payload_count - self.released_payloads,
            "lap_credit_available": self.lap_credit_available,
            "active_task": self.active_task,
        }
