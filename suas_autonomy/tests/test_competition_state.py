import unittest

from suas_autonomy.mission.competition_state import CompetitionMission, Event, Phase


class CompetitionMissionTests(unittest.TestCase):
    def ready_for_tasks(self) -> CompetitionMission:
        mission = CompetitionMission(payload_count=2)
        mission.dispatch(Event.PRECHECK_PASSED)
        mission.dispatch(Event.AIRBORNE)
        mission.dispatch(Event.LAP_COMPLETED)
        return mission

    def test_takeoff_requires_lap_before_tasking(self):
        mission = CompetitionMission()
        self.assertEqual(mission.dispatch(Event.PRECHECK_PASSED).action, "command_takeoff")
        self.assertEqual(mission.dispatch(Event.AIRBORNE).current, Phase.WAYPOINT_LAP)
        self.assertEqual(mission.dispatch(Event.LAP_COMPLETED).current, Phase.TASKING)

    def test_drop_consumes_lap_credit(self):
        mission = self.ready_for_tasks()
        mission.dispatch(Event.REQUEST_DROP)
        mission.dispatch(Event.DROP_RELEASED)
        self.assertEqual(mission.phase, Phase.WAYPOINT_LAP)
        self.assertFalse(mission.lap_credit_available)
        with self.assertRaisesRegex(ValueError, "invalid"):
            mission.dispatch(Event.REQUEST_DROP)

    def test_mapping_and_search_do_not_consume_lap_credit(self):
        mission = self.ready_for_tasks()
        mission.dispatch(Event.REQUEST_MAPPING)
        mission.dispatch(Event.TASK_COMPLETED)
        mission.dispatch(Event.REQUEST_SEARCH)
        mission.dispatch(Event.TASK_COMPLETED)
        self.assertTrue(mission.lap_credit_available)

    def test_abort_always_requests_rtl(self):
        mission = self.ready_for_tasks()
        transition = mission.dispatch(Event.ABORT)
        self.assertEqual(transition.current, Phase.ABORTED)
        self.assertEqual(transition.action, "command_rtl")


if __name__ == "__main__":
    unittest.main()
