import unittest

from suas_autonomy.gcs.dashboard import StateStore, SyntheticStateSource


class GcsTests(unittest.TestCase):
    def test_synthetic_snapshot_has_competition_units_and_trail(self):
        store = StateStore(SyntheticStateSource(), [[-35.36, 149.16], [-35.35, 149.17]])
        snapshot = store.snapshot()
        aircraft = snapshot["aircraft"]
        self.assertGreater(aircraft["groundspeed_kt"], 0)
        self.assertGreater(aircraft["altitude_msl_ft"], aircraft["altitude_agl_ft"])
        self.assertEqual(len(snapshot["trail"]), 1)
        self.assertEqual(len(snapshot["boundary"]), 2)


if __name__ == "__main__":
    unittest.main()
