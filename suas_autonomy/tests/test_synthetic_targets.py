import unittest

from suas_autonomy.vision.synthetic_targets import detect_test_targets, evaluate, generate_scene


class SyntheticTargetTests(unittest.TestCase):
    def test_generated_targets_are_detected_and_localized(self):
        image, truths = generate_scene(width=640, height=360, target_count=3, seed=11)
        detections = detect_test_targets(
            image,
            metres_per_pixel=0.08,
            origin_lat=-35.363262,
            origin_lon=149.165237,
        )
        result = evaluate(truths, detections)
        self.assertEqual(result["matched_count"], 3)
        self.assertEqual(result["recall"], 1.0)
        self.assertLess(result["mean_centre_error_px"], 3.0)


if __name__ == "__main__":
    unittest.main()
