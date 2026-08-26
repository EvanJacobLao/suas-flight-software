import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from suas_autonomy.companion.benchmark import benchmark_resolution
from suas_autonomy.companion.capture_pipeline import SyntheticCamera, SyntheticTelemetry, run_pipeline
from suas_autonomy.mapping.dataset import DatasetReplay, validate_dataset, write_orthomosaic_manifest
from suas_autonomy.mapping.odm_handoff import prepare_odm_project
from suas_autonomy.mapping.planning import CameraGeometry, GeoPoint, capture_geometry, plan_survey
from suas_autonomy.mapping.preview_stitcher import analyze_pairs, stitch_images


class SurveyPlanningTests(unittest.TestCase):
    def test_capture_geometry(self):
        geometry = capture_geometry(CameraGeometry(81, 4096, 2160), 60, 0.75, 0.65, 18)
        self.assertGreater(geometry.footprint_width_m, 100)
        self.assertGreater(geometry.footprint_height_m, 50)
        self.assertAlmostEqual(geometry.capture_interval_s, geometry.along_track_spacing_m / 18)

    def test_plan_convex_boundary(self):
        boundary = [
            GeoPoint(-35.0, 149.0),
            GeoPoint(-35.0, 149.002),
            GeoPoint(-34.998, 149.002),
            GeoPoint(-34.998, 149.0),
        ]
        lines = plan_survey(boundary, heading_deg=0, along_spacing_m=20, cross_spacing_m=30)
        self.assertGreaterEqual(len(lines), 5)
        self.assertTrue(all(len(line.capture_points) >= 2 for line in lines))
        self.assertTrue(all(line.length_m > 150 for line in lines))


class DatasetTests(unittest.TestCase):
    def test_validate_manifest_and_replay(self):
        with tempfile.TemporaryDirectory() as temporary:
            dataset = Path(temporary)
            run_pipeline(SyntheticCamera(320, 180, 100), SyntheticTelemetry(), dataset, 3, 0)
            report, valid = validate_dataset(dataset)
            self.assertFalse(report.failed)
            self.assertEqual(report.valid_count, 3)
            self.assertEqual(len(list(DatasetReplay(valid))), 3)
            manifest = dataset / "orthomosaic_manifest.json"
            write_orthomosaic_manifest(valid, manifest)
            document = json.loads(manifest.read_text(encoding="utf-8"))
            self.assertEqual(document["capture_count"], 3)
            handoff = prepare_odm_project(dataset, dataset / "odm", "test_project")
            self.assertEqual(handoff["image_count"], 3)
            geo_lines = (dataset / "odm" / "test_project" / "geo.txt").read_text(encoding="utf-8").splitlines()
            self.assertEqual(geo_lines[0], "EPSG:4326")
            self.assertEqual(len(geo_lines), 4)

    def test_hash_tampering_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            dataset = Path(temporary)
            captures = run_pipeline(SyntheticCamera(160, 90, 100), SyntheticTelemetry(), dataset, 1, 0)
            captures[0][0].write_bytes(captures[0][0].read_bytes() + b"tampered")
            report, valid = validate_dataset(dataset)
            self.assertTrue(report.failed)
            self.assertEqual(valid, [])


class StitchingTests(unittest.TestCase):
    def test_overlapping_crops_match_and_stitch(self):
        rng = np.random.default_rng(7)
        base = rng.integers(0, 256, size=(320, 900, 3), dtype=np.uint8)
        for x in range(40, 860, 100):
            cv2.circle(base, (x, 160), 25, (0, 255, 0), 4)
            cv2.putText(base, str(x), (x - 25, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        images = [base[:, 0:450].copy(), base[:, 225:675].copy(), base[:, 450:900].copy()]
        paths = [Path(f"crop_{index}.jpg") for index in range(3)]
        matches = analyze_pairs(paths, images)
        self.assertTrue(all(match.acceptable for match in matches))
        status, panorama, _ = stitch_images(images)
        self.assertEqual(status, cv2.Stitcher_OK)
        self.assertIsNotNone(panorama)
        self.assertGreater(panorama.shape[1], 700)


class BenchmarkTests(unittest.TestCase):
    def test_small_benchmark(self):
        result = benchmark_resolution(320, 180, 2)
        self.assertGreater(result.quality_fps, 0)
        self.assertGreater(result.jpeg_fps, 0)


if __name__ == "__main__":
    unittest.main()
