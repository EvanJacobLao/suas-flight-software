import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from suas_autonomy.companion.capture_pipeline import (
    SyntheticCamera,
    SyntheticTelemetry,
    image_quality,
    run_pipeline,
)
from suas_autonomy.companion.orin_diagnostics import check_memory, check_storage


class CapturePipelineTests(unittest.TestCase):
    def test_quality_rejects_blank_dark_frame(self):
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        result = image_quality(frame)
        self.assertFalse(result["acceptable"])
        self.assertGreater(result["dark_fraction"], 0.9)

    def test_synthetic_capture_writes_image_and_metadata(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            captures = run_pipeline(
                SyntheticCamera(320, 180, frame_rate=100),
                SyntheticTelemetry(),
                output,
                capture_count=2,
                capture_interval_s=0,
            )
            self.assertEqual(len(captures), 2)
            for image_path, metadata_path in captures:
                self.assertTrue(image_path.exists())
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                self.assertEqual(metadata["schema_version"], 1)
                self.assertEqual(metadata["telemetry"]["source"], "synthetic")
                self.assertEqual(metadata["gimbal"]["pitch_deg"], -90.0)
                self.assertEqual(len(metadata["image_sha256"]), 64)


class DiagnosticsTests(unittest.TestCase):
    def test_memory_check_returns_known_status(self):
        self.assertIn(check_memory().status, {"PASS", "WARN"})

    def test_storage_check_current_directory(self):
        self.assertIn(check_storage(Path.cwd()).status, {"PASS", "WARN"})


if __name__ == "__main__":
    unittest.main()
