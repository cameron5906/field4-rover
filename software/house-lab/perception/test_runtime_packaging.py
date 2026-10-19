"""Offline tests of artifact verification and portable archive assembly."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import zipfile
from build_windows_runtime import ROOT, checked_artifact, extract_archive


class RuntimePackagingTests(unittest.TestCase):
    def test_pinned_models(self):
        for record, model in [("model_provenance.json", "efficientdet_lite0.tflite"), ("pose_model_provenance.json", "pose_landmarker_lite.task")]:
            metadata = json.loads((ROOT / record).read_text())
            model_path = ROOT / "models" / model
            if not model_path.is_file():
                self.skipTest("Optional upstream model weights are not bundled in this source-only subset")
            data = model_path.read_bytes()
            self.assertEqual(len(data), metadata["bytes"])
            self.assertEqual(hashlib.sha256(data).hexdigest(), metadata["sha256"])

    def test_manifest_covers_pinned_requirements(self):
        manifest = json.loads((ROOT / "windows_runtime_manifest.json").read_text())
        norm = lambda value: value.lower().replace("_", "-")
        requirements = {norm(line.strip()) for line in (ROOT / "requirements.txt").read_text().splitlines() if line and not line.startswith("#")}
        wheels = {norm(item["name"] + "==" + item["version"]) for item in manifest["wheels"]}
        self.assertEqual(requirements, wheels)
        for item in manifest["wheels"]:
            self.assertTrue(item["url"].startswith("https://files.pythonhosted.org/"))
            self.assertEqual(len(item["sha256"]), 64)
            self.assertTrue(item["filename"].endswith(("win_amd64.whl", "none-any.whl")))

    def test_hash_mismatch_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory)
            (cache / "bad.zip").write_bytes(b"tampered")
            with self.assertRaises(ValueError):
                checked_artifact({"filename": "bad.zip", "sha256": "0" * 64, "bytes": 8}, cache, offline=True)

    def test_offline_missing_file_does_not_download(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileNotFoundError):
                checked_artifact({"filename": "missing.zip"}, Path(directory), offline=True)

    def test_zip_traversal_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            archive = Path(directory) / "bad.zip"
            with zipfile.ZipFile(archive, "w") as zipped:
                zipped.writestr("../escape.txt", "no")
            with self.assertRaises(ValueError):
                extract_archive(archive, Path(directory) / "extract")
            self.assertFalse((Path(directory) / "escape.txt").exists())

    def test_wheel_data_relocation(self):
        with tempfile.TemporaryDirectory() as directory:
            archive = Path(directory) / "test.whl"
            with zipfile.ZipFile(archive, "w") as zipped:
                zipped.writestr("demo/__init__.py", "")
                zipped.writestr("demo.data/purelib/extra.py", "test = True")
                zipped.writestr("demo.data/scripts/unneeded.exe", "not installed")
            output = Path(directory) / "extract"
            extract_archive(archive, output, wheel=True)
            self.assertTrue((output / "demo/__init__.py").exists())
            self.assertTrue((output / "extra.py").exists())
            self.assertFalse((output / "demo.data").exists())

    def test_assembled_runtime_path(self):
        runtime = ROOT / "runtime_windows"
        if not runtime.exists():
            self.skipTest("Cross-assembled runtime absent")
        self.assertEqual((runtime / "python312._pth").read_text().splitlines(), ["python312.zip", ".", "..", "Lib/site-packages", "import site"])
        for path in ["python.exe", "pythonw.exe", "python312.dll", "LICENSE.txt", "Lib/site-packages/mediapipe/tasks/c/libmediapipe.dll"]:
            self.assertTrue((runtime / path).is_file(), path)


if __name__ == "__main__":
    unittest.main()
