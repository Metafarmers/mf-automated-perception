from pathlib import Path
import pytest

from mflib.perception.automated_perception.grain.grain_factory import GrainFactory
from mflib.perception.automated_perception.grain.grain_base import GrainBase
from mflib.perception.automated_perception.grain.defs._dummy import Dummy

# test order
from mflib.perception.automated_perception.tests.integration_phase import IntegrationPhase
pytestmark = [
  pytest.mark.integration,
  pytest.mark.order(IntegrationPhase.LOCATE_ROSBAG_3_GRAIN_FACTORY),
]


def test_grain_factory_real_environment():
  """
  Real-run test.

  Assumes grain package is already installed and importable,
  with the following structure:

    grain/
      defs/
        raw_rosbag.py
        raw_rosbag_path.py
        _dummy.py
  """

  # --------------------------------------------------
  # registry sanity
  # --------------------------------------------------
  keys = GrainFactory.list_keys()
  assert len(keys) > 0

  # _dummy should be skipped
  assert all(not any(k.startswith("_") for k in key) for key in keys)

  # --------------------------------------------------
  # representative real grain resolve
  # --------------------------------------------------
  key = ("raw", "rosbag", "path")
  cls = GrainFactory.resolve_class(key)

  assert issubclass(cls, GrainBase)
  assert cls.__name__ == "RawRosbagPath"


def test_display_latest_raw_rosbag_path():
  """
  Real-run smoke test.

  - Find latest ('raw','rosbag','path') grain
  - If not exists: silently return
  - If exists:
      - print_grain_summary()
      - summarize_db()
  """

  # --------------------------------------------------
  # locate grain package root (installed environment)
  # --------------------------------------------------

  key = ("raw", "rosbag", "path")

  # --------------------------------------------------
  # locate grain_data_root
  # (assumes GrainBase global config is already loaded)
  # --------------------------------------------------
  dummy_grain = Dummy()
  grain_data_root = dummy_grain.grain_data_root

  # --------------------------------------------------
  # try loading latest grain
  # --------------------------------------------------
  grain = GrainFactory.load_latest_grain(
    key=key,
    grain_data_root=grain_data_root,
  )
  if grain is None:
    # no grain found → nothing to test
    print("No latest ('raw','rosbag','path') grain found; skipping test.")
    return

  # --------------------------------------------------
  # display grain info (side-effect based test)
  # --------------------------------------------------
  print("\n========== LATEST RAW ROSBAG PATH GRAIN ==========")
  grain.print_grain_summary()

  try:
    grain.summarize_db()
  except Exception as e:
    pytest.fail(f"summarize_db() failed: {e}")
