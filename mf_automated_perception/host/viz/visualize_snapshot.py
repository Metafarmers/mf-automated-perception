from pathlib import Path

import rerun as rr
import cv2

from mf_automated_perception.env import GRAIN_DATA_ROOT
from mf_automated_perception.grain.grain_factory import GrainFactory
from mf_automated_perception.realtime.sync.time_utils import to_sec
from mf_automated_perception.realtime.sync.engine import (
  SnapshotTimeline,
  load_sync_results_from_snapshot_db,
)
from mf_automated_perception.realtime.datatypes.image import MfImage


def _load_image_for_rerun(img: MfImage):
  """
  Load image data for rerun.
  """
  path = GRAIN_DATA_ROOT / Path(img.path_to_raw)
  if not path.exists():
    raise FileNotFoundError(path)

  # OpenCV loads BGR
  bgr = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
  if bgr is None:
    raise RuntimeError(f"Failed to read image: {path}")

  # Convert to RGB for rerun
  rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
  return rgb


def visualize_snapshot_rerun() -> None:
  # --------------------------------------------------
  # load latest snapshot grain
  # --------------------------------------------------
  GrainFactory.build_registry()

  snapshot_grain = GrainFactory.load_latest_grain(
    key=("snapshot",),
    grain_data_root=GRAIN_DATA_ROOT,
  )

  synced: SnapshotTimeline = load_sync_results_from_snapshot_db(
    snapshot_grain=snapshot_grain
  )

  if not synced:
    raise RuntimeError("No snapshot data found")

  # --------------------------------------------------
  # rerun init
  # --------------------------------------------------
  rr.init(
    application_id="mf-eye-snapshot",
    spawn=True,   # viewer 자동 실행
  )

  # --------------------------------------------------
  # visualize
  # --------------------------------------------------
  for anchor_ns, sensor_map in synced.items():
    sec = to_sec(anchor_ns)
    # set timeline
    rr.set_time("anchor_ns", timestamp=sec)

    for sensor_name, entry in sensor_map.items():
      img = entry.payload
      if not isinstance(img, MfImage):
        continue

      rgb = _load_image_for_rerun(img)

      rr.log(
        f"snapshot/{sensor_name}",
        rr.Image(rgb),
      )


if __name__ == "__main__":
  visualize_snapshot_rerun()
