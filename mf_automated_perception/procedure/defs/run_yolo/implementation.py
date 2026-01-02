# mf_automated_perception/procedure/defs/run_yolo/implementation.py

import json
from typing import Dict, List, Optional

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey
from mf_automated_perception.procedure.defs.run_yolo.definition import (
  RunYolo,
)
from mf_automated_perception.realtime.datatypes.image import MfImage
from mf_automated_perception.realtime.mot.yolo_ultralytics import (
  YoloUltralyticsWrapper,
)


# ==================================================
# helpers
# ==================================================

def images_load_from_grain(
  input_grain: GrainBase,
  target_sensor_names: List[str],
) -> Dict[str, List[MfImage]]:
  """
  Load all rows from image table and convert to MfImage list.
  """
  rows = input_grain.dict_view(table="image", limit=-1)

  images: Dict[str, List[MfImage]] = {}

  for row in rows:
    sensor_name = row["sensor_name"]
    if sensor_name not in target_sensor_names:
      continue

    if sensor_name not in images:
      images[sensor_name] = []

    d = dict(row)

    if d.get("K") is not None:
      d["K"] = json.loads(d["K"])
    if d.get("D") is not None:
      d["D"] = json.loads(d["D"])

    images[sensor_name].append(MfImage.from_dict(d))

  return images


def image_insert_and_return_id(
  *,
  conn,
  img: MfImage,
) -> int:
  """
  Insert one MfImage into image table and return image_id.
  """
  cur = conn.execute(
    """
    INSERT INTO image (
      sensor_name,
      timestamp_sec,
      timestamp_nsec,
      path_to_raw,
      width,
      height,
      encoding,
      K,
      D
    )
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
    """,
    (
      img.sensor_name,
      img.timestamp_sec,
      img.timestamp_nsec,
      img.path_to_raw,
      img.width,
      img.height,
      img.encoding,
      json.dumps(img.K) if img.K is not None else None,
      json.dumps(img.D) if img.D is not None else None,
    ),
  )

  return cur.lastrowid


# ==================================================
# procedure implementation
# ==================================================

class RunYoloImpl(RunYolo):

  def _run(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]],
    output_grain: Optional[GrainBase],
    config,
    logger,
  ) -> None:
    if input_grains is None:
      raise RuntimeError(
        f"{self.key}: input_grains must be provided"
      )
    if len(input_grains) > 1:
      raise RuntimeError(
        f"{self.key}: only one input grain is expected, "
        f"got: {list(input_grains.keys())}"
      )
    if output_grain is None:
      raise RuntimeError(
        f"{self.key}: output_grain must be provided"
      )

    rosbag_grain = input_grains[("raw", "rosbag", "decoded")]
    logger.info(
      f"table names: {rosbag_grain.list_tables()}"
    )

    all_mf_images = images_load_from_grain(
      rosbag_grain,
      config.target_sensor_names,
    )

    yolo_wrapper = YoloUltralyticsWrapper(
      logger=logger,
      cfg_file=config.yolo_model_config_file,
    )

    # 실제 YOLO 추론 및 결과 저장 로직은
    # 여기 아래에 기존 코드 그대로 이어서 구현하면 된다.
