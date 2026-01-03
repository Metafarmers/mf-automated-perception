# mf_automated_perception/procedure/defs/run_yolo/implementation.py

import json
from pathlib import Path
from typing import Dict, List, Optional

import cv2

from mf_automated_perception.env import GRAIN_DATA_ROOT
from mf_automated_perception.grain.grain_base import GrainBase, GrainKey
from mf_automated_perception.procedure.defs.run_yolo.definition import (
  RunYolo,
)
from mf_automated_perception.realtime.datatypes.image import MfImage
from mf_automated_perception.realtime.mot.yolo_ultralytics import (
  YoloUltralyticsWrapper,
)
from tqdm import tqdm

from mf_automated_perception.realtime.datatypes.yolo import (
  add_bbox_to_sql,
  add_seg_mask_to_sql,
  add_keypoint_to_sql,
)



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

    conn = output_grain.open()
    conn = output_grain.open()

    for sensor_name, mf_images in all_mf_images.items():
      logger.info(
        f"Running YOLO on sensor '{sensor_name}' with "
        f"{len(mf_images)} images"
      )

      for mf_image in tqdm(
        mf_images,
        desc=f"YOLO [{sensor_name}]",
        unit="image",
        leave=False,
      ):
        sec = mf_image.timestamp_sec
        nsec = mf_image.timestamp_nsec

        image_id = mf_image.image_id

        path = GRAIN_DATA_ROOT / Path(mf_image.path_to_raw)
        im = cv2.imread(str(path))
        if im is None:
          logger.warning(f"Failed to read image: {path}")
          continue

        detections = yolo_wrapper.predict(im, sec, nsec)

        if yolo_wrapper.model.task == "detect":
          for det in detections:
            add_bbox_to_sql(
              conn,
              image_id=image_id,
              mfbbox=det,
            )

        elif yolo_wrapper.model.task == "segment":
          for det in detections:
            bbox_id = add_bbox_to_sql(
              conn,
              image_id=image_id,
              mfbbox=det,
            )

            add_seg_mask_to_sql(
              conn,
              bbox_id=bbox_id,
              mfmask=det,
            )

        elif yolo_wrapper.model.task == "pose":
          for det in detections:
            bbox_id = add_bbox_to_sql(
              conn,
              image_id=image_id,
              mfbbox=det,
            )

            add_keypoint_to_sql(
              conn,
              bbox_id=bbox_id,
              mfkp=det,
              keypoint_format=config.keypoint_format,
            )

        else:
          raise RuntimeError(
            f"Unsupported YOLO task: {self.model.task}"
          )

    output_grain.close()
