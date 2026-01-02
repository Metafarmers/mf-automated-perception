# mf_automated_perception/procedure/defs/decode_rosbags/implementation.py

import json
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import CompressedImage, Image
from tqdm import tqdm

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey
from mf_automated_perception.procedure.defs.decode_rosbags.definition import (
  DecodeRosbags,
)
from mf_automated_perception.utils.analyze_rosbag import (
  map_image_topics_and_intrinsics,
  parse_rosbag_metadata,
)

# ==================================================
# image
# ==================================================

def save_image_msg(
  *,
  conn,
  msg_type: str,
  data: bytes,
  topic: str,
  image_topics_with_intrinsics: dict,
  image_data_root_abs,
  grain_data_dir_abs,
  grain_data_dir_rel,
  bridge,
  sensor_name: str,
  logger,
):
  

  topic_l = topic.lower()

  if ("rgb" in topic_l) or ("color" in topic_l):
    subdir = sensor_name if sensor_name else "rgb"
    ext = "jpg"
    encoding = "bgr8"
  elif "depth" in topic_l:
    subdir = sensor_name if sensor_name else "depth"
    ext = "png"
    encoding = "16uc1"
  else:
    return

  out_dir = image_data_root_abs / subdir
  out_dir.mkdir(parents=True, exist_ok=True)

  if msg_type == "sensor_msgs/msg/Image":
    msg = deserialize_message(data, Image)
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

  elif msg_type == "sensor_msgs/msg/CompressedImage":
    msg = deserialize_message(data, CompressedImage)
    if topic_l.endswith("compressed"):
      img = bridge.compressed_imgmsg_to_cv2(msg)
    elif topic_l.endswith("compresseddepth"):
      np_arr = np.frombuffer(msg.data, np.uint8)
      img = cv2.imdecode(np_arr[12:], cv2.IMREAD_UNCHANGED)
    else:
      return
  else:
    return

  ts = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
  out_path = out_dir / f"{ts}.{ext}"

  if not cv2.imwrite(str(out_path), img, [cv2.IMWRITE_JPEG_QUALITY, 85]):
    logger.warning(f"Failed to write image: {out_path}")
    return

  if topic in image_topics_with_intrinsics:
    intr = image_topics_with_intrinsics[topic]
    K_json = json.dumps(intr["K"])
    D_json = json.dumps(intr["D"])
  else:
    K_json = json.dumps([0.0] * 9)
    D_json = json.dumps([])

  rel_path = grain_data_dir_rel / out_path.relative_to(grain_data_dir_abs)

  conn.execute(
    """
    INSERT INTO image (
      timestamp_sec,
      timestamp_nsec,
      sensor_name,
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
      msg.header.stamp.sec,
      msg.header.stamp.nanosec,
      sensor_name,
      str(rel_path),
      img.shape[1],
      img.shape[0],
      encoding,
      K_json,
      D_json,
    ),
  )


# ==================================================
# pointcloud
# ==================================================

def save_pointcloud_msg(
  *,
  conn,
  data: bytes,
  topic: str,
  pointcloud_data_root_abs,
  grain_data_dir_abs,
  grain_data_dir_rel,
  sensor_name: str,
  logger,
):
  import numpy as np
  import open3d as o3d
  import json
  from rclpy.serialization import deserialize_message
  from sensor_msgs.msg import PointCloud2
  from sensor_msgs_py import point_cloud2

  msg = deserialize_message(data, PointCloud2)

  points = [
    (p[0], p[1], p[2])
    for p in point_cloud2.read_points(
      msg,
      field_names=["x", "y", "z"],
      skip_nans=True,
    )
  ]

  if not points:
    logger.warning(f"Empty point cloud after parsing: topic={topic}")
    return

  xyz = np.asarray(points, dtype=np.float32)

  ts = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
  out_path = pointcloud_data_root_abs / f"{topic.replace('/', '_')}_{ts}.pcd"

  pcd = o3d.geometry.PointCloud()
  pcd.points = o3d.utility.Vector3dVector(xyz)

  if not o3d.io.write_point_cloud(str(out_path), pcd):
    logger.error(f"Failed to write PCD: {out_path}")
    return

  rel_path = grain_data_dir_rel / out_path.relative_to(grain_data_dir_abs)

  conn.execute(
    """
    INSERT INTO pointcloud (
      timestamp_sec,
      timestamp_nsec,
      sensor_name,
      num_points,
      fields,
      path_to_raw
    )
    VALUES (?, ?, ?, ?, ?, ?)
    """,
    (
      msg.header.stamp.sec,
      msg.header.stamp.nanosec,
      sensor_name,
      int(xyz.shape[0]),
      json.dumps(["x", "y", "z"]),
      str(rel_path),
    ),
  )


# ==================================================
# procedure implementation
# ==================================================

class DecodeRosbagsImpl(DecodeRosbags):

  def _run(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]],
    output_grain: Optional[GrainBase],
    config,
    logger,
  ) -> None:
    

    if input_grains is None or output_grain is None:
      raise RuntimeError(
        f"{self.key}: input_grains and output_grain must be provided"
      )

    rosbag_path_grain = input_grains[("raw", "rosbag", "path")]
    paths = rosbag_path_grain.dict_view(table="path_to_raw")

    image_data_root_abs = output_grain.grain_data_dir_abs / "image"
    pointcloud_data_root_abs = output_grain.grain_data_dir_abs / "pointclouds"
    image_data_root_abs.mkdir(parents=True, exist_ok=True)
    pointcloud_data_root_abs.mkdir(parents=True, exist_ok=True)

    conn = output_grain.open()

    for p in paths:
      bag_dir = p["bag_dir"]
      logger.info(f"Decoding rosbag: {bag_dir}")

      storage_id, msg_count, topic_type_dict = parse_rosbag_metadata(bag_dir)
      image_topics_with_intrinsics = map_image_topics_and_intrinsics(
        bag_dir=bag_dir,
        logger=logger,
      )

      reader = SequentialReader()
      reader.open(
        StorageOptions(uri=bag_dir, storage_id=storage_id),
        ConverterOptions("", ""),
      )

      image_topics = {
        t for t, typ in topic_type_dict.items()
        if typ in {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}
      }
      pointcloud_topics = {
        t for t, typ in topic_type_dict.items()
        if typ == "sensor_msgs/msg/PointCloud2"
      }

      bridge = CvBridge()

      with tqdm(total=msg_count, desc=f"Decoding {Path(bag_dir).name}") as pbar:
        while reader.has_next():
          topic, data, _ = reader.read_next()

          sensor_name = (
            config.topic_to_sensor_name_map.get(topic)
            if config.topic_to_sensor_name_map
            and topic in config.topic_to_sensor_name_map
            else topic.replace("/", "_").strip("_")
          )

          if config.decode_images and topic in image_topics:
            save_image_msg(
              conn=conn,
              msg_type=topic_type_dict[topic],
              data=data,
              topic=topic,
              image_topics_with_intrinsics=image_topics_with_intrinsics,
              image_data_root_abs=image_data_root_abs,
              grain_data_dir_abs=output_grain.grain_data_dir_abs,
              grain_data_dir_rel=output_grain.grain_data_dir_rel,
              bridge=bridge,
              sensor_name=sensor_name,
              logger=logger,
            )

          if config.decode_pointclouds and topic in pointcloud_topics:
            save_pointcloud_msg(
              conn=conn,
              data=data,
              topic=topic,
              pointcloud_data_root_abs=pointcloud_data_root_abs,
              grain_data_dir_abs=output_grain.grain_data_dir_abs,
              grain_data_dir_rel=output_grain.grain_data_dir_rel,
              sensor_name=sensor_name,
              logger=logger,
            )

          pbar.update(1)

      conn.commit()
      logger.info(f"Finished decoding rosbag: {bag_dir}")

    output_grain.close()
