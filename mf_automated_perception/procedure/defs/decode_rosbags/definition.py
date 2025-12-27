from typing import ClassVar, Dict, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey
from mf_automated_perception.procedure.core.procedure_base import ProcedureBase
from mf_automated_perception.utils.analyze_rosbag import (
  map_image_topics_and_intrinsics,
  parse_rosbag_metadata,
)
import time


def save_image_msg(
  *,
  conn,
  msg_type: str,
  data: bytes,
  topic: str,
  image_topics_with_intrinsics: dict,
  image_data_root,
  bridge,
  logger,
):
  from rclpy.serialization import deserialize_message
  from sensor_msgs.msg import Image, CompressedImage
  import cv2
  import numpy as np
  import json

  topic_l = topic.lower()

  if ("rgb" in topic_l) or ("color" in topic_l):
    subdir = "rgb"
    ext = "jpg"
    encoding = "bgr8"
  elif "depth" in topic_l:
    subdir = "depth"
    ext = "png"
    encoding = "depth"
  else:
    return

  out_dir = image_data_root / subdir
  out_dir.mkdir(parents=True, exist_ok=True)

  # ---------- decode ----------
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
  out_path = out_dir / f"{topic.replace('/', '_')}_{ts}.{ext}"

  if not cv2.imwrite(
    str(out_path),
    img,
    [cv2.IMWRITE_JPEG_QUALITY, 85],
  ):
    logger.warning(f"Failed to write image: {out_path}")
    return

  # ---------- intrinsics ----------
  if topic in image_topics_with_intrinsics:
    intr = image_topics_with_intrinsics[topic]
    K_json = json.dumps(intr["K"])
    D_json = json.dumps(intr["D"])
  else:
    K_json = json.dumps([0.0] * 9)
    D_json = json.dumps([])

  # ---------- DB insert ----------
  cur = conn.cursor()
  cur.execute(
    """
    INSERT INTO images (
      timestamp_sec,
      timestamp_nsec,
      path_to_raw,
      width,
      height,
      encoding,
      K,
      D
    )
    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
    """,
    (
      msg.header.stamp.sec,
      msg.header.stamp.nanosec,
      str(out_path),
      img.shape[1],
      img.shape[0],
      encoding,
      K_json,
      D_json,
    ),
  )


def save_pointcloud_msg(
  *,
  conn,
  data: bytes,
  topic: str,
  pointcloud_data_root,
  logger,
):
  import numpy as np
  import open3d as o3d
  import json
  from rclpy.serialization import deserialize_message
  from sensor_msgs.msg import PointCloud2
  from sensor_msgs_py import point_cloud2

  # ---------- deserialize ----------
  msg = deserialize_message(data, PointCloud2)

  # ---------- PointCloud2 -> xyz ----------
  points = [
    (p[0], p[1], p[2])
    for p in point_cloud2.read_points(
      msg,
      field_names=("x", "y", "z"),
      skip_nans=True,
    )
  ]

  if not points:
    logger.warning(f"Empty point cloud after parsing: topic={topic}")
    return

  xyz = np.asarray(points, dtype=np.float32)

  # ---------- save PCD ----------
  ts = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
  out_path = pointcloud_data_root / f"{topic.replace('/', '_')}_{ts}.pcd"

  pcd = o3d.geometry.PointCloud()
  pcd.points = o3d.utility.Vector3dVector(xyz)

  if not o3d.io.write_point_cloud(str(out_path), pcd):
    logger.error(f"Failed to write PCD: {out_path}")
    return

  # ---------- DB insert ----------
  cur = conn.cursor()
  cur.execute(
    """
    INSERT INTO pointclouds (
      timestamp_sec,
      timestamp_nsec,
      num_points,
      fields,
      path_to_raw
    )
    VALUES (?, ?, ?, ?, ?)
    """,
    (
      msg.header.stamp.sec,
      msg.header.stamp.nanosec,
      int(xyz.shape[0]),
      json.dumps(["x", "y", "z"]),
      str(out_path),
    ),
  )







class DecodeRosbagsConfig(BaseModel):
  decode_images: bool = True
  decode_pointclouds: bool = True
  # decode_imu: bool = True
  # decode_tf: bool = True
  # tf_from_to_frames: Optional[List[Tuple[str, str]]] = None

class DecodeRosbags(ProcedureBase):
  key: ClassVar[str] = "decode_rosbags"
  version: ClassVar[str] = "1.0.0"
  ParamModel: ClassVar[Optional[Type]] = DecodeRosbagsConfig
  description: ClassVar[str] = "Locate ROS2 rosbags and store their paths as raw grains."
  input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (("raw", "rosbag", "path"),)
  output_grain_key: ClassVar[GrainKey] = ("raw", "rosbag", "decoded")


  def _run(
    self,
    *,
    input_grains: Dict[GrainKey, GrainBase],
    output_grain: GrainBase,
    config,
    logger,
  ) -> None:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from cv_bridge import CvBridge
    from tqdm import tqdm


    rosbag_path_grain: GrainBase = input_grains[('raw', 'rosbag', 'path')]
    paths = rosbag_path_grain.dict_view(table='path_to_raw')

    image_topic_types = {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'}
    image_data_root = output_grain.grain_data_dir / 'images'
    image_data_root.mkdir(parents=True, exist_ok=True)
    pointcloud_topic_types = {'sensor_msgs/msg/PointCloud2'}
    pointcloud_data_root = output_grain.grain_data_dir / 'pointclouds'
    pointcloud_data_root.mkdir(parents=True, exist_ok=True)

    conn = output_grain.open()
    for p in paths:
      bag_dir = p['bag_dir']
      logger.info(f"Decoding rosbag: {bag_dir}")

      storage_id, msg_count, topic_type_dict = parse_rosbag_metadata(bag_dir)
      image_topics_with_intrinsics = map_image_topics_and_intrinsics(
        bag_dir=bag_dir,
        logger=logger
      )


      reader = SequentialReader()
      storage_options = StorageOptions(
        uri=bag_dir,
        storage_id=storage_id,
      )
      converter_options = ConverterOptions("", "")
      reader.open(storage_options, converter_options)

      # topic type filter
      image_topics = {
        t for t, typ in topic_type_dict.items()
        if typ in image_topic_types
      }
      pointcloud_topics = {
        t for t, typ in topic_type_dict.items()
        if typ in pointcloud_topic_types
      }

      logger.info(f"Image topics: {image_topics}")
      logger.info(f"PointCloud topics: {pointcloud_topics}")

      bridge = CvBridge()

      cnt = 0
      while reader.has_next():
        cnt += 1
        if cnt % 1000 == 0:
          logger.info(f"  Processed {cnt}/{msg_count} messages...")
          conn.commit()

        topic, data, t = reader.read_next()

        # IMAGE
        if config.decode_images and topic in image_topics:
          save_image_msg(
            conn=conn,
            msg_type=topic_type_dict[topic],
            data=data,
            topic=topic,
            image_topics_with_intrinsics=image_topics_with_intrinsics,
            image_data_root=image_data_root,
            bridge=bridge,
            logger=logger,
          )

        # POINTCLOUD
        if config.decode_pointclouds and topic in pointcloud_topics:
          save_pointcloud_msg(
            conn=conn,
            data=data,
            topic=topic,
            pointcloud_data_root=pointcloud_data_root,
            logger=logger,
          )



      logger.info(f"Finished decoding rosbag: {bag_dir}")
      conn.commit()

    output_grain.close()
