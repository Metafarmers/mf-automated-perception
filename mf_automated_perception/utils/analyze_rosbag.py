# --- Standard library ---
from pathlib import Path

# --- Third-party libs ---
from typing import Dict

import yaml


def parse_rosbag_metadata(bag_dir):
  """
  Returns:
    storage_identifier: str
    topics: dict[name -> type]
  """
  metadata_path = Path(bag_dir) / "metadata.yaml"
  with open(metadata_path, "r") as f:
    meta = yaml.safe_load(f)

  info = meta["rosbag2_bagfile_information"]

  storage_identifier = info["storage_identifier"]
  msg_count = info["message_count"]

  topics = {}
  for t in info["topics_with_message_count"]:
    name = t["topic_metadata"]["name"]
    typ = t["topic_metadata"]["type"]
    topics[name] = typ

  return storage_identifier, msg_count, topics


def map_image_topics_and_intrinsics(
  *,
  bag_dir: str,
  logger
) -> Dict[str, Dict] | None:
  """
  Returns:
    {
      image_topic: {
        "camera_info_topic": str,
        "K": list[float],
        "D": list[float],
      }
    }
  """
  from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
  from rclpy.serialization import deserialize_message
  from sensor_msgs.msg import CameraInfo


  # -------------------------------------------------
  # 1. parse metadata.yaml
  # -------------------------------------------------
  storage_identifier, _, topics = parse_rosbag_metadata(bag_dir)

  # -------------------------------------------------
  # 2. collect image & camera_info topics
  # -------------------------------------------------
  image_topics = []
  camera_info_topics = []

  for name, typ in topics.items():
    if typ in (
      "sensor_msgs/msg/Image",
      "sensor_msgs/msg/CompressedImage",
    ):
      image_topics.append(name)

    elif typ == "sensor_msgs/msg/CameraInfo":
      camera_info_topics.append(name)

  if not image_topics:
    logger.debug("No image topics found in metadata")
    return None

  if not camera_info_topics:
    logger.debug("No CameraInfo topics found in metadata")
    return None

  # -------------------------------------------------
  # 3. match image ↔ camera_info (prefix similarity)
  # -------------------------------------------------
  def common_prefix_length(a: str, b: str) -> int:
    n = min(len(a), len(b))
    for i in range(n):
      if a[i] != b[i]:
        return i
    return n

  image_to_caminfo: Dict[str, str] = {}

  for img_topic in image_topics:
    best = None
    best_score = -1

    for cam_topic in camera_info_topics:
      score = common_prefix_length(img_topic, cam_topic)
      if score > best_score:
        best_score = score
        best = cam_topic

    if best is None:
      logger.debug(
        f"Failed to find CameraInfo topic for image topic '{img_topic}'"
      )
      return None

    image_to_caminfo[img_topic] = best

  # -------------------------------------------------
  # 4. open rosbag and read CameraInfo messages
  # -------------------------------------------------
  storage_options = StorageOptions(
    uri=str(bag_dir),
    storage_id=storage_identifier,
  )
  converter_options = ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr",
  )
  print(str(bag_dir))
  print(storage_identifier)

  reader = SequentialReader()
  reader.open(storage_options, converter_options)

  caminfo_data: Dict[str, Dict[str, list]] = {}

  while reader.has_next():
    topic, data, _ = reader.read_next()

    if topic not in camera_info_topics:
      continue

    if topic in caminfo_data:
      continue

    msg = deserialize_message(data, CameraInfo)

    caminfo_data[topic] = {
      "K": list(msg.k),
      "D": list(msg.d),
    }

    if len(caminfo_data) == len(camera_info_topics):
      break

  # -------------------------------------------------
  # 5. build final result
  # -------------------------------------------------
  result: Dict[str, Dict] = {}

  for img_topic, cam_topic in image_to_caminfo.items():
    if cam_topic not in caminfo_data:
      raise RuntimeError(
        f"CameraInfo not found for image topic '{img_topic}' "
        f"(matched to '{cam_topic}')"
      )

    result[img_topic] = {
      "camera_info_topic": cam_topic,
      "K": caminfo_data[cam_topic]["K"],
      "D": caminfo_data[cam_topic]["D"],
    }

  return result
