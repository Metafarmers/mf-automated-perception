# --- Standard library ---
import numpy as np
from pathlib import Path
import yaml

# --- Third-party libs ---
import rclpy
from cv_bridge import CvBridge
from tqdm import tqdm
from sensor_msgs.msg import Image, CompressedImage, PointCloud2, CameraInfo
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from cv_bridge import CvBridge


# --- Project / ROS packages ---
from mflib.perception.automated_perception.raw_dtype_def import (
  MfRawImage, MfTimestamp
)
from mflib.perception.automated_perception.snapshot import (
  Snapshot, SnapshotQueue
)

def parse_rosbag_metadata(metadata_path: Path):
  """
  Returns:
    storage_identifier: str
    topics: dict[name -> type]
  """
  import yaml

  with open(metadata_path, "r") as f:
    meta = yaml.safe_load(f)

  info = meta["rosbag2_bagfile_information"]

  storage_identifier = info["storage_identifier"]

  topics = {}
  for t in info["topics_with_message_count"]:
    name = t["topic_metadata"]["name"]
    typ = t["topic_metadata"]["type"]
    topics[name] = typ

  return storage_identifier, topics

def find_image_topics_with_intrinsics(
  *,
  bag_path: Path,
  storage_identifier: str,
  topics: dict[str, str],
):
  """
  Returns:
    dict[image_topic] = {
      "camera_info_topic": str,
      "K": list[float],
      "D": list[float],
    }
  """

  bag_path = Path(bag_path)

  # -------------------------------------------------
  # select image topics & camera_info topics
  # -------------------------------------------------
  image_topics = []
  camera_info_topics = []

  for name, typ in topics.items():
    if typ == "sensor_msgs/msg/CompressedImage":
      if name.endswith("compressed") or name.endswith("compressedDepth"):
        image_topics.append(name)

    elif typ == "sensor_msgs/msg/CameraInfo":
      camera_info_topics.append(name)

  if not image_topics:
    raise RuntimeError("No image topics found")

  if not camera_info_topics:
    raise RuntimeError("No CameraInfo topics found")

  print("\n[Image topics]")
  for t in image_topics:
    print(f"  {t}")

  print("\n[CameraInfo topics]")
  for t in camera_info_topics:
    print(f"  {t}")

  # -------------------------------------------------
  # 3. match image ↔ camera_info (prefix similarity)
  # -------------------------------------------------
  def common_prefix_length(a: str, b: str) -> int:
    n = min(len(a), len(b))
    for i in range(n):
      if a[i] != b[i]:
        return i
    return n

  image_to_cam = {}

  print("\n[Matching image -> CameraInfo]")
  for img_topic in image_topics:
    best = None
    best_score = -1

    for cam_topic in camera_info_topics:
      score = common_prefix_length(img_topic, cam_topic)
      if score > best_score:
        best_score = score
        best = cam_topic

    image_to_cam[img_topic] = best
    print(f"  {img_topic} -> {best} (score={best_score})")

  # -------------------------------------------------
  # 4. read CameraInfo messages (open bag, scan)
  # -------------------------------------------------
  caminfo_data = {}

  storage_options = StorageOptions(
    uri=str(bag_path),
    storage_id=storage_identifier
  )
  converter_options = ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr"
  )

  reader = SequentialReader()
  reader.open(storage_options, converter_options)

  print("\n[Reading CameraInfo messages]")

  while reader.has_next():
    topic, data, _ = reader.read_next()

    if topic not in camera_info_topics:
      continue

    if topic in caminfo_data:
      continue  # already read

    msg = deserialize_message(data, CameraInfo)

    K = list(msg.k)  # length 9
    D = list(msg.d)  # variable length

    caminfo_data[topic] = {
      "K": K,
      "D": D,
    }

    print(f"\nCameraInfo topic: {topic}")
    print(f"  K (len={len(K)}): {K}")
    print(f"  D (len={len(D)}): {D}")

    if len(caminfo_data) == len(camera_info_topics):
      break

  # -------------------------------------------------
  # 5. build final mapping
  # -------------------------------------------------
  result = {}

  print("\n[Final image topic intrinsics mapping]")
  for img_topic, cam_topic in image_to_cam.items():
    if cam_topic not in caminfo_data:
      print(f"  {img_topic}: CameraInfo not found")
      continue

    K = caminfo_data[cam_topic]["K"]
    D = caminfo_data[cam_topic]["D"]

    result[img_topic] = {
      "camera_info_topic": cam_topic,
      "K": K,
      "D": D,
    }

    print(f"\nImage topic: {img_topic}")
    print(f"  CameraInfo: {cam_topic}")
    print(f"  K = {K}")
    print(f"  D = {D}")

  return result
