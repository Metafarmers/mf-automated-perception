import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage

from mflib.perception.automated_perception.raw_dtype_def import (
  MfRawOdometry, MfTimestamp, MfRawImage
)
from mflib.perception.automated_perception.snapshot import Snapshot, SnapshotQueue, SyncEngine
from mflib.perception.automated_perception.example import params_apple_counting as P

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from tqdm import tqdm

def load_tf_into_buffer(bag_path, tf_buffer):
  storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
  converter_options = ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr'
  )

  reader = SequentialReader()
  reader.open(storage_options, converter_options)

  first_time = None
  last_time = None

  while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic not in ["/tf", "/tf_static"]:
      continue

    tf_msg = deserialize_message(data, TFMessage)

    for tr in tf_msg.transforms:
      for tr in tf_msg.transforms:
        if topic == "/tf_static":
          tf_buffer.set_transform_static(tr, "bag_read")
        else:
          tf_buffer.set_transform(tr, "bag_read")

      t = tr.header.stamp
      t_sec = float(t.sec) + float(t.nanosec) * 1e-9

      if first_time is None:
        first_time = t_sec
      last_time = t_sec

  return first_time, last_time

def generate_odometry_queue_from_tf(bag_path, step_sec=0.1):
  rclpy.init()
  node = rclpy.create_node("tf_to_odom")

  tf_buffer = Buffer(cache_time=Duration(seconds=600.0))

  odom_q = SnapshotQueue("odom")

  # --- 1) Load all TF into buffer ---
  t_start, t_end = load_tf_into_buffer(bag_path, tf_buffer)
  if t_start is None:
    raise RuntimeError("No TF data found in bag.")

  # --- 2) Iterate from t_start → t_end with step_sec ---
  total_steps = int((t_end - t_start) / step_sec) + 1
  pbar = tqdm(total=total_steps, desc="Generating odometry from TF", unit="step")

  t_cur = t_start
  while t_cur <= t_end:
    ros_time = Time(seconds=t_cur)

    try:
      tr = tf_buffer.lookup_transform(
        P.base_frame,
        P.body_frame,
        ros_time,
        # timeout=Duration(seconds=0.01)
      )
    except Exception as e:
      # print(f"Warning: TF lookup failed at t={t_cur:.2f}s: {e}")
      # progress update
      t_cur += step_sec
      pbar.update(1)
      continue

    # extract translation
    trans = np.array([
      tr.transform.translation.x,
      tr.transform.translation.y,
      tr.transform.translation.z
    ], dtype=np.float32)

    # extract quaternion
    quat = np.array([
      tr.transform.rotation.x,
      tr.transform.rotation.y,
      tr.transform.rotation.z,
      tr.transform.rotation.w
    ], dtype=np.float32)

    # timestamp
    ts = MfTimestamp(
      sec=tr.header.stamp.sec,
      nsec=tr.header.stamp.nanosec
    )

    raw_odom = MfRawOdometry(
      pose=trans,
      orientation=quat,
      vel=None,
      angvel=None
    )

    snap = Snapshot.from_raw("odom", raw_odom, ts)
    odom_q.add(snap)

    # progress update
    t_cur += step_sec
    pbar.update(1)

  pbar.close()
  node.destroy_node()
  rclpy.shutdown()
  return odom_q

def generate_rgb_queue(bag_path):
  """
  Reads RGB frames from P.rgb_topic.
  Automatically supports both Image and CompressedImage.
  """
  storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
  converter_options = ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr'
  )

  reader = SequentialReader()
  reader.open(storage_options, converter_options)

  rgb_q = SnapshotQueue("rgb")
  bridge = CvBridge()

  is_compressed = ("compressed" in P.rgb_topic)

  pbar = tqdm(desc="Reading RGB frames", unit="img")

  while reader.has_next():
    topic, data, _ = reader.read_next()

    if topic != P.rgb_topic:
      continue

    # Deserialize appropriate message type
    if is_compressed:
      msg = deserialize_message(data, CompressedImage)
      img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    else:
      msg = deserialize_message(data, Image)
      img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Convert to MfRawImage
    mf_img = MfRawImage(
      img,
      encoding="8UC3",
      filename=None
    )

    ts = MfTimestamp(
      sec=msg.header.stamp.sec,
      nsec=msg.header.stamp.nanosec
    )

    snap = Snapshot.from_raw("rgb", mf_img, ts)
    rgb_q.add(snap)

    pbar.update(1)

  pbar.close()
  return rgb_q


if __name__ == "__main__":
  odom_queue = generate_odometry_queue_from_tf(P.bag_path, step_sec=0.1)
  print("Odom queue size:", len(odom_queue))
  rgb_queue = generate_rgb_queue(P.bag_path)
  print("RGB queue size:", len(rgb_queue))

  timediff_rules = {
    ("rgb", "odom"): 200, # ms
  }

  engine = SyncEngine(timediff_rules)

  ok, result = engine.sync(
    ["rgb", "odom"],
    [rgb_queue, odom_queue],
    squeeze=True,
    report_remaining=True
  )
