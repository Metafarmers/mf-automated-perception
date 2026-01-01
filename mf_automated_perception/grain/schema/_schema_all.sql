PRAGMA foreign_keys = ON;

-- One row per image frame

CREATE TABLE IF NOT EXISTS image (
  image_id INTEGER PRIMARY KEY AUTOINCREMENT,

  -- epoch timestamp
  timestamp_sec INTEGER NOT NULL,
  timestamp_nsec INTEGER NOT NULL,
  sensor_name TEXT NOT NULL,

  -- image metadata
  path_to_raw TEXT NOT NULL,
  width INTEGER NOT NULL,
  height INTEGER NOT NULL,
  encoding TEXT NOT NULL,

  -- camera intrinsics (3x3 matrix, flattened, JSON array)
  K TEXT,

  -- camera distortion coefficients (variable length, JSON array)
  D TEXT
);

CREATE INDEX IF NOT EXISTS idx_image_timestamp
ON image(timestamp_sec, timestamp_nsec);

-- Point cloud metadata
-- Raw data stored as .npz or .pcd files

CREATE TABLE IF NOT EXISTS pointcloud (
  pointcloud_id INTEGER PRIMARY KEY AUTOINCREMENT,

  -- epoch timestamp
  timestamp_sec INTEGER NOT NULL,
  timestamp_nsec INTEGER NOT NULL,
  sensor_name TEXT NOT NULL,

  num_points INTEGER NOT NULL,
  fields TEXT NOT NULL,

  path_to_raw TEXT NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_pointcloud_timestamp
ON pointcloud(timestamp_sec, timestamp_nsec);

CREATE TABLE IF NOT EXISTS ply (
  ply_id INTEGER PRIMARY KEY AUTOINCREMENT,
  rosbag_path TEXT NOT NULL UNIQUE,
  path_to_raw TEXT NOT NULL
);
CREATE UNIQUE INDEX IF NOT EXISTS idx_ply_rosbag
ON ply(rosbag_path);

CREATE TABLE IF NOT EXISTS snapshot_item (
  item_id INTEGER PRIMARY KEY AUTOINCREMENT,
  snapshot_id INTEGER NOT NULL,

  -- sensor identity
  sensor_name TEXT NOT NULL,

  -- payload routing
  payload_table_name TEXT NOT NULL,   
  payload_id INTEGER NOT NULL,  

  FOREIGN KEY (snapshot_id) REFERENCES snapshot(snapshot_id)
);

-- one payload per sensor per snapshot
CREATE UNIQUE INDEX IF NOT EXISTS uq_snapshot_item_snapshot_sensor
ON snapshot_item(snapshot_id, sensor_name);

-- snapshot scan
CREATE INDEX IF NOT EXISTS idx_snapshot_item_snapshot_id
ON snapshot_item(snapshot_id);

CREATE TABLE IF NOT EXISTS snapshot (
  snapshot_id INTEGER PRIMARY KEY AUTOINCREMENT,

  -- anchor / sync timestamp
  anchor_timestamp_sec INTEGER NOT NULL,
  anchor_timestamp_nsec INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_snapshot_anchor_time
ON snapshot(anchor_timestamp_sec, anchor_timestamp_nsec);

CREATE TABLE IF NOT EXISTS yolo_bbox (
  bbox_id INTEGER PRIMARY KEY AUTOINCREMENT,

  -- source image
  image_id INTEGER NOT NULL,

  -- detection result
  class_id INTEGER NOT NULL,
  score REAL NOT NULL,

  -- bounding box (image coordinate)
  x REAL NOT NULL,
  y REAL NOT NULL,
  w REAL NOT NULL,
  h REAL NOT NULL,

  FOREIGN KEY (image_id) REFERENCES raw_image(image_id)
);

CREATE INDEX IF NOT EXISTS idx_yolo_bbox_image
ON yolo_bbox(image_id);

CREATE INDEX IF NOT EXISTS idx_yolo_bbox_class
ON yolo_bbox(class_id);

CREATE TABLE IF NOT EXISTS yolo_keypoint (
  bbox_id INTEGER PRIMARY KEY,

  -- ordered keypoint coordinates
  xs TEXT NOT NULL,      -- JSON array: [x1, x2, ...]
  ys TEXT NOT NULL,      -- JSON array: [y1, y2, ...]
  scores TEXT,           -- JSON array: [s1, s2, ...] (optional)

  -- semantic definition of order
  keypoint_format TEXT NOT NULL,  -- e.g. 'strawberry_v1'

  FOREIGN KEY (bbox_id) REFERENCES yolo_bbox(bbox_id)
);

CREATE TABLE IF NOT EXISTS yolo_seg_mask (
  seg_mask_id INTEGER PRIMARY KEY AUTOINCREMENT,
  bbox_id INTEGER NOT NULL,

  -- path to npz file containing mask coordinates
  path_to_npz TEXT NOT NULL,

  -- number of points in the mask (for quick inspection)
  num_points INTEGER,

  FOREIGN KEY (bbox_id) REFERENCES yolo_bbox(bbox_id)
);

CREATE UNIQUE INDEX IF NOT EXISTS uq_yolo_seg_mask_bbox
ON yolo_seg_mask(bbox_id);
  
-- 6DoF odometry
-- One row per pose estimate

CREATE TABLE IF NOT EXISTS odometry (
  odom_id INTEGER PRIMARY KEY AUTOINCREMENT,

  -- normalized timestamp
  timestamp_sec INTEGER NOT NULL,
  timestamp_nsec INTEGER NOT NULL,

  -- position
  px REAL NOT NULL,
  py REAL NOT NULL,
  pz REAL NOT NULL,

  -- orientation (quaternion xyzw)
  qx REAL NOT NULL,
  qy REAL NOT NULL,
  qz REAL NOT NULL,
  qw REAL NOT NULL,

  -- optional linear velocity
  vx REAL,
  vy REAL,
  vz REAL,

  -- optional angular velocity
  wx REAL,
  wy REAL,
  wz REAL
);

CREATE INDEX IF NOT EXISTS idx_odometry_timestamp
ON odometry(timestamp_sec, timestamp_nsec);

CREATE TABLE IF NOT EXISTS path_to_raw (
  bag_dir TEXT PRIMARY KEY NOT NULL,
  storage_identifier TEXT NOT NULL,
  detected_by TEXT NOT NULL,
  scanned_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

