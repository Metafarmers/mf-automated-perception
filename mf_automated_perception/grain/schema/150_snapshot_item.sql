CREATE TABLE IF NOT EXISTS snapshot_item (
  item_id INTEGER PRIMARY KEY AUTOINCREMENT,
  snapshot_id INTEGER NOT NULL,

  -- sensor identity
  sensor_name TEXT NOT NULL,
  sensor_type TEXT NOT NULL,
  payload_kind TEXT NOT NULL,

  -- original source timestamp
  src_timestamp_sec INTEGER NOT NULL,
  src_timestamp_nsec INTEGER NOT NULL,

  -- anchor timestamp (duplicated for fast query / standalone read)
  anchor_timestamp_sec INTEGER NOT NULL,
  anchor_timestamp_nsec INTEGER NOT NULL,

  -- interpolation flag
  is_interpolated INTEGER NOT NULL DEFAULT 0,

  -- -----------------------------
  -- image payload
  -- -----------------------------
  image_path_to_data TEXT,
  image_width INTEGER,
  image_height INTEGER,
  image_encoding TEXT,
  image_K TEXT,
  image_D TEXT,

  -- -----------------------------
  -- pointcloud payload
  -- -----------------------------
  pc_path_to_data TEXT,
  pc_num_points INTEGER,
  pc_fields TEXT,

  -- -----------------------------
  -- odometry payload
  -- -----------------------------
  odom_px REAL,
  odom_py REAL,
  odom_pz REAL,
  odom_qx REAL,
  odom_qy REAL,
  odom_qz REAL,
  odom_qw REAL,
  odom_vx REAL,
  odom_vy REAL,
  odom_vz REAL,
  odom_wx REAL,
  odom_wy REAL,
  odom_wz REAL,

  FOREIGN KEY (snapshot_id) REFERENCES snapshot(snapshot_id)
);

-- 핵심: snapshot + sensor lookup
CREATE UNIQUE INDEX IF NOT EXISTS uq_snapshot_item_snapshot_sensor
ON snapshot_item(snapshot_id, sensor_name);

-- snapshot 전체 scan
CREATE INDEX IF NOT EXISTS idx_snapshot_item_snapshot_id
ON snapshot_item(snapshot_id);

-- 시간 기반 디버깅 / 검색
CREATE INDEX IF NOT EXISTS idx_snapshot_item_src_time
ON snapshot_item(src_timestamp_sec, src_timestamp_nsec);

CREATE INDEX IF NOT EXISTS idx_snapshot_item_anchor_time
ON snapshot_item(anchor_timestamp_sec, anchor_timestamp_nsec);

