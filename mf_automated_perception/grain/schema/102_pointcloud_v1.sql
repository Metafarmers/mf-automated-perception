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
