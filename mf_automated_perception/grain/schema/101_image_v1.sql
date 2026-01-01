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
