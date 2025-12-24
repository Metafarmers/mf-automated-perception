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
