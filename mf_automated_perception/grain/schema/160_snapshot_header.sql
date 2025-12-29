CREATE TABLE IF NOT EXISTS snapshot (
  snapshot_id INTEGER PRIMARY KEY AUTOINCREMENT,

  -- anchor / sync timestamp
  anchor_timestamp_sec INTEGER NOT NULL,
  anchor_timestamp_nsec INTEGER NOT NULL,

  -- optional sequential index (offline friendly)
  snapshot_index INTEGER,

  -- bookkeeping
  created_at_sec INTEGER NOT NULL,
  created_at_nsec INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_snapshot_anchor_time
ON snapshot(anchor_timestamp_sec, anchor_timestamp_nsec);
