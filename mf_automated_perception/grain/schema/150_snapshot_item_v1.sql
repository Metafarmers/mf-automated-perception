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
