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
