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
