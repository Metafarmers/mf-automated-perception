-- YOLO detection grouping table
-- Links bbox and optional keypoints

CREATE TABLE IF NOT EXISTS yolo_detections (
  yolo_id INTEGER PRIMARY KEY AUTOINCREMENT,

  image_id INTEGER NOT NULL,
  bbox_id INTEGER NOT NULL,
  keypoint_id INTEGER,

  FOREIGN KEY(image_id) REFERENCES images(image_id)
    ON DELETE CASCADE,

  FOREIGN KEY(bbox_id) REFERENCES bboxes(bbox_id)
    ON DELETE CASCADE,

  FOREIGN KEY(keypoint_id) REFERENCES keypoints(keypoint_id)
    ON DELETE SET NULL
);

CREATE INDEX IF NOT EXISTS idx_yolo_image
ON yolo_detections(image_id);
