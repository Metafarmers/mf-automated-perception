-- Keypoint sets associated with a bounding box

CREATE TABLE IF NOT EXISTS keypoints (
  keypoint_id INTEGER PRIMARY KEY AUTOINCREMENT,

  bbox_id INTEGER NOT NULL,

  xs TEXT NOT NULL,
  ys TEXT NOT NULL,
  confs TEXT NOT NULL,

  FOREIGN KEY(bbox_id) REFERENCES bboxes(bbox_id)
    ON DELETE CASCADE
);
