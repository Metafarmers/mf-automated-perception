-- 2D bounding boxes
-- Linked to image frames

CREATE TABLE IF NOT EXISTS bboxes (
  bbox_id INTEGER PRIMARY KEY AUTOINCREMENT,

  image_id INTEGER NOT NULL,

  -- tlwh format
  x REAL NOT NULL,
  y REAL NOT NULL,
  w REAL NOT NULL,
  h REAL NOT NULL,

  score REAL NOT NULL,
  label INTEGER NOT NULL,

  FOREIGN KEY(image_id) REFERENCES images(image_id)
    ON DELETE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_bboxes_image
ON bboxes(image_id);

CREATE INDEX IF NOT EXISTS idx_bboxes_label
ON bboxes(label);
