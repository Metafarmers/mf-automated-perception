CREATE TABLE IF NOT EXISTS yolo_seg_mask (
  seg_mask_id INTEGER PRIMARY KEY AUTOINCREMENT,
  bbox_id INTEGER NOT NULL,

  -- path to npz file containing mask coordinates
  path_to_npz TEXT NOT NULL,

  -- number of points in the mask (for quick inspection)
  num_points INTEGER,

  FOREIGN KEY (bbox_id) REFERENCES yolo_bbox(bbox_id)
);

CREATE UNIQUE INDEX IF NOT EXISTS uq_yolo_seg_mask_bbox
ON yolo_seg_mask(bbox_id);
  