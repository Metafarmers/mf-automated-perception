from typing import Dict, List, Literal, Optional

import cv2
import numpy as np
import torch
import yaml
from pydantic import BaseModel, Field
from ultralytics import YOLO

from mf_automated_perception.env import PROJECT_ROOT
from mf_automated_perception.realtime.datatypes.yolo import (
  MfBBox,
  MfKeypoint,
  MfSegMaskMemory,
)
from mf_automated_perception.utils.visualize import index_to_distinct_rgb


class YoloIoUConfig(BaseModel):
  enabled: bool
  max_allowed_overlap_per_label: Dict[int, float]


class YoloFilterConfig(BaseModel):
  enabled: bool
  valid_labels: List[int]
  reject_margin_percent: float


class YoloRuntimeConfig(BaseModel):
  device: Literal["auto", "cpu", "cuda"]
  warmup_iters: int
  verbose: bool


class YoloConfig(BaseModel):
  # model
  model_path: str

  # labels
  label_map: Dict[int, List[str]]

  # thresholds
  conf_thresh: float
  global_iou_thresh: float

  # iou
  custom_iou: YoloIoUConfig

  # filter
  filter: YoloFilterConfig

  # runtime
  runtime: YoloRuntimeConfig



def compute_iou_xyxy(a, b) -> float:
  ax1, ay1, ax2, ay2 = a
  bx1, by1, bx2, by2 = b

  inter_x1 = max(ax1, bx1)
  inter_y1 = max(ay1, by1)
  inter_x2 = min(ax2, bx2)
  inter_y2 = min(ay2, by2)

  if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
    return 0.0

  inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
  area_a = (ax2 - ax1) * (ay2 - ay1)
  area_b = (bx2 - bx1) * (by2 - by1)

  return inter_area / float(area_a + area_b - inter_area)

# compute IoU between two tlwh boxes
def compute_iou_tlwh(a, b):
  ax1, ay1, aw, ah = a
  bx1, by1, bw, bh = b
  ax2, ay2 = ax1 + aw, ay1 + ah
  bx2, by2 = bx1 + bw, by1 + bh

  ix1, iy1 = max(ax1, bx1), max(ay1, by1)
  ix2, iy2 = min(ax2, bx2), min(ay2, by2)
  if ix2 <= ix1 or iy2 <= iy1:
    return 0.0

  inter = (ix2 - ix1) * (iy2 - iy1)
  area_a = aw * ah
  area_b = bw * bh
  return inter / float(area_a + area_b - inter)


class YoloUltralyticsWrapper:
  def __init__(self, logger, cfg_file):
    self.logger = logger

    with open(cfg_file, "r") as r:
      cfg = yaml.safe_load(r)
    self.cfg = YoloConfig.model_validate(cfg)

    self.model = YOLO(PROJECT_ROOT / self.cfg.model_path)
    self.device = self._resolve_device(self.cfg.runtime.device)
    self.model.to(self.device)
    self.model.eval()

    self._warmed_up = False
    self.last_raw_result = None
    self.last_results = []

    if self.model.task not in ["detect", "pose", "segment"]:
      raise ValueError(f'Unsupported YOLO task type: {self.model.task}')
    self.logger.info(f'yolo model loaded: {self.cfg.model_path}, task={self.model.task}')
    self.logger.info(f'device: {self.device}')
    self.logger.info('label names -')
    for label_id, label_name in self.model.names.items():
      self.logger.info(f'  {label_id}: {label_name}')

  # resolve device string
  def _resolve_device(self, dev):
    if dev == "auto":
      return "cuda" if torch.cuda.is_available() else "cpu"
    return dev

  # lazy warmup using zero image
  def _warmup_if_needed(self, h, w):
    if self._warmed_up:
      return

    iters = self.cfg.runtime.warmup_iters
    if iters <= 0:
      self._warmed_up = True
      return

    self.logger.info(f'warmup start: iters={iters}, size=({h},{w})')
    dummy = np.zeros((h, w, 3), dtype=np.uint8)

    for _ in range(iters):
      self.model.predict(
        dummy,
        conf=self.cfg.conf_thresh,
        verbose=False,
      )

    self._warmed_up = True
    self.logger.info('warmup done')

  def _bbox_pass_filter(self, tlwh, label, img_h, img_w):
    # filter disabled
    if not self.cfg.filter.enabled:
      return True

    # label filter
    if self.cfg.filter.valid_labels:
      if label not in self.cfg.filter.valid_labels:
        return False

    # margin reject filter
    margin_pct = self.cfg.filter.reject_margin_percent
    if margin_pct > 0.0:
      mx = img_w * margin_pct / 100.0 / 2.0
      my = img_h * margin_pct / 100.0 / 2.0
      x, y, w, h = tlwh
      if x <= mx or y <= my:
        return False
      if x + w >= img_w - mx or y + h >= img_h - my:
        return False

    return True

  def _iou_prune_indices(self, idxs, result):
    kept = []

    # group indices by label
    label_to_idxs = {}
    for i in idxs:
      label = int(result.boxes[i].cls[0])
      label_to_idxs.setdefault(label, []).append(i)

    # process each label independently
    for label, label_idxs in label_to_idxs.items():
      thresh = self.cfg.custom_iou.max_allowed_overlap_per_label.get(label, None)

      # no iou rule means keep all
      if thresh is None or thresh <= 0:
        kept.extend(label_idxs)
        continue

      groups = []

      # build IoU groups
      for i in label_idxs:
        box_i = result.boxes[i]
        x1, y1, x2, y2 = box_i.xyxy[0].tolist()
        tlwh_i = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))

        assigned = False
        for g in groups:
          # compare with first element of group
          j = g[0]
          box_j = result.boxes[j]
          x1, y1, x2, y2 = box_j.xyxy[0].tolist()
          tlwh_j = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))

          if compute_iou_tlwh(tlwh_i, tlwh_j) > thresh:
            g.append(i)
            assigned = True
            break

        if not assigned:
          groups.append([i])

      # select max-score from each group
      for g in groups:
        best = max(
          g,
          key=lambda idx: float(result.boxes[idx].conf[0]),
        )
        kept.append(best)

    return kept


  def _filter_indices(self, idxs, result, img_h, img_w):
    # bbox geometry and label filter
    kept = []
    for i in idxs:
      box = result.boxes[i]
      label = int(box.cls[0])
      x1, y1, x2, y2 = box.xyxy[0].tolist()
      tlwh = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))

      if not self._bbox_pass_filter(tlwh, label, img_h, img_w):
        continue

      kept.append(i)

    # score sort
    kept.sort(key=lambda i: float(result.boxes[i].conf[0]), reverse=True)

    # iou prune
    if self.cfg.custom_iou.enabled:
      kept = self._iou_prune_indices(kept, result)

    return kept

  def _build_outputs_from_indices(self, idxs, result, sec, nsec):
    outputs = []

    for i in idxs:
      box = result.boxes[i]
      label = int(box.cls[0])
      score = float(box.conf[0])
      x1, y1, x2, y2 = box.xyxy[0].tolist()
      tlwh = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))

      if self.model.task == "detect":
        outputs.append(
          MfBBox(sec, nsec, label, score, tlwh)
        )

      elif self.model.task == "pose":
        kp = result.keypoints[i]
        outputs.append(
          MfKeypoint(
            sec,
            nsec,
            label,
            score,
            tlwh,
            keypoint_xy=[int(v) for v in kp.xy.flatten()],
            keypoint_score=[float(v) for v in kp.conf[0].tolist()],
          )
        )

      elif self.model.task == "segment":
        pts = result.masks.xy[i]  # already in image coordinates
        outputs.append(
          MfSegMaskMemory(
            sec,
            nsec,
            label,
            score,
            tlwh,
            mask_xy=pts,
          )
        )



    return outputs


  def predict(self, cv_img, sec, nsec):
    h, w = cv_img.shape[:2]
    self._warmup_if_needed(h, w)

    result = self.model.predict(
      cv_img,
      conf=self.cfg.conf_thresh,
      iou=self.cfg.global_iou_thresh,
      verbose=self.cfg.runtime.verbose,
    )[0].cpu()

    self.last_raw_result = result

    # stage 1: collect candidate indices
    idxs = list(range(len(result.boxes)))

    # stage 2: filtering pipeline
    idxs = self._filter_indices(idxs, result, h, w)

    # stage 3: build outputs
    outputs = self._build_outputs_from_indices(idxs, result, sec, nsec)

    self.last_results = outputs
    return outputs


  # draw result on image
  def draw(self, cv_img, draw_raw=False):
    if draw_raw:
      if self.last_raw_result is None:
        return cv_img
      return self.last_raw_result.plot(img=cv_img)

    img = cv_img.copy()
    for result in self.last_results:
      x, y, w, h = result.tlwh
      r,g,b = index_to_distinct_rgb(result.label)
      cv2.rectangle(img, (x, y), (x + w, y + h), (b, g, r), 2)

      label_id = result.label
      score = result.score

      x, y, w, h = result.tlwh
      cv2.putText(
        img,
        f"{label_id}: {score:.2f}",
        (x, y - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (b, g, r),
        1,
      )


      if isinstance(result, MfKeypoint):
        for i in range(0, len(result.keypoint_xy), 2):
          cv2.circle(
            img,
            (result.keypoint_xy[i], result.keypoint_xy[i + 1]),
            3,
            (255, 0, 0),
            -1,
          )

      if isinstance(result, MfSegMaskMemory):
        pts = result.mask_xy.astype(np.int32)
        cv2.fillPoly(img, [pts], (b, g, r))

    return img


def test_yolo_on_rgb_overlay_depth(
  logger,
  cfg_file,
  rgb_path,
  depth_path,
  output_path,
):
  import cv2
  import yaml
  import numpy as np
  from pathlib import Path

  # init yolo wrapper
  yolo = YoloUltralyticsWrapper(logger, cfg_file)

  # load rgb image
  rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
  if rgb is None:
    raise RuntimeError(f"failed to load rgb: {rgb_path}")

  # load depth image (uint16)
  depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
  if depth is None:
    raise RuntimeError(f"failed to load depth: {depth_path}")
  if depth.dtype != np.uint16:
    raise RuntimeError("depth image must be uint16")

  # run inference on rgb
  yolo.predict(rgb, sec=0, nsec=0)

  # percentile-based grayscale normalize depth excluding max value
  depth_f = depth.astype(np.float32)
  invalid_mask = depth_f >= 65535.0
  valid_vals = depth_f[~invalid_mask]

  if valid_vals.size == 0:
    depth_vis = np.zeros_like(depth_f, dtype=np.uint8)
  else:
    lo = np.percentile(valid_vals, 5)
    hi = np.percentile(valid_vals, 95)
    depth_vis = np.clip((depth_f - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)
    depth_vis[invalid_mask] = 0

  # convert grayscale depth to BGR
  depth_gray_bgr = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

  # draw yolo result
  rgb_overlay = yolo.draw(rgb, draw_raw=False)
  depth_overlay = yolo.draw(depth_gray_bgr, draw_raw=False)

  # resize depth images to match rgb size if needed
  if depth_gray_bgr.shape[:2] != rgb.shape[:2]:
    depth_gray_bgr = cv2.resize(
      depth_gray_bgr,
      (rgb.shape[1], rgb.shape[0]),
      interpolation=cv2.INTER_NEAREST,
    )
    depth_overlay = cv2.resize(
      depth_overlay,
      (rgb.shape[1], rgb.shape[0]),
      interpolation=cv2.INTER_NEAREST,
    )

  # top row: original images
  top_row = np.hstack([rgb, depth_gray_bgr])

  # bottom row: overlay images
  bottom_row = np.hstack([rgb_overlay, depth_overlay])

  # full grid
  concat = np.vstack([top_row, bottom_row])

  # save result
  output_path = Path(output_path)
  output_path.parent.mkdir(parents=True, exist_ok=True)
  cv2.imwrite(str(output_path), concat)


  logger.info(f"saved concat result: {output_path}")



if __name__ == "__main__":
  from mf_automated_perception.utils.gp_logger import get_logger
  logger = get_logger("yolo_test", file_dir='/tmp')

  test_yolo_on_rgb_overlay_depth(
    logger=logger,
    cfg_file="/workspace/params/ai/straw_all_seg.yaml",
    # cfg_file="/workspace/params/ai/strawberry2024DB.yaml",
    rgb_path="/workspace/params/ai/sample_rgb.jpg",
    depth_path="/workspace/params/ai/sample_depth.png",
    output_path="/workspace/params/ai/debug_overlay.png",
  )
