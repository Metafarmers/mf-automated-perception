from pydantic import BaseModel
from pathlib import Path
import os
import yaml
from typing import List
import numpy as np
import cv2

# my library
from mflib.perception.mot.yolo_ultralytics import (
  YoloUltralytics,
  annotate_bbox_img
)
from mflib.perception.automated_perception.procedures.default_logger import get_logger

def iou_tlwh(a, b):
  ax, ay, aw, ah = a
  bx, by, bw, bh = b

  ax2, ay2 = ax + aw, ay + ah
  bx2, by2 = bx + bw, by + bh

  ix1 = max(ax, bx)
  iy1 = max(ay, by)
  ix2 = min(ax2, bx2)
  iy2 = min(ay2, by2)

  iw = max(0.0, ix2 - ix1)
  ih = max(0.0, iy2 - iy1)

  inter = iw * ih
  union = aw * ah + bw * bh - inter

  if union == 0:
    return 0.0
  return inter / union

def suppress_by_iou_keep_large(bboxes, iou_thresh=0.6):
  keep = []

  for b in bboxes:
    bx = b.tlwh
    bw, bh = bx[2], bx[3]
    b_area = bw * bh

    discard = False
    for k in keep:
      kx = k.tlwh
      kw, kh = kx[2], kx[3]
      k_area = kw * kh

      if iou_tlwh(bx, kx) > iou_thresh:
        # IoU가 크면 큰 놈만 남김
        if b_area > k_area:
          keep.remove(k)
        else:
          discard = True
        break

    if not discard:
      keep.append(b)

  return keep



class MfConfig(BaseModel):
  image_dir: str
  depth_dir: str
  flower_model: str
  flower_labels: List[int]
  branch_model: str
  branch_labels: List[int]
  leaf_seg_model: str
  leaf_seg_labels: List[int]

  target_inference_size: List[int]
  result_dir: str

  def load_config_from_yaml(yaml_path: str | Path) -> "MfConfig":
    yaml_path = Path(yaml_path)
    with yaml_path.open("r") as f:
      data = yaml.safe_load(f)

    return MfConfig.model_validate(data)

def process_flower_model(images, yolo_params, config: MfConfig, logger):
  # load models
  flower_model_params = yolo_params.copy()
  flower_model_params['model_path'] = config.flower_model
  flower_model_params['valid_bbox_labels'] = config.flower_labels
  flower_model_params['inference_output_type'] = 'bbox'
  flower_model = YoloUltralytics(logger, flower_model_params)

  # run flower detection
  for i, (im_name, image) in enumerate(images):
    print(f'Processing image {i+1}/{len(images)} with size {image.shape}')
    flower_result = flower_model.predict(image)
    debug_image = annotate_bbox_img(image, flower_result, logger)
    cv2.imwrite(
      str(Path(config.result_dir) / f"flower_debug_{im_name}"),
      debug_image
    )

def process_branch_model(images, yolo_params, config: MfConfig, logger):
  # load models
  branch_model_params = yolo_params.copy()
  branch_model_params['model_path'] = config.branch_model
  branch_model_params['valid_bbox_labels'] = config.branch_labels
  branch_model_params['inference_output_type'] = 'bbox'
  branch_model = YoloUltralytics(logger, branch_model_params)

  # run branch detection
  results = []
  for i, (im_name, image, _, _) in enumerate(images):
    print(f'Processing image {i+1}/{len(images)} with size {image.shape}')
    branch_result = branch_model.predict(image, to_ros_msg=False)
    results.append(branch_result)

  for i in range(len(images)):
    n_pre = len(results[i])
    results[i] = suppress_by_iou_keep_large(
      results[i],
      iou_thresh=0.5
    )
    n_post = len(results[i])
    logger.info(f'Image {i}: suppressed {n_pre} ->  {n_post} branches by IoU')

  for i in range(len(images)):
    n_leaves = len(results[i])
    debug_image = annotate_bbox_img(images[i][1], results[i], logger)
    cv2.putText(
      debug_image,
      f'{n_leaves*3} leaves (# of branch * 3)',
      (10, 30),
      cv2.FONT_HERSHEY_SIMPLEX,
      1, (0, 255, 0),
      2
    )
    cv2.imwrite(
      str(Path(config.result_dir) / f"branch_debug_{images[i][0]}"),
      debug_image
    )
  
  for re in results[-1]:
    center = re.tlwh[0] + re.tlwh[2]/2, re.tlwh[1] + re.tlwh[3]/2
    area = re.tlwh[2] * re.tlwh[3]
    print(f'center: {center}, area: {area}', re.tlwh)


def process_leaf_model(images, yolo_params, config: MfConfig, logger):
  # load models
  leaf_model_params = yolo_params.copy()
  leaf_model_params['model_path'] = config.leaf_seg_model
  leaf_model_params['valid_bbox_labels'] = config.leaf_seg_labels
  leaf_model_params['inference_output_type'] = 'instance_segmentation'
  leaf_model = YoloUltralytics(logger, leaf_model_params)

  # run leaf detection
  results = []
  for i, (im_name, image, _, _) in enumerate(images):
    print(f'Processing image {i+1}/{len(images)} with size {image.shape}')
    result = leaf_model.predict(image, to_ros_msg=False)
    results.append(result)
  
  for i in range(len(images)):
    n_pre = len(results[i])
    results[i] = suppress_by_iou_keep_large(
      results[i],
      iou_thresh=0.3
    )
    n_post = len(results[i])
    logger.info(f'Image {i}: suppressed {n_pre} ->  {n_post} branches by IoU')

  for i, (im_name, image, _, _) in enumerate(images):
    debug_image = annotate_bbox_img(image, results[i], logger)
    path = str(Path(config.result_dir) / f"leaf_debug_{im_name}")
    print(f'Saving leaf debug image to {path}')
    cv2.imwrite(
      path,
      debug_image
    )

def process_leaf_model2(images, yolo_params, config: MfConfig, logger):
  # run leaf detection
  leaf_model_params = yolo_params.copy()
  leaf_model_params['model_path'] = config.leaf_seg_model
  leaf_model_params['valid_bbox_labels'] = config.leaf_seg_labels
  leaf_model_params['inference_output_type'] = 'instance_segmentation'
  leaf_model = YoloUltralytics(logger, leaf_model_params)

  # run leaf detection
  _, rgb, _, depth = images[0]
  result = leaf_model.predict(rgb)
  # print(rgb.shape)
  # print(result[0].tlwh)
  # print(depth.shape)

  # 1. RGB -> depth scale factor 계산
  scale_x = depth.shape[1] / rgb.shape[1]
  scale_y = depth.shape[0] / rgb.shape[0]

  print(f"scale_x={scale_x:.6f}, scale_y={scale_y:.6f}")

  # 가정한 intrinsic (대충, depth 카메라 기준)
  fx = 600.0
  fy = 600.0

  rgb_vis_depth = rgb.copy()
  rgb_vis_axis = rgb.copy()

  bboxes_tlwh = [det.tlwh for det in result]
  for i, (x, y, w, h) in enumerate(bboxes_tlwh):
    x = int(round(x))
    y = int(round(y))
    w = int(round(w))
    h = int(round(h))

    # RGB -> depth 좌표계
    dx = int(round(x * scale_x))
    dy = int(round(y * scale_y))
    dw = int(round(w * scale_x))
    dh = int(round(h * scale_y))

    x0 = max(dx, 0)
    y0 = max(dy, 0)
    x1 = min(dx + dw, depth.shape[1])
    y1 = min(dy + dh, depth.shape[0])

    if x1 <= x0 or y1 <= y0:
      continue

    depth_crop = depth[y0:y1, x0:x1]
    valid_depth = depth_crop[depth_crop > 0]

    if valid_depth.size == 0:
      median_depth_mm = np.nan
    else:
      median_depth_mm = float(np.median(valid_depth))

    # mm -> m 변환
    median_depth_m = median_depth_mm * 0.001 if np.isfinite(median_depth_mm) else np.nan

    print(f"[bbox {i}] Z = {median_depth_mm:.1f} mm")

    # ---------- median depth 디버그 ----------
    cv2.rectangle(
      rgb_vis_depth,
      (x, y),
      (x + w, y + h),
      (0, 255, 0),
      2
    )

    depth_text = f"Z={median_depth_mm:.0f}mm" if np.isfinite(median_depth_mm) else "Z=nan"

    cv2.putText(
      rgb_vis_depth,
      depth_text,
      (x, max(y - 5, 15)),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.5,
      (0, 255, 0),
      1,
      cv2.LINE_AA
    )

    # ---------- 축 길이 계산 (depth 해상도 기준) ----------
    if np.isfinite(median_depth_m):
      width_m  = dw * median_depth_m / fx
      height_m = dh * median_depth_m / fy
    else:
      width_m = np.nan
      height_m = np.nan

    # ---------- 축 길이 디버그 ----------
    cv2.rectangle(
      rgb_vis_axis,
      (x, y),
      (x + w, y + h),
      (0, 255, 0),
      2
    )

    axis_text1 = f"W={width_m:.3f}m" if np.isfinite(width_m) else "W=nan"
    axis_text2 = f"H={height_m:.3f}m" if np.isfinite(height_m) else "H=nan"

    cv2.putText(
      rgb_vis_axis,
      axis_text1,
      (x, max(y - 18, 18)),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.45,
      (0, 255, 0),
      1,
      cv2.LINE_AA
    )

    cv2.putText(
      rgb_vis_axis,
      axis_text2,
      (x, max(y - 4, 32)),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.45,
      (0, 255, 0),
      1,
      cv2.LINE_AA
    )

  cv2.imwrite("rgb_depth_overlay.png", rgb_vis_depth)
  cv2.imwrite("rgb_axis_overlay.png", rgb_vis_axis)





def main():
  logger = get_logger(
    Path(__file__).stem,
    file_dir='/workspace/data/procedure_logs'
  )
  config = MfConfig.load_config_from_yaml("run_model_with_images.yaml")
  yolo_params = {
    'model_path': None,
    'conf_thresh': 0.02,
    'valid_bbox_labels': None, # [?, ?, ...]
    'inference_output_type': None, # bbox, keypoint, instance_segmentation
    'imgsz': config.target_inference_size,

    # ultralytics
    'use_cuda': True,
    'iou_thresh': 0.3,
    'verbose_yolo_predict': False,

    # bbox params
    'filter_bbox_by_labels': True,
    'margin_percent_for_bbox_reject': 1 # percent length margin used to reject bbox close to border
  }

  # 1. read images
  im_names = os.listdir(config.image_dir)
  im_names.sort()
  d_im_names = os.listdir(config.depth_dir)
  d_im_names.sort()
  images = []
  for rgb_name, depth_name in zip(im_names, d_im_names):
    rgb_path = os.path.join(config.image_dir, rgb_name)
    im = cv2.imread(rgb_path)
    im = cv2.resize(im, (config.target_inference_size[1], config.target_inference_size[0]))
    im = YoloUltralytics.ensure_multiple_of_32(im)

    depth_path = os.path.join(config.depth_dir, depth_name)
    depth_im = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

    images.append((rgb_name, im, depth_name, depth_im))
  

  # process_flower_model(images, yolo_params, config, logger)
  # process_branch_model(images, yolo_params, config, logger) # 720, 960
  process_leaf_model2(images, yolo_params, config, logger) # 387, 504 by depth anytbing v3

if __name__ == '__main__':
  main()