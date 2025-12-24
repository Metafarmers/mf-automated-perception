import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
import rclpy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
    lifespan=rclpy.duration.Duration(seconds=0),  # Infinite
    deadline=rclpy.duration.Duration(seconds=0),  # Infinite
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=rclpy.duration.Duration(seconds=0)  # Infinite
)

bag_path = '/workspace/data/drone/apple2025/tree17_number1'

## shared params
rgb_topic               = '/d405/color/image_rect_raw/compressed'
rgb_info_topic          = '/d405/color/camera_info'
depth_topic             = '/d405/depth/image_rect_raw/compressedDepth'
depth_info_topic        = '/d405/depth/camera_info'
rgb_K = None
rgb_D = None
depth_K = None
depth_D = None


base_frame = 'map_imu_init'
body_frame = 'cam_downward'

# yolo_debug_topic        = '/mot_by_bbox/yolo_debug/compressed'
# bbox_topic              = '/mot_by_bbox/bbox_out'
# yolo_depth_debug_topic  = '/mot_by_bbox/yolo_depth_debug/compressed'
# fps = 15

# debug_camera_fov = True


# ## yolo detection
# yolo_params = {
#   'model_path': '/opt/config/perception/ai_models/strawberry2024DB.pt', # for x86_64
#   # 'model_path': '/opt/config/perception/ai_models/redlight_detection.pt', # for x86_64 # TODO: ISU-perception (모델 성능 확인 후 교체)
#    # ultralytics
#   'use_cuda': True,
#   'conf_thresh': 0.4,
#   'iou_thresh': 0.01,
#   'verbose_yolo_predict': False,

#    # topics
#   'yolo_debug_topic': '/mot_by_bbox/yolo_debug/compressed',
#   'yolo_raw_debug_topic': '/mot_by_bbox/yolo_raw_debug/compressed',
#   'yolo_depth_debug_topic': '/mot_by_bbox/yolo_depth_debug/compressed',

#    # node configuration
#   'publish_yolo_debug': True,
#   'publish_yolo_raw_debug': False,
#   'debug_on_depth': False,
#   'max_allowed_rgbd_timediff_ms': 1000/fps/2 - 1,
#   'depth_min_mm': 70,
#   'depth_max_mm': 600,

#   'inference_output_type': ['bbox'], # bbox, keypoint

#    # bbox params
#   'filter_bbox_by_labels': True,
#   'valid_bbox_labels': [6, 7], # None refers to all labels
#   # 'valid_bbox_labels': [0], # red flower model
#   'margin_percent_for_bbox_reject': 1 # percent length margin used to reject bbox close to border
# }

# # data synchronization
# image_queue_size = 100
# light_data_queue_size = 100

# # mot configuration
# dt_mot_timer = 0.05
# print_mot_health_check = False

# # odometry

# tf_timer_period_seconds = 0.05
# if tf_timer_period_seconds < dt_mot_timer:
#   tf_timer_period_seconds = dt_mot_timer

# # sync params
# verbose_sync = False
# sync_params = {
#   'max_allowed_rgbd_timediff_ms': yolo_params['max_allowed_rgbd_timediff_ms'],
#   'max_allowed_odom_time_diff_ms': 500
# }

# # 3D object detection
# detection3d_verbose = False
# debug_det3d = True
# det3d_params = {
#   'det3d_algorithm_type': 'Detection3DCreaterBBox', #'TSDFfromDepthROI', Detection3DCreaterBBox
#   'valid_bbox_labels': yolo_params['valid_bbox_labels'],
#   'out_mask_tlbr': (0,0,0,0),
#   'max_allowed_nan_ratio': 0.9,
#   'depth_min_mm': yolo_params['depth_min_mm'], # yolo.py
#   'depth_max_mm': yolo_params['depth_max_mm'], # yolo.py
# }

# trajectory_set_type = trajectory_manager.MfMotTrajectorySetCube3D

# trajectory_params = {
#    'n_max_trajectory': 10000,

#   # trajectory management
#   'query_xyz_radius': 0.1,
#   'n_max_neighbor': 20,
#   'do_associate_cost_thresh': 0.02, # meter
#   'do_pruning': False,
#   'pruning_no_update_count_thresh': 5,

#   # trajectory data management
#   'det2d_max_score': 0.5,
#   'det2d_update_factor': 0.5,
#   'volume_update_w': 0.7,
#   'pose_update_w': 0.7,
#   'n_required_association': 5,
#   'thresh_confirmation': 1.0,
# }

# debug_trajectory_mode = 'valid' # 'valid' 'unreliable' 'active'
# debug_trajectory_min_det3d_count = 3

# # other utils
# use_manipulator_adapter = False
