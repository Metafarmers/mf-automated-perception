# mf_automated_perception/procedure/defs/run_yolo/definition.py

from typing import ClassVar, List, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.grain.grain_base import GrainKey
from mf_automated_perception.procedure.core.procedure_base import ProcedureBase


class RunYoloConfig(BaseModel):
  yolo_model_config_file: str
  target_sensor_names: List[str]
  save_images_with_detections: bool
  save_images_step_size: int


class RunYolo(ProcedureBase):
  """
  Run YOLO inference on decoded rosbag images (definition only).
  """

  key: ClassVar[str] = "run_yolo"
  version: ClassVar[str] = "0.0.1"

  docker_image: ClassVar[str] = "mf-mantis-eye"
  docker_image_tag: ClassVar[str] = "latest"

  ParamModel: ClassVar[Optional[Type[BaseModel]]] = RunYoloConfig
  description: ClassVar[str] = "Run YOLO inference on decoded rosbag images."

  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (
    ("raw", "rosbag", "decoded"),
  )
  output_grain_key: ClassVar[GrainKey] = ("detection2d", 'yolo')
