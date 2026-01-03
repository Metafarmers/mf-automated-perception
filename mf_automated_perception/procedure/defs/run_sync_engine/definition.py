# mf_automated_perception/procedure/defs/run_sync_engine/definition.py

from typing import ClassVar, Dict, List, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.grain.grain_base import GrainKey
from mf_automated_perception.procedure.core.procedure_base import ProcedureBase


class RunSyncEngineConfig(BaseModel):
  target_sensor_names: List[str]
  pivot_sensor_name: str
  max_time_diff_sec_dict: Dict[str, float]


class RunSyncEngine(ProcedureBase):
  """
  Run time synchronization engine (definition only).
  """

  key: ClassVar[str] = "run_sync_engine"
  version: ClassVar[str] = "0.0.1"

  docker_image: ClassVar[str] = "mf-mantis-eye"
  docker_image_tag: ClassVar[str] = "latest"

  ParamModel: ClassVar[Optional[Type[BaseModel]]] = RunSyncEngineConfig
  description: ClassVar[str] = (
    "Time-sync grains from multiple sensory data."
  )

  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (
    ("raw", "rosbag", "decoded"),
    ("yolo", "detections"),
  )
  output_grain_key: ClassVar[GrainKey] = ("snapshot",)
