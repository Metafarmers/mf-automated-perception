from typing import Any, ClassVar, Dict, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.procedure.core.procedure_base import (
  GrainBase,
  GrainKey,
  ProcedureBase,
)

class OfflineGlimConfig(BaseModel):
  livox_topic: str
  livox_imu_topic: str

class OfflineGlim(ProcedureBase):
  key: ClassVar[str] = "offline_glim"
  version: ClassVar[str] = "1.0.0"
  docker_image: ClassVar[str] = 'glim'
  docker_image_tag: ClassVar[str] = 'sync_engine'
  ParamModel: ClassVar[Optional[Type[BaseModel]]] = OfflineGlimConfig
  description: ClassVar[str] = "Offline GLIM procedure."
  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (('raw', 'rosbag', 'path'),)
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (('slam3d', 'glim'),) #pcd, 6d pose with loop closure
  output_grain_key: ClassVar[GrainKey] = ("dummy", )

  def _run(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]],
    output_grain: Optional[GrainBase],
    config,
    logger,
  ) -> None:
    logger.info("Running Dummy procedure")

    if not input_grains:
      raise RuntimeError(
        f"{self.key}: input_grains must be provided."
      )
    if output_grain is None:
      raise RuntimeError(f"{self.key}: output_grain must be provided.")

    path_grain: GrainBase = input_grains[('raw', 'rosbag', 'path')]
    logger.info(path_grain.dict_view(table='path_to_raw', limit=-1))