import json
from typing import ClassVar, Dict, List, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey
from mf_automated_perception.procedure.core.procedure_base import ProcedureBase
from mf_automated_perception.realtime.datatypes.image import MfImage
from mf_automated_perception.realtime.sync.engine import SyncEngine
from mf_automated_perception.realtime.sync.time_utils import nsec_to_sec_nsec


def images_load_from_grain(input_grain, target_sensor_names: List[str]) -> Dict[str, List[MfImage]]:
  """
  Load all rows from image table and convert to MfImage list.
  """
  rows = input_grain.dict_view(table="image", limit=-1)

  images: dict[str, List[MfImage]] = {}

  for row in rows:
    # handle sensor_name indexing
    sensor_name = row["sensor_name"]
    if sensor_name not in target_sensor_names:
      continue
    if sensor_name not in images:
      images[sensor_name] = []
    d = dict(row)

    # JSON decode camera params if present
    if d.get("K") is not None:
      d["K"] = json.loads(d["K"])
    if d.get("D") is not None:
      d["D"] = json.loads(d["D"])

    images[sensor_name].append(MfImage.from_dict(d))

  return images

def image_insert_and_return_id(
  *,
  conn,
  img: MfImage,
) -> int:
  """
  Insert one MfImage into image table and return image_id.
  """

  cur = conn.execute(
    """
    INSERT INTO image (
      sensor_name,
      timestamp_sec,
      timestamp_nsec,

      path_to_raw,
      width,
      height,
      encoding,
      K,
      D
    )
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
    """,
    (
      img.sensor_name,
      img.timestamp_sec,
      img.timestamp_nsec,

      img.path_to_raw,
      img.width,
      img.height,
      img.encoding,
      json.dumps(img.K) if img.K is not None else None,
      json.dumps(img.D) if img.D is not None else None,
    ),
  )

  return cur.lastrowid



class RunYoloConfig(BaseModel):
  yolo_model_config_file: str
  target_sensor_names: List[str]

class RunYolo(ProcedureBase):
  key: ClassVar[str] = "run_yolo"
  version: ClassVar[str] = "0.0.1"
  docker_image: ClassVar[str] = 'mf-mantis-eye'
  docker_image_tag: ClassVar[str] = 'latest'
  ParamModel: ClassVar[Optional[Type[BaseModel]]] = RunYoloConfig
  description: ClassVar[str] = "Time-sync grains from multiple (but not limited to) sensory data  "
  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (('raw', 'rosbag', 'decoded'),)
  output_grain_key: ClassVar[GrainKey] = ('snapshot',)


  def _run(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]],
    output_grain: Optional[GrainBase],
    config: RunYoloConfig,
    logger,
  ) -> None:
    if input_grains is None:
      raise RuntimeError(f"{self.key}: input_grains must be provided")
    if len(input_grains) > 1:
      raise RuntimeError(
        f"{self.key}: only one input grain is expected, got: {list(input_grains.keys())}"
      )
    if output_grain is None:
      raise RuntimeError(f"{self.key}: output_grain must be provided")

    rosbag_grain: GrainBase = input_grains[('raw', 'rosbag', 'decoded')]
    logger.info(f'table names: {rosbag_grain.list_tables()}')

    #['image_id', 'timestamp_sec', 'timestamp_nsec', 'sensor_name', 'path_to_raw', 'width', 'height', 'encoding', 'K', 'D']

    # to python instance
    all_mf_images = images_load_from_grain(rosbag_grain, config.target_sensor_names)
    logger.info(f'loaded image sensor names: {list(all_mf_images.keys())}')
