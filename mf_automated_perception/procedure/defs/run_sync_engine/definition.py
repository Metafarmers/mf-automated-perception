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



class RunSyncEngineConfig(BaseModel):
  target_sensor_names: List[str]
  pivot_sensor_name: str
  max_time_diff_sec_dict: Dict[str, float]

class RunSyncEngine(ProcedureBase):
  key: ClassVar[str] = "run_sync_engine"
  version: ClassVar[str] = "0.0.1"
  docker_image: ClassVar[str] = 'mf-mantis-eye'
  docker_image_tag: ClassVar[str] = 'latest'
  ParamModel: ClassVar[Optional[Type[BaseModel]]] = RunSyncEngineConfig
  description: ClassVar[str] = "Time-sync grains from multiple (but not limited to) sensory data  "
  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = \
    (('raw', 'rosbag', 'decoded'), ('yolo', 'detections'),)
  output_grain_key: ClassVar[GrainKey] = ('snapshot',)


  def _run(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]],
    output_grain: Optional[GrainBase],
    config: RunSyncEngineConfig,
    logger,
  ) -> None:
    if input_grains is None:
      raise RuntimeError(f"{self.key}: input_grains must be provided")
    if output_grain is None:
      raise RuntimeError(f"{self.key}: output_grain must be provided")

    pivot_sensor = config.pivot_sensor_name
    if pivot_sensor not in config.target_sensor_names:
      raise ValueError(f'pivot_sensor_name {pivot_sensor} not in target_sensor_names')

    input_grain: GrainBase = input_grains[('raw', 'rosbag', 'decoded')]
    logger.info(f'table names: {input_grain.list_tables()}')

    #['image_id', 'timestamp_sec', 'timestamp_nsec', 'sensor_name', 'path_to_raw', 'width', 'height', 'encoding', 'K', 'D']
    # logger.info(f'columns in images table: {input_grain.list_column_names(table="images")}')

    # to python instance
    all_mf_images = images_load_from_grain(input_grain, config.target_sensor_names)
    logger.info(f'loaded image sensor names: {list(all_mf_images.keys())}')

    # sync engine
    sync_engine = SyncEngine()

    # register sensors
    for sensor_name in config.target_sensor_names:
      max_time_diff_sec = config.max_time_diff_sec_dict[sensor_name]
      sync_engine.register_sensor(
        sensor_name=sensor_name,
        sensor_type='camera',
        max_dt_ns=int(max_time_diff_sec * 1e9),
      )

    # add samples
    for sensor_name, mf_images in all_mf_images.items():
      if sensor_name not in config.target_sensor_names:
        raise ValueError(f'sensor_name {sensor_name} not in target_sensor_names. It supposed to be filtered previously.')
      for img in mf_images:
        sync_engine.add_sample(
          sensor_name=sensor_name,
          sec=img.timestamp_sec,
          nsec=img.timestamp_nsec,
          payload=img,
        )

    # perform sync
    synced, error = sync_engine.sync_over_pivot_sensor(pivot_sensor_name=pivot_sensor)
    if synced is None:
      logger.info(f'sync failed: {error}. No results to write.')

    logger.info(f'synced count: {len(synced)}')

    conn = output_grain.open()

    try:
      for anchor_ns, sync_decisions in synced.items():
        anchor_sec, anchor_nsec = nsec_to_sec_nsec(anchor_ns)

        # --------------------------------------------------
        # snapshot
        # --------------------------------------------------
        cur = conn.execute(
          """
          INSERT INTO snapshot (
            anchor_timestamp_sec,
            anchor_timestamp_nsec
          )
          VALUES (?, ?)
          """,
          (
            anchor_sec,
            anchor_nsec,
          ),
        )
        snapshot_id = cur.lastrowid

        # --------------------------------------------------
        # snapshot_item per sensor
        # --------------------------------------------------
        for sensor_name, decision in sync_decisions.items():
          img: MfImage = decision.payload

          # 1) payload 저장
          image_id = image_insert_and_return_id(
            conn=conn,
            img=img,
          )

          # 2) snapshot_item 라우팅
          conn.execute(
            """
            INSERT INTO snapshot_item (
              snapshot_id,
              sensor_name,
              payload_table_name,
              payload_id
            )
            VALUES (?, ?, ?, ?)
            """,
            (
              snapshot_id,
              sensor_name,
              "image",
              image_id,
            ),
          )

      conn.commit()

    finally:
      output_grain.close()

