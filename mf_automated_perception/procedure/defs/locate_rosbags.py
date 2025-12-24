from pydantic import BaseModel
from typing import Dict, List, ClassVar, Type
from pathlib import Path
import yaml
import subprocess

from mflib.perception.automated_perception.grain.grain_base import GrainBase, GrainKey
from mflib.perception.automated_perception.procedure.procedure_base import ProcedureBase
from mflib.perception.automated_perception.grain.defs.raw_rosbag_path import RawRosbagPath

def try_rosbag_reindex(bag_dir: Path, logger) -> bool:
  """
  Try to reindex a rosbag directory.
  Return True if reindex succeeded, False otherwise.
  """
  cmd = ["ros2", "bag", "reindex", str(bag_dir)]

  logger.debug(f"Running reindex: {' '.join(cmd)}")

  try:
    proc = subprocess.run(
      cmd,
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
      text=True,
      check=False,
    )
  except FileNotFoundError:
    logger.error("ros2 CLI not found in PATH")
    return False

  if proc.returncode != 0:
    logger.debug(f"reindex failed for {bag_dir}")
    logger.debug(proc.stderr.strip())
    return False

  logger.info(f"reindex succeeded for {bag_dir}")
  return True


def find_leaf_directories(root: Path) -> List[Path]:
  """
  Find leaf directories under root.
  Leaf directory = directory with no subdirectories.
  """
  leaf_dirs: List[Path] = []

  for p in root.rglob("*"):
    if not p.is_dir():
      continue

    has_subdir = any(child.is_dir() for child in p.iterdir())
    if not has_subdir:
      leaf_dirs.append(p)

  return leaf_dirs

class LocateRosbagsConfig(BaseModel):
  search_root: str

class LocateRosbags(ProcedureBase):
  key: ClassVar[str] = "locate_rosbags"
  version: str = "1.0.0"
  ParamModel: ClassVar[Type] = LocateRosbagsConfig
  description: ClassVar[str] = "Locate ROS2 rosbags and store their paths as raw grains."
  input_grain_keys: ClassVar[List[GrainKey]] = []
  output_grain_key: ClassVar[GrainKey] = ("raw", "rosbag", "path")

  def write_results_to_db(
    self,
    *,
    grain_out: GrainBase,
    results: List[Dict],
    logger,
  ) -> None:
    if not results:
      logger.info("No results to write into DB")
      return

    conn = grain_out.open()
    try:
      cur = conn.cursor()

      for r in results:
        try:
          cur.execute(
            """
            INSERT OR REPLACE INTO path_to_raw
              (bag_dir, storage_identifier, detected_by)
            VALUES (?, ?, ?)
            """,
            (
              r["bag_dir"],
              r["storage_identifier"],
              r["detected_by"],
            ),
          )
        except Exception as e:
          logger.error(f"Failed to insert {r}: {e}")

      conn.commit()
      logger.info(f"Wrote {len(results)} rows into path_to_raw")

    finally:
      grain_out.close()


  def _run(
    self,
    *,
    input_grains: Dict[GrainKey, GrainBase],
    output_grain: GrainBase,
    config,
    logger,
  ) -> None:
    logger.debug('########################')

    root = Path(config.search_root)
    if not root.is_dir():
      raise ValueError(f"Invalid search_root: {root}")

    results: List[Dict] = []

    leaf_dirs = find_leaf_directories(root)
    logger.info(f"Found {len(leaf_dirs)} leaf directories under {root}")
    # for l in leaf_dirs:
    #   logger.debug(f"Leaf dir: {l}")

    for leaf in leaf_dirs:
      db3_files = list(leaf.glob("*.db3"))
      mcap_files = list(leaf.glob("*.mcap"))

      if not db3_files and not mcap_files:
        continue

      # ---------- determine storage_identifier by file extension ----------
      if db3_files and mcap_files:
        logger.error(
          f"Mixed rosbag formats found in {leaf}: "
          f"{len(db3_files)} *.db3, {len(mcap_files)} *.mcap. Abort this directory."
        )
        continue

      if db3_files:
        storage_id = "sqlite3"
      elif mcap_files:
        storage_id = "mcap"
      else:
        logger.debug("How do you see this?")
        continue

      logger.debug(
        f"Detected rosbag storage format in {leaf}: {storage_id}"
      )

      # ---------- metadata handling ----------
      metadata_path = leaf / "metadata.yaml"
      detected_by = "metadata"

      if not metadata_path.exists():
        logger.info(f"metadata.yaml not found in {leaf}, trying reindex")

        if not try_rosbag_reindex(leaf, logger):
          continue
        detected_by = "reindex"

        if not metadata_path.exists():
          logger.debug(f"reindex did not generate metadata.yaml in {leaf}")
          continue

      try:
        with metadata_path.open("r") as f:
          metadata = yaml.safe_load(f)
      except Exception as e:
        logger.debug(f"Failed to load metadata.yaml in {leaf}: {e}")
        continue

      # ---------- sanity check ----------
      try:
        msg_count = metadata["rosbag2_bagfile_information"]["message_count"]
      except KeyError:
        logger.debug(f"message_count not found in metadata.yaml in {leaf}")
        continue

      logger.debug(
        f"Found valid rosbag directory: {leaf} "
        f"storage={storage_id}, messages={msg_count}"
      )

      results.append(
        {
          "bag_dir": str(leaf),
          "storage_identifier": storage_id,
          "detected_by": detected_by,
        }
      )


    # logger.info(f'n results: {len(results)}')
    # for r in results:
    #   logger.debug(r)
    self.write_results_to_db(
      grain_out=output_grain,
      results=results,
      logger=logger,
    )

