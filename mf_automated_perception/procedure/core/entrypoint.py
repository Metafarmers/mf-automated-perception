import argparse
import logging
import sys
from pathlib import Path
from typing import Dict, Optional

import yaml

from mf_automated_perception.env import GRAIN_DATA_ROOT, LOG_DIR_ROOT, PROJECT_ROOT
from mf_automated_perception.grain.grain_base import GrainKey
from mf_automated_perception.grain.grain_factory import GrainFactory
from mf_automated_perception.procedure.core.procedure_base import ProcedureRunResult
from mf_automated_perception.procedure.core.procedure_factory import ProcedureFactory
from mf_automated_perception.utils.gp_logger import get_logger
from mf_automated_perception.utils.time_utils import now


def parse_input_grain_uuids(
  args,
  procedure,
) -> Optional[Dict[GrainKey, str]]:
  if not args.input_grain_uuid:
    return None

  single_uuid = args.input_grain_uuid
  uuids = {}

  for key in procedure.required_input_grain_keys:
    uuids[key] = single_uuid

  for key in procedure.optional_input_grain_keys:
    uuids[key] = single_uuid

  return uuids


def parse_config(config_file: Optional[Path], param_model_cls):
  if param_model_cls is None:
    return None

  if config_file is None:
    raise ValueError("Config file required but not provided")

  if not config_file.is_absolute():
    config_file = PROJECT_ROOT / config_file

  data = yaml.safe_load(config_file.read_text())
  return param_model_cls.model_validate(data)


def main() -> None:
  parser = argparse.ArgumentParser(
    description="Run an automated perception procedure"
  )

  parser.add_argument("--procedure", required=True)
  parser.add_argument("--config", type=Path)
  parser.add_argument("--creator", required=True)
  parser.add_argument("--load-grain-rule",
                      choices=["latest", "uuid"], required=True)
  parser.add_argument("--log-level", default="INFO",
                      choices=["DEBUG", "INFO", "WARNING", "ERROR"])
  parser.add_argument("--input-grain-uuid", type=str)
  parser.add_argument(
    "--workflow-uuid",
    type=str,
    help="Workflow UUID propagated across procedures",
  )

  args = parser.parse_args()

  started_at = now()
  exit_code = 0
  status = "success"
  error_message = None
  output_grain_uuid = None

  ProcCls = ProcedureFactory.resolve_class(args.procedure)
  procedure = ProcCls()

  logger = get_logger(
    name=procedure.key,
    level=getattr(logging, args.log_level),
  )

  GrainFactory.build_registry()

  if args.load_grain_rule == "uuid" and not args.input_grain_uuid:
    raise ValueError("--input-grain-uuid required when --load-grain-rule=uuid")

  input_grain_uuids = parse_input_grain_uuids(args, procedure)

  try:
    input_grains = procedure.load_inputs(
      grain_data_root=GRAIN_DATA_ROOT,
      load_rule=args.load_grain_rule,
      input_grain_uuids=input_grain_uuids,
      logger=logger,
    )

    config = parse_config(args.config, procedure.ParamModel)

    if len(procedure.output_grain_key) > 0:
      output_grain_type = GrainFactory.resolve_class(procedure.output_grain_key)
      output_grain = output_grain_type()
    else:
      output_grain = None

    procedure.run(
      input_grains=input_grains,
      output_grain=output_grain,
      config=config,
      context={
        "creator": args.creator,
        "workflow_uuid": args.workflow_uuid,
      },
      logger=logger,
    )


    if output_grain is not None:
      output_grain_uuid = output_grain.uuid

  except Exception as e:
    logger.exception("Procedure execution failed")
    status = "failed"
    exit_code = 1
    error_message = str(e)

  finally:
    finished_at = now()

    result = ProcedureRunResult(
      procedure_key=procedure.key,
      procedure_version=getattr(procedure, "version", None),
      creator=args.creator,
      workflow_uuid=args.workflow_uuid,
      input_grain_uuid=args.input_grain_uuid,
      output_grain_uuid=output_grain_uuid,
      status=status,
      exit_code=exit_code,
      started_at=started_at,
      finished_at=finished_at,
      duration_sec=(finished_at - started_at).total_seconds(),
      log_dir=LOG_DIR_ROOT,
      error_message=error_message,
    )


    metadata_file = LOG_DIR_ROOT / "last_run.json"
    metadata_file.write_text(
      result.model_dump_json(indent=2)
    )

    if "output_grain" in locals() and output_grain is not None:
      try:
        output_grain.close()
      except Exception:
        logger.exception("Failed to close output grain")

    if status == "failed":
      sys.exit(exit_code)

  logger.info("Procedure finished successfully")


if __name__ == "__main__":
  main()
