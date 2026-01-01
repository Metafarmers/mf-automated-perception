import argparse
import logging
from pathlib import Path

from mf_automated_perception.env import GRAIN_DATA_ROOT, LOG_DIR_ROOT
from mf_automated_perception.grain.grain_factory import GrainFactory
from mf_automated_perception.procedure.core.procedure_factory import ProcedureFactory
from mf_automated_perception.utils.gp_logger import get_logger


def main() -> None:
  parser = argparse.ArgumentParser(
    description="Run an automated perception procedure"
  )

  parser.add_argument(
    "--procedure",
    required=True,
    help="Procedure key (e.g. locate_rosbags)",
  )

  parser.add_argument(
    "--config",
    type=Path,
    required=False,
    help="Path to procedure config YAML file",
  )

  parser.add_argument(
    "--creator",
    required=True,
    help="Creator name recorded in provenance",
  )

  parser.add_argument(
    "--log-name",
    default="procedure",
    help="Logger name",
  )

  parser.add_argument(
    "--log-level",
    default="INFO",
    choices=["DEBUG", "INFO", "WARNING", "ERROR"],
  )

  parser.add_argument(
    "--load-input-rule",
    default=None,
    choices=[None, "latest"],
    help="Input grain loading rule",
  )

  args = parser.parse_args()

  logger = get_logger(
    name=args.log_name,
    file_dir=LOG_DIR_ROOT,
    level=getattr(logging, args.log_level),
  )

  ProcCls = ProcedureFactory.resolve_class(args.procedure)
  procedure = ProcCls()

  logger.info(f"Starting procedure: {args.procedure}")
  logger.info(f"Config file: {args.config}")
  logger.info(f"Creator: {args.creator}")
  logger.info(f"Load input rule: {args.load_input_rule}")

  GrainFactory.build_registry()

  # -------------------------------------------------
  # resolve input_grains in Runner
  # -------------------------------------------------
  input_grains = None

  print('args.load_input_rule:', args.load_input_rule)
  if args.load_input_rule == "latest":
    input_grains = {}

    # required input grains
    for key in procedure.required_input_grain_keys:
      logger.info(f"Loading latest grain for key: {key}")
      grain = GrainFactory.load_latest_grain(
        key=key,
        grain_data_root=Path(GRAIN_DATA_ROOT),
      )
      input_grains[key] = grain

    # optional input grains
    print('optional_input_grain_keys:', procedure.optional_input_grain_keys)
    for key in procedure.optional_input_grain_keys:
      try:
        logger.info(f"Loading latest grain for key: {key}")
        grain = GrainFactory.load_latest_grain(
          key=key,
          grain_data_root=Path(GRAIN_DATA_ROOT),
        )
        input_grains[key] = grain
      except Exception as e:
        logger.warning(f"Failed to load optional grain {key}: {e}")

  # -------------------------------------------------
  # prepare output grain
  # -------------------------------------------------
  if len(procedure.output_grain_key) > 0:
    output_grain_type = GrainFactory.resolve_class(procedure.output_grain_key)
    output_grain = output_grain_type()
  else:
    output_grain = None

  try:
    procedure.run(
      input_grains=input_grains,
      output_grain=output_grain,
      config_file=args.config,
      context={
        "creator": args.creator,
      },
      logger=logger,
    )

  except Exception:
    logger.exception("Procedure execution failed")
    raise

  finally:
    try:
      if output_grain is not None:
        output_grain.close()
    except Exception:
      logger.exception("Failed to close output grain")

  logger.info("Procedure finished successfully")

if __name__ == "__main__":
  main()