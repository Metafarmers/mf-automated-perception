from pathlib import Path
import os

DATA_ROOT = Path(
  os.environ.get("MF_DATA_ROOT", "/data")
).resolve()
if not DATA_ROOT.is_dir():
  raise RuntimeError(f"Data root not found: {DATA_ROOT}")

SCHEMA_ROOT = Path(
  "/workspace/mf_automated_perception/grain/schema"
).resolve()
if not SCHEMA_ROOT.is_dir():
  raise RuntimeError(f"Schema root not found: {SCHEMA_ROOT}")