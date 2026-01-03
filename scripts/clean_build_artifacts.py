#!/usr/bin/env python3

from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]

DIR_PATTERNS = {
  "__pycache__",
  ".pytest_cache",
  ".mypy_cache",
  ".ruff_cache",
  "build",
  "dist",
}

FILE_SUFFIXES = {
  ".pyc",
  ".pyo",
  ".pyd",
}

FILE_NAMES = {
  "out.txt",
}

def remove_dir(path: Path) -> None:
  print(f"[DIR]  removing {path}")
  shutil.rmtree(path, ignore_errors=True)

def remove_file(path: Path) -> None:
  print(f"[FILE] removing {path}")
  path.unlink(missing_ok=True)

def main() -> None:
  for path in ROOT.rglob("*"):
    if path.is_dir():
      if path.name in DIR_PATTERNS:
        remove_dir(path)
        continue

      if path.name.endswith(".egg-info"):
        remove_dir(path)
        continue

    if path.is_file():
      if path.suffix in FILE_SUFFIXES:
        remove_file(path)
        continue

      if path.name in FILE_NAMES:
        remove_file(path)
        continue

      # remove pytest-generated images only
      if (
        path.suffix == ".png"
        and "tests" in path.parts
        and "test_" in path.name
      ):
        remove_file(path)

if __name__ == "__main__":
  main()

