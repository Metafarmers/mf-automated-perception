#!/usr/bin/env python3

import argparse
import subprocess
from pathlib import Path
import sys

DOCKER_ROOT = Path("docker")


def list_available_images():
  if not DOCKER_ROOT.exists():
    return []

  return [
    p.name
    for p in DOCKER_ROOT.iterdir()
    if p.is_dir()
  ]


def read_version(image_dir: Path) -> str:
  version_file = image_dir / "version"

  if not version_file.exists():
    print(f"[ERROR] version file not found: {version_file}")
    sys.exit(1)

  version = version_file.read_text().strip()

  if not version:
    print(f"[ERROR] version file is empty: {version_file}")
    sys.exit(1)

  return version


def parse_args():
  parser = argparse.ArgumentParser(
    description="Build docker image from docker/<image_name>/Dockerfile"
  )

  parser.add_argument(
    "--image_name",
    required=True,
    help="Docker image name (directory under docker/)"
  )

  return parser.parse_args()


def main():
  args = parse_args()

  image_name = args.image_name
  image_dir = DOCKER_ROOT / image_name
  dockerfile = image_dir / "Dockerfile"

  if not image_dir.exists():
    print(f"[ERROR] Docker directory not found: {image_dir}")
    print("")
    print("Available images:")
    for name in list_available_images():
      print(f"  - {name}")
    sys.exit(1)

  if not dockerfile.exists():
    print(f"[ERROR] Dockerfile not found: {dockerfile}")
    sys.exit(1)

  version = read_version(image_dir)

  version_tag = f"{image_name}:{version}"
  latest_tag = f"{image_name}:latest"

  build_cmd = [
    "docker", "build",
    "-t", version_tag,
    "-f", str(dockerfile),
    ".",
  ]

  print("Building docker image")
  print(f"  image   : {version_tag}")
  print(f"  context : .")
  print("")
  print("Command:")
  print(" ", " ".join(build_cmd))
  print("")

  try:
    subprocess.run(build_cmd, check=True)
  except subprocess.CalledProcessError as e:
    print("[ERROR] Docker build failed")
    sys.exit(e.returncode)

  print("")
  print(f"Tagging image as latest: {latest_tag}")

  tag_cmd = [
    "docker", "tag",
    version_tag,
    latest_tag,
  ]

  try:
    subprocess.run(tag_cmd, check=True)
  except subprocess.CalledProcessError as e:
    print("[ERROR] Docker tag failed")
    sys.exit(e.returncode)

  print("")
  print("Done.")
  print(f"Built image : {version_tag}")
  print(f"Also tagged : {latest_tag}")


if __name__ == "__main__":
  main()
