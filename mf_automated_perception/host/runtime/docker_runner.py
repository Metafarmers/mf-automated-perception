from pathlib import Path
import subprocess
import os


def run_procedure(
  *,
  procedure: str,
  docker_image: str,
  creator: str,
  params_file: Path | None = None,
  bash: bool = False,
) -> subprocess.CompletedProcess:
  if params_file is not None and not params_file.exists():
    print(f"Params file not found: {params_file}")
    raise FileNotFoundError(params_file)

  data_root = os.environ.get("MF_DATA_ROOT")
  external_data_root = os.environ.get("MF_EXTERNAL_DATA_ROOT")

  if data_root is None:
    raise RuntimeError("MF_DATA_ROOT is not set")
  if external_data_root is None:
    raise RuntimeError("MF_EXTERNAL_DATA_ROOT is not set")

  cmd = [
    "docker", "run", "--rm",
  ]

  if bash:
    cmd += ["-it"]

  cmd += [
    "-v", f"{Path.cwd()}:/workspace",
    "-v", f"{Path(data_root).resolve()}:/data",
    "-v", f"{Path(external_data_root).resolve()}:/external_data",
  ]

  if bash:
    cmd += [
      docker_image,
      "/bin/bash",
    ]
  else:
    cmd += [
      docker_image,
      "python3", "-m",
      "mf_automated_perception.procedure.core.entrypoint",
      "--procedure", procedure,
      "--creator", creator,
      "--config", str(
        params_file
        if params_file is not None
        else f"/workspace/params/{procedure}.yaml"
      ),
    ]

  return subprocess.run(cmd, check=True)
