from pathlib import Path
import subprocess


def run_procedure(
  *,
  procedure: str,
  params_file: Path,
  image: str = "mf-eye-runtime:humble",
  creator: str = "unknown",
) -> None:
  if not params_file.exists():
    raise FileNotFoundError(params_file)

  cmd = [
    "docker", "run", "--rm",
    "-v", f"{Path.cwd()}:/workspace",
    "-v", f"{(Path.cwd() / 'data').resolve()}:/data",
    "-v", "/home/tw/mf/project_manage_vcs/spagri/perception/data/"
          "mantis-eye/dongtan_automated_perception/dongtan_mot_routine:/external_data",
    image,
    "python3", "-m",
    "mf_automated_perception.procedure.core.entrypoint",
    "--procedure", procedure,
    "--creator", creator,
    "--config", f"/workspace/params/{procedure}.yaml",
  ]

  subprocess.run(cmd, check=True)
