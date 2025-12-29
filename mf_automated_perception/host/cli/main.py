from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import typer
from rich.console import Console
from rich.table import Table

from mf_automated_perception.host.runtime.docker_runner import run_procedure
from mf_automated_perception.procedure.core.procedure_factory import (
  ProcedureFactory,
  ProcedureBase,
)

class MfEyeCLI:
  """
  CLI facade for mf-eye.

  This class owns CLI state (runner, config, console) and exposes
  bound methods as Typer commands.
  """

  def __init__(self) -> None:
    self.console = Console()
    self.runner = None  # 나중에 Runner.default()로 교체


def create_app(cli: MfEyeCLI) -> typer.Typer:
  app = typer.Typer(
    name="mf-eye",
    help="CLI for automated perception procedures",
    no_args_is_help=True,
  )

  # -----------------------------------------------------------------
  # version
  # -----------------------------------------------------------------
  @app.command()
  def version() -> None:
    """
    Show mf-eye version.
    """
    cli.console.print("mf-eye version 0.1.0")

  # -----------------------------------------------------------------
  # list-procedures
  # -----------------------------------------------------------------
  @app.command("list-procedures")
  def list_procedures() -> None:
    """
    List available procedures.
    """
    cli.console.print("[bold]list-procedures called[/bold]")

    ProcedureFactory.build_registry()
    detail = ProcedureFactory.list_procedures()

    table = Table(title="Procedures")
    table.add_column("KEY")
    table.add_column("docker_image")
    table.add_column("version")
    table.add_column('description')

    for _, classtype in detail.items():
      table.add_row(
        classtype.key,
        f'{classtype.docker_image}:{classtype.docker_image_tag}',
        classtype.version,
        classtype.description
      )
    cli.console.print(table)

  # -----------------------------------------------------------------
  # show-procedure
  # -----------------------------------------------------------------
  @app.command("show-procedure")
  def show_procedure(
    key: str = typer.Argument(..., help="Procedure key"),
  ) -> None:
    """
    Show detailed information about a procedure.
    """
    cli.console.print(
      f"[bold]show-procedure called[/bold]: key={key}"
    )

  # -----------------------------------------------------------------
  # list-grains
  # -----------------------------------------------------------------
  @app.command("list-grains")
  def list_grains() -> None:
    """
    List available grains.
    """
    cli.console.print("[bold]list-grains called[/bold]")

    table = Table(title="Grains")
    table.add_column("KEY")
    table.add_column("TYPE")

    table.add_row("raw_rosbag", "RawRosbagGrain")
    cli.console.print(table)

  # -----------------------------------------------------------------
  # show-grain
  # -----------------------------------------------------------------
  @app.command("show-grain")
  def show_grain(
    key: str = typer.Argument(..., help="Grain key"),
  ) -> None:
    """
    Show detailed information about a grain.
    """
    cli.console.print(f"[bold]show-grain called[/bold]: key={key}")

  # -----------------------------------------------------------------
  # run
  # -----------------------------------------------------------------
  @app.command()
  def run(
    procedure_key: str = typer.Argument(..., help="Procedure key"),
    params: Optional[str] = typer.Option(
      None, "--params", help="Path to params file (yaml/json)"
    ),
    bash: bool = typer.Option(
      False, "--bash", help="Run docker container with bash instead of procedure"
    ),
  ) -> None:
    """
    Run a procedure.
    """
    # query image info from ProcedureFactory
    ProcedureFactory.build_registry()
    procedure = ProcedureFactory.resolve_class(procedure_key)
    docker_image = f"{procedure.docker_image}:{procedure.docker_image_tag}"

    if params is None:
      if procedure.ParamModel is not None:
        params_path = Path("params") / f"{procedure_key}.yaml"
      else:
        params_path = None
    else:
      params_path = Path(params)

    cli.console.print(f"Running procedure: {procedure_key}")
    cli.console.print(f"params = {params_path}")


    try:
      run_procedure(
        procedure=procedure_key,
        params_file=params_path,
        creator="tw",
        docker_image=docker_image,
        bash=bash,   # ← 여기만 추가
      )
    except Exception as e:
      cli.console.print(f"Failed: {e}")
      raise typer.Exit(code=1)

    cli.console.print(
      f"Procedure '{procedure_key}' finished"
    )



  # -----------------------------------------------------------------
  # doctor
  # -----------------------------------------------------------------
  @app.command()
  def doctor() -> None:
    """
    Check environment status.
    """
    cli.console.print("[bold]doctor called[/bold]")
    cli.console.print(f"python = {sys.version.split()[0]}")

    try:
      import docker  # type: ignore

      cli.console.print("docker = available")
    except Exception:
      cli.console.print("docker = not available")

  return app


# ---------------------------------------------------------------------
# entrypoint
# ---------------------------------------------------------------------
def main() -> None:
  cli = MfEyeCLI()
  app = create_app(cli)
  app()


# Typer entrypoint expects a callable
app = create_app(MfEyeCLI())


if __name__ == "__main__":
  main()
