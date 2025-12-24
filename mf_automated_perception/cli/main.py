from __future__ import annotations

import sys
from typing import Optional

import typer
from rich.console import Console
from rich.table import Table


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

    table = Table(title="Procedures")
    table.add_column("KEY")
    table.add_column("VERSION")
    table.add_column("EXECUTOR")

    table.add_row("locate_rosbags", "1.0.0", "docker")
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
  ) -> None:
    """
    Run a procedure.
    """
    cli.console.print("[bold]run called[/bold]")
    cli.console.print(f"procedure_key = {procedure_key}")
    cli.console.print(f"params = {params}")

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
