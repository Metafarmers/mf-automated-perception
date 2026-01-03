from mf_automated_perception.host.runtime.docker_runner import run_procedure
from mf_automated_perception.procedure.core.procedure_base import ProcedureRunResult


def test_run_procedure_basic_integration():
  """
  Integration test:
  Actually runs Docker container via run_procedure.
  """

  result = run_procedure(
    procedure="test_basic_procedures",
    docker_image="mf-mantis-eye:latest",
    creator="test_user",
    params_file=None,
    bash=False,
  )
  print("ProcedureRunResult:", result)

  # must return subprocess.CompletedProcess
  assert isinstance(result, ProcedureRunResult)

  assert result.status == "success"
