import pytest
from mflib.perception.automated_perception.tests.integration_phase import IntegrationPhase

"""
Integration test orchestration

Phase order:
  10 - GRAIN_BASE
  20 - GRAIN_FACTORY
  30 - PROCEDURE_LOCATE

These tests rely on filesystem side effects and must be executed sequentially.
"""

@pytest.mark.integration
@pytest.mark.order(IntegrationPhase.LOCATE_ROSBAG_1_GRAIN)
def test_locate_rosbag1():
  pass

@pytest.mark.integration
@pytest.mark.order(IntegrationPhase.LOCATE_ROSBAG_2_PROCEDURE)
def test_locate_rosbag2():
  pass

@pytest.mark.integration
@pytest.mark.order(IntegrationPhase.LOCATE_ROSBAG_3_GRAIN_FACTORY)
def test_locate_rosbag3():
  pass


