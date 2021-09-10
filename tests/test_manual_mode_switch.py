import pytest
import time
from dronekit import VehicleMode, LocationGlobalRelative
from smartdrone.modes import PLMode
from smartdrone.states import PL_ManualControl, PL_LandingPadSearch, PL_LandingPadGo, PL_LandingPadLand, PL_IRBeaconSearch

@pytest.mark.simulation
def test_PLSmartDrone_Initialization(PLvehicle, plmode):
    assert PLvehicle.smartmode == 'PL'
    assert PLvehicle.smartmode.state == 'ManualControl'

@pytest.mark.simulation
def test_ManualControl2LandingPadSearch(PLvehicle, plmode):
    state = PL_ManualControl(PLvehicle, plmode)
    assert state.complete_code == 0
    PLvehicle.mode = VehicleMode('LOITER')
    time.sleep(1)
    PLvehicle.mode = VehicleMode('GUIDED')
    time.sleep(1)
    state._verify_complete_code()
    assert state.complete_code == 1

@pytest.mark.simulation
def test_ManualControlNot2LandingPadSearch(PLvehicle, plmode):
    state = PL_ManualControl(PLvehicle, plmode)
    assert state.complete_code == 0
    PLvehicle.mode = VehicleMode('ALT_HOLD')
    time.sleep(1)
    PLvehicle.mode = VehicleMode('GUIDED')
    time.sleep(1)
    state._verify_complete_code()
    assert state.complete_code == 0

@pytest.mark.simulation
def test_LandingPadSearch2ManualControl(PLvehicle, plmode):
    PLvehicle.mode = VehicleMode('GUIDED')
    time.sleep(1)
    state = PL_LandingPadSearch(PLvehicle, plmode)
    assert state.complete_code == 0
    PLvehicle.mode = VehicleMode('LOITER')
    time.sleep(1)
    state._verify_complete_code()
    assert state.complete_code == 2

@pytest.mark.simulation
def test_LandingPadGo2ManualControl(PLvehicle, plmode):
    PLvehicle.mode = VehicleMode('GUIDED')
    time.sleep(1)
    state = PL_LandingPadGo(LocationGlobalRelative(None,None), PLvehicle, plmode)
    assert state.complete_code == 0
    PLvehicle.mode = VehicleMode('LOITER')
    time.sleep(1)
    state._verify_complete_code()
    assert state.complete_code == 2

@pytest.mark.simulation
def test_LandingPadLand2ManualControl(PLvehicle, plmode):
    PLvehicle.mode = VehicleMode('GUIDED')
    time.sleep(1)
    state = PL_LandingPadLand(PLvehicle, plmode)
    assert state.complete_code == 0
    assert PLvehicle.mode == VehicleMode('LAND')
    PLvehicle.mode = VehicleMode('LOITER')
    time.sleep(1)
    state._verify_complete_code()
    assert state.complete_code == 2

@pytest.mark.simulation
def test_IRBeaconSearch2ManualControl(PLvehicle, plmode):
    PLvehicle.mode = VehicleMode('LAND')
    time.sleep(1)
    state = PL_IRBeaconSearch(PLvehicle, plmode)
    assert state.complete_code == 0
    assert PLvehicle.mode == VehicleMode('GUIDED')
    PLvehicle.mode = VehicleMode('LOITER')
    time.sleep(1)
    state._verify_complete_code()
    assert state.complete_code == 2
