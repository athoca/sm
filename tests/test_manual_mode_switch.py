import pytest
import time
from dronekit import VehicleMode
from smartdrone.modes import PLMode
from smartdrone.states import PL_ManualControl, PL_LandingPadSearch, PL_LandingPadGo, PL_LandingPadLand, PL_IRBeaconSearch

def test_one(PLvehicle, plmode):
    state = PL_ManualControl(PLvehicle, plmode)
    assert state.complete_code == 0
    PLvehicle.mode = VehicleMode('LOITER')
    time.sleep(1)
    PLvehicle.mode = VehicleMode('GUIDED')
    time.sleep(1)
    state._verify_complete_code()
    PLvehicle.mode
    assert state.complete_code == 1

def test_two(PLvehicle, plmode):
    PLvehicle.mode = VehicleMode('GUIDED')
    time.sleep(1)
    state = PL_LandingPadSearch(PLvehicle, plmode)
    assert state.complete_code == 0
    PLvehicle.mode = VehicleMode('LOITER')
    time.sleep(1)
    state._verify_complete_code()
    assert state.complete_code == 2
