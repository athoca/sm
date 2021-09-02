import pytest
import time
from dronekit import VehicleMode
from smartdrone.vehicles import connect_to_PLdrone
from smartdrone.modes import PLMode
from smartdrone.states import PL_ManualControl, PL_LandingPadSearch, PL_LandingPadGo, PL_LandingPadLand, PL_IRBeaconSearch

@pytest.fixture(scope="module")
def PLvehicle():
    connection_string = "udpin:127.0.0.1:14551"
    return connect_to_PLdrone(connection_string)

@pytest.fixture(scope="module")
def plmode(PLvehicle):
     return PLMode(PLvehicle)