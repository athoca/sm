import dronekit
from smartdrone.core import SmartDrone
from smartdrone.modes import ArdupilotMode, PLMode
from smartdrone.utils import sd_logger

class PLSmartDrone(SmartDrone):
    """Specific Smart Drone need specific mavlink info and fly controlling function.
    """
    def __init__(self, *args):
        super(PLSmartDrone, self).__init__(*args)
        self.dronename = "PLSmartDrone"
        self.smartmode = PLMode(self)
    
    def check_mode_change(self):
        # TODO: update logic later. Now auto update to PLMode
        if isinstance(self.smartmode, ArdupilotMode):
            lastmode = self.smartmode.name
            self.smartmode = PLMode(self)
            sd_logger.debug("Change mode from {} to {}".format(lastmode, self.smartmode.name))


def connect_to_PLdrone(connection_string, baud_rate=None):
    """Connect with drone autopilot using dronekit.

    Parameters
    ----------
    connection_string : string

    baud_rate : int

    Returns
    -------
    vehicle : a SmartDrone or None
    """
    # Try to connect to autopilot via dronekit
    drone = None
    try:
        if baud_rate:
            drone = dronekit.connect(connection_string, baud=baud_rate, wait_ready=True, vehicle_class=PLSmartDrone)
        else:
            drone = dronekit.connect(connection_string, wait_ready=True, vehicle_class=PLSmartDrone)
    except ConnectionError:
        sd_logger.info('******Cannot connect to drone autopilot !******')
    return drone
