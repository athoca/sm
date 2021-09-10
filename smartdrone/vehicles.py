import dronekit
from smartdrone.core import SmartDrone
from smartdrone.modes import ArdupilotMode, PLMode
from smartdrone.utils import sd_logger, PLND

class PLSmartDrone(SmartDrone):
    """Specific Smart Drone need specific mavlink info and fly controlling function.
    """
    def __init__(self, *args):
        super(PLSmartDrone, self).__init__(*args)
        self.dronename = "PLSmartDrone"
        self.smartmode = PLMode(self)
        # Create an Vehicle.plnd object with initial values set to None.
        self.plnd = PLND()

        @self.on_message('RANGEFINDER')
        def _callback(self, _, message):
            self.plnd.H = message.distance
            self.plnd.ts = message._timestamp
            
        @self.on_message('AHRS3')
        def _callback(self, _, message):
            self.plnd.pX = message.roll
            self.plnd.pY = message.pitch
            self.plnd.vX = message.yaw
            self.plnd.vY = message.altitude
            self.plnd.TAcq = message.lat
            self.plnd.LastMeasMS = message.lng
            self.plnd.ts = message._timestamp
            # sd_logger.debug(message)

        # TODO: attitude: rad
    def get_height(self):
        if self.plnd.H is not None:
            return self.plnd.H
        else:
            return self.location.global_relative_frame.alt

    def rotate_gimbal_WE(self, wait_ready=True, timeout=1):
        """TODO: rotate, check pitch yaw, if not rerotate until timeout
        """
        sd_logger.debug("rotate_gimbal_WE")
    
    def rotate_gimbal_NS(self, wait_ready=True, timeout=1):
        """TODO: rotate, check pitch yaw, if not rerotate until timeout
        """
        sd_logger.debug("rotate_gimbal_NS")

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
