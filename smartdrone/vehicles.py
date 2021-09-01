from smartdrone.core import SmartDrone
from smartdrone.modes import ArdupilotMode, PLMode
from smartdrone.utils import sd_logger

class PLSmartDrone(SmartDrone):
    """Specific Smart Drone need specific mavlink info and fly controlling function.
    """
    def __init__(self, *args):
        super(PLSmartDrone, self).__init__(*args)
        self.dronename = "PLSmartDrone"
        self.smartmode = ArdupilotMode(self)
    
    def check_mode_change(self):
        # TODO: update logic later.
        if isinstance(self.smartmode, ArdupilotMode):
            lastmode = self.smartmode.name
            self.smartmode = PLMode(self)
            sd_logger.debug("Change mode from {} to {}".format(lastmode, self.smartmode.name))



