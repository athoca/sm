from smartdrone.core import SmartDrone

class PLSmartDrone(SmartDrone):
    """Specific Smart Drone need specific mavlink info and fly controlling function.
    """
    def __init__(self, *args):
        super(PLSmartDrone, self).__init__(*args)
        pass

