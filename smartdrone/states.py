from smartdrone.core import ModeState
from smartdrone.utils import sd_logger
import random
from dronekit import VehicleMode

n = random.randint(0,22)

class PL_ManualControl(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'ManualControl'
    
    def _execute(self):
        """ Smart drone do nothing, waiting for switching state by change ardupilot mode from LOITER to GUIDED
        """
        sd_logger.info("Smart drone do nothing, waiting for switching state by change ardupilot mode from LOITER to GUIDED...")
        # TODO: add complete code for change smartmode. Now only one smartmode.
        # sd_logger.debug(self.vehicle.last_mode)
        self._update_complete_code()

    def _update_navigation(self):
        pass
    def _update_doing(self):
        pass
    def _update_complete_code(self):
        if self.vehicle.mode == VehicleMode('GUIDED') and self.vehicle.last_mode == VehicleMode('LOITER'):
                self.complete_code = 1

class PL_LandingPadSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadGo, 2 => back: ManualControl
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadSearch'

    def _execute(self):
        """
        """
        sd_logger.debug("Executing landing pad search")
        # TODO: add complete code for change smartmode. Now only one smartmode.
        self.complete_code = random.randint(0,2)

class PL_LandingPadGo(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadGo'

    def _execute(self):
        """
        """
        sd_logger.debug("Executing landing pad go")
        # TODO: add complete code for change smartmode. Now only one smartmode.
        self.complete_code = random.randint(0,3)

class PL_LandingPadLand(ModeState):
    # complete code: 0 => not completed, 1 => next: IRBeaconSearch, 2 => back: ManualControl = Landed
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadLand'

    def _execute(self):
        """
        """
        sd_logger.debug("Executing landing pad land")
        # TODO: add complete code for change smartmode. Now only one smartmode.
        self.complete_code = random.randint(0,2)

class PL_IRBeaconSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'IRBeaconSearch'

    def _execute(self):
        """
        """
        sd_logger.debug("Executing IR beacon search")
        # TODO: add complete code for change smartmode. Now only one smartmode.
        self.complete_code = random.randint(0,3)




    