from smartdrone.core import SmartMode
from smartdrone.states import PL_ManualControl, PL_LandingPadSearch, PL_LandingPadGo, PL_LandingPadLand, PL_IRBeaconSearch
from smartdrone.utils import sd_logger

class PLMode(SmartMode):
    """Specific Smart Mode need specific info and State Machine algorithm flow.
    """
    def __init__(self, *args):
        super(PLMode, self).__init__(*args)
        self.name = 'PL' # precision_landing
        self.state = PL_ManualControl(self.vehicle, self)
        self._original_heading = None # TODO: temporary support for develop yaw detection, remove in future
    
    def check_current_state_complete(self):
        # ManualControl     : 0 => not completed, 1 => next: LandingPadSearch
        # LandingPadSearch  : 0 => not completed, 1 => next: LandingPadGo, 2 => back: ManualControl
        # LandingPadGo      : 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
        # LandingPadLand    : 0 => not completed, 1 => next: IRBeaconSearch, 2 => back: ManualControl = Landed
        # IRBeaconSearch    : 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
        complete_code = self.state.complete_code
        # TODO: add complete code for change mode
        # TODO: use dict for elegant and easy to verify State Machine algorithm

        # TODO: check current mode change to LOITER => not overwrite by below.
        if complete_code:
            self.state._logger("Complete code {}".format(complete_code))
            if complete_code == 2:
                self.state = PL_ManualControl(self.vehicle, self)
            elif complete_code == 3:
                self.state = PL_LandingPadSearch(self.vehicle, self)
            elif complete_code == 1:
                if self.state == 'ManualControl': 
                    self.state = PL_LandingPadSearch(self.vehicle, self)
                elif self.state == 'LandingPadSearch': 
                    self.state = PL_LandingPadGo(self.state.detected_target, self.vehicle, self)
                elif self.state == 'LandingPadGo': 
                    self.state = PL_LandingPadLand(self.vehicle, self)
                elif self.state == 'LandingPadLand': 
                    self.state = PL_IRBeaconSearch(self.vehicle, self)
                elif self.state == 'IRBeaconSearch': 
                    self.state = PL_LandingPadLand(self.vehicle, self)
            

class ArdupilotMode(SmartMode):
    """Mode controlled by ardupilot. Do nothing. Need no state. Drone work in normal ardupilot modes.
    """
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'ardupilot'
        self._flag = False # only display log once

    def precheck_mode_failsafe(self):
        if not self._flag:
            sd_logger.debug("Check nothing. All failsafe handled by ardupilot firmware")
    def run(self):
        if not self._flag:
            sd_logger.info("Running ardupilot mode, smart drone is in lazy time...")
            self._flag = True

