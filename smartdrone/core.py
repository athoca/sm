"""Use to declare all main Classes and Interface"""

import dronekit
from smartdrone.utils import sd_logger, wait_1s

class SmartDrone(dronekit.Vehicle):
    def __init__(self, *args):
        super(SmartDrone, self).__init__(*args) # temporary remove to build the framework
        self.dronename = "SmartDrone"
        # pass vehicle to get update of pixhawk controller
        self.smartmode = SmartMode(self)
        self.last_mode = self.mode
        self._current_mode = self.mode

        @self.on_attribute('mode')
        def _callback(self, _, message):
            self.last_mode = self._current_mode
            self._current_mode = message

    def start_main_control_loop(self):
        sd_logger.info("Start main control loop!")
        while True:
            self.smartmode.precheck_mode_failsafe()
            self.smartmode.run()
            self.check_mode_change()

    def check_mode_change(self):
        """Not implemented. We need only 1 smart mode now.
        """
        pass

    def core_fly_type_control_funcs(self):
        pass


class SmartMode:
    def __init__(self, vehicle, name=None, state=None):
        self.vehicle = vehicle
        self.name = name if name else 'smart_mode'
        # Each SmartMode know its State Machine Flow and the FIRST ONE
        # Pass vehicle to get update of pixhawk controller
        self.state = state if state else ModeState(self.vehicle, self) 
    
    def precheck_mode_failsafe(self):
        """Check if drone status is ready to run the mode. Mode ready means all state ready.
        """
        if True:
            sd_logger.info("----------------------------------")
            sd_logger.info("NOT IMPLEMENTED. Drone status, prechecked failsafe for mode {}".format(self.name))
        else:
            sd_logger.info("Drone status prechecked, detect failsafe for mode {}".format(self.name))
            sd_logger.info("Back to AltHold mode")

    def run(self):
        sd_logger.info("Run smart mode {}".format(self.name))
        self.state.handle()
        self.check_current_state_complete()

    def check_current_state_complete(self):
        if self.state.complete:
            self.state = 'next_state'
            self.state.reset()


class ModeState:
    def __init__(self, vehicle, mode, name=None):
        self.vehicle = vehicle
        self.mode = mode
        self.name = name if name else 'noname_state'
        self.reset()

    def reset(self):
        self.complete_code = 0

    def handle(self):
        sd_logger.info("Run state {} in smart mode {}".format(self.name, self.mode.name))
        self._execute()
        self._update_navigation()
        self._update_doing()
        #TODO: update wait for a total time = 1-2s each loop
        wait_1s() # wait here enable drone enough time to do updated command, especially change ardupilot modes if needed.
        self._verify_complete_code()
        
    
    def _execute(self):
        """From current status, compute next update of navigation or doing if needed.
        Not use a simple list of commands to facilitate complex algorithm.
        Standard execution should be less than 2s
        """
        sd_logger.debug("smart_algorithm")

    def _update_navigation(self):
        sd_logger.debug("update_navigation")

    def _update_doing(self):
        # Change mode, Failsafe
        sd_logger.debug("update_doing")

    def _verify_complete_code(self):
        # TODO: add complete code for change smartmode. Now only one smartmode.
        sd_logger.debug("_verify_complete_code")

    def __eq__(self, name):
        if self.name==name:
            return True
        else:
            return False





def func(x):
    return x + 1

def raise_exception():
    raise SystemExit(1)