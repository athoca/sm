"""Use to declare all main Classes and Interface"""

import dronekit
from smartdrone.utils import sd_logger, wait_5s

class SmartDrone(dronekit.Vehicle):
    def __init__(self, smartmode, *args):
        # super(SmartDrone, self).__init__(*args) # temporary remove to build the framework
        # pass vehicle to get update of pixhawk controller
        self.dronename = "SmartDrone"
        self.smartmode = SmartMode(self) # default is a Ardupilot mode

    def start_main_control_loop(self):
        sd_logger.info("Start main control loop!")
        while True:
            self.smartmode.precheck_mode_failsafe()
            self.smartmode.run()
            self.check_mode_change()
            wait_5s()

    def check_mode_change(self):
        """Not implemented. We need only 1 smart mode now.
        """
        pass

    def core_fly_type_control_funcs(self):
        pass


class SmartMode:
    def __init__(self, vehicle, name=None, state=None):
        self.vehicle = vehicle
        self.name = name if name else 'ardupilot_mode'
        # Each SmartMode know its State Machine Flow and the FIRST ONE
        # pass vehicle to get update of pixhawk controller
        self.state = state if state else ModeState(self.vehicle, self) 
    
    def precheck_mode_failsafe(self):
        """Check if drone status is ready to run the mode. Mode ready means all state ready.
        """
        if True:
            sd_logger.info("NOT IMPLEMENTED. Drone status, prechecked failsafe for mode {}".format(self.name))
        else:
            sd_logger.info("Drone status prechecked is not failsafe for mode {}".format(self.name))
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
        self.complete = False

    def handle(self):
        sd_logger.info("Run state {} in smart mode".format(self.name, self.mode.name))
        self.execute()
        self.update_navigation()
        self.update_doing()
    
    def execute(self):
        """From current status, compute next update of navigation or doing if needed.
        Not use a simple list of commands to facilitate complex algorithm.
        Standard execution should be less than 2s
        """
        sd_logger.debug("smart_algorithm")

    def update_navigation(self):
        sd_logger.debug("update_navigation")

    def update_doing(self):
        # Change mode, Failsafe
        sd_logger.debug("update_doing")





def func(x):
    return x + 1

def raise_exception():
    raise SystemExit(1)