"""Use to declare all main Classes and Interface"""

import dronekit
from pymavlink import mavutil # Needed for command message definitions
import time
from smartdrone.utils import sd_logger, get_location_difference_metres, FLYING_TEST_ID
from smartdrone.utils import r
import redis



class SmartDrone(dronekit.Vehicle):
    def __init__(self, *args):
        super(SmartDrone, self).__init__(*args) # temporary remove to build the framework
        self.dronename = "SmartDrone" # SmartDrone
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
            failsafe = self.smartmode.precheck_mode_failsafe()
            if failsafe:
                #TODO: check when should set the channel overrides which avoids crashing when mode changed to loiter
                sd_logger.info("Set throttle to 1500 via channel overrides.")
                self.channels.overrides['3'] = 1500
                time.sleep(1)
            else:
                self.smartmode.run()
                self.check_mode_change()

    def check_mode_change(self):
        """Not implemented. We need only 1 smart mode now.
        """
        pass

    def core_fly_type_control_funcs(self):
        pass

    def condition_yaw(self, heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.

        By default the yaw of the vehicle will follow the direction of travel. After setting 
        the yaw using this function there is no way to return to the default yaw "follow direction 
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

        For more information see: 
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.send_mavlink(msg)


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
        try:
            ping_ok = r.ping()
        except redis.exceptions.ConnectionError as e:
            sd_logger.error("FAILSAFE: Redis connection error")
            ping_ok = False
        if ping_ok:
            # sd_logger.info("[{}]NOT IMPLEMENTED failsafe check".format(self.name))
            self.state._logger("-----------------{}----------------- NOT IMPLEMENTED failsafe check".format(FLYING_TEST_ID))
            MODE = self.vehicle.mode.name
            PLND_ANGLE_MAX = self.vehicle.parameters['PLND_ANGLE_MAX']
            PLND_TACQ_DURATION = self.vehicle.parameters['PLND_TAR_ACQUIRE']
            PLND_YAW_ALIGN = self.vehicle.parameters['PLND_YAW_ALIGN']
            WPNAV_ANGLE_MAX = self.vehicle.parameters['WPNAV_ANGLE_MAX']
            ANGLE_MAX = self.vehicle.parameters['ANGLE_MAX']
            # self.state._logger("Mode: {} :: PLND_AM: {} :: WPNAV_AM: {} :: AM: {} :: Tacq: {}ms :: YawAlign :: {}".\
            #     format(MODE, PLND_ANGLE_MAX, WPNAV_ANGLE_MAX, ANGLE_MAX, PLND_TACQ_DURATION, PLND_YAW_ALIGN))
            sd_logger.info("Mode: {} :: PLND_AM: {} :: WPNAV_AM: {} :: AM: {} :: Tacq: {}ms :: YawAlign :: {}".\
                format(MODE, PLND_ANGLE_MAX, WPNAV_ANGLE_MAX, ANGLE_MAX, PLND_TACQ_DURATION, PLND_YAW_ALIGN))
            sd_logger.info(self.vehicle.battery)
            sd_logger.info(self.vehicle.channels)

            # sd_logger.info(self.vehicle.plnd)
            # sd_logger.info(self.vehicle.attitude)
            # gimbal_angles = (self.vehicle.gimbal.pitch, self.vehicle.gimbal.roll, self.vehicle.gimbal.yaw)
            # sd_logger.info("Gimbal pitch, roll, yaw: {}".format(gimbal_angles))
            # current_location = self.vehicle.location.global_relative_frame
            # home_location = self.vehicle.home_location # not Relative but absolut location.
            # if current_location is not None and home_location is not None:
            #     NED = get_location_difference_metres(current_location, home_location)
            #     sd_logger.info("Current location: {}".format(current_location))
            #     sd_logger.info("To home location NED: {}".format(NED))
            return 0
        else:
            sd_logger.info("Drone status prechecked, detect failsafe for mode {}".format(self.name))
            sd_logger.info("Back to AltHold mode")
            return 1

    def run(self):
        self.state.handle()
        self.check_current_state_complete()

    def check_current_state_complete(self):
        if self.state.complete:
            self.state = 'next_state'
            self.state.reset()

    def __eq__(self, name):
        if self.name==name:
            return True
        else:
            return False


class ModeState:
    def __init__(self, vehicle, mode, name=None):
        self.vehicle = vehicle
        self.mode = mode
        self.name = name if name else 'noname_state'
        self.reset()

    def reset(self):
        self.complete_code = 0

    def handle(self):
        self._compute_mission()
        self._update_navigation()
        self._update_doing()
        #TODO: update wait for a total time = 1-2s each loop
        self.wait_and_monitor_vehicle_mode_change(1) # wait here not too much logging each loop, only check ardupilot mode changed if needed.
        self._verify_complete_code()
        
    def wait_and_monitor_vehicle_mode_change(self, t=1):
        """ Wait t second while monitoring if mode change.
        """
        initial_mode = self.vehicle.mode
        start = time.time()
        while time.time()-start< t:
            if self.vehicle.mode != initial_mode:
                self._logger("Mode changed when waiting!")
                return 1
            time.sleep(0.2) # monitoring interval
        return 0
    
    def _compute_mission(self):
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
    
    def _logger(self, message):
        sd_logger.info("[{}][{}] ".format(self.mode.name, self.name) + message)

    def state_logging(self):
        sd_logger.info(self.vehicle.plnd)
        sd_logger.info(self.vehicle.attitude)
        gimbal_angles = (self.vehicle.gimbal.pitch, self.vehicle.gimbal.roll, self.vehicle.gimbal.yaw)
        sd_logger.info("Gimbal pitch, roll, yaw: {}".format(gimbal_angles))
        current_location = self.vehicle.location.global_relative_frame
        home_location = self.vehicle.home_location # not Relative but absolut location.
        if current_location is not None and home_location is not None:
            NED = get_location_difference_metres(current_location, home_location)
            sd_logger.info("Current location: {}".format(current_location))
            sd_logger.info("To home location NED: {}".format(NED))