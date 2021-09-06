from logging import error
from os import name
from smartdrone.core import ModeState
from smartdrone.utils import sd_logger, wait_1s, do_nothing, get_distance_metres, get_location_metres
from smartdrone.config import LandingPadSearch_Config, LandingPadGo_Config
import random
import time
from dronekit import VehicleMode
from dronekit import LocationGlobal, LocationGlobalRelative
class PL_ManualControl(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'ManualControl'
    
    def _compute_mission(self):
        """ Smart engine do nothing, waiting for switching state by change ardupilot mode from LOITER to GUIDED
        """
        sd_logger.info("Wait for switching state by change ardupilot mode from LOITER to GUIDED...")
        #TODO: check when should set the channel overrides which avoids crashing when mode changed to loiter
        sd_logger.info("setting throttle channel to 1500 via channel overrides")
        self.vehicle.channels.overrides['3'] = 1500
        wait_1s() # time to received mavlink mode update

    def _update_navigation(self):
        do_nothing()

    def _update_doing(self):
        do_nothing()

    def _verify_complete_code(self):
        if self.vehicle.mode == VehicleMode('GUIDED') and self.vehicle.last_mode == VehicleMode('LOITER'):
                self.complete_code = 1
                return

class PL_LandingPadSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadGo, 2 => back: ManualControl (if mode != GUIDED)
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadSearch'
        self.detected_target = None
        self._is_detected = False
        self._original_location = self.vehicle.location.global_relative_frame
        self.target_altitude = LandingPadSearch_Config.ALTITUDE
        self.error_threshold = LandingPadSearch_Config.ERROR_THRESHOLD
        # set first target location
        self.target_location = {"idx":[0,0],
            "loc":LocationGlobalRelative(self._original_location.lat, self._original_location.lon, self.target_altitude)}
        self.from_ground = self._is_from_ground()
        # repeated 2 steps in the state:
        self._navigation = True
        self._doing = False

    def reset(self):
        super().reset()

    def _is_from_ground(self):
        return (not self.vehicle.armed) or (self._original_location.alt < 0.7)

    def _update_target_location(self, square_size=10, nb_squares=1):
        # TODO future: increase square_size after a long time.
        # TODO: update square_size and nb_squares from config
        current_i = self.target_location['idx'][0]
        current_j = self.target_location['idx'][1]
        li = list(range(-nb_squares, nb_squares+1))
        li.remove(current_i)
        lj = list(range(-nb_squares, nb_squares+1))
        lj.remove(current_j)
        i = random.choice(li)
        j = random.choice(lj)
        original_target_location = LocationGlobalRelative(self._original_location.lat, self._original_location.lon, self.target_altitude)
        self.target_location = {"idx":[i,j],
            "loc": get_location_metres(original_target_location, j*square_size, i*square_size)}

    def _compute_mission(self):
        """Repeat wandering in a square area then detecting landing pad until detected.
        """
        if self._navigation:
            current_location = self.vehicle.location.global_relative_frame
            dist = get_distance_metres(current_location, self.target_location['loc'])
            sd_logger.debug("Distance to the target location: {}".format(dist))
            if dist < self.error_threshold:
                self._navigation = False
                self._doing = True
                self.from_ground = False
                self._update_target_location()
            return
            
    def _update_navigation(self):
        if self._navigation:
            if not self.from_ground:
                sd_logger.debug("Waiting for GO TO target location {}".format(self.target_location))
                self.vehicle.simple_goto(self.target_location['loc'])    
            else:
                if not self.vehicle.is_armable:
                    sd_logger.debug(" Waiting for vehicle armable to initialise...")
                    return
                else:
                    if not self.vehicle.armed:
                        sd_logger.debug("Waiting for arming motors")
                        self.vehicle.armed = True
                        return
                sd_logger.debug("Waiting for TAKE OFF to target altitude {} m".format(self.target_altitude))
                sd_logger.debug(" Altitude: {}".format(self.vehicle.location.global_relative_frame.alt))
                self.vehicle.simple_takeoff(self.target_altitude) # Take off to target altitude

    def _update_doing(self):
        if self._doing:
            sd_logger.debug("Waiting for doing detection")
            self._do_detection()
            #TODO future: for simplicity, do command is return done immediately. Update if needed later.
            self._navigation = True
            self._doing = False

    def _verify_complete_code(self):
        if self.vehicle.mode != VehicleMode('GUIDED'):
                self.complete_code = 2
                return
        if self._is_detected:
            self.complete_code = 1
            return
    
    def _do_detection(self):
        # TODO: implement logic + set target position from detection (for next state)
        self._is_detected = random.choice([0,1,1])
        current_location = self.vehicle.location.global_relative_frame
        self.detected_target = LocationGlobalRelative(current_location.lat, current_location.lon, 0)
        wait_1s()


class PL_LandingPadGo(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, target, *args):
        super().__init__(*args)
        self.name = 'LandingPadGo'
        self.detected_target = target
        self.detected_yaw = None
        self._lost_target = False
        self._target_acquired = False
        self.error_threshold = LandingPadGo_Config.ERROR_THRESHOLD
        self.h2 = LandingPadGo_Config.H2
        self.h1 = LandingPadGo_Config.H1
        self._target_location_h2 = LocationGlobalRelative(target.lat, target.lon, self.h2)
        self._target_location_h1 = LocationGlobalRelative(target.lat, target.lon, self.h1)
        self._target_yaw = None
        # one-way sequence of 6 steps of the state:
        self._move_on_h2 = True
        self._doing_on_h2 = False
        self._move_on_h1 = False
        self._doing_on_h1 = False
        self._is_yawing = False
        self._precheck_land = False

    def _compute_mission(self):
        """ Follow three steps: Move, Approach, then Yaw
        """
        # Move to h2 on top, then detect again, if not => search again
        if self._move_on_h2:
            current_location = self.vehicle.location.global_relative_frame
            dist = get_distance_metres(current_location, self._target_location_h2)
            sd_logger.debug("Distance to the h2 target location: {}".format(dist))
            if dist < self.error_threshold:
                self._move_on_h2 = False
                self._doing_on_h2 = True
            return
        # Approach to h1, then detect again, if not => search again
        if self._move_on_h1:
            current_location = self.vehicle.location.global_relative_frame
            dist = get_distance_metres(current_location, self._target_location_h1)
            sd_logger.debug("Distance to the h1 target location: {}".format(dist))
            if dist < self.error_threshold:
                self._move_on_h1 = False
                self._doing_on_h1 = True
            return
        # Yawing and check target acquired, if not => search again
        if self._is_yawing:
            # TODO: check self._target_yaw - current yaw < error, if ok => precheck_land()
            if True:
                self._is_yawing = False
                self._precheck_land = True

    def _update_navigation(self):
        if self._move_on_h2:
            sd_logger.debug("Waiting for GO TO H2 target location {}".format(self._target_location_h2))
            self.vehicle.simple_goto(self._target_location_h2)
            return
        if self._move_on_h1:
            sd_logger.debug("Waiting for GO TO H1 target location {}".format(self._target_location_h1))
            self.vehicle.simple_goto(self._target_location_h1)
            return
        if self._is_yawing:
            # TODO: send request yawing!
            pass

    def _update_doing(self):
        if self._doing_on_h2:
            sd_logger.debug("Waiting for doing detection on H2")
            self._do_detection()
            #TODO future: for simplicity, do command is return done immediately. Update if needed later.
            if not self._is_detected:
                self._lost_target = True
            else:
                self._doing_on_h2 = False
                self._move_on_h1 = True
                # TODO: update _target_location_h1 from detection result
                # self._target_location_h1 = self.detected_target
            return

        if self._doing_on_h1:
            sd_logger.debug("Waiting for doing detection on H1")
            self._do_detection()
            #TODO future: for simplicity, do command is return done immediately. Update if needed later.
            if not self._is_detected:
                self._lost_target = True
            else:
                self._doing_on_h1 = False
                self._is_yawing = True
                # TODO: update _target_yaw based on detection result.
                # self._target_yaw = self.detected_yaw
            return

        if self._precheck_land:
            # TODO: check if target acquired => set _target_acquired, else set _lost_target. End sequence of steps.
            self.complete_code = random.choice([1,1,1,1,1,1,1,3])

    def _verify_complete_code(self):
        if self.vehicle.mode != VehicleMode('GUIDED'):
                self.complete_code = 2
                return
        elif self._lost_target:
            self.complete_code = 3
            return
        elif self._target_acquired:
            self.complete_code = 1
            return
    
    def _do_detection(self):
        self._is_detected = random.choice([0,1,1,1])
        if not self._is_detected: # retry
            self._is_detected = random.choice([0,1,1,1])
        # TODO: update detected_target based on detection result.
        # TODO: update detected_yaw based on detection result.
        wait_1s()


class PL_LandingPadLand(ModeState):
    # complete code: 0 => not completed, 1 => next: IRBeaconSearch, 2 => back: ManualControl = Landed
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadLand'
        self._set_vehicle_land_mode(wait_ready=True, wait_time=2)

    def _compute_mission(self):
        """ Correct yaw after approaching landing pad
        """
        sd_logger.info("Control yaw while let pixhark control land")
        # TODO: while h > H config to slow down => let land
        # TODO: correct yaw at H slow down? (necessary?)
        do_nothing()
        wait_1s()

    def _update_navigation(self):
        do_nothing()

    def _update_doing(self):
        do_nothing()

    def _verify_complete_code(self):
        if self.vehicle.mode != VehicleMode('LAND'):
                self.complete_code = 2
                return
        else:
            if not self.vehicle.armed:
                self.complete_code = 2
                return
            # TODO: if disarm (landed successful) => complete_code = 2 + notification. Testable
            # TODO: if target acquired lost for > 2000ms => code = 1, IRBeaconSearch. Testable.
            self.complete_code = random.choice([0,0,0,0,0,0,0,1])
            return

    def _set_vehicle_land_mode(self, wait_ready=False, wait_time=2):
        """To make sure set mode have enough time to be done, before verify_complete_code run"""
        if wait_ready:
            self.vehicle.mode = VehicleMode('LAND')
            time.sleep(0.2) # wait time for set mode done
            start = time.time()
            while self.vehicle.mode != VehicleMode('LAND') or time.time()-start<wait_time:
                self.vehicle.mode = VehicleMode('LAND')
                time.sleep(0.2) # wait time for set mode done
        else:
            self.vehicle.mode = VehicleMode('LAND')

class PL_IRBeaconSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'IRBeaconSearch'
        self._set_vehicle_guided_mode(wait_ready=True, wait_time=2)

    def _compute_mission(self):
        """ If above a configured level, switch to LandingPadSearch, if not fly up to the level and check.
        If target acquired, switching back LandingPadLand, else LandingPadSearch.
        """
        # TODO: fly up to h1 in move on top when rotate yaw, if target acquired, yaw then to land, else search
        sd_logger.debug("Executing IR beacon search")
        wait_1s()
    def _update_navigation(self):
        wait_1s()
    def _update_doing(self):
        wait_1s()

    def _verify_complete_code(self):
        if self.vehicle.mode != VehicleMode('GUIDED'):
                self.complete_code = 2
                return
        else:
            self.complete_code = random.choice([0,1,3])
            return

    def _set_vehicle_guided_mode(self, wait_ready=False, wait_time=2):
        """To make sure set mode have enough time to be done, before verify_complete_code run"""
        if wait_ready:
            self.vehicle.mode = VehicleMode('GUIDED')
            time.sleep(0.2) # wait time for set mode done
            start = time.time()
            while self.vehicle.mode != VehicleMode('GUIDED') or time.time()-start<wait_time:
                self.vehicle.mode = VehicleMode('GUIDED')
                time.sleep(0.2) # wait time for set mode done
        else:
            self.vehicle.mode = VehicleMode('GUIDED')




    