from logging import error
from os import name
from re import search
from smartdrone.core import ModeState
from smartdrone.utils import sd_logger, wait_0_5s, wait_1s, wait_5s, do_nothing, \
                        get_distance_metres, get_location_metres, get_location_difference_metres, rad2degree
from smartdrone.config import LandingPadSearch_Config, LandingPadGo_Config, LandingPadLand_Config
import random
import time
from dronekit import VehicleMode
from dronekit import LocationGlobal, LocationGlobalRelative
class PL_ManualControl(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'ManualControl'
        self._last_override_time = time.time()

    def _compute_mission(self):
        """ Smart engine do nothing, waiting for switching state by change ardupilot mode from LOITER to GUIDED
        """
        self._logger("Wait mode change from LOITER to GUIDED...")
        # TODO: test setup gimbal at the begin. Overwrite channel 6 and 8?
        # self._logger("Rotate GIMBAL to default.")

        #TODO: check when should set the channel overrides which avoids crashing when mode changed to loiter
        self._logger("Set throttle to 1500 via channel overrides. Last time: {}".format(time.time() - self._last_override_time))
        self.vehicle.channels.overrides['3'] = 1500
        self._last_override_time = time.time()
        wait_1s() # time to received mavlink mode update, it's safe to be slow here.
        time.sleep(0.5) # temporary add to reduce frequence of overriding.
        if self.mode._original_heading is None:
            if self.vehicle.heading is not None:
                self.mode._original_heading = self.vehicle.heading

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
        self.nb_squares = LandingPadSearch_Config.WANDERING_NB_SQUARES
        self.square_size = LandingPadSearch_Config.WANDERING_SQUARE_SIZE

        # set first target location
        self.target_location = {"idx":[0,0],
            "loc":LocationGlobalRelative(self._original_location.lat, self._original_location.lon, self.target_altitude)}
        self.from_ground = self._is_from_ground()
        # repeated 2 steps in the state:
        self._navigation = True
        self._doing = False
        self._stop_yaw_when_goto = False # TODO: temporary support for goto without yawing

    def _is_from_ground(self):
        # return (not self.vehicle.armed) or (self._original_location.alt < 0.7)
        return (not self.vehicle.armed) or (self.vehicle.get_height() < 0.2)

    def _update_target_location(self, square_size=8, nb_squares=1):
        # TODO future: increase square_size after a long time.
        square_size = self.square_size
        nb_squares = self.nb_squares
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
            self._logger("Distance to target location: {}".format(dist))
            if dist < self.error_threshold:
                self._navigation = False
                self._doing = True
                self.from_ground = False
                self._update_target_location()
            return
            
    def _update_navigation(self):
        if self._navigation:
            if not self.from_ground:
                if not self._stop_yaw_when_goto:
                    self.vehicle.condition_yaw(self.vehicle.heading)
                    self._stop_yaw_when_goto = True
                self._logger("GOING TO target location {}".format(self.target_location['idx']))
                self.vehicle.simple_goto(self.target_location['loc'])    
            else:
                if not self.vehicle.is_armable:
                    self._logger(" Waiting for vehicle armable to initialise...")
                    return
                else:
                    if not self.vehicle.armed:
                        self._logger("Waiting for arming motors")
                        self.vehicle.armed = True
                        return
                self._logger("TAKING OFF to target altitude {} m".format(self.target_altitude))
                # self._logger(" Altitude: {}".format(self.vehicle.location.global_relative_frame.alt))
                self._logger("Altitude: {}".format(self.vehicle.get_height()))
                self.vehicle.simple_takeoff(self.target_altitude) # Take off to target altitude

    def _update_doing(self):
        if self._doing:
            self._logger("DOING DETECTION")
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
        wait_1s()
        wait_1s()
        # TODO: implement logic + set target position from detection (for next state)
        # logging current position - home_location, convert to meters (N, E, H), attitude
        current_location = self.vehicle.location.global_relative_frame
        home_location = self.vehicle.home_location # not Relative but absolut location.
        sd_logger.info(self.vehicle.attitude)
        NED = get_location_difference_metres(current_location, home_location)
        self._logger("To target NED: {}".format(NED))
        self._logger("Height from rangefinder: {}".format(self.vehicle.get_height()))
        self.vehicle.channels.overrides['6'] = 1340 # 1642
        wait_1s()
        wait_1s()
        self._logger("Rotated gimbal by overwriting channel 6 = 1340")
        wait_1s()
        self.vehicle.channels.overrides['6'] = 1642
        self._logger("Rotated gimbal by overwriting channel 6 = 1642")
        self._is_detected = random.choice([0,0,1])
        self.detected_target = LocationGlobalRelative(home_location.lat, home_location.lon, 0)
        


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
        self._is_yawing = False # first time yaw
        self._precheck_land = False

    def _compute_mission(self):
        """ Follow three steps: Move, Approach, then Yaw
        """
        # Move to h2 on top, then detect again, if not => search again
        if self._move_on_h2:
            current_location = self.vehicle.location.global_relative_frame
            dist = get_distance_metres(current_location, self._target_location_h2)
            self._logger("Distance to the H2 TARGET LOCATION: {}".format(dist))
            if dist < self.error_threshold:
                self._move_on_h2 = False
                self._doing_on_h2 = True
            return
        # Approach to h1, then detect again, if not => search again
        if self._move_on_h1:
            current_location = self.vehicle.location.global_relative_frame
            dist = get_distance_metres(current_location, self._target_location_h1)
            self._logger("Distance to the H1 TARGET LOCATION: {}".format(dist))
            if dist < self.error_threshold:
                self._move_on_h1 = False
                self._doing_on_h1 = True
            return
        # Yawing and check target acquired, if not => search again
        if self._is_yawing:
            current_yaw = rad2degree(self.vehicle.attitude.yaw)
            dist = abs(current_yaw - self._target_yaw)
            self._logger("Distance to CORRECT YAW: {}".format(dist))
            if dist < 5: # in degree
                # TODO: in case of 0 degree, the current yaw oscilate 0 and 360.
                # TODO: check self._target_yaw - current yaw < error, if ok => precheck_land()
                self._is_yawing = False
                self._precheck_land = True

    def _update_navigation(self):
        if self._move_on_h2:
            self._logger("GOING TO H2 target location {}".format(self._target_location_h2))
            self.vehicle.simple_goto(self._target_location_h2)
            return
        if self._move_on_h1:
            self._logger("GOING TO H1 target location {}".format(self._target_location_h1))
            self.vehicle.simple_goto(self._target_location_h1)
            return
        if self._is_yawing:
            self._logger("YAWING to absolute target {}".format(self._target_yaw))
            # TODO: send request yawing!
            self.vehicle.condition_yaw(self._target_yaw)
            

    def _update_doing(self):
        if self._doing_on_h2:
            self._logger("DOING DETECTION ON H2")
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
            self._logger("DOING DETECTION ON H1")
            self._do_detection()
            #TODO future: for simplicity, do command is return done immediately. Update if needed later.
            if not self._is_detected:
                self._lost_target = True
            else:
                self._doing_on_h1 = False
                self._is_yawing = True
                # TODO: update _target_yaw based on detection result.
                # self._target_yaw = self.detected_yaw
                self._target_yaw = 120
            return

        if self._precheck_land:
            self._logger("CHECK TARGET ACQUIRED ON H1")
            wait_0_5s()
            # TODO: check if target acquired => set _target_acquired, else set _lost_target. End sequence of steps.
            # self.complete_code = random.choice([1,1,1,1,1,1,1,3]) # for simulation test
            self._target_acquired = True


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
        wait_1s()
        wait_1s()
        # TODO: update detected_target based on detection result.
        # TODO: update detected_yaw based on detection result.
        # logging current position - home_location, convert to meters (N, E, H), attitude
        current_location = self.vehicle.location.global_relative_frame
        home_location = self.vehicle.home_location # not Relative but absolut location.
        sd_logger.info(self.vehicle.attitude)
        NED = get_location_difference_metres(current_location, home_location)
        self._logger("To target NED: {}".format(NED))
        self._logger("Height from rangefinder: {}".format(self.vehicle.get_height()))
        self._logger("Current heading: {}".format(self.vehicle.heading))
        self._logger("Original heading: {}".format(self.mode._original_heading))

        self._is_detected = random.choice([0,1,1,1])
        if not self._is_detected: # retry
            self._is_detected = random.choice([0,1,1,1])
        
        


class PL_LandingPadLand(ModeState):
    # complete code: 0 => not completed, 1 => next: IRBeaconSearch, 2 => back: ManualControl = Landed
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadLand'
        self.H = LandingPadLand_Config.H_YAW
        self._target_acquired = True
        # sequence of 3 steps of the state:
        self._descending = True         # descend
        self._doing_on_H = False    # detect yaw
        self._is_yawing = False     # second time yaw
        self._set_vehicle_land_mode(wait_ready=True, wait_time=2)

    def _compute_mission(self):
        """ Correct yaw after approaching landing pad
        """
        if self._descending:
            dist = self.vehicle.get_height() - self.H
            self._logger("DESCEND in LAND to {}m, Distance = {}".format(self.H, dist))
            if dist < 0.3:
                self._doing_on_H = True
                self._descending = False
                self._set_vehicle_guided_mode(wait_ready=True, wait_time=2)
            return

        if self._is_yawing:
            current_yaw = rad2degree(self.vehicle.attitude.yaw)
            dist = abs(current_yaw - self._target_yaw)
            self._logger("Distance to CORRECT YAW: {}".format(dist))
            if dist < 5: # in degree
            # TODO: check self._target_yaw - current yaw < error, if ok => check target acquired
                self._is_yawing = False
                self._logger("SWITCH TO LOITER MODE FIRST. WAIT 1s.")
                #TODO: check when should set the channel overrides which avoids crashing when mode changed to loiter
                self._logger("Set throttle to 1500 via channel overrides.")
                self.vehicle.channels.overrides['3'] = 1500
                self._set_vehicle_loiter_mode(wait_ready=True, wait_time=2)
                
                time.sleep(1)
                self._set_vehicle_land_mode(wait_ready=True, wait_time=2)
                self._logger("SWITCH BACK LAND MODE SECOND.")
            return

        self._logger("Current H = {}".format(self.vehicle.get_height()))

    def _update_navigation(self):
        if self._is_yawing:
            self._logger("YAWING to absolute target {}".format(self._target_yaw))
            # TODO: send request yawing!
            self.vehicle.condition_yaw(self._target_yaw)

    def _update_doing(self):
        if self._doing_on_H:
            self._logger("DOING DETECTION YAW")
            self._detect_yaw()
            #TODO future: for simplicity, do command is return done immediately. Update if needed later.
            self._doing_on_H = False
            self._is_yawing = True
            # TODO: update _target_yaw based on detection result.
            # self._target_yaw = self.detected_yaw
            # self._target_yaw = 300
            self._target_yaw = self.mode._original_heading
            return

    def _verify_complete_code(self):
        if self.vehicle.mode not in [VehicleMode('LAND'),VehicleMode('GUIDED')]:
                self.complete_code = 2
                return
        else:
            if not self.vehicle.armed: # succesful landing.
                self._logger("***************************FINISHED!")
                self.complete_code = 2
                return
            # TODO: if target acquired lost for > 2000ms => code = 1, 
            # Now assumpt that precision landing sucessfully, _target_acquired always True => never IRBeaconSearch
            if not self._target_acquired:
                self.complete_code = 1
                return    

    def _set_vehicle_loiter_mode(self, wait_ready=False, wait_time=2):
        """To make sure set mode have enough time to be done, before verify_complete_code run"""
        if wait_ready:
            self.vehicle.mode = VehicleMode('LOITER')
            time.sleep(0.2) # wait time for set mode done
            start = time.time()
            while self.vehicle.mode != VehicleMode('LOITER') or time.time()-start<wait_time:
                self.vehicle.mode = VehicleMode('LOITER')
                time.sleep(0.2) # wait time for set mode done
        else:
            self.vehicle.mode = VehicleMode('LOITER')

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
    
    def _detect_yaw(self):
        wait_1s()
        wait_1s()
        # TODO: update detected_yaw based on detection result.
        # logging current position - home_location, convert to meters (N, E, H), attitude. Original_Heading, Height (from Rangefinder if available)
        current_location = self.vehicle.location.global_relative_frame
        home_location = self.vehicle.home_location # not Relative but absolut location.
        sd_logger.info(self.vehicle.attitude)
        NED = get_location_difference_metres(current_location, home_location)
        self._logger("To target NED: {}".format(NED))
        self._logger("Height from rangefinder: {}".format(self.vehicle.get_height()))
        self._logger("Current heading: {}".format(self.vehicle.heading))
        self._logger("Original heading: {}".format(self.mode._original_heading))


class PL_IRBeaconSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'IRBeaconSearch'
        self.h1 = LandingPadGo_Config.H1
        self._set_vehicle_guided_mode(wait_ready=True, wait_time=2)

    def _compute_mission(self):
        """ If above a configured level, switch to LandingPadSearch, if not fly up to the level and check.
        If target acquired, switching back LandingPadLand, else LandingPadSearch.
        """
        # tam thoi hoan thien nhung co the ko can dung state nay
        # TODO: fly up to h1 in move on top when rotate yaw, if target acquired, yaw then to land, else search
        # flyup h1, check target acquired, if yes => LAND, not => Search
        self._logger("Executing IR beacon search")
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

