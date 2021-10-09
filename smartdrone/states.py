from logging import error
from os import name
from re import search
from smartdrone.core import ModeState
from smartdrone.utils import sd_logger, do_nothing, \
                        get_distance_metres, get_location_metres, get_location_difference_metres, rad2degree, degree2degree
from smartdrone.utils import detect_landingpad, detect_yaw
from smartdrone.config import AutoMission_Config, LandingPadSearch_Config, LandingPadGo_Config, LandingPadLand_Config
from smartdrone.config import TIME_STABLE_AFTER_NAVIGATION, TIME_STABLE_AFTER_GIMBAL, TIME_STABLE_AFTER_YAW, DELTA_YAW_GIMBAL, USE_FAKE_TACQ
import random
import time
from dronekit import VehicleMode
from dronekit import LocationGlobal, LocationGlobalRelative

class PL_ManualControl(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadSearch, 4 => AutoMission
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
        
        # time to received mavlink mode update, it's safe to be slow here. Small than 3s so channels overrides updated.
        is_mode_changed = self.wait_and_monitor_vehicle_mode_change(1.5)
        if is_mode_changed:
            return

        # TODO: move the one-time setting in correct place
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
        if self.vehicle.mode == VehicleMode('GUIDED') and self.vehicle.last_mode == VehicleMode('LAND'):
                self.complete_code = 4
                return

class PL_AutoMission(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadSearch, 2 => ManualControl
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'AutoMission'
        self._original_location = self.vehicle.location.global_relative_frame # set current position to go back before switching LandingPadSearch
        self.takeoff_altitude = AutoMission_Config.TAKEOFF_ALTITUDE
        self.mission_altitude = AutoMission_Config.MISSION_ALTITUDE
        self.to_north = AutoMission_Config.NORTH
        self.to_east = AutoMission_Config.EAST
        self.error_threshold = AutoMission_Config.ERROR_THRESHOLD

        self.from_ground = self._is_from_ground()
        self.target_nb = 1
        self.target_location = None
        self._update_target_location()

        # repeated 2 steps in the state:
        self._navigation = True
        self._is_complete = False
        self._stop_yaw_when_goto = False # TODO: temporary support for goto without yawing

        # If not start from ground, stop navigation, switch back to ManualControl
        if not self.from_ground:
           self._navigation = False
           self.complete_code = 2

    def _is_from_ground(self):
        return (not self.vehicle.armed) or (self.vehicle.get_height() < 0.2)

    def _update_target_location(self):
        """ 3 targets, then _is_complete = 1 and _navigation = False
        """
        if self.target_nb == 1:
            self.target_location = LocationGlobalRelative(self._original_location.lat, self._original_location.lon, self.takeoff_altitude)
            self.target_nb = 2
        elif self.target_nb == 2:
            original_target_location = LocationGlobalRelative(self._original_location.lat, self._original_location.lon, self.mission_altitude)
            self.target_location = get_location_metres(original_target_location, self.to_north, self.to_east)
            self.target_nb = 3
        elif self.target_nb == 3:
            self.target_location = LocationGlobalRelative(self._original_location.lat, self._original_location.lon, self.takeoff_altitude)
            self.target_nb = 4
        elif self.target_nb == 4:
            self._is_complete = True
            self._navigation = False

    def _compute_mission(self):
        """
        """
        if self._navigation:
            current_location = self.vehicle.location.global_relative_frame
            dist = get_distance_metres(current_location, self.target_location)
            self._logger("Distance to target location {}: {}".format(self.target_nb - 1, dist))
            if dist < self.error_threshold:
                self.from_ground = False
                self._update_target_location()
            return

    def _update_navigation(self):
        if self._navigation:
            if not self.from_ground:
                if not self._stop_yaw_when_goto:
                    self.vehicle.condition_yaw(self.vehicle.heading)
                    self._stop_yaw_when_goto = True
                self._logger("GOING FOR MISSION")
                self.vehicle.simple_goto(self.target_location)    
            else:
                if not self.vehicle.is_armable:
                    self._logger(" Waiting for vehicle armable to initialise...")
                    return
                else:
                    if not self.vehicle.armed:
                        self._logger("Waiting for arming motors")
                        self.vehicle.armed = True
                        return
                self._logger("TAKING OFF to target altitude {} m".format(self.takeoff_altitude))
                self._logger("Altitude: {}".format(self.vehicle.get_height()))
                self.vehicle.simple_takeoff(self.takeoff_altitude) # Take off to target altitude

    def _update_doing(self):
        do_nothing()

    def _verify_complete_code(self):
        if self.vehicle.mode != VehicleMode('GUIDED'):
            self.complete_code = 2
            return
        if self._is_complete:
            self.complete_code = 1
            return

class PL_LandingPadSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadGo, 2 => back: ManualControl (if mode != GUIDED)
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadSearch'
        self.detected_target = None
        self._is_detected = False
        self._original_location = self.vehicle.location.global_relative_frame # set current position when switch to LandingPadSearch as original_location
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
            if self._is_detected:
                self._navigation = False
                self._doing = False
            else:
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
        # wait so the drone stable after moving
        is_mode_changed = self.wait_and_monitor_vehicle_mode_change(TIME_STABLE_AFTER_NAVIGATION)
        if is_mode_changed:
            return
        self.state_logging()
        self._logger("Detecting landing pad at 1642.")
        H = self.vehicle.get_height()
        self._logger("Height from rangefinder: {}".format(H))
        current_heading = self.vehicle.heading
        self._logger("Current heading: {}".format(current_heading))
        current_location = self.vehicle.location.global_relative_frame
        home_location = self.vehicle.home_location
        # TODO: remove home_location argument
        self._is_detected, self.detected_target = detect_landingpad(H=H, heading=current_heading, current_location=current_location, is_gimbal_rotated=False, home_location=home_location)
        self._logger("LandingPad Detection is {}".format(self._is_detected))
        if self._is_detected:
            self._logger("LandingPad target is at {}".format(self.detected_target))

        if not self._is_detected:
            self.vehicle.channels.overrides['6'] = 1340 # 1340: gimbal rotated, 1642: gimbal original
            # wait so the gimbal stable after rotate gimbal
            is_mode_changed = self.wait_and_monitor_vehicle_mode_change(TIME_STABLE_AFTER_GIMBAL)
            if is_mode_changed:
                return
            self.state_logging()
            self._logger("Rotated gimbal by overwriting channel 6 = 1340. Detecting landing pad at 1340.")
            H = self.vehicle.get_height()
            self._logger("Height from rangefinder: {}".format(H))
            current_heading = self.vehicle.heading
            self._logger("Current heading: {}".format(current_heading))
            current_location = self.vehicle.location.global_relative_frame
            # TODO: remove home_location argument
            self._is_detected, self.detected_target = detect_landingpad(H=H, heading=current_heading, current_location=current_location, is_gimbal_rotated=False, home_location=home_location)
            self._logger("LandingPad Detection is {}".format(self._is_detected))
            if self._is_detected:
                self._logger("LandingPad target is at {}".format(self.detected_target))
            self.vehicle.channels.overrides['6'] = 1642
            # wait so gimbal rotate to orignal position
            is_mode_changed = self.wait_and_monitor_vehicle_mode_change(TIME_STABLE_AFTER_GIMBAL)
            if is_mode_changed:
                return
            self._logger("Rotated gimbal by overwriting channel 6 = 1642")

class PL_LandingPadGo(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, target, *args):
        super().__init__(*args)
        self.name = 'LandingPadGo'
        self.detected_target = target
        self.detected_yaw = None
        self._is_detected = None
        self._lost_target = False
        self._target_acquired = False
        self.error_threshold = LandingPadGo_Config.ERROR_THRESHOLD
        self.angle_error_threshold = LandingPadGo_Config.ANGLE_ERROR_THRESHOLD
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
        # Move to h2 on top, then detect again, if not => search again, if yes => update self._target_location_h1 
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
            if dist < self.angle_error_threshold: # in degree
                # TODO: in case of 0 degree, the current yaw oscilate 0 and 360.
                # check self._target_yaw - current yaw < error, if ok => precheck_land()
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
            self.vehicle.condition_yaw(self._target_yaw)
            return
            

    def _update_doing(self):
        if self._doing_on_h2:
            self._logger("DOING DETECTION ON H2")
            self._do_detection_on_h2()
            #TODO future: for simplicity, do command is return done immediately. Update if needed later.
            if not self._is_detected:
                self._lost_target = True
            else:
                self._doing_on_h2 = False
                self._move_on_h1 = True
                # Update target at h1 from detection
                self._target_location_h1 = LocationGlobalRelative(self.detected_target.lat, self.detected_target.lon, self.h1)
            return

        if self._doing_on_h1:
            self._logger("DOING DETECTION ON H1")
            self._do_detection_on_h1()
            #TODO future: for simplicity, do command is return done immediately. Update if needed later.
            if not self._is_detected:
                self._lost_target = True
            else:
                self._doing_on_h1 = False
                self._is_yawing = True
                # TODO: update value for delta_yaw
                delta_yaw = self.vehicle.heading - DELTA_YAW_GIMBAL # magic value?
                self._target_yaw = degree2degree(self.detected_yaw + delta_yaw)
            return

        if self._precheck_land:
            self._logger("CHECK TARGET ACQUIRED ON H1")
            self._do_preland_check()
            if not self._target_acquired:
                self._lost_target = True


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
    
    def _do_detection_on_h2(self):
        # wait so the drone stable after moving
        is_mode_changed = self.wait_and_monitor_vehicle_mode_change(TIME_STABLE_AFTER_NAVIGATION)
        if is_mode_changed:
            return
        self.state_logging()
        H = self.vehicle.get_height()
        self._logger("Height from rangefinder: {}".format(H))
        current_heading = self.vehicle.heading
        self._logger("Current heading: {}".format(current_heading))
        self._logger("Detecting landing pad at 1642.")
        current_location = self.vehicle.location.global_relative_frame
        home_location = self.vehicle.home_location
        # TODO: remove home_location argument, update current H
        self._is_detected, self.detected_target = detect_landingpad(H=H, heading=current_heading, current_location=current_location, is_gimbal_rotated=False, home_location=home_location)
        self._logger("LandingPad Detection is {}".format(self._is_detected))
        if self._is_detected:
            self._logger("LandingPad target is at {}".format(self.detected_target))


    def _do_detection_on_h1(self):
        # wait so the drone stable after moving
        is_mode_changed = self.wait_and_monitor_vehicle_mode_change(TIME_STABLE_AFTER_NAVIGATION)
        if is_mode_changed:
            return
        self.state_logging()
        H = self.vehicle.get_height()
        self._logger("Height from rangefinder: {}".format(H))
        self._logger("Current heading: {}".format(self.vehicle.heading))
        self._logger("Detecting yaw at 1642.")
        self._is_detected, self.detected_yaw = detect_yaw(H=H, is_gimbal_rotated=False)
        self._logger("Yaw Detection is {}".format(self._is_detected))
        if self._is_detected:
            self._logger("Yaw target is {}".format(self.detected_yaw))

    
    def _do_preland_check(self):
        # wait so the drone stable after yawing
        is_mode_changed = self.wait_and_monitor_vehicle_mode_change(TIME_STABLE_AFTER_YAW)
        if is_mode_changed:
            return
        self.state_logging()
        H = self.vehicle.get_height()
        self._logger("Height from rangefinder: {}".format(H))
        self._logger("Current heading: {}".format(self.vehicle.heading))
        self._logger("Original heading: {}".format(self.mode._original_heading))
        # TODO: check PLND TAcq
        if USE_FAKE_TACQ:
            self._target_acquired = True
        else:
            self._target_acquired = self.vehicle.plnd.TAcq > 0.5      
        # self._target_acquired = self.vehicle.plnd.TAcq > 0.5
        # self._target_acquired = True


class PL_LandingPadLand(ModeState):
    # complete code: 0 => not completed, 1 => next: IRBeaconSearch, 2 => back: ManualControl = Landed
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadLand'
        self.H = LandingPadLand_Config.H_YAW
        self.angle_error_threshold = LandingPadLand_Config.ANGLE_ERROR_THRESHOLD
        self.detected_yaw = None
        self._is_detected = None
        self._target_yaw = None
        self._target_acquired = True # Now assumpt that precision landing sucessfully, _target_acquired always True => never IRBeaconSearch
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
            # TODO: in case of 0 degree, the current yaw oscilate 0 and 360.
            dist = abs(current_yaw - self._target_yaw)
            self._logger("Distance to CORRECT YAW: {}".format(dist))
            if dist < self.angle_error_threshold: # in degree
                # wait for stable after yawing, then set land again
                is_mode_changed = self.wait_and_monitor_vehicle_mode_change(TIME_STABLE_AFTER_YAW)
                if is_mode_changed:
                    return

                self._is_yawing = False
                self._set_vehicle_land_mode(wait_ready=True, wait_time=2)
                self._logger("SWITCH BACK LAND MODE SECOND.")
            return
        
        self._logger("Current H = {}".format(self.vehicle.get_height()))

    def _update_navigation(self):
        if self._is_yawing:
            self._logger("YAWING to absolute target {}".format(self._target_yaw))
            self.vehicle.condition_yaw(self._target_yaw)
            return

    def _update_doing(self):
        if self._doing_on_H:
            self._logger("DOING DETECTION YAW")
            self._do_detection_yaw()
            #TODO future: for simplicity, do command is return done immediately. Update if needed later.
            self._doing_on_H = False
            self._is_yawing = True
            if not self._is_detected:
                # If not detected, no yaw correction
                self._target_yaw = self.vehicle.heading
            else:
                # TODO: update value for delta_yaw
                delta_yaw = self.vehicle.heading - DELTA_YAW_GIMBAL # magic value?
                self._target_yaw = degree2degree(self.detected_yaw + delta_yaw)
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
    
    def _do_detection_yaw(self):
        # wait so the drone stable after moving
        is_mode_changed = self.wait_and_monitor_vehicle_mode_change(TIME_STABLE_AFTER_NAVIGATION)
        if is_mode_changed:
            return
        self.state_logging()
        H = self.vehicle.get_height()
        self._logger("Height from rangefinder: {}".format(H))
        self._logger("Current heading: {}".format(self.vehicle.heading))
        self._logger("Detecting yaw at 1642.")
        self._is_detected, self.detected_yaw = detect_yaw(H=H, is_gimbal_rotated=False)
        self._logger("Yaw Detection is {}".format(self._is_detected))
        if self._is_detected:
            self._logger("Yaw target is {}".format(self.detected_yaw))


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
        # TODO future: fly up to h1 in move on top when rotate yaw, if target acquired, yaw then to land, else search
        # flyup h1, check target acquired, if yes => LAND, not => Search
        self._logger("Executing IR beacon search")
        do_nothing()
    def _update_navigation(self):
        do_nothing()
    def _update_doing(self):
        do_nothing()

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

