from smartdrone.core import ModeState
from smartdrone.utils import sd_logger, wait_1s, do_nothing
import random
import time
from dronekit import VehicleMode

class PL_ManualControl(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'ManualControl'
    
    def _compute_mission(self):
        """ Smart engine do nothing, waiting for switching state by change ardupilot mode from LOITER to GUIDED
        """
        sd_logger.info("Wait for switching state by change ardupilot mode from LOITER to GUIDED...")
        wait_1s() # time to received mavlink mode update

    def _update_navigation(self):
        do_nothing()

    def _update_doing(self):
        do_nothing()

    def _verify_complete_code(self):
        if self.vehicle.mode == VehicleMode('GUIDED') and self.vehicle.last_mode == VehicleMode('LOITER'):
                self.complete_code = 1
                return

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

    def _update_navigation(self):
        do_nothing()

    def _update_doing(self):
        do_nothing()

    def _verify_complete_code(self):
        if self.vehicle.mode != VehicleMode('LAND'):
                self.complete_code = 2
                return
        else:
            # TODO: if disarm (landed successful) => complete_code = 2 + notification. Testable
            # TODO: if target acquired lost for > 2000ms => code = 1, IRBeaconSearch. Testable.
            self.complete_code = random.choice([0,1])
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


class PL_LandingPadSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadGo, 2 => back: ManualControl (if mode != GUIDED)
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadSearch'

    def _compute_mission(self):
        """Repeat wandering in a square area then detecting landing pad until detected.
        """
        # TODO: if disarm => check armable and takeoff to H_search. Then start. If not armable => log then back Manual Mode
        # TODO: if armed, if distance != H_search => go up to H_search then start.
        # Get next position for wandering, goto, take 2 photo and detect, continue

        sd_logger.debug("Executing landing pad search")
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
            self.complete_code = random.choice([0,1])
            return

class PL_LandingPadGo(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadGo'

    def _compute_mission(self):
        """ Follow three steps: Move, Approach, then Yaw
        """
        # TODO: move to h2 on top, then detect again if not => search again
        # TODO: approach to h1, do detection and combine target acquired
        # TODO: yaw and check target acquired
        sd_logger.debug("Executing landing pad go")
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




    