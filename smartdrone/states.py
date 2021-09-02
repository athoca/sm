from smartdrone.core import ModeState
from smartdrone.utils import sd_logger
import random
import time
from dronekit import VehicleMode

class PL_ManualControl(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'ManualControl'
    
    def _execute(self):
        """ Smart drone do nothing, waiting for switching state by change ardupilot mode from LOITER to GUIDED
        """
        sd_logger.info("Smart drone do nothing, waiting for switching state by change ardupilot mode from LOITER to GUIDED...")
        # sd_logger.debug(self.vehicle.last_mode)

    def _update_navigation(self):
        pass
    def _update_doing(self):
        pass
    def _verify_complete_code(self):
        if self.vehicle.mode == VehicleMode('GUIDED') and self.vehicle.last_mode == VehicleMode('LOITER'):
                self.complete_code = 1

class PL_LandingPadSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadGo, 2 => back: ManualControl (if mode != GUIDED)
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadSearch'

    def _execute(self):
        """
        """
        sd_logger.debug("Executing landing pad search")

    def _verify_complete_code(self):
        if self.vehicle.mode != VehicleMode('GUIDED'):
                self.complete_code = 2
        else:
            self.complete_code = random.choice([0,1])

class PL_LandingPadGo(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadGo'

    def _execute(self):
        """
        """
        sd_logger.debug("Executing landing pad go")
    
    def _verify_complete_code(self):
        if self.vehicle.mode != VehicleMode('GUIDED'):
                self.complete_code = 2
        else:
            self.complete_code = random.choice([0,1,3])

class PL_LandingPadLand(ModeState):
    # complete code: 0 => not completed, 1 => next: IRBeaconSearch, 2 => back: ManualControl = Landed
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadLand'
        self._set_vehicle_land_mode(wait_ready=True, wait_time=2)

    def _execute(self):
        """
        """
        sd_logger.debug("Executing landing pad land")
    
    def _verify_complete_code(self):
        if self.vehicle.mode != VehicleMode('LAND'):
                self.complete_code = 2
        else:
            # TODO: if disarm (landed successful) => complete_code = 2
            self.complete_code = random.choice([0,1])

    def _set_vehicle_land_mode(self, wait_ready=False, wait_time=2):
        """To make sure set mode have enough time to be done, before verify_complete_code run"""
        if wait_ready:
            self.vehicle.mode = VehicleMode('LAND')
            time.sleep(0.2) # wait time for set mode done
            start = time.time()
            while self.vehicle.mode != VehicleMode('LAND') or time.time()-start<wait_time:
                self.vehicle.mode = VehicleMode('LAND')
                time.sleep(0.2) # wait time for set mode done


class PL_IRBeaconSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'IRBeaconSearch'
        self._set_vehicle_guided_mode(wait_ready=True, wait_time=2)

    def _execute(self):
        """
        """
        sd_logger.debug("Executing IR beacon search")

    def _verify_complete_code(self):
        if self.vehicle.mode != VehicleMode('GUIDED'):
                self.complete_code = 2
        else:
            self.complete_code = random.choice([0,1,3])

    def _set_vehicle_guided_mode(self, wait_ready=False, wait_time=2):
        """To make sure set mode have enough time to be done, before verify_complete_code run"""
        if wait_ready:
            self.vehicle.mode = VehicleMode('GUIDED')
            time.sleep(0.2) # wait time for set mode done
            start = time.time()
            while self.vehicle.mode != VehicleMode('GUIDED') or time.time()-start<wait_time:
                self.vehicle.mode = VehicleMode('GUIDED')
                time.sleep(0.2) # wait time for set mode done




    