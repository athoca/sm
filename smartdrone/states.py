from smartdrone.core import ModeState
from smartdrone.utils import sd_logger
import random

n = random.randint(0,22)

class PL_ManualControl(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'ManualControl'
    
    def execute(self):
        """
        """
        sd_logger.debug("Executing manual control")
        # TODO: add complete code for change mode
        self.complete_code = random.randint(0,1)


class PL_LandingPadSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadGo, 2 => back: ManualControl
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadSearch'

    def execute(self):
        """
        """
        sd_logger.debug("Executing landing pad search")
        # TODO: add complete code for change mode
        self.complete_code = random.randint(0,2)

class PL_LandingPadGo(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadGo'

    def execute(self):
        """
        """
        sd_logger.debug("Executing landing pad go")
        # TODO: add complete code for change mode
        self.complete_code = random.randint(0,3)

class PL_LandingPadLand(ModeState):
    # complete code: 0 => not completed, 1 => next: IRBeaconSearch, 2 => back: ManualControl = Landed
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'LandingPadLand'

    def execute(self):
        """
        """
        sd_logger.debug("Executing landing pad land")
        # TODO: add complete code for change mode
        self.complete_code = random.randint(0,2)

class PL_IRBeaconSearch(ModeState):
    # complete code: 0 => not completed, 1 => next: LandingPadLand, 2 => back: ManualControl, 3 => LandingPadSearch
    def __init__(self, *args):
        super().__init__(*args)
        self.name = 'IRBeaconSearch'

    def execute(self):
        """
        """
        sd_logger.debug("Executing IR beacon search")
        # TODO: add complete code for change mode
        self.complete_code = random.randint(0,3)




    