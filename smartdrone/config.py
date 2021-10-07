class AutoMission_Config:
    TAKEOFF_ALTITUDE = 15 # in meter
    MISSION_ALTITUDE = 20 # in meter
    NORTH = 0 # in meter
    EAST = 15 # in meter
    ERROR_THRESHOLD = 1.2 # in meter, the limit drone must be able acheive, if not it will stand without moving.
    
class LandingPadSearch_Config:
    ALTITUDE = 15 # in meter
    ERROR_THRESHOLD = 1.2 # in meter, the limit drone must be able acheive, if not it will stand without moving.
    WANDERING_SQUARE_SIZE = 8
    WANDERING_NB_SQUARES = 1

class LandingPadGo_Config:
    H2 = 12 # in meter
    H1 = 6
    ERROR_THRESHOLD = 1.2 # in meter, the limit drone must be able acheive, if not it will stand without moving.
    ANGLE_ERROR_THRESHOLD = 4 # in degree

    PITCH_ANGLE_MAX = 10
    ROLL_ANGLE_MAX = 10

class LandingPadLand_Config:
    H_YAW = 3 # in meter, quan tinh => se xuong thap hon.
    ANGLE_ERROR_THRESHOLD = 4 # in degree


