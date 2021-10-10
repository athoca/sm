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

USE_FAKE_DETECTIONS = False
USE_FAKE_TACQ = False

DELTA_YAW_GIMBAL = 3 # 6 => 3 # CCW # Difference between heading of drone and head of gimbal camera
TIME_STABLE_AFTER_NAVIGATION = 5
TIME_STABLE_AFTER_YAW = 5
TIME_STABLE_AFTER_GIMBAL = 3
