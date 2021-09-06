class LandingPadSearch_Config(object):
    ALTITUDE = 15 # in meter
    ERROR_THRESHOLD = 1.2 # in meter, the limit drone must be able acheive, if not it will stand without moving.
    WANDERING_SQUARE_SIZE = 10
    WANDERING_NB_SQURE = 1

    PITCH_ANGLE_MAX = 10
    ROLL_ANGLE_MAX = 10

class LandingPadGo_Config(object):
    H2 = 12 # in meter
    H1 = 8
    ERROR_THRESHOLD = 1.2 # in meter, the limit drone must be able acheive, if not it will stand without moving.

    PITCH_ANGLE_MAX = 10
    ROLL_ANGLE_MAX = 10

