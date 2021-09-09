from smartdrone.utils import sd_logger
from smartdrone.vehicles import connect_to_PLdrone
import time
import math


def rad2degree(rad):
    # convert range -pi to pi to range 180 - 360=0 - 180
    if rad < 0:
        rad = 2*math.pi + rad
    return rad/math.pi*180

def degree2degree(degree):
    # convert any degree (even <0 or >360) to range 180 - 360=0 - 180
    return degree % 360 



# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-c','--connect', help="Vehicle connection target string.", required=True)
args = parser.parse_args()
connection_string = args.connect
sd_logger.info("Connection string: {}".format(connection_string))

smartdrone = connect_to_PLdrone(connection_string)
smartdrone._rngfnd_distance = 10
print("AAAAAAAAA")
# for i in range(1):
#     smartdrone.condition_yaw(-40, relative=True)
#     time.sleep(2)
while True:
    sd_logger.info(smartdrone.plnd)
    currentLocation = smartdrone.location.global_relative_frame
    sd_logger.info("ALT : {}".format(currentLocation.alt))
    # sd_logger.info(smartdrone.attitude)
    sd_logger.info("PLND_ANGLE_MAX : {}".format(smartdrone.parameters['PLND_ANGLE_MAX']))
    sd_logger.info("WPNAV_ANGLE_MAX : {}".format(smartdrone.parameters['WPNAV_ANGLE_MAX']))
    sd_logger.info("ANGLE_MAX : {}".format(smartdrone.parameters['ANGLE_MAX']))
    sd_logger.info("PLND_SMART_MODE : {}".format(smartdrone.parameters['PLND_SMART_MODE']))
    
    # homeLocation = smartdrone.home_location
    # sd_logger.info(currentLocation)
    # sd_logger.info(homeLocation)
    
    # deg = 0
    # deg = degree2degree(deg)
    # smartdrone.condition_yaw(degree2degree(deg))
    # sd_logger.info(smartdrone.heading)
    # sd_logger.info(rad2degree(smartdrone.attitude.yaw))
    # sd_logger.info(smartdrone.attitude.yaw)
    

    # #set the channel overrides which avoids crashing when mode changed to loiter
    # print("setting throttle channel to 1500 via channel overrides")
    # sd_logger.info(smartdrone.channels)
    # sd_logger.info(smartdrone.channels.overrides)
    # smartdrone.channels.overrides['3'] = 1500
    # sd_logger.info(smartdrone.channels)
    time.sleep(3)