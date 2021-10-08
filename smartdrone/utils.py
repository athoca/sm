import math
import os
import random
from smartdrone.config import USE_FAKE_DETECTIONS
# Create ID for each test
FLYING_TEST_ID = random.randint(1,10000000)

def is_on_Xavier():
    name = os.uname()[1]
    if name == "bootai":
        return True
    else:
        return False

def get_logging_file():
    Xavier_logging_folder = "/home/bootai/workspace/sm"
    filename = "smartdrone.log"
    if os.path.exists(Xavier_logging_folder):
        return os.path.join(Xavier_logging_folder, filename)
    else:
        return filename

import logging
sd_logger = logging.getLogger('sm') # smartdrone
# log_formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
log_formatter = logging.Formatter("%(asctime)s [%(levelname)-5.5s]  %(message)s")
logging_file = get_logging_file()
file_handler = logging.FileHandler(logging_file)
file_handler.setFormatter(log_formatter)
sd_logger.addHandler(file_handler)
sd_logger.setLevel(logging.DEBUG)

import time
from dronekit import LocationGlobal, LocationGlobalRelative
import struct
import redis
import numpy as np
import math

# Redis connection
r = redis.Redis(host='localhost', port=6379, db=0)

def save_stack_to_redis(r, data, name):
    """Data is dict including timestamp e.g {"stack": stack_in_numpy_array, "timestamp": timestamp}
    """
    array = data["stack"]
    h, w, c = array.shape
    shape = struct.pack('>III',h,w,c)
    encoded = shape + array.tobytes()
    timestamp = data["timestamp"]
    encoded_dict = {"stack": encoded, "timestamp": timestamp}
    r.hmset(name, encoded_dict)
    return

def load_stack_from_redis(r, name, dtype=np.uint8):
    """Retrieve Stack and timestamp from Redis key name"""
    redis_encoded_dict = r.hgetall(name)
    if redis_encoded_dict:
        encoded = redis_encoded_dict[b"stack"]
        hwc_offset = 4*3
        h, w, c = struct.unpack('>III', encoded[:hwc_offset])
        array = np.frombuffer(encoded, dtype=dtype, offset=hwc_offset).reshape(h, w, c)
        timestamp = float(redis_encoded_dict[b"timestamp"].decode("utf=8"))
        return {"stack": array, "timestamp": timestamp}
    else:
        return {}


def do_nothing():
    pass

def wait_5s():
    time.sleep(5)

def wait_1s():
    time.sleep(1)

def wait_0_5s():
    time.sleep(0.5)

def rad2degree(rad):
    # convert range -pi to pi to range 180 - 360=0 - 180
    if rad < 0:
        rad = 2*math.pi + rad
    return rad/math.pi*180

def degree2degree(degree):
    # convert any degree (even <0 or >360) to range 180 - 360=0(North) - 180
    return degree % 360 


class PLND(object):
    """
    The PLND from mavlink message.

    :param TAcq:	True if landing target is detected
    :param LastMeasMS:	Time when target was last detected
    :param pX:	Target position relative to vehicle, X-Axis (0 if target not found)
    :param pY:	Target position relative to vehicle, Y-Axis (0 if target not found)
    :param vX:	Target velocity relative to vehicle, X-Axis (0 if target not found)
    :param vY:	Target velocity relative to vehicle, Y-Axis (0 if target not found)
    :param mX:	Target’s relative to origin position as 3-D Vector, X-Axis
    :param mY:	Target’s relative to origin position as 3-D Vector, Y-Axis
    :param mZ:	Target’s relative to origin position as 3-D Vector, Z-Axis
    """
    def __init__(self, TAcq=None, LastMeasMS=None, pX=None, pY=None, vX=None, vY=None, H=None, ts=None):
        """
        PLND object constructor.
        """
        self.pX = pX
        self.pY = pY
        self.vX = vX
        self.vY = vY
        self.TAcq = TAcq
        self.LastMeasMS = LastMeasMS
        self.ts = ts
        self.H = H
        
    def __str__(self):
        """
        String representation used to print the PLND object. 
        """
        return "PLND: TAcq={},LastMeasMS={},pX={},pY={},vX={},vY={},H={},ts={}".format(self.TAcq, self.LastMeasMS, self.pX,self.pY,self.vX,self.vY,self.H,self.ts)

class FailSafe(object):
    """
    The FailSafe from SYS_STATUS onboard_control_sensors_health from mavlink message.
    """
    def __init__(self, radio=0, ts=None):
        """
        PLND object constructor.
        """
        self.radio = radio
        self.ts = ts
        
    def __str__(self):
        """
        String representation used to print the PLND object. 
        """
        return "Failsafe: Radio={}, ts={}".format(self.radio, self.ts)



def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    dalt = aLocation2.alt - aLocation1.alt
    dlen = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    dist = math.sqrt(dalt*dalt + dlen*dlen)
    return dist

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def get_location_difference_metres(location1, location2):
    """
    Based on get_location_metres
    """
    if type(location1) is LocationGlobalRelative and type(location2) is LocationGlobal:
        H = location1.alt
    elif type(location1) is LocationGlobalRelative and type(location2) is LocationGlobalRelative:
        H = location1.alt - location2.alt
    else:
        raise Exception("Invalid Location object passed")
        
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = (location2.lat - location1.lat)*math.pi/180 # in rad
    dLon = (location2.lon - location1.lon)*math.pi/180 # in rad

    dNorth = dLat * earth_radius
    dEast = dLon*earth_radius*math.cos(math.pi*location1.lat/180)
    
    return (dNorth, dEast, H)

def detect_landingpad(H, heading, location, is_gimbal_rotated=False, home_location=None):
    """ TODO
        - get frame, logging current timestamp - frame timestamps
        - run detection on frame, get bboxes
        - if bboxes > 0, set self._is_detected = 1, build self.detected_target = from NED depend on is_gimbal_rotated
        """
    if USE_FAKE_DETECTIONS:
        if H >13:
            is_detected = random.choice([0,0,1])
        elif H > 10:
            is_detected = random.choice([0,1,1,1,1,1,1,1])
        else:
            is_detected = 1

        detected_target = None
        if is_detected:
            if home_location is not None:
                detected_target = LocationGlobalRelative(home_location.lat, home_location.lon, 0)
        return is_detected, detected_target
    else:
        # TODO: get current frame using redis client
        stack = load_stack_from_redis(r, "new_frame")
        if not stack:
            sd_logger.error("NO FRAME FOR KEY new_frame.")
            return 0, None
        else:
            sd_logger.info("GET FRAME FOR KEY new_frame for detection.")
            # TODO calculate detected_target based on frame, H and corrected using is_gimbal_rotated
            # TODO: log detected_target in m in North and East from current position
            # TODO: detect landing pad here
            is_detected = 0
            detected_target = None
            to_North = 10
            to_East = 10

            # Save frame and detection for debug
            sd_logger.info("SAVE FRAME AND DETECTION for debug")
            ts = stack["timestamp"]
            key_name = ":".join([str(FLYING_TEST_ID), str(ts)])
            detection_key_name = ":".join([key_name,"detection:target"])
            save_stack_to_redis(r, stack, key_name)
            # TODO: add more infor in the detection_dict
            detection_dict = {"is_detected": is_detected}
            r.hmset(detection_key_name, detection_dict)

            return is_detected, detected_target
            # return 1, LocationGlobalRelative(home_location.lat, home_location.lon, 0)



def detect_yaw(H, is_gimbal_rotated=False):
    """ TODO
        - get frame, logging current timestamp - frame timestamps
        - run detection on frame, get bboxes
        - if bboxes > 0, detect_yaw
        """
    if USE_FAKE_DETECTIONS:
        if H > 4:
            is_detected = random.choice([0,1,1,1,1,1,1,1])
            detected_yaw = 0
        else:
            is_detected = 1
            detected_yaw = 0
        return is_detected, detected_yaw
    else:
        # TODO: get current frame using redis client
        stack = load_stack_from_redis(r, "new_frame")
        if not stack:
            sd_logger.error("NO FRAME FOR KEY new_frame.")
            return 0, 0
        else:
            sd_logger.info("GET FRAME FOR KEY new_frame for detection.")
            # TODO calculate detected_target based on frame, H and corrected using is_gimbal_rotated
            # TODO: log detected_target in m in North and East from current position
            # TODO: detect yaw here
            is_detected = 1
            detected_yaw = 0

            # Save frame and detection for debug
            sd_logger.info("SAVE FRAME AND DETECTION for debug")
            ts = stack["timestamp"]
            key_name = ":".join([str(FLYING_TEST_ID), str(ts)])
            detection_key_name = ":".join([key_name,"detection:yaw"])
            save_stack_to_redis(r, stack, key_name)
            # TODO: add more infor in the detection_dict
            detection_dict = {"is_detected": is_detected, "detected_yaw": detected_yaw}
            r.hmset(detection_key_name, detection_dict)

            return is_detected, detected_yaw