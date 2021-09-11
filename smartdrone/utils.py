import math
import logging
sd_logger = logging.getLogger('sm') # smartdrone
# log_formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
log_formatter = logging.Formatter("%(asctime)s [%(levelname)-5.5s]  %(message)s")
file_handler = logging.FileHandler('smartdrone.log')
file_handler.setFormatter(log_formatter)
sd_logger.addHandler(file_handler)
sd_logger.setLevel(logging.DEBUG)
import time

from dronekit import LocationGlobal, LocationGlobalRelative


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
    # convert any degree (even <0 or >360) to range 180 - 360=0 - 180
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
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = (location2.lat - location1.lat)*math.pi/180 # in rad
    dLon = (location2.lon - location1.lon)*math.pi/180 # in rad

    dNorth = dLat * earth_radius
    dEast = dLon*earth_radius*math.cos(math.pi*location1.lat/180)
    if type(location1) is LocationGlobalRelative and type(location2) is LocationGlobal:
        H = location1.alt
    elif type(location1) is LocationGlobalRelative and type(location2) is LocationGlobalRelative:
        H = location1.alt - location2.alt
    else:
        raise Exception("Invalid Location object passed")
    return (dNorth, dEast, H)