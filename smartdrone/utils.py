import logging
sd_logger = logging.getLogger('smartdrone')
log_formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
file_handler = logging.FileHandler('smartdrone.log')
file_handler.setFormatter(log_formatter)
sd_logger.addHandler(file_handler)
sd_logger.setLevel(logging.DEBUG)

import time
def do_nothing():
    pass

def wait_5s():
    time.sleep(5)

def wait_1s():
    time.sleep(1)

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

class RFND(object):
    """
    The RFND from mavlink message.

    :param Dist:	Reported distance from sensor
    :param ts: timestampe from pixhark? Time since system startup
    """
    def __init__(self, Dist=None, ts=None):
        """
        RFND object constructor.
        """
        self.Dist = Dist
        self.ts = ts
        
    def __str__(self):
        """
        String representation used to print the PLND object. 
        """
        return "PLND: Dist={}, ts={}".format(self.Dist, self.ts)