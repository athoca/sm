import os
name = os.uname()[1]
if name == "bootai":
    import sys
    sys.path.append('/home/bootai/.local/lib/python3.6/site-packages')

from smartdrone.utils import sd_logger
from smartdrone.vehicles import connect_to_PLdrone


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-c','--connect', help="Vehicle connection target string.", required=True)
args = parser.parse_args()
connection_string = args.connect
sd_logger.info("Connection string: {}".format(connection_string))

smartdrone = connect_to_PLdrone(connection_string)
smartdrone.start_main_control_loop()