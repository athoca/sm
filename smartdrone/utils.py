import logging
sd_logger = logging.getLogger('smartdrone')
log_formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
console_handler = logging.StreamHandler()
console_handler.setFormatter(log_formatter)
sd_logger.addHandler(console_handler)
sd_logger.setLevel(logging.DEBUG)

import time

def do_nothing():
    pass

def wait_5s():
    time.sleep(5)
