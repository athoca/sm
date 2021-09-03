import time
print("Start simulator (SITL)")
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()
print(connection_string)
count = 0
while True:
    print(count)
    time.sleep(10)
    count += 1