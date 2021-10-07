* Start dronekit simulation:
conda activate smart_drone 
python start_sitl.py
* Forward port:
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551

* OR start ardupilot simulation:
conda activate smart_drone 
python ardupilot_silt/Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild
*** Start with -w - it will wipe your parameters (which are stored in eeprom.bin) => can access through wifi
python ardupilot_silt/Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild -w

* Forward port:
mavproxy.py --master tcp:127.0.0.1:5762 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
For tcp forwarding:
mavproxy.py --master tcp:127.0.0.1:5762 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:5763

* Connecting to additional Ground Station: turn on Qgroundcontrol
* Run main script:
python main.py --connect udpin:127.0.0.1:14551


* temporary:
python main2.py --connect /dev/cu.usbmodem1433201
python main2.py --connect udpin:127.0.0.1:14551
python main2.py --connect udpin:127.0.0.1:14550
python main2.py --connect tcp:127.0.0.1:5762

python3 main2.py --connect /dev/ttyACM0
python3 main2.py --connect tcp:192.168.0.102:5763