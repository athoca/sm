* Start drone simulation:
conda activate smart_drone 
python start_sitl.py
* Forward port:
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
* Connecting to additional Ground Station: turn on Qgroundcontrol
* Run main script:
python main.py --connect udpin:127.0.0.1:14551
