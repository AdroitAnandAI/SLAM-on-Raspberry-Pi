# SLAM on Raspberry Pi

_Simultaneous Localization and Mapping (SLAM) with RP LIDAR A1 on Raspberry Pi with remote visualization using MQTT_



## How to use?

```
git clone https://github.com/simondlevy/BreezySLAM.git
cd to BreezySLAM/examples
sudo python3 setup.py install
```

```
git clone https://github.com/simondlevy/PyRoboViz.git
change to PyRoboViz base directory
sudo python3 setup.py install
```

To enable save of map visualization, replace the **__init__.py** file in roboviz directory and execute PyRoboViz setup script again using the above command.

To execute SLAM on Raspberry Pi,
```
python3 rpslam-thread.py
```

The map will be visualized in RPi itself. To enable visualization in a remote system, you can execute the MQTT version,
```
python3 rpslam-mqtt.py
```

If the MQTT visualization is slow, then the bytearray transfer might be the bottleneck. You can either connect the RPi to router using a LAN cable (to improve the speed) or reduce the dimensions of the map to reduce the size of the bytearray. Instead you can reduce the MQTT publish frequency as well.





