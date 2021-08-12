# SLAM on Raspberry Pi

_Simultaneous Localization and Mapping (SLAM) using RP LIDAR A1 on Raspberry Pi with remote visualization using MQTT_

<< blog link>>
<< slam gif>>
<< mqtt gif>>
  

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

To execute SLAM on Raspberry Pi,
```
python3 rpslam-thread.py
```

The map will be visualized in RPi itself. To enable visualization in a remote system, you can execute the MQTT version,
```
python3 rpslam-mqtt.py
```

Note: To enable save of the visualization map, please replace the **__init__.py** file in roboviz directory with the one in this repo and execute PyRoboViz setup script again from the PyRoboViz base directory, using the command below.
```
sudo python3 setup.py install
```

Before execution, create a directory named 'gif' inside the base directory of slam script to let the map images saved. These images are saved in time sequence so that a gif animation can be easily created.




If the MQTT visualization is slow, then the bytearray transfer might be the bottleneck. You can either connect the RPi to router using a LAN cable (to improve the speed) or reduce the dimensions of the map to reduce the size of the bytearray. Instead you can reduce the MQTT publish frequency as well.

# References
1. https://github.com/simondlevy/BreezySLAM
2. https://github.com/simondlevy/PyRoboViz
3. https://www.udacity.com/course/computer-vision-nanodegree--nd891
4. https://www.thinkautonomous.ai/blog/?p=lidar-and-camera-sensor-fusion-in-self-driving-cars


