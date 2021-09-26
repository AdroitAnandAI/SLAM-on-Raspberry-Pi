# SLAM on Raspberry Pi

_Simultaneous Localization and Mapping (SLAM) Navigation using RP LIDAR A1 on Raspberry Pi with remote point cloud visualization using MQTT_


<< blog link>>

SLAM (Simultaneous Localization And Mapping) algorithms use LiDAR and IMU data to simultaneously locate the robot in real-time and generate a coherent map of surrounding landmarks such as buildings, trees, rocks, and other world features, at the same time.

Though a classic chicken and egg problem, it has been approximately solved using methods like Particle Filter, Extended Kalman Filter (EKF), Covariance intersection, and GraphSLAM. SLAM enables accurate mapping where GPS localization is unavailable, such as indoor spaces.

Reasonably so, SLAM is the core algorithm being used in robot navigation, autonomous cars, robotic mapping, virtual reality, augmented reality, etc. If we can do robot localization on RPi then it is easy to make a moving car or walking robot that can ply indoors autonomously.

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




Let's discuss GraphSLAM and do a custom implementation. Then we will attempt to integrate RPi with RPLidar A1 M8, running on a 5V 3A battery, and do SLAM along with visualization of the LIDAR point cloud map to assist navigation or even to generate a floor map. Finally, the LIDAR point cloud map is visualized on a remote machine using MQTT.

Graph SLAM
Assume a robot in the 2D world, tries to move 10 units to the right from x to x'. Due to motion uncertainty, x' = x + 10 may not hold, but it will be a Gaussian centered around x + 10. The Gaussian should peak when x' approaches x + 10
Robot movement from x0 to x1 to x2 is characterized by 2 Gaussian functionsIf x1 is away from x0 by 10 units, the Kalman Filter models the uncertainty using the Gaussian with (x1 – x0 –10). Hence, there is still a probability associated with locations < 10 and > 10.
There is another similar Gaussian at x2 with a higher spread. The total probability of the entire route is the product of the two Gaussians. We can drop the constants, as we just need to maximize the likelihood of the position x1, given x0. Thus the product of Gaussian becomes the sum of exponent terms, i.e. the constraint has only x's and sigma.
Graph SLAM models the constraints as System of Linear Equations (SLEs), with a Ω matrix containing the coefficients of the variables and a ξ vector that contains the limiting value of the constraints. Every time an observation is made between 2 poses, a 'local addition' is done on the 4 matrix elements (as the product of gaussian becomes the sum of exponents).
Let's say, the robot moves from x0 to x1 to x2 which are 5 and -4 units apart.
Omega Matrix and Xi vector after 2 movementsThe coefficient of x's and RHS values are added to corresponding cells. Consider the landmark L0 is at a distance of 9 units from x1.





## References
1. https://github.com/simondlevy/BreezySLAM
2. https://github.com/simondlevy/PyRoboViz
3. https://www.udacity.com/course/computer-vision-nanodegree--nd891
4. https://www.thinkautonomous.ai/blog/?p=lidar-and-camera-sensor-fusion-in-self-driving-cars


