"""

This file as been modified etensively to implement SLAM in RPLidar

Concepts taken from rpslam.py : BreezySLAM Python with SLAMTECH RP A1 Lidar
https://github.com/simondlevy/BreezySLAM

Consume LIDAR measurement file and create an image for display.

Adafruit invests time and resources providing this open source code.
Please support Adafruit and open source hardware by purchasing
products from Adafruit!

Written by Dave Astels for Adafruit Industries
Copyright (c) 2019 Adafruit Industries
Licensed under the MIT license.

All text above must be included in any redistribution.
"""

import os
from math import cos, sin, pi, floor
# import pygame
from adafruit_rplidar import RPLidar, RPLidarException
import numpy as np
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel


# Screen width & height
W = 640
H = 480


MAP_SIZE_PIXELS         = 200
MAP_SIZE_METERS         = 10
MIN_SAMPLES   = 150

SCAN_BYTE = b'\x20'
SCAN_TYPE = 129

slamData = []

# This is the Publisher
client = mqtt.Client()
client.connect("test.mosquitto.org", 1883, 600)

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

# Create an RMHC SLAM object with a laser model and optional robot model
slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

# Initialize an empty trajectory
trajectory = []

# Initialize empty map
mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

# used to scale data to fit on the screen
max_distance = 0

#pylint: disable=redefined-outer-name,global-statement
def process_data(data):
    global max_distance
    lcd.fill((0,0,0))
    point = ( int(W / 2) , int(H / 2) )
    
    pygame.draw.circle(lcd,pygame.Color(255, 255, 255),point,10 )
    pygame.draw.circle(lcd,pygame.Color(100, 100, 100),point,100 , 1 )
    pygame.draw.line( lcd,pygame.Color(100, 100, 100) , ( 0, int(H/2)),( W , int(H/2) ) )
    pygame.draw.line( lcd,pygame.Color(100, 100, 100) , ( int(W/2),0),( int(W/2) , H ) )

    for angle in range(360):
        distance = data[angle]
        if distance > 0:                  # ignore initially ungathered data points
            max_distance = max([min([5000, distance]), max_distance])
            radians = angle * pi / 180.0
            x = distance * cos(radians)
            y = distance * sin(radians)
            point = ( int(W / 2) + int(x / max_distance * (W/2)), int(H/2) + int(y / max_distance * (H/2) ))
            pygame.draw.circle(lcd,pygame.Color(255, 0, 0),point,2 )
    pygame.display.update()


scan_data = [0]*360

def _process_scan(raw):
    '''Processes input raw data and returns measurment data'''
    new_scan = bool(raw[0] & 0b1)
    inversed_new_scan = bool((raw[0] >> 1) & 0b1)
    quality = raw[0] >> 2
    if new_scan == inversed_new_scan:
        raise RPLidarException('New scan flags mismatch')
    check_bit = raw[1] & 0b1
    if check_bit != 1:
        raise RPLidarException('Check bit not equal to 1')
    angle = ((raw[1] >> 1) + (raw[2] << 7)) / 64.
    distance = (raw[3] + (raw[4] << 8)) / 4.
    return new_scan, quality, angle, distance

def lidar_measurments(self, max_buf_meas=500):
       
        lidar.set_pwm(800)
        status, error_code = self.health
        
        cmd = SCAN_BYTE
        self._send_cmd(cmd)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != 5:
            raise RPLidarException('Wrong info reply length')
        if is_single:
            raise RPLidarException('Not a multiple response mode')
        if dtype != SCAN_TYPE:
            raise RPLidarException('Wrong response data type')
        while True:
            raw = self._read_response(dsize)
            self.log_bytes('debug', 'Received scan response: ', raw)
            if max_buf_meas:
                data_in_buf = self._serial_port.in_waiting
                if data_in_buf > max_buf_meas*dsize:
                    self.log('warning',
                             'Too many measurments in the input buffer: %d/%d. '
                             'Clearing buffer...' %
                             (data_in_buf//dsize, max_buf_meas))
                    self._serial_port.read(data_in_buf//dsize*dsize)
            yield _process_scan(raw)


def lidar_scans(self, max_buf_meas=800, min_len=100):
        
        scan = []
        iterator = lidar_measurments(lidar,max_buf_meas)
        for new_scan, quality, angle, distance in iterator:
            if new_scan:
                if len(scan) > min_len:
                    yield scan
                scan = []
            if quality > 0 and distance > 0:
                scan.append((quality, angle, distance))

try:

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles = None
    scan_count = 0

    for scan in lidar_scans(lidar):

        scan_count += 1

        # Extract (quality, angle, distance) triples from current scan
        items = [item for item in scan]

        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles = [item[1] for item in items]

        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            slam.update(distances, scan_angles_degrees=angles)
            previous_distances = distances.copy()
            previous_angles    = angles.copy()

        # If not adequate, use previous
        elif previous_distances is not None:
            slam.update(previous_distances, scan_angles_degrees=previous_angles)

        # Get current robot position
        x, y, theta = slam.getpos()

        # print("x = " + str(x) + " y = " + str(y) + "theta = " + str(theta))

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)

        data2Transmit = np.array([x, y, theta])

        if scan_count % 30 == 0:
            client.publish("safetycam/topic/slamviz", data2Transmit.tobytes() + mapbytes)



except KeyboardInterrupt:
    #print('Stopping the LIDAR SLAM')
    raise

finally:
    lidar.stop()
    lidar.disconnect()
