"""

This file as been modified extensively to implement SLAM in RPLidar.
It can run well on Raspberry Pi 3B or 4

Many concepts are taken from rpslam.py : BreezySLAM in Python with SLAMTECH RP A1 Lidar
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
import time
from math import cos, sin, pi, floor
# import pygame
from adafruit_rplidar import RPLidar, RPLidarException
import numpy as np
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt
from threading import Thread


from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
# from rplidar import RPLidar as Lidar
# from adafruit_rplidar import RPLidar as Lidar
from roboviz import MapVisualizer



# Screen width & height
W = 640
H = 480


MAP_SIZE_PIXELS         = 250
MAP_SIZE_METERS         = 15
MIN_SAMPLES   = 150

SCAN_BYTE = b'\x20'
SCAN_TYPE = 129

slamData = []

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

# Create an RMHC SLAM object with a laser model and optional robot model
slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

# # Set up a SLAM display
viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM', show_trajectory=True)

# Initialize an empty trajectory
trajectory = []

# To exit lidar scan thread gracefully
runThread = True

# Initialize empty map
mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

# used to scale data to fit on the screen
max_distance = 0
# x, y, theta = 0, 0, 0

# Pose will be modified in our threaded code
pose = [0, 0, 0]

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


def slam_compute(pose, mapbytes):

    try:

        # We will use these to store previous scan in case current scan is inadequate
        previous_distances = None
        previous_angles = None
        scan_count = 0

        for scan in lidar_scans(lidar):

            # To stop the thread
            if not runThread:
                break

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

            # Get new position
            pose[0], pose[1], pose[2] = slam.getpos()

            # Get current map bytes as grayscale
            slam.getmap(mapbytes)

    except KeyboardInterrupt:
        lidar.stop()
        lidar.disconnect()
        raise



# Launch the slam computation thread
thread = Thread(target=slam_compute,
                args=(pose, mapbytes))
thread.daemon = True
thread.start()

try:
    # Loop forever,displaying current map and pose
    while True:

        #print("x = " + str(pose[0]) + " y = " + str(pose[1]) + "theta = " + str(pose[2]))
        if not viz.display(pose[0]/1000., pose[1]/1000., pose[2], mapbytes):
            raise KeyboardInterrupt


except KeyboardInterrupt:
    runThread = False
    thread.join()
    lidar.stop()
    lidar.disconnect()
    exit(0)