#!/usr/bin/env python
import time
from sys import exit
from roboviz import MapVisualizer
import numpy as np



try:
    import paho.mqtt.client as mqtt
except ImportError:
    exit("This example requires the paho-mqtt module\nInstall with: sudo pip install paho-mqtt")


MQTT_SERVER = "test.mosquitto.org"
MQTT_PORT = 1883
MQTT_TOPIC = "safetycam/topic/slamviz"

# Set these to use authorisation
MQTT_USER = None
MQTT_PASS = None


MAP_SIZE_PIXELS         = 200
MAP_SIZE_METERS         = 10
# Set up a SLAM display
viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM', show_trajectory=True)

print("""
Public MQTT messages from {server} on port {port} to visualize SLAM!

It will monitor the {topic} topic by default, and decodes the bytearray
""".format(
    server=MQTT_SERVER,
    port=MQTT_PORT,
    topic=MQTT_TOPIC
))

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):

    robotPos_bytes = msg.payload[:24]
    map_bytes = msg.payload[24:]

    robotPos = np.frombuffer(robotPos_bytes, dtype='float64')
    robotPos = np.array(robotPos)

    x, y, theta = robotPos
    viz.display(x / 1000., y / 1000., theta, map_bytes)

    return



client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

if MQTT_USER is not None and MQTT_PASS is not None:
    print("Using username: {un} and password: {pw}".format(un=MQTT_USER, pw="*" * len(MQTT_PASS)))
    client.username_pw_set(username=MQTT_USER, password=MQTT_PASS)

client.connect(MQTT_SERVER, MQTT_PORT, 60)

client.loop_forever()