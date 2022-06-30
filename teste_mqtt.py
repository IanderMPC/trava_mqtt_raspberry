import paho.mqtt.client as mqtt
import random
import json
from picamera.array import PiRGBArray # Generates a 3D RGB array
from picamera import PiCamera # Provides a Python interface for the RPi Camera Module
import requests as client
import time # Provides time-related functions
import cv2 # OpenCV library
import numpy as np # Import NumPy library
import base64
import RPi.GPIO as GPIO

cont =-2
# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)

# Set pin 11 as an output, and set servo as pin 11 as PWM
GPIO.setup(36,GPIO.OUT)
servo = GPIO.PWM(36,50) # Note 11 is pin, 50 = 50Hz pulse

#start PWM running, but with value of 0 (pulse off)
servo.start(0)

def lock():
    servo.ChangeDutyCycle(12)
    time.sleep(0.7)
    servo.ChangeDutyCycle(0)

def unlock():
    servo.ChangeDutyCycle(2)
    time.sleep(0.7)
    servo.ChangeDutyCycle(0)
    time.sleep(10)
    lock()
    
# Definitions
# put here your device token
device_token = '159c730b-54db-4473-84b7-670f76eb9ef9'

broker = "mqtt.tago.io"
broker_port = 1883
mqtt_keep_alive = 60

# MQTT publish topic must be tago/data/post
mqtt_topic = "tago/data/post"

# put any name here, TagoIO doesn't validate this username.
mqtt_username = 'tagoio'

# MQTT password must be the device token (TagoIO does validate this password)
mqtt_password ='159c730b-54db-4473-84b7-670f76eb9ef9'



def on_connect(client, userdata, flags, rc):
    # This will be called once the client connects
    print(f"Connected with result code {rc}")
    # Subscribe here!
    client.subscribe(mqtt_topic)
def on_message(client, userdata, msg):
    global cont
    
    if msg.retain!=1 and cont ==0:
        print(f"Message received [{msg.topic}]: {msg.payload}")
        status = json.loads(msg.payload.decode('utf-8'))
        unlock()
        cont = 0;
    else:
        cont += 1
    
    
    
print("[STATUS] Initializing MQTT...")
client = mqtt.Client(protocol=mqtt.MQTTv31)
client.username_pw_set(mqtt_username, mqtt_password)
# client.tls_set_context()
client.on_connect = on_connect
#client.subscribe("tago/data/post")
#client.on_publish = on_publish


x = client.connect(broker, broker_port)
client.on_message = on_message
try:
    client.loop_forever()
except KeyboardInterrupt:
    pass

client.disconnect()

client.loop_stop()

GPIO.cleanup() 

