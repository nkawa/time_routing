import random
import time
import sys

from paho.mqtt import client as mqtt_client

import argparse

import json

id = 1
x = 20
y = 0


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            publish(client)
        else:
            print("Failed to connect, return code %d\n", rc)

    broker = '192.168.207.234' #diana
    port = 1883
    
    client = mqtt_client.Client()
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def publish(client):
    global id
    topic = f"robot/dest/{id}"
    msg = json.dumps({'header':{"frame_id": "map"}, "pose": {"position":{"x":x, "y":y, "z":0}, "orientation": {"x":0, "y":0, "z":0, "w":1}}})
    result = client.publish(topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")
    


def run():
    client = connect_mqtt()
    client.loop_forever()

    
if __name__ == '__main__':
    # Initialize parser
    parser = argparse.ArgumentParser()
 
    # Adding optional argument
    parser.add_argument("-i", "--Id", help = "robot id", type=int, default=1)
    parser.add_argument("-x", "--X", help = "x", type=float, default=20)
    parser.add_argument("-y", "--Y", help = "y", type=float, default=0)

    
    # Read arguments from command line
    args = parser.parse_args()
    
    id = args.Id
    x = args.X
    y = args.Y


    run()