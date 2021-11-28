import random
import time
import sys

from paho.mqtt import client as mqtt_client

import argparse

import json

id = 0

synarios = [
    [(0,0), (0,0), (0,0)], # 0に集合
    [(40,0), (45, -3), (-23, -2)], # 互いに入れ替わり
    [(-15,23), (9.3, 23.9), (32.27, 23.8)], #上の方に並ぶ
    [(47.7, -4.0), (41, 24.3), (-21, 5.6)] # random

]



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
    global id, synarios

    for i, pos in enumerate(synarios[id]):
        topic = f"robot/dest/{i+1}"
        msg = json.dumps({'header':{"frame_id": "map"}, "pose": {"position":{"x":pos[0], "y":pos[1], "z":0}, "orientation": {"x":0, "y":0, "z":0, "w":1}}})
        result = client.publish(topic, msg)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")
        time.sleep(0.1)


def run():
    client = connect_mqtt()
    client.loop_forever()

    
if __name__ == '__main__':
    # Initialize parser
    parser = argparse.ArgumentParser()
 
    # Adding optional argument
    parser.add_argument("-i", "--Id", help = "robot id", type=int, default=0)

    
    # Read arguments from command line
    args = parser.parse_args()
    
    id = args.Id


    run()