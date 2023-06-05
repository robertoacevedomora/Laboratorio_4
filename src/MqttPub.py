import serial
import paho.mqtt.client as mqtt
import json
import time
import os
from datetime import datetime

ser = serial.Serial('COM4',115200,timeout =0)


ACCESS_TOKEN = 'JAdeAfZaduFGAbQJxtTx'
broker = 'demo.thingsboard.io'
port = 1883

def on_publish(client, userdata, result):
    print("data published to thingsboard /n")
    pass
client1 = mqtt.Client("Giroscopio")
client1.on_publish = on_publish
client1.username_pw_set(ACCESS_TOKEN)
client1.connect(broker,port,keepalive=60)
dict = dict()


while (1):
    ejes = ser.readline().strip().decode('utf-8')
    ejes = ejes.split('\t')
    dict["X"] = ejes[0]
    dict["Y"] = ejes[1]
    dict["Z"] = ejes[2]
    dict["Nivel"] = ejes[3]
    hola = json.dumps(dict)
    client1.publish("v1/devices/me/telemetry",hola)
    print(hola)
    time.sleep(1)
    



