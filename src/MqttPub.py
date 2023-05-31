import serial
#import paho.mqtt.client as mqtt
#import json
#import ssl, socket


#Puerto
ser = serial.Serial('COM4',115200,timeout =0)

while True:
    data = ser.readline()
    data_sensor = data.decode('utf8')
    print(data_sensor)

#Broker
#broker = "iot.eie.ucr.ac.cr"
#port = 1883
#topic= "vI/devices/me/telemetry"
#username = "holadevice"
#password = "vd42cc4bzxr5eoow71oo"

#button = {"enabled":False}

#while True:
    #try:
      #  data = ser.readline().strip().decode('utf-8')
       # x, y, z = data.split('\t')
        
        
        # Creamos un diccionario con los valores de X, Y, Z y la batería
        #json_data = {"x": float(x), "y": float(y), "z": float(z)}
            
        # Convertimos el diccionario a formato JSON y lo publicamos por MQTT
        #client.publish(topic, json.dumps(json_data))
        #print(json_data)
    #except:
     #   pass
