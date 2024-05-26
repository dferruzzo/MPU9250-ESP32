# Referência:
# https://www.emqx.com/en/blog/how-to-use-mqtt-in-python
# 
# Para monitorar os pacotes MQTT instale no computador o MQTT Explorer http://mqtt-explorer.com.
# 
import random
import json

from paho.mqtt import client as mqtt_client

broker = '192.168.0.73'
port = 1883
topic = "Dados"
# Generate a Client ID with the subscribe prefix.
client_id = f'subscribe-{random.randint(0, 100)}'
# username = 'emqx'
# password = 'public'

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
            
    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        # str to dict
        payload_dict=json.loads(msg.payload.decode())
        print("AccX = %5.5f" % payload_dict["AccX"])
        print("AccY = %5.5f" % payload_dict["AccY"])
        print("AccZ = %5.5f\n" % payload_dict["AccZ"])  

    client.subscribe(topic)
    client.on_message = on_message

def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()

if __name__ == '__main__':
    run()
