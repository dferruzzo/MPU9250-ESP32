'''
mqtt_client.py
--------------
Este módulo contém funções para conectar a um broker MQTT e se inscrever em um tópico.

Referências:
https://www.emqx.com/en/blog/how-to-use-mqtt-in-python

Para monitorar os pacotes MQTT, instale no computador o MQTT Explorer http://mqtt-explorer.com.

Importações:
- random: para gerar um ID de cliente aleatório.
- json: para decodificar a carga útil da mensagem MQTT.
- paho.mqtt.client: cliente MQTT.

Variáveis Globais:
- broker: endereço IP do broker MQTT.
- port: porta do broker MQTT.
- topic: tópico MQTT para se inscrever.
- client_id: ID do cliente MQTT, gerado aleatoriamente.

Funções:
- connect_mqtt(): Conecta ao broker MQTT e retorna o cliente MQTT.
- subscribe(client): Se inscreve no tópico MQTT e define a função de callback para quando uma mensagem é recebida.
'''
import random
import json
import time
from paho.mqtt import client as mqtt_client # pip3 install "paho-mqtt<2.0.0"

broker = '192.168.0.73'
port = 1883
topic = "Dados"
# Generate a Client ID with the subscribe prefix.
client_id = f'subscribe-{random.randint(0, 100)}'
# username = 'emqx'
# password = 'public'
userdata = {}

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
        userdata = json.loads(msg.payload.decode())
        # str to dict
        #payload_dict=json.loads(msg.payload.decode())
        # Adicionar aqui o código para processar a mensagem recebida.
        #print("AccX = %5.5f" % payload_dict["AccX"])
        #print("AccY = %5.5f" % payload_dict["AccY"])
        #print("AccZ = %5.5f\n" % payload_dict["AccZ"])        
        print(userdata) 
    client.subscribe(topic)
    client.on_message = on_message

def run():
    client = connect_mqtt()
    client.loop_start()
    subscribe(client)
    time.sleep(4)
    client.loop_stop()
    #client.loop_forever()

if __name__ == '__main__':
    run()
