"""
    Description: Este script se conecta a um servidor MQTT e plota os valores recebidos em tempo real.
    Returns:
        _type_: _description_
    Recommendations: Use Grafana para visualização de dados em tempo real em lugar desse script.
"""
# TODO: Try Grafana with InfluxDB
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import json

# Configurações do MQTT
MQTT_BROKER = "192.168.0.73"  # Substitua pelo endereço do seu servidor MQTT
MQTT_PORT = 1883  # Porta padrão do MQTT
MQTT_TOPIC = "Dados"  # Substitua pelo tópico ao qual você quer se inscrever

BUFFER_SIZE = 100
# Buffer para armazenar os dados recebidos
data = deque(maxlen=BUFFER_SIZE)  # Mantém os últimos 100 valores recebidos
# Inicializa o buffer das labels de tempo
LABELS_SIZE = BUFFER_SIZE
time_labels = deque(maxlen=LABELS_SIZE)
for i in range(LABELS_SIZE):
    time_labels.append(str(i))
#tempo_data = deque(maxlen=BUFFER_SIZE)

# Função chamada quando a conexão com o broker é estabelecida
def on_connect(client, userdata, flags, rc):
    print(f"Conectado com o código {rc}")
    client.subscribe(MQTT_TOPIC)

# Função chamada quando uma mensagem é recebida
def on_message(client, userdata, msg):
    try:
        # Tenta converter a payload para float e adiciona ao buffer
        pl = json.loads(msg.payload.decode())
        tempo = round(pl["Time"]/1000,1) # Converte o tempo para segundos
        gx = pl["GyrX"]
        data.append(float(gx))
        time_labels.append(str(tempo))
        #print(tempo_data)
    except ValueError:
        print("Valor recebido não é um número válido")

# Configuração do cliente MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Inicializa o buffer com zeros
data.extend([0] * data.maxlen)

# Função de inicialização do gráfico
def init():
    ax.set_xlim(0, data.maxlen)
    #ax.set_ylim(auto=True)  # Ajuste os limites do eixo Y conforme necessário
    line.set_ydata([0] * data.maxlen)
    ax.set_xlabel("Tempo (s)")
    return line,

# Função para atualizar o gráfico
def update(frame):
    line.set_ydata(data)
    ax.set_ylim(min(data), max(data))
    ax.set_xticks(range(0,BUFFER_SIZE))
    ax.xaxis.set_ticklabels(time_labels, rotation=45)
    return line,

# Configurações do gráfico
fig, ax = plt.subplots(figsize=(15,5))
ax.tick_params(axis='x', labelsize=6)
line, = ax.plot(range(data.maxlen), data)

# Configura a animação
ani = animation.FuncAnimation(fig, update, init_func=init, interval=100, save_count=50)

# Inicia o loop MQTT em uma thread separada
client.loop_start()

# Mostra o gráfico
plt.show()

# Quando a janela do gráfico é fechada, para o loop MQTT
client.loop_stop()
client.disconnect()