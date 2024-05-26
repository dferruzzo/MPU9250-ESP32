# MPU-9250 e ESP32

## Descrição do Projeto

Projeto para a determinação da atitude utilizando MPU-9250 e ESP32. São lidos os seguintes  dados:

* Giroscópio, 3 eixos,
* Acelerômetro, 3 eixos,
* Magnetômetro, 3 eixos,
* Temperatura em Celsius.

Utilizando filtro de Kalman, são gerados os seguintes dados:

* [ ] atitude em ângulos de Euler, (\phi,~\theta,~\psi), rolagem (roll), arfagem (pitch), guinada (yaw).
* [ ] atitude em quaterniões.
* [ ] taxas (p,q,r).

## Pré-requisitos

* PlatformIO em VSCode.
* MPU-9250
* ESP32

## Instalação

1. Clone o repositório
2. Instale as dependências
3. Abra a pasta no VScode.

## Uso

Implemente o circuito mostrado.

![Diagrama do circuito](circuit_diagram.png)

## Trabalho pendentes

* [x] Transmitir dados via WiFi.
* [x] Utilizar protocolo MQTT.
* [ ] Finalizar visualização em tempo-real via WiFi.
* [ ] Implementar filtro de Kalman.
* [ ] Implementar fusão sensorial.

## Licença

GNU General Public License (GPL).

## Contato

Diego Paolo Ferruzzo Correa

diego.ferruzzo@ufabc.edu.br
