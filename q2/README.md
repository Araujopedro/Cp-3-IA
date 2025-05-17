## Questão 2 – Transformando o Sensor Ultrassônico em um Dispositivo IoT

### Objetivo do Desafio

Transformar um projeto tradicional com sensor de distância (HC-SR04 + display LCD I2C) em um dispositivo IoT real, que publica dados via MQTT e envia informações para um sistema supervisório com painel visual e persistência em banco de dados.

A ideia é que o ESP32 funcione como um `nó` inteligente de coleta de dados, integrando-se a uma infraestrutura de monitoramento remoto.

### O que já está pronto

Um código funcional em Arduino que:

- Mede a distância com o sensor HC-SR04.
- Exibe o valor medido em um display LCD via I2C.
- Imprime os dados via Serial.

### Sua Tarefa

A partir desse código, você deverá:

- Integrar o ESP32 com MQTT, utilizando como referência o código da Questão 1:
- Configure o Wi-Fi e o broker MQTT.
- Crie um tópico, que faça sentido e sejá unico, para publicar os dados do sensor.
- O payload deve estar em formato JSON, por exemplo:

Desenvolver um sistema supervisório com as seguintes funcionalidades:

- Interface no Node-RED Dashboard que exiba os dados recebidos em tempo real.
- Um gráfico com histórico da distância medida.
- Exibição textual do valor atual.
- Armazenamento dos dados em um banco de dados, pode ser local (como SQLite) ou online (como MongoDB Atlas, se preferir).

### Dicas

Reutilize a estrutura de conexão e publicação MQTT da Q1.
Pesquise por tutoriais que ajudem na conexão com o banco de dados.