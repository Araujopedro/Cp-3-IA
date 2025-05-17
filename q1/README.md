## Questão 1 – Comunicação MQTT com ESP32 no Wokwi

### Objetivo do Desafio

Explorar a comunicação entre um dispositivo IoT (ESP32) e um sistema externo utilizando o protocolo MQTT, amplamente adotado em aplicações de Internet das Coisas. O projeto já inclui um código completo em Arduino, que simula um sensor de temperatura e umidade e um LED controlável remotamente. O objetivo principal é criar uma interface no Node-RED que receba e envie mensagens MQTT para interagir com esse dispositivo.

### O que já está pronto

O código Arduino está completo e funcional:

- Ele conecta o ESP32 ao Wi-Fi.
- Lê periodicamente os valores do sensor DHT22.
- Publica os dados de temperatura, umidade e estado do LED no tópico fiap/iot/temphumi.
- Escuta comandos no tópico fiap/iot/led, ativando ou desativando o LED via JSON.

Você não deve modificar o código do ESP32 neste momento.

### Sua Tarefa

Configurar o Node-RED para se conectar ao mesmo broker MQTT do ESP32.

`Assinar os tópicos:`

- Exibir na interface os valores recebidos: temperatura, umidade e status do LED.

`Criar um painel de controle MQTT:`

- Adicione botões ou interruptores que permitam enviar mensagens para controlar o LED remotamente.

O payload deve estar em formato JSON.

