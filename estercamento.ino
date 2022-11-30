#include <SPI.h>
#include "mcp2515_can.h"

// Constanstes do programa:
// Pinos Encoder
#define encoderPinA      2     // A -> Pino 2
#define pino_reflexivo   4
#define pino_optico      2
#define pino_sentido     8
#define pino_PWM         10
#define pino_acionamento 12
// Leitura direta do rbarramenta
#define readPinA        bitRead(PIND,2) 
// Tempo de Aquisição
#define tempoAquisicao  1000
#define PI 3.1415926535897932384626433832795


// Variáveis do programa:
volatile long int contadorPassos = 0;
long int contadorProtegido = 0;
long int tempoInicial = 0;
long int tempoAtual = 0;
long int tempoDecorrido = 0;

int sensor_reflexivo = 0;//Criação de variável
int sensor_optico = 0;//Criação de variável
int test_giro = 0;//variavel de controle do teste

int Pulsos_totais = 0;
int giro_new = 0;
int sentido_giro = 0;
int pulsos_angulo = 0;
int Pulsos_req = 0;
int giro_old = 0;

////////// VARIÁVEIS CAN
const int SPI_CS_PIN = 5;
const int CAN_INT_PIN = 3;
unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned long can_ID = 0;
unsigned char buf[8];
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

void MCP2515_ISR() {
    flagRecv = 1;
}

void home(){
    
    sensor_reflexivo = digitalRead(pino_reflexivo);//variável sensor_reflexivo armazena dados do sensor reflexivo.
    sensor_optico = digitalRead(pino_optico);//variável sensor_optico armazena dados do sensor óptico.
    
    while (sensor_reflexivo == 1 && sensor_optico == 1){ //enquanto os sensores estão desativados.
        Serial.println(sensor_reflexivo);
        Serial.println(sensor_optico);
        //Sentido do giro:
        digitalWrite(pino_sentido,LOW);//giro anti-horário do eixo (o sentido de giro definido pelos sinais "LOW" ou "HIGH", são modificados para cada roda).
        delay(10);//espera de 20 milissegundos
        //Liga:
        digitalWrite(pino_acionamento,LOW);//comando de acionamento do motor de esterçamento.
        //PWM:
        analogWrite(pino_PWM,70);//Envio de sinal PWM (escala de 0 até 255).
        Serial.println("Giro Anti-horario.");//print da operação "giro anti-horário".

        //Atualização dos dados de entrada
        sensor_reflexivo = digitalRead(pino_reflexivo);//atualização da variável sensor_reflexivo, dados do sensor reflexivo.
        sensor_optico = digitalRead(pino_optico);//atualização da variável sensor_reflexivo, dados do sensor óptico.
    }
    //Desacionamento do motor de esterçamento:
    delay(10);//espera de 20 milisegundos
    analogWrite(pino_PWM, LOW);//Envio de sinal PWM (escala de 0 até 255).
    delay(10);//espera de 20 milisegundos
    digitalWrite(pino_acionamento, HIGH);//comando de desligamento do motor de esterçamento.

    Serial.println(sensor_reflexivo);
    Serial.println(sensor_optico);
    
    while (sensor_reflexivo == 0 || sensor_reflexivo == 1 && sensor_optico == 1){ //caso em que sensor reflexivo é acionado e posterior desacionamento dos sensores.
        //Sentido do giro:
        digitalWrite(pino_sentido, HIGH);//giro horário do eixo (o sentido de giro definido pelos sinais "LOW" ou "HIGH", são modificados para cada roda).
        delay(10);
        //Liga:
        digitalWrite(pino_acionamento, LOW);//comando de acionamento do motor de esterçamento.
        //PWM:
        analogWrite(pino_PWM, 70);//Envio de sinal PWM (escala de 0 até 255).
        Serial.println("Giro Horario.");//print da operação "giro horário".

        //Atualização dos dados de entrada
        sensor_reflexivo = digitalRead(pino_reflexivo);//atualização da variável sensor_reflexivo, dados do sensor reflexivo.
        sensor_optico = digitalRead(pino_optico);//atualização da variável sensor_reflexivo, dados do sensor óptico.
    }
        //Desacionamento do motor de esterçamento:
    delay(10);//espera de 20 milisegundos
    analogWrite(pino_PWM, LOW);//Envio de sinal PWM, duty cycle de 0% (escala de 0 até 255).
    delay(10);//espera de 20 milisegundos
    digitalWrite(pino_acionamento, HIGH);//comando de desligamento do motor de esterçamento.

    Serial.println("Sistema alinhado.");//print de operação finalizada "sistema alinhado".
}

int data_extract(unsigned char* buf){
  int angulo = 0;
  angulo = buf[1];
  for(int i = 0; i<4; i++){
    bitWrite(angulo,i+8,bitRead(buf[0],i));
  }
    return angulo;
}

void error(unsigned long can_ID){
    unsigned long erro = 0;
    bitWrite(erro, 0, bitRead(can_ID, 5));
    bitWrite(erro, 1, bitRead(can_ID, 6));

    switch(erro){
      case 00:
        Serial.println("ERRO DE HOME");
        //Falha no motor
        break;
      case 01:
        Serial.println("ERRO NO MOTOR");
        //erro de leitura do encoder
        break;
      case 10:
        Serial.println("ERRO NO ENCODER");
        //outro tipo de erro
        break;
      case 11:
        Serial.println("SEM ERRO"); 
        //ok
        break;
    }
}

void comando_valor(unsigned char* buf){
  
    switch(bitRead(buf[0],7)){
        case 0: // comando para a roda
            switch(bitRead(buf[0],6)){ //Verificação do acionamento do motor
                case 1://ON
                    if(bitRead(buf[0],4)==0){
                        home();
                    }else if(bitRead(buf[0],4)==1){
                        //impõe angulo para o nó
                    }
                    break;
                case 0://OFF
                  digitalWrite(12,LOW);//desliga o motor
                  Serial.println("motor desligado");
                  analogWrite(10, 0);        
                  break;
            }
            break;
        case 1:
            break;
            //pergunta valor pro nó. 

    }

}

void isrA(){
    contadorPassos++;
}

void setup() {
    pinMode(pino_optico, INPUT);//identificação de dados no pino 5, provenientes sensor óptico.
    pinMode(pino_reflexivo,INPUT);//identificação de dados no pino 7, provenientes sensor reflexivo.
    pinMode(pino_sentido,OUTPUT);//transmissão de dados via pina 8.
    pinMode(pino_PWM,OUTPUT);//transmissão de dados via pina 10.
    pinMode(pino_acionamento,OUTPUT);//transmissão de dados via pina 12.
    
    // Definindo a velocidade da comunicação serial
    Serial.begin(115200);
    while(!Serial){};
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_ISR, FALLING); // start interrupt
    while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
    //home();
    contadorPassos = 0;
    // Definindo as entradas com um resistor de Pull Up
    pinMode(encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA),isrA, CHANGE);  
}

void loop() {
    noInterrupts();
    contadorProtegido = contadorPassos;
    interrupts();

    if(flagRecv){
    flagRecv = 0;                // clear flag
    CAN.readMsgBuf(&len,buf);    // read data,  len: data length, buf: data buf
    can_ID = CAN.getCanId();
    Serial.println(CAN.getCanId());
    int angulo = map(data_extract(buf),0,4095,0,360);
    Serial.println(angulo);
    //error(can_ID);
    //comando_valor(buf);
    }
}
