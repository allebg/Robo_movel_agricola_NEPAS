#include <SPI.h>
#include "mcp2515_can.h"

// Constanstes do programa:
// Pinos Encoder
#define encoderPinA     2     // A -> Pino 2
/*#define encoderPinB     3     // B -> Pino 3
// Leitura direta do rbarrament0
#define readPinA        bitRead(PIND,2) 
#define readPinB        bitRead(PIND,3)*/
// Tempo de Aquisição
#define tempoAquisicao  1000
//Pi
#define PI 3.1415926535897932384626433832795
#define  D 0.254 //Diâmetro da roda em metros (10")


////////// VARIÁVEIS e CONSTANTES CAN
const int SPI_CS_PIN = 5;
const int CAN_INT_PIN = 3;
unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned long can_ID = 0;
unsigned char buf[8];// = {0b01111111, 0b11111111, 0,0,0,0};

mcp2515_can CAN(SPI_CS_PIN); // Set CS pin


////////// VARIAVEIS TRAÇÃO
volatile long int contadorPassos = 0;
long int contadorProtegido = 0;
long int tempoInicial = 0;
long int tempoAtual = 0;
long int tempoDecorrido = 0;
int dist1 = 0;
int dist2 = 0;
unsigned long tempo1 = 0;
unsigned long tempo2 = 0;
boolean flagSentidoHorario;



int deslocamento = 0;
int acionamento = 0;
int vel = 0;
int voltas = 0;
long aux=0;

void setup() {
  pinMode(8,OUTPUT);//transmissão de dados via pina 8. SENTIDO
  pinMode(10,OUTPUT);//transmissão de dados via pina 10. PWM
  pinMode(12,OUTPUT);//transmissão de dados via pina 12. ACIONAMENTO

  
  Serial.begin(115200);
  while(!Serial){};
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_ISR, FALLING); // start interrupt
  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)) {             // init can bus : baudrate = 500k
      SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
      delay(100);
  }
  SERIAL_PORT_MONITOR.println("CAN init ok!");
  
  //Contorno dos enconders
  contadorPassos = 0;
  // Definindo as entradas com um resistor de Pull Up
  pinMode(encoderPinA, INPUT_PULLUP);
  //pinMode(encoderPinB, INPUT_PULLUP);
  // Configurando as interrupções
  attachInterrupt(digitalPinToInterrupt(encoderPinA),isrA, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoderPinB),isrB, CHANGE);
}

void MCP2515_ISR() {flagRecv = 1;}

void loop() {
  aux++;
  noInterrupts();
  contadorProtegido = contadorPassos;
  /*Serial.print("Passos: ");
  Serial.println(contadorProtegido);
  Serial.print("Deslocamento: ");
  Serial.println(Deslocamento);*/
  //tempoInicial = millis();
  interrupts();

  if(flagRecv){
    flagRecv = 0;                // clear flag
    CAN.readMsgBuf(&len,buf);    // read data,  len: data length, buf: data buf
    can_ID = CAN.getCanId();
    Serial.println(CAN.getCanId());
    error(can_ID);
    comando_valor(buf);
  }

  //Cálculo da distância:
  deslocamento = contadorProtegido*((PI*D)/100);//46200 é o número de pulsos por rotação
  odometer_hourmeter(deslocamento);
  instant_vel(deslocamento);

    /*do{
    tempoAtual = millis();
    //if(tempoAtual < tempoInicial){
      //tempoDecorrido = 4294967295 - tempoInicial + tempoAtual;
    //}
    //else{
      tempoDecorrido = tempoAtual - tempoInicial;
    //}
  }while(tempoDecorrido < tempoAquisicao);*/

}

void odometer_hourmeter(int deslocamento){
  if(aux == 50000){
    Serial.print("Odômetro: ");
    Serial.print(deslocamento);
    Serial.println(" metros");
    Serial.print("Horímetro: ");
    Serial.print(millis()/1000);
    Serial.println(" segundos");
  }
}

void instant_vel(int deslocamento){
  if(aux == 50000){
    aux = 0;
    dist2 = dist1;
    dist1 = deslocamento;
    tempo2 = tempo1;
    tempo1 = millis();

    vel = (dist1-dist2)/((tempo1-tempo2)/1000);
    Serial.print("Velocidade atual: ");
    Serial.print(vel);
    Serial.println(" m/s");
  }  
}


int data_extract(unsigned char* buf){
  int data=0;
  data = buf[1];
  for(int i = 0; i<4; i++){
    bitWrite(data,i+8,bitRead(buf[0],i));
  }
    return data;
}

void error(unsigned long can_ID){
    unsigned long erro = 0;
    bitWrite(erro, 0, bitRead(can_ID, 5));
    bitWrite(erro, 1, bitRead(can_ID, 6));

    switch(erro){
      case 00:
        Serial.println("Falha no motor");
        //Falha no motor
        break;
      case 01:
        Serial.println("erro de leitura encoder");
        //erro de leitura do encoder
        break;
      case 10:
        Serial.println("outro tipo de erro");
        //outro tipo de erro
        break;
      case 11:
        Serial.println("ok"); 
        //ok
        break;
    }
}

void comando_valor(unsigned char* buf){
  
switch(bitRead(buf[0],7)){
    case 0: // comando para a roda
  switch(bitRead(buf[0],5)){ //Verificação do sentido de Giro
    case 0: //para trás
      if(flagSentidoHorario){
        digitalWrite(12,LOW);//desliga o motor
        delay(100)
      }
      digitalWrite(8,LOW); //giro anti-horário do eixo
      flagSentidoHorario = false;
      digitalWrite(12,HIGH);//liga o motor
      Serial.println("Sentido de Giro anti horário");
      delay(10);
      break;
    case 1: //para frente
        if(!flagSentidoHorario){
            digitalWrite(12,LOW);//desliga o motor
            delay(100)
        }
      digitalWrite(8,HIGH); //giro horário do eixo
      flagSentidoHorario = true;
      delay(10);
      digitalWrite(12,HIGH);//liga o motor 
      Serial.println("Sentido de Giro horário");
      delay(10);
      break;
  }

        switch(bitRead(buf[0],6)){ //Verificação do acionamento do motor
          case 1:
            digitalWrite(12, HIGH);//aciona o motor
            analogWrite(10, data_extract(buf));        
            Serial.println("motor acionado");
            break;
          case 0:
            digitalWrite(12,LOW);//desliga o motor
            Serial.println("motor desligado");
            analogWrite(10, 0);        
            break;    
        }
    case 1:  //Requisição de valor para o nó
    break;

}

void isrA(){
    contadorPassos++;
}
/*
void isrB(){
  if(readPinA == readPinB)
    contadorPassos++;
  else
    contadorPassos--;
}*/
