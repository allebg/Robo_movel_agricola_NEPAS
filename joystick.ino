#include <SPI.h>
#include <math.h>
#include "mcp2515_can.h"

#define PI 3.14159265

#define BUTTON_UP 2
#define BUTTON_RIGHT 3
#define BUTTON_DOWN 4
#define BUTTON_LEFT 5
#define PIN_ANALOG_X 0
#define PIN_ANALOG_Y 1

const int SPI_CS_PIN = 10;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
bool flag_motor_frente = 0;
bool flag_motor_tras = 0;



void setup() {
    Serial.begin(115200);
    while(!Serial){};
    
    while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_8MHz)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
    pinMode(BUTTON_UP, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT, INPUT_PULLUP);
    pinMode(BUTTON_DOWN, INPUT_PULLUP);
    pinMode(BUTTON_LEFT, INPUT_PULLUP);
}

void loop() {

 if(digitalRead(BUTTON_UP) == LOW) {
    if(flag_motor_tras == 1){
        flag_motor_tras = 0;
        stmp[0] = 0b00111111; //comando de desacionamento do motor
        CAN.sendMsgBuf(0x11100000, 0, sizeof(stmp), stmp);
        Serial.println("CAN BUS sendMsgBuf ok! - desligar motor (f)");      
        delay(30);                       
    }    
    stmp[0] = 0b01110000;
    stmp[1] = 0b11111111;   //comando de acionamento do motor para frente
    CAN.sendMsgBuf(0x11100000, 0, sizeof(stmp), stmp);
    delay(100);
    SERIAL_PORT_MONITOR.println("CAN BUS sendMsgBuf ok! - ligar motor");
    flag_motor_frente = 1;
 }
 if(digitalRead(BUTTON_LEFT) == LOW) {
    stmp[0] = 0b00110000; //comando de desacionamento do motor
    CAN.sendMsgBuf(0x11100000, 0, sizeof(stmp), stmp);
    delay(100);
    SERIAL_PORT_MONITOR.println("CAN BUS sendMsgBuf ok! - desligar motor");
    flag_motor_frente = 0;
    flag_motor_tras = 0;
   
 } if(digitalRead(BUTTON_RIGHT) == LOW) {
   Serial.println("Button 0 is pressed");
 }

 else if(digitalRead(BUTTON_DOWN) == LOW) {
    if(flag_motor_frente == 1){
      flag_motor_frente = 0;
        stmp[0] = 0b00111111; //comando de desacionamento do motor
        CAN.sendMsgBuf(0x11100000, 0, sizeof(stmp), stmp);
        Serial.println("CAN BUS sendMsgBuf ok! - desligar motor(t)");      
        delay(30);                       
    }
    stmp[0] = 0b01010000;
    stmp[1] = 0b11111111;
    CAN.sendMsgBuf(0x11100000, 0, sizeof(stmp), stmp);
    Serial.println("CAN BUS sendMsgBuf ok! - ligar motor para trás");
    flag_motor_tras = 1;     
    delay(100);                       
  }
 /*
  Serial.print("x: ");
  Serial.println(analogRead(PIN_ANALOG_X));
  Serial.print("y: ");
  Serial.println(analogRead(PIN_ANALOG_Y));
  */
  // Some delay to clearly observe your values on serial monitor.
  //joystick_angle(analogRead(PIN_ANALOG_X), analogRead(PIN_ANALOG_Y));

}

void joystick_angle(int x, int y){
  int angle = 0;
  int val = 180.0 / PI;

  x -= 518;
  y -= 523;
  if(x<4 && x>-4) x = 0; {}
  if(y<4 && y>-4) y = 0; {}
  angle = atan2 (y,x) * val;
  if (angle<0) angle+=360; {}
  Serial.println(x);
  Serial.println(y);
  Serial.println(angle);
  delay(25);
}