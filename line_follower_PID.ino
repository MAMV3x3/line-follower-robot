#include <QTRSensors.h>

#define led_indicador 9
#define start_button 8
#define pwmi  3   //PWM LEFT MOTOR
#define izq1  5   //LEFT1 MOTOR
#define izq2  4   //LEFT2 MOTOR
#define pwmd  11  //PWM RIGHT MOTOR
#define der1  6   //RIGHT1 MOTOR
#define der2  7   //RIGHT2 MOTOR
//#define GO 10

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Settings
int tipo_de_pista = 1; //1 para pista negra, 0 para pista blanca

float KP = 0.09;
float KD = 0.3;
float KI = 0.0;
int max_speed = 150;

int setpoint = 3500;

// Variables
int proporcional = 0;
int last_error = 0;
int derivativo = 0;
int integral = 0;
float diferencial = 0;
int error = 0;

void setup() {
  //PWM Frequency
  TCCR2B = TCCR2B & B11111000 | B00000011;  

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setEmitterPin(2);

  pinMode(led_indicador, OUTPUT);
  pinMode(start_button, INPUT_PULLUP);
  pinMode(izq1, OUTPUT);
  pinMode(izq2, OUTPUT);
  pinMode(der1, OUTPUT);
  pinMode(der2, OUTPUT);
  
  while(digitalRead(start_button) == HIGH);
  digitalWrite(led_indicador, HIGH);
  
  // Calibrate sensors
  for (int i = 0; i < 100; i++) {
    qtr.calibrate();
  }
  digitalWrite(led_indicador, LOW);
  while(digitalRead(start_button) == HIGH);  
}

void loop() {
  // Read RRT
  //int GO = digitalRead(GO);    
  while(true){
    //int GO = digitalRead(GO); 
    //frenos();
    int lectura = position();
    diferencial = PID(lectura);
    controlMotores(diferencial, max_speed);
    // if(GO == 0){
    //   moverMotores(-20, -20);       
    //   break;
    // }
  }
  // while(true){
  //   moverMotores(0, 0);
  // }  
}

int position() {
  qtr.read(sensorValues);
  int position;
  if(tipo_de_pista){
    position = qtr.readLineBlack(sensorValues);
  }
  else{
    position = qtr.readLineWhite(sensorValues);
  }
  return position;
}

float PID(int lectura) {
  error = setpoint - lectura;
  proporcional = error;
  integral = (integral + proporcional) * ((integral*proporcional > 0));
  derivativo = error - last_error;
  float diff = KP * proporcional + KD * derivativo + KI * integral;
  last_error = error;
  return diff;
}

void controlMotores(int diferencial, int speed) {
  if (diferencial > speed) diferencial = speed;
  if (diferencial < -speed) diferencial = -speed;
  if (diferencial < 0) {
    moverMotores(speed, speed+diferencial);
  } else {
    moverMotores(speed-diferencial, speed);
  }
}

void moverMotores(int izq, int der) {
  // Mueve los motores con la velocidad indicada
  // izq: velocidad del motor izquierdo (valor entre -255 y 255)
  // der: velocidad del motor derecho (valor entre -255 y 255)

  // Control del motor IZQUIERDO
  if (izq >= 0) {
    digitalWrite(izq1, HIGH);
    digitalWrite(izq2, LOW);
  } else {
    digitalWrite(izq1, LOW);
    digitalWrite(izq2, HIGH);
    izq = -izq;
  }
  analogWrite(pwmi, izq);

  // Control del motor DERECHO
  if (der >= 0) {
    digitalWrite(der1, LOW);
    digitalWrite(der2, HIGH);
  } else {
    digitalWrite(der1, HIGH);
    digitalWrite(der2, LOW);
    der = -der;
  }
  analogWrite(pwmd, der);
}

void frenos() {
  if(position<=150){
    moverMotores(-max_speed, max_speed);    
  }
  if(position>=6850){
    moverMotores(max_speed, -max_speed);
  }
}
