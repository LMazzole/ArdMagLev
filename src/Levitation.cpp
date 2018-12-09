#include <PID_v1.h>
#include "Arduino.h"

// #define MYDEBUG

#define CONTROLL
// #define TESTX
// #define TESTY
// #define TESTXY

//Define Debug-Function
#ifdef MYDEBUG
  #define DEBUG_PRINT(x)        Serial.print (x)
  #define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
  #define DEBUG_PRINTLN(x)      Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTDEC(x)
  #define DEBUG_PRINTLN(x)
#endif

// const byte IN1 = 4; //Enable for Out 2
// const byte IN2 = 3; //Enable for Out 2
// const byte IN3 = 7; //Enable for Out 3
// const byte IN4 = 8; //Enable for Out 3
// const byte ENA = 6;
// const byte ENB = 5;
// const byte BL = 2;

const byte IN1 = 9; //Enable for Out 1 X_Plus
const byte IN2 = 8; //Enable for Out 2 X_Minus
const byte IN3 = 7; //Enable for Out 3 Y_Plus
const byte IN4 = 6; //Enable for Out 4 Y_Minus
const byte ENA = 10;
const byte ENB = 5;
const byte HALLX = A1;
const byte HALLY = A0;
// const byte BL = 2;

double Setpoint_X, Input_X, Output_X,X_plus;
double p_X = 5.0,i_X = 0,d_X = 0.08;
// double p_X = 5.0,i_X = 0.0,d_X = 0.01;


double Setpoint_Y, Input_Y, Output_Y,Y_plus;
double p_Y = 5.0,i_Y = 0,d_Y = 0.08;
// double p_Y = 5.0,i_Y = 0.0,d_Y = 0.01;


int i,on_put=1;

#define anzahlMittelWerte 3
int werteHallX[anzahlMittelWerte], zaehlerMittelWerteX=0;
int werteHallY[anzahlMittelWerte], zaehlerMittelWerteY=0;
int mittelWertX(int neuerWert);
int mittelWertY(int neuerWert);
// unsigned long time;

PID PID_X(&Input_X, &Output_X, &Setpoint_X, p_X,i_X,d_X, DIRECT);
PID PID_Y(&Input_Y, &Output_Y, &Setpoint_Y, p_Y,i_Y,d_Y, DIRECT);

// char inByte='9',nullByte,run_flag,run_dirict;
// float go_step;

void turn_X(int a);
void turn_Y(int a);
void read_sensor();
void turn_off();
double pid_ctrl_X(double error);
double pid_ctrl_Y(double error);


void setup(){
  Serial.begin(57600);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  // pinMode(BL,OUTPUT);
  digitalWrite(IN1,0);
  digitalWrite(IN2,0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,0);
  analogWrite(ENA,0);
  analogWrite(ENB,0);


  Setpoint_X = 550;//560;
  PID_X.SetTunings(p_X,i_X,d_X);
  PID_X.SetOutputLimits(-255,255);
  PID_X.SetSampleTime(1);
  PID_X.SetMode(AUTOMATIC);

  Setpoint_Y = 560;//560;
  PID_Y.SetTunings(p_Y,i_Y,d_Y);
  PID_Y.SetOutputLimits(-255,255);
  PID_Y.SetSampleTime(1);
  PID_Y.SetMode(AUTOMATIC);

}

void loop(){
  // Input_X = mittelWertX(analogRead(HALLX));
  // Input_Y = mittelWertY(analogRead(HALLY));
  Input_X = analogRead(HALLX);
  Input_Y = analogRead(HALLY);

    read_sensor();
  #ifdef CONTROLL //Regeln
    //Compunt correction Value
    //===PID====
    PID_X.Compute();
    PID_Y.Compute();
    // Output_X = map(Input_X, 0, 1023, -255, 255);
    // Output_Y = map(Input_Y, 0, 1023, -255, 255);

    //==I/O=======
    // if (abs((Output_X-Setpoint_X))>10){
    //   Output_X=255;
    // }
    // else{
    //   Output_X=-255;
    // }
    // if (abs((Output_Y-Setpoint_Y))>10){
    //   Output_Y=255;
    // }
    // else{
    //   Output_Y=-255;
    // }
    turn_X(Output_X);
    turn_Y(Output_Y);
    // read_sensor();
    // delay(1);
  #endif

  #ifdef TESTX
    turn_X(255); //Oben
    delay(10000);
    turn_X(-255); //Unten
    delay(10000);
  #endif

  #ifdef TESTY
    turn_Y(255); //Rechts
    delay(10000);
    turn_Y(-255); //Links
    delay(10000);
  #endif

  #ifdef TESTXY
    DEBUG_PRINTLN("=====TESTXY=====");
    int delayTest=750;
    turn_X(255); //Oben
    delay(delayTest);
    turn_off();
    read_sensor();
    turn_Y(255); //Rechts
    delay(delayTest);
    turn_off();
    read_sensor();
    turn_X(-255); //Unten
    delay(delayTest);
    turn_off();
    read_sensor();
    turn_Y(-255); //Links
    delay(delayTest);
    turn_off();
    read_sensor();
    // delay(5000);
  #endif
}

void turn_X(int a){
  DEBUG_PRINT("turn_X: "); DEBUG_PRINT(a);
  if(a>=0)  {
    DEBUG_PRINTLN(" Oben");
    digitalWrite(IN1,1);
    digitalWrite(IN2,0);
    analogWrite(ENA,a);
  }
  else  {
    DEBUG_PRINTLN(" Unten");
    a=-a;
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    analogWrite(ENA,a);
  }
}

void turn_Y(int a){
DEBUG_PRINT("turn_Y: "); DEBUG_PRINT(a);
  if(a>=0)  {
    DEBUG_PRINTLN(" Rechts");
    digitalWrite(IN3,1);
    digitalWrite(IN4,0);
    analogWrite(ENB,a);
  }
  else{
    DEBUG_PRINTLN(" Links");
    a=-a;
    digitalWrite(IN3,0);
    digitalWrite(IN4,1);
    analogWrite(ENB,a);
  }
}

void turn_off(){
  digitalWrite(IN1,0);
  digitalWrite(IN2,0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,0);
  analogWrite(ENA,0);
  analogWrite(ENB,0);
  delay(100);
}

void read_sensor(){
  Input_X = analogRead(A1);
  Input_Y = analogRead(A0);
  DEBUG_PRINT("Input_X: ");
  DEBUG_PRINT(Input_X);
  DEBUG_PRINT("      ");
  DEBUG_PRINT("Input_Y: ");
  DEBUG_PRINTLN(Input_Y);
}

int mittelWertX(int neuerWert) {
// neuen int Wert eingeben und den Mittelwert als float zurück geben
//
// Matthias Busse 9.2.2016 Version 1.0
float mittel, summe=0;
  werteHallX[zaehlerMittelWerteX] = neuerWert;
  for(int k=0; k < anzahlMittelWerte; k++) summe += werteHallX[k];
  mittel=(float) summe / anzahlMittelWerte;
  zaehlerMittelWerteX++;
  if(zaehlerMittelWerteX >= anzahlMittelWerte) zaehlerMittelWerteX=0;
  return mittel;
}

int mittelWertY(int neuerWert) {
// neuen int Wert eingeben und den Mittelwert als float zurück geben
//
// Matthias Busse 9.2.2016 Version 1.0
float mittel, summe=0;
  werteHallY[zaehlerMittelWerteY] = neuerWert;
  for(int k=0; k < anzahlMittelWerte; k++) summe += werteHallY[k];
  mittel=(float) summe / anzahlMittelWerte;
  zaehlerMittelWerteY++;
  if(zaehlerMittelWerteY >= anzahlMittelWerte) zaehlerMittelWerteY=0;
  return mittel;
}

// double pid_ctrl_X(double error) { //PID without library
//   static double X_esum;
//   static double X_ealt;
//   double y;
//
//   esum = esum + eerror;
//   y = X_Kp * error + X_Ki * x_Ta * esum + X_Kd * (error – ealt)/X_Ta;
//   ealt = error;
//
//   return y;
// }

// double pid_ctrl_Y(double e) { //PID without library
//   static double esum;
//   static double ealt;
//   double y;
//
//   esum = esum + e;
//   y = Kp * e + Ki * Ta * esum + Kd * (e – ealt)/Ta;
//   ealt = e;
//
//   return y;
// }
