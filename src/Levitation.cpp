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

const byte ENA = 10;
const byte IN1 = 9; //Enable for Out 1 X_Plus
const byte IN2 = 8; //Enable for Out 2 X_Minus
const byte IN3 = 7; //Enable for Out 3 Y_Plus
const byte IN4 = 6; //Enable for Out 4 Y_Minus
const byte ENB = 5;
const byte HALLX = A1;
const byte HALLY = A0;
// const byte BL = 2;

double Setpoint_X, Input_X, Output_X,X_plus;
double p_X = 1.0,i_X = 0,d_X = 0.05;
// double p_X = 1.0,i_X = 0.0,d_X = 0.01;


double Setpoint_Y, Input_Y, Output_Y,Y_plus;
double p_Y = 1.0,i_Y = 0,d_Y = 0.05;
// double p_Y = 1.0,i_Y = 0.0,d_Y = 0.01;


int i,on_put=1;

#define anzahlMittelWerte 3
int werteHallX[anzahlMittelWerte], zaehlerMittelWerteX=0;
int werteHallY[anzahlMittelWerte], zaehlerMittelWerteY=0;
int mittelWertX(int neuerWert);
int mittelWertY(int neuerWert);
// int h_min = 61;
// int h_max = 122;
// unsigned long time;

PID PID_X(&Input_X, &Output_X, &Setpoint_X, p_X,i_X,d_X, DIRECT);
PID PID_Y(&Input_Y, &Output_Y, &Setpoint_Y, p_Y,i_Y,d_Y, DIRECT);

// char inByte='9',nullByte,run_flag,run_dirict;
// float go_step;

void turn_X(int a);
void turn_Y(int a);
void read_sensor();
void turn_off();
int corr_sensor_y(double pwm_y);
int corr_sensor_x(double pwm_x);
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


  Setpoint_X = 560;//560;
  PID_X.SetTunings(p_X,i_X,d_X);
  PID_X.SetOutputLimits(-255,255);
  PID_X.SetSampleTime(10);
  PID_X.SetMode(AUTOMATIC);

  Setpoint_Y = 560;//560;
  PID_Y.SetTunings(p_Y,i_Y,d_Y);
  PID_Y.SetOutputLimits(-255,255);
  PID_Y.SetSampleTime(10);
  PID_Y.SetMode(AUTOMATIC);

turn_off();
}

void loop(){
  // Input_X = mittelWertX(analogRead(HALLX));
  // Input_Y = mittelWertY(analogRead(HALLY));
  Input_X = analogRead(HALLX)-corr_sensor_y(Output_X);
  Input_Y = analogRead(HALLY)-corr_sensor_y(Output_Y);

    // read_sensor();
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
  int valx = 0;
  int delaytestx=750;
  while (valx<255) {
    Output_X=valx;
    turn_X(Output_X);
    read_sensor();
    valx+=10;
    delay(delaytestx);
  }
  Output_X=0;
  turn_X(Output_X);
  DEBUG_PRINTLN("=======");
  delay(1000);
  valx = 0;
  while (valx<255) {
    Output_X=-valx;
    turn_X(Output_X);
    read_sensor();
    valx+=10;
    delay(delaytestx);
  }
  Output_X=0;
  turn_X(Output_X);
    // turn_X(255); //Oben
    // delay(10000);
    // turn_X(-255); //Unten
    // delay(10000);
  #endif

  #ifdef TESTY
  int valy = 0;
  int delaytesty=750;
  while (valy<255) {
    Output_Y=valy;
    turn_Y(Output_Y);
    // Input_Y = analogRead(A0);
    // DEBUG_PRINT("Input_Y: ");DEBUG_PRINT("\t");
    // DEBUG_PRINTLN(Input_Y);DEBUG_PRINT("\t");
    // DEBUG_PRINTLN(Input_Y-corr_sensor_y(valy));
    read_sensor();
    valy+=10;
    delay(delaytesty);
  }
  Output_Y=0;
  turn_Y(Output_Y);
  DEBUG_PRINTLN("=======");
  delay(1000);
  valy = 0;
  while (valy<255) {
    Output_Y=-valy;
    turn_Y(Output_Y);
    read_sensor();
    valy+=10;
    delay(delaytesty);
  }
  Output_Y=0;
  turn_Y(Output_Y);
    delay(5000);
  #endif

  #ifdef TESTXY
    DEBUG_PRINTLN("=====TESTXY=====");
    int delayTest=2000;
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
    delay(5000);
  #endif
}

void turn_X(int a){
  Output_X=a;
  DEBUG_PRINT("turn_X: ");DEBUG_PRINT("\t"); DEBUG_PRINT(a);DEBUG_PRINT("\t");
  if(a>=0)  {
    DEBUG_PRINT(" Oben");
    digitalWrite(IN2,0);
    digitalWrite(IN1,1);
    analogWrite(ENA,a);
  }
  else  {
    DEBUG_PRINT(" Unten");
    a=-a;
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    analogWrite(ENA,a);
  }
  // DEBUG_PRINTLN("");
}

void turn_Y(int a){
  Output_Y=a;
DEBUG_PRINT("turn_Y: ");DEBUG_PRINT("\t"); DEBUG_PRINT(a);DEBUG_PRINT("\t");
  if(a>=0)  {
    DEBUG_PRINT(" Rechts");
    digitalWrite(IN4,0);
    digitalWrite(IN3,1);
    analogWrite(ENB,a);
  }
  else{
    DEBUG_PRINT(" Links");
    a=-a;
    digitalWrite(IN3,0);
    digitalWrite(IN4,1);
    analogWrite(ENB,a);
  }
  // DEBUG_PRINTLN("");
}

void turn_off(){
  digitalWrite(IN1,0);
  digitalWrite(IN2,0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,0);
  Output_X=0;
  Output_Y=0;
  analogWrite(ENA,Output_X);
  analogWrite(ENB,Output_Y);
  delay(100);
}

void read_sensor(){
  Input_X = analogRead(A1);
  Input_Y = analogRead(A0);
  DEBUG_PRINT("\t");DEBUG_PRINT("\t");DEBUG_PRINT("\t");DEBUG_PRINT("\t");
  DEBUG_PRINT("Input_X: ");DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_X);DEBUG_PRINT("\t");
  DEBUG_PRINT("corr: ");DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_X-corr_sensor_y(Output_X));
  // DEBUG_PRINTLN("");//DEBUG_PRINT("\t");
  DEBUG_PRINT("\t");
  DEBUG_PRINT("Input_Y: ");DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_Y);DEBUG_PRINT("\t");
  DEBUG_PRINT("corr: ");DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_Y-corr_sensor_y(Output_Y));DEBUG_PRINTLN("");
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

int corr_sensor_y(double x){
  if (x==0){
    return 0;
  }
  else{
    double corrval;
    corrval=(0.000009*x*x-0.0001*x-0.08)*x; //corrval=(0.000009*x*x-0.0001*x-0.08)*x;
    // DEBUG_PRINTLN(corrval);
    // DEBUG_PRINTLN((int)corrval);
    return (int)corrval;
  }
}

int corr_sensor_x(double x){
  if (x==0){
    return 0;
  }
  else{
    double corrval;
    corrval=(0.000008*x*x-0.0001*x+0.006)*x;
    // DEBUG_PRINTLN(corrval);
    // DEBUG_PRINTLN((int)corrval);
    return (int)corrval;
  }
}
// double pid_ctrl_X(double error) { //PID without library
//   static double X_esum;
//   static double X_ealt;
//   double y;
//
//   esum = X_esum + error;
//   y = p_X * error + i_X * x_Ta * esum + d_X * (error – X_ealt)/X_Ta;
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
