#include <PID_v1.h>
#include "Arduino.h"

// #define MYDEBUG
// #define MYTIMING

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

#ifdef MYTIMING
  #define TIMING_PRINT(x)        Serial.print (x)
  #define TIMING_PRINTDEC(x)     Serial.print (x, DEC)
  #define TIMING_PRINTLN(x)      Serial.println (x)
#else
  #define TIMING_PRINT(x)
  #define TIMING_PRINTDEC(x)
  #define TIMING_PRINTLN(x)
#endif

const byte ENA = 10;
const byte IN1 = 9; //9Enable for Out 1 X_Plus
const byte IN2 = 8; //8Enable for Out 2 X_Minus
const byte IN3 = 7; //Enable for Out 3 Y_Plus
const byte IN4 = 6; //Enable for Out 4 Y_Minus
const byte ENB = 5;
const byte HALLX = A1;
const byte HALLY = A0;

const byte anzahlMittelWerte = 3;
int werteHallX[anzahlMittelWerte], zaehlerMittelWerteX=0;
int werteHallY[anzahlMittelWerte], zaehlerMittelWerteY=0;
int mittelWertX(int neuerWert);
int mittelWertY(int neuerWert);

unsigned long time_loop_start;
unsigned long time_loop_stop;

double Setpoint_X, X_plus;
static double Input_X, Output_X;
double p_X = 2,i_X = 0,d_X = 0.005;
// double p_X = 1.0,i_X = 0.0,d_X = 0.01;

double Setpoint_Y, Y_plus;
static double Input_Y, Output_Y;
double p_Y = 2,i_Y = 0,d_Y = 0.005;
// double p_Y = 1.0,i_Y = 0.0,d_Y = 0.01;
PID PID_X(&Input_X, &Output_X, &Setpoint_X, p_X,i_X,d_X, DIRECT);
PID PID_Y(&Input_Y, &Output_Y, &Setpoint_Y, p_Y,i_Y,d_Y, DIRECT);

void turn_X(int a);
void turn_Y(int a);
void read_sensor();
void turn_off();
int corr_sensor_y(double pwm_y);
int corr_sensor_x(double pwm_x);
double pid_ctrl_X(double error);
double pid_ctrl_Y(double error);


void setup(){
  Serial.begin(4800);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  digitalWrite(IN1,0);
  digitalWrite(IN2,0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,0);
  analogWrite(ENA,0);
  analogWrite(ENB,0);


  Setpoint_X = 560;//560;
  PID_X.SetTunings(p_X,i_X,d_X);
  PID_X.SetOutputLimits(-255,255);
  PID_X.SetSampleTime(10);//10
  PID_X.SetMode(AUTOMATIC);
  PID_X.SetControllerDirection(REVERSE);

  Setpoint_Y = 560;//560;
  PID_Y.SetTunings(p_Y,i_Y,d_Y);
  PID_Y.SetOutputLimits(-255,255);
  PID_Y.SetSampleTime(10);//10
  PID_Y.SetMode(AUTOMATIC);
  PID_Y.SetControllerDirection(REVERSE);//DIRECT or REVERSE

turn_off();
}

void loop(){
  // DEBUG_PRINTLN("");
  TIMING_PRINT("Time passed [ms]:"); TIMING_PRINTLN((double)(time_loop_stop-time_loop_start));
  time_loop_start=millis();
  Input_X = analogRead(HALLX)-corr_sensor_y(Output_X);
  Input_Y = analogRead(HALLY)-corr_sensor_y(Output_Y);
  // Input_X = mittelWertX(Input_X);
  // Input_Y = mittelWertY(Input_Y);
    // read_sensor();
  #ifdef CONTROLL //Regeln
  read_sensor();
    //Compute correction Value
    //===PID====
    // time_loop_stop=millis();
    // TIMING_PRINT("Time passed [ms]:"); TIMING_PRINTLN((double)(time_loop_stop-time_loop_start));
    PID_X.Compute();
    PID_Y.Compute();
    // time_loop_start=millis();
    turn_X(Output_X);
    turn_Y(Output_Y);
  #endif

  #ifdef TESTX
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("===TESTX====");
    int valx = 0;
    int delaytestx=750;
    while (valx<255) {
      turn_X(valx);
      read_sensor();
      valx+=10;
      delay(delaytestx);
    }
    turn_X(0);
    DEBUG_PRINTLN("=======");
    delay(1000);
    valx = 0;
    while (valx<255) {
      turn_X(-valx);
      read_sensor();
      valx+=10;
      delay(delaytestx);
    }
    turn_X(0);
    // delay(10000);
  #endif

  #ifdef TESTY
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("===TESTY====");
    int valy = 0;
    int delaytesty=750;
    while (valy<255) {
      turn_Y(valy);
      read_sensor();
      valy+=10;
      delay(delaytesty);
    }
    turn_Y(0);
    DEBUG_PRINTLN("=======");
    delay(1000);
    valy = 0;
    while (valy<255) {
      turn_Y(-valy);
      read_sensor();
      valy+=10;
      delay(delaytesty);
    }
    turn_Y(0);
    delay(5000);
  #endif

  #ifdef TESTXY
    DEBUG_PRINTLN("=====TESTXY=====");
    int delayTest=2000;
    turn_X(255); //Oben
    delay(delayTest);
    read_sensor();
    turn_off();
    turn_Y(255); //Rechts
    delay(delayTest);
    read_sensor();
    turn_off();
    turn_X(-255); //Unten
    delay(delayTest);
    read_sensor();
    turn_off();
    turn_Y(-255); //Links
    delay(delayTest);
    read_sensor();
    turn_off();
    read_sensor();
    delay(5000);
  #endif

  time_loop_stop=millis();
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
  // DEBUG_PRINT("\t");
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
  // DEBUG_PRINT("\t");
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
  DEBUG_PRINT("\t");
  // DEBUG_PRINT("\t");DEBUG_PRINT("\t");DEBUG_PRINT("\t");
  DEBUG_PRINT("Input_X: ");DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_X);DEBUG_PRINT("\t");
  DEBUG_PRINT("corr: ");DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_X-corr_sensor_x(Output_X));
  // DEBUG_PRINTLN("");//DEBUG_PRINT("\t");
  DEBUG_PRINT("\t");
  DEBUG_PRINT("Input_Y: ");DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_Y);DEBUG_PRINT("\t");
  DEBUG_PRINT("corr: ");DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_Y-corr_sensor_y(Output_Y));DEBUG_PRINTLN("");
}

int mittelWertX(int neuerWert) {
// neuen int Wert eingeben und den Mittelwert als fSloat zurück geben
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
  // return 0;
  if (x==0){
    return 0;
  }
  else{
    double corrval;
    // corrval= 0.01*x*x*x - 0.88*x*x + 22*x;// -176;
    corrval=1.13/100000*x*x*x+9.70/1000000*x*x-0.08*x+0.113;
    // corrval= (0.1132*x - 0.0828)*x*x;
    // corrval=(0.000009*x*x-0.0001*x-0.08)*x; //corrval=(0.000009*x*x-0.0001*x-0.08)*x;
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
    corrval=1.13/100000*x*x*x+9.70/1000000*x*x-0.08*x+0.113;
    // corrval=(0.000008*x*x-0.0001*x+0.006)*x;
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
