/*%
-----------------------------------------------------------------------
% Dateiname: Levitation.cpp

% Funktion: Regelung eines fliegenden Magneten
%           Grundlage: https://www.instructables.com/id/Arduino-Air-Bonsai-Levitation/
% Autor: Luca Mazzoleni  lmazzole@hsr.ch
%
% Erstellungsdatum: 08.12.2018
% Letzte Änderung:  19.12.2018
%-----------------------------------------------------------------------
*/


#include <PID_v1.h>
#include "Arduino.h"

// #define MYDEBUG
// #define MYTIMING

#define CONTROLL
// #define TESTX
// #define TESTY
// #define TESTXY

#define BREAKERMODE
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

// PINMAPPING
#ifdef BUILD_FOR_NANO
  const byte ENA = 10;
  const byte IN1 = 9; //9Enable for Out 1 X_Plus
  const byte IN2 = 8; //8Enable for Out 2 X_Minus
  const byte IN3 = 7; //Enable for Out 3 Y_Plus
  const byte IN4 = 6; //Enable for Out 4 Y_Minus
  const byte ENB = 5;
  const byte HALLX = A1;
  const byte HALLY = A0;
#endif

#ifdef BUILD_FOR_LEONARDO
  const byte ENA = 13; //Enable X
  const byte IN1 = 10; //Enable for Out 1 X_Plus
  const byte IN2 = 11; //Enable for Out 2 X_Minus

  const byte ENB = 6; //Enable Y
  const byte IN3 = 3; //Enable for Out 3 Y_Plus
  const byte IN4 = 5; //Enable for Out 4 Y_Minus

  const byte HALL1 = A1;
  const byte HALL2 = A2;
  const byte HALL3 = A3;
  const byte HALL4 = A4;
#endif
//

const byte anzahlMittelWerte = 3;
int werteHallX[anzahlMittelWerte], zaehlerMittelWerteX=0;
int werteHallY[anzahlMittelWerte], zaehlerMittelWerteY=0;
int mittelWertX(int neuerWert);
int mittelWertY(int neuerWert);

unsigned long time_loop_start;
unsigned long time_loop_stop;

double Setpoint_X, X_plus;
static double Input_X, Output_X;
double p_X = 0.8,i_X = 0.0,d_X = 0.008;
// double p_X = 1.0,i_X = 0.0,d_X = 0.01;

double Setpoint_Y, Y_plus;
static double Input_Y, Output_Y;
double p_Y = 0.8,i_Y = 0.0,d_Y = 0.008;
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
  #ifdef MYDEBUG
    Serial.begin(4*4800);
  #endif

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  Setpoint_X = 0;//560;
  PID_X.SetTunings(p_X,i_X,d_X);
  PID_X.SetOutputLimits(-255,255);
  PID_X.SetSampleTime(5);//10
  PID_X.SetMode(AUTOMATIC);
  PID_X.SetControllerDirection(DIRECT);

  Setpoint_Y = 0;//560;
  PID_Y.SetTunings(p_Y,i_Y,d_Y);
  PID_Y.SetOutputLimits(-255,255);
  PID_Y.SetSampleTime(5);//10
  PID_Y.SetMode(AUTOMATIC);
  PID_Y.SetControllerDirection(DIRECT);//DIRECT or REVERSE

  turn_off();
}

void loop(){
  // DEBUG_PRINTLN("");
  #ifdef MYTIMING
    TIMING_PRINT("Time passed [ms]:"); TIMING_PRINTLN((double)(time_loop_stop-time_loop_start));
    time_loop_start=millis();
  #endif
  // Input_X = analogRead(HALLX);//-corr_sensor_x(Output_X);
  // Input_Y = analogRead(HALLY);//-corr_sensor_y(Output_Y);
  // Input_X = mittelWertX(Input_X);
  // Input_Y = mittelWertY(Input_Y);
  read_sensor();
  #ifdef CONTROLL //Regeln
    PID_X.Compute();
    PID_Y.Compute();
    turn_X(Output_X);
    turn_Y(Output_Y);
  #endif

  #ifdef TESTX
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("===TESTX====");
    int valx = 0;
    int delaytestx=750/2;
    while (valx<255) {
      turn_X(valx);
      delay(delaytestx);
      read_sensor();
      delay(delaytestx);
      valx+=10;
    }
    turn_X(0);
    DEBUG_PRINTLN("=======");
    delay(1000);
    valx = 0;
    while (valx<255) {
      turn_X(-valx);
      delay(delaytestx);
      read_sensor();
      delay(delaytestx);
      valx+=10;
    }
    turn_X(0);
  #endif

  #ifdef TESTY
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("===TESTY====");
    int valy = 0;
    int delaytesty=750/2;
    while (valy<255) {
      turn_Y(valy);
      delay(delaytesty);
      read_sensor();
      delay(delaytesty);
      valy+=10;
    }
    turn_Y(0);
    DEBUG_PRINTLN("=======");
    delay(1000);
    valy = 0;
    while (valy<255) {
      turn_Y(-valy);
      delay(delaytesty);
      read_sensor();
      delay(delaytesty);
      valy+=10;
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
    #ifdef MYTIMING
    time_loop_stop=millis();
  #endif
}
//=============================================================================
void turn_X(int a){
  Output_X=a;
  DEBUG_PRINT("turn_X: ");DEBUG_PRINT("\t"); DEBUG_PRINT(a);DEBUG_PRINT("\t");
  if(a>=0)  {
    DEBUG_PRINT("Rechts");
    digitalWrite(IN2,0);
    #ifdef BREAKERMODE
      analogWrite(IN1,a);
    #else
      digitalWrite(IN1,1);
      analogWrite(ENA,a);
    #endif
  }
  else  {
    DEBUG_PRINT("Links");
    a=-a;
    digitalWrite(IN1,0);
    #ifdef BREAKERMODE
      analogWrite(IN2,a);
    #else
      digitalWrite(IN2,1);
      analogWrite(ENA,a);
    #endif
  }
  DEBUG_PRINT("\t");
  // DEBUG_PRINTLN("");
}

void turn_Y(int a){
  Output_Y=a;
  DEBUG_PRINT("turn_Y: ");DEBUG_PRINT("\t"); DEBUG_PRINT(a);DEBUG_PRINT("\t");
  if(a>=0)  {
    DEBUG_PRINT("Oben");
    digitalWrite(IN4,0);
    #ifdef BREAKERMODE
      analogWrite(IN3,a);
    #else
      digitalWrite(IN3,1);
      analogWrite(ENB,a);
    #endif
  }
  else{
    DEBUG_PRINT("Unten");
    a=-a;
    digitalWrite(IN3,0);
    #ifdef BREAKERMODE
      analogWrite(IN4,a);
    #else
      digitalWrite(IN4,1);
      analogWrite(ENB,a);
    #endif
  }
  DEBUG_PRINT("\t");
  // DEBUG_PRINTLN("");
}

void turn_off(){
  digitalWrite(IN1,0);
  digitalWrite(IN2,0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,0);
  Output_X=0;
  Output_Y=0;
  analogWrite(ENA,0);
  analogWrite(ENB,0);
  delay(100);
}

void read_sensor(){
  int Input_sens_1 = analogRead(HALL1);
  int Input_sens_2 = analogRead(HALL2);
  int Input_sens_3 = analogRead(HALL3);
  int Input_sens_4 = analogRead(HALL4);
  Input_X = Input_sens_2-Input_sens_4;
  Input_Y = Input_sens_1-Input_sens_3;
    // DEBUG_PRINT("\t");DEBUG_PRINT("\t");DEBUG_PRINT("\t");
    // DEBUG_PRINT("In_1: ");DEBUG_PRINT("\t");
    // DEBUG_PRINT(Input_sens_1);DEBUG_PRINT("\t");
    // DEBUG_PRINT("In_2: ");DEBUG_PRINT("\t");
    // DEBUG_PRINT(Input_sens_2);DEBUG_PRINT("\t");
    // DEBUG_PRINT("In_3: ");DEBUG_PRINT("\t");
    // DEBUG_PRINT(Input_sens_3);DEBUG_PRINT("\t");
    // DEBUG_PRINT("In_4: ");DEBUG_PRINT("\t");
    // DEBUG_PRINT(Input_sens_4);DEBUG_PRINT("\t");
  DEBUG_PRINT("In_X: ");DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_X);DEBUG_PRINT("\t");
  DEBUG_PRINT("In_Y: ");DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_Y);DEBUG_PRINT("\t");
  DEBUG_PRINTLN("");
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
    #ifdef PROTO1
      // 2.04171635083766e-05	-0.00157012196594661	0.0549065517747006	15.6842388523726
      corrval=2.04/100000*x*x*x-0.0016*x*x+0.055*x+15.684;
    #endif
    #ifdef PROTO2
      // -6.91597888416238e-06	-4.50387411858792e-05	0.0126020514747854	-0.386905705678503
      corrval=-6.9159/1000000*x*x*x-4.5038/100000*x*x+0.01260*x;
    #endif
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
    #ifdef PROTO1
      // 1.52619475978755e-05	-0.00179062463855232	0.314493807509023	17.2766057392217
      corrval=0.3*(1.526/100000*x*x*x-0.0018*x*x+0.314*x+17.2766);
    #endif
    #ifdef PROTO2
      // 4.06833420891875e-06	-9.96144176102952e-05	0.0335466492784287	-0.316197422365213
      corrval=0.3*(4.0683/1000000*x*x*x-9.9614/100000*x*x+0.0335*x);
    #endif
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
