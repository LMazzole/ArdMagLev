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


// #include <PID_v1.h>
#include "Arduino.h"

// #define MYDEBUG
// #define MYCONTROLLDEBUG
// #define MYTIMING

// #define TESTX
// #define TESTY

#define CONTROLL
// #define AUTOSHUTDOWN
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

#ifdef MYCONTROLLDEBUG
  #define CONTROLLDEBUG_PRINT(x)        Serial.print (x)
  #define CONTROLLDEBUG_PRINTDEC(x)     Serial.print (x, DEC)
  #define CONTROLLDEBUG_PRINTLN(x)      Serial.println (x)
#else
  #define CONTROLLDEBUG_PRINT(x)
  #define CONTROLLDEBUG_PRINTDEC(x)
  #define CONTROLLDEBUG_PRINTLN(x)
#endif


#ifdef MYTIMING
  #define TIMING_PRINT(x)        Serial.print (x)
  #define TIMING_PRINTDEC(x)     Serial.print (x, DEC)
  #define TIMING_PRINTLN(x)      Serial.println (x)
  unsigned long time_loop_start;
  unsigned long time_loop_stop;
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
  const byte ENA = 12;
  const byte IN1 = 3; //Enable for Out 1 X_Plus
  const byte IN2 = 5; //Enable for Out 2 X_Minus
  const byte IN3 = 10; //Enable for Out 3 Y_Plus
  const byte IN4 = 11; //Enable for Out 4 Y_Minus
  const byte ENB = 12;

  const byte HALL1 = A1;
  const byte HALL2 = A2;
  const byte HALL3 = A3;
  const byte HALL4 = A4;
#endif
//

#ifdef AUTOSHUTDOWN
  const int shutdown_treshhold = 200;
  int shutdown_val=0;
#endif

// double ratio = 0.05;
static double esum_X;
static double ealt_X;
static unsigned long timeCallPidFuncalt_X=micros();

// double Setpoint_X, X_plus;
static double Input_X, Output_X;
double p_X = 1.0,i_X = 0.00,d_X = 0.80;
// double p_X = 0.8,i_X = 0.00,d_X = 0.16;
// double p_X = 0.8,i_X = 0.00*ratio,d_X = 0.008/ratio;
// double p_X = 1.0,i_X = 0.0,d_X = 0.01;

static double esum_Y;
static double ealt_Y;
static unsigned long timeCallPidFuncalt_Y=micros();

// double Setpoint_Y, Y_plus;
static double Input_Y, Output_Y;
double p_Y = 1.0,i_Y = 0.00,d_Y = 0.80;
// double p_Y = 0.8,i_Y = 0.00,d_Y = 0.16;
// double p_Y = 0.8,i_Y = 0.00*ratio,d_Y = 0.008/ratio;
// double p_Y = 1.0,i_Y = 0.0,d_Y = 0.01;

//Functionprototyp
double pid_ctrl(  double *error,
                  double *Kp, double *Ki, double *Kd,
                  double *errorsum, double *erroralt,
                  unsigned long *timeCallPidFuncalt, int outMin, int outMax);

void turn_X(int a);
void turn_Y(int a);
void read_sensor();
void turn_off();
int corr_sensor_y(double pwm_y);
int corr_sensor_x(double pwm_x);


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

  turn_off();
}

void loop(){
  // DEBUG_PRINTLN("");
  #ifdef MYTIMING
    TIMING_PRINT("Time passed [ms]:"); TIMING_PRINTLN((double)(time_loop_stop-time_loop_start));
    time_loop_start=millis();
  #endif

  read_sensor();
  Input_X-=0.8*corr_sensor_x(Output_X);
  Input_Y-=0.8*corr_sensor_y(Output_Y);

  #ifdef CONTROLL //Regeln
    Output_X= pid_ctrl(&Input_X, &p_X, &i_X, &d_X, &esum_X, &ealt_X, &timeCallPidFuncalt_X, -255,255);
    Output_Y= pid_ctrl(&Input_Y, &p_Y, &i_Y, &d_Y, &esum_Y, &ealt_Y, &timeCallPidFuncalt_Y, -255,255);

    #ifdef AUTOSHUTDOWN
      if(abs(Output_X)>254 || abs(Output_X)>254){
          shutdown_val+=1;
          DEBUG_PRINTLN("");
          DEBUG_PRINT("shutdown_val");DEBUG_PRINTLN(shutdown_val);
      }
      else{
        shutdown_val=0;
      }
      if(shutdown_val>shutdown_treshhold){
        DEBUG_PRINTLN("");
        DEBUG_PRINTLN("TURN OFF");
        turn_off();
        delay(3000);//cooldown
        while(abs(Input_X) >10 && abs(Input_Y) >10 ){
          DEBUG_PRINTLN("WAIT FOR INITPOS");
          read_sensor();
          delay(100);
        }
        shutdown_val=0;
      }
    #endif
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
  #ifdef BREAKERMODE
    analogWrite(ENA,0);
    analogWrite(ENB,0);
  #endif
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
  // DEBUG_PRINT(Input_X);DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_X-corr_sensor_x(Output_X));DEBUG_PRINT("\t");
  DEBUG_PRINT("In_Y: ");DEBUG_PRINT("\t");
  // DEBUG_PRINT(Input_Y);DEBUG_PRINT("\t");
  DEBUG_PRINT(Input_Y-corr_sensor_y(Output_Y));DEBUG_PRINT("\t");
  DEBUG_PRINTLN("");
}

int corr_sensor_y(double x){
  // return 0;
  if (x==0){
    return 0;
  }
  else{
    double corrval;
    corrval=x*-0.29;
      // corrval=0.3*(4.0683/1000000*x*x*x-9.9614/100000*x*x+0.0335*x);
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
    corrval=x*-0.29;
      // corrval=0.3*(4.0683/1000000*x*x*x-9.9614/100000*x*x+0.0335*x);
    // DEBUG_PRINTLN(corrval);
    // DEBUG_PRINTLN((int)corrval);
    return (int)corrval;
  }
}



// double pid_ctrl_Y(double e) { //PID without library
//   static double esum;
//   static double ealt;
//   double y;

//   esum = esum + e;
//   y = Kp * e + Ki * Ta * esum + Kd * (e – ealt)/Ta;
//   ealt = e;

//   return y;
// }

double inline pid_ctrl(double *error, double *Kp, double *Ki, double *Kd, double *errorsum, double *erroralt, unsigned long *timeCallPidFuncalt, int outMin, int outMax) { //PID without library
  unsigned long timeCallPidFunc=micros();
  double Ta;
  double output;
    // CONTROLLDEBUG_PRINT("micros(): ");CONTROLLDEBUG_PRINT(micros());DEBUG_PRINT("\t");
    // CONTROLLDEBUG_PRINT("timeCallalt: ");CONTROLLDEBUG_PRINT(*timeCallPidFuncalt);CONTROLLDEBUG_PRINT("\t");
    // CONTROLLDEBUG_PRINT("timeCall: ");CONTROLLDEBUG_PRINT(timeCallPidFunc);CONTROLLDEBUG_PRINT("\t");
  Ta = (timeCallPidFunc-*timeCallPidFuncalt);
    CONTROLLDEBUG_PRINT("E: ");CONTROLLDEBUG_PRINT(*error);CONTROLLDEBUG_PRINT("\t");
    CONTROLLDEBUG_PRINT("Ealt:");CONTROLLDEBUG_PRINT(*erroralt);CONTROLLDEBUG_PRINT("\t");
    CONTROLLDEBUG_PRINT("Ta:");CONTROLLDEBUG_PRINT(Ta);CONTROLLDEBUG_PRINT("\t");
    // CONTROLLDEBUG_PRINT("Kp:");CONTROLLDEBUG_PRINT(*Kp);CONTROLLDEBUG_PRINT("\t");
    // CONTROLLDEBUG_PRINT("Kd:");CONTROLLDEBUG_PRINT(*Kd);CONTROLLDEBUG_PRINT("\t");
  *timeCallPidFuncalt=timeCallPidFunc;
  *errorsum += *error;
  output = -((*Kp)*(*error) + (*Ki)*Ta*1000000.0*(*errorsum) + (*Kd)*1000.0*((*error)-(*erroralt))/Ta);
    CONTROLLDEBUG_PRINT("KPges:");CONTROLLDEBUG_PRINT((*Kp)* (*error));CONTROLLDEBUG_PRINT("\t");
    CONTROLLDEBUG_PRINT("Kdges:");CONTROLLDEBUG_PRINT((*Kd)*1000.0*((*error)-(*erroralt))/Ta);CONTROLLDEBUG_PRINT("\t");
    CONTROLLDEBUG_PRINT("out:");CONTROLLDEBUG_PRINTLN(output);
  *erroralt = *error;
  if(output > outMax){
    output = outMax;
  }
  else if(output < outMin){
    output = outMin;
  } 
  return output;
}
