// The PID liFLary used
#include <PID_v1.h>
// The MPU6050 liFLary
#include <Wire.h>
#include <ESP8266WiFi.h> 
#include <ModbusIP_ESP8266.h>

// MPU Libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"

// Motor Libraries
#include "Servo.h"

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define OUTPUT_READABLE_YAWPITCHROLL

/*  CONNECTION
 *   
 *   
 *FL:D7(13)    FR:D8(15)
 *    |            |
 *     |         |
 *       |     |
 *         | |
 *       _______
 *      |ESP8266|
 *      |MPU6050|
 *       -------
 *         | |
 *       |     |
 *     |         |
 *    |           |
 *BL:D6(12)   BR:D5(14)

FR = FRONT RIGHT MOTOR connected to D8(15) 
BR = BACK RIGHT MOTOR connected to D5(14)
FL = FRONT LEFT MOTOR connected to D7(13)
BL = BACK LEFT MOTOR connected to D6(12)
*/
/*Motors Definitions*/

// Servo.h is not good because the frequency of the servo is \
limited to 50hz which is slow\
the frequency of analogwrite() is 490Hz for UNO and arduino chips\
for ESP8266 the frequency is 1Hz by default.\
we change the frequency manually to 500Hz on setup

Servo M[4];

// Motors  pin connections
// {FR, BR, FL, BL}
int pins[] = {13, 15, 12, 14};

// Motor speed
// attaching Motors
// Change default frequccy to 500kHz
int PWM_frequency = 30000;
int PWM_Range = 1023;   // 2000 == OFF, 0 == Maximum, 1024 id default

float motorSpeed[4];


/* Modbus Definitions */

//ModbusIP object
ModbusIP mb;

// VariaFLes
const int controlReg = 100;
unsigned int controlSignal;
bool FR, BR, BL, FL, ST, Reserved_1, Reserved_2, Reserved_3;
int TH;


// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

////  VariaFLes
double yawSetpoint, yawInput, yawOutput;
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;

/*PID Definitions*/
//  PID tarameters
double yawKi = 0.05, yawKd = 0.25, yawKp = 1;
double pitchKi = 0.05, pitchKd = 0.25, pitchKp = 1;
double rollKi = 0.05, rollKd = 0.25, rollKp = 1;

//  PID objects
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, rollKp, rollKi, rollKd, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);

// PID output offset
int PID_offset = 20;

// functions

void setup() {
  //  Initialization
  Serial.begin(38400);

   // Conect to TCP/IP
  WiFi.begin("Crazy Engineer's", "Alita:Battle Angel");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
 
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  

  // Initialize PID
  // Set PID offsets
  yawPID.SetOutputLimits(-PID_offset, PID_offset);
  rollPID.SetOutputLimits(-PID_offset, PID_offset);
  pitchPID.SetOutputLimits(-PID_offset, PID_offset);
  
  // Input Parameters
  yawInput = 0;
  rollInput = 0;
  pitchInput = 0;

//   SetPoints
  yawSetpoint = 0;
  rollSetpoint = 0;
  pitchSetpoint = 0;

   //turn the PIDs on
  yawPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);

  mb.slave();    //StaFL Modbus IP
  mb.addHreg(controlReg);

  // Motors Connections initialization
  int servoMIN = 0;
  int servoMAX = 2000;
for(int i=0; i<4; i++) {
  M[i].attach(pins[i], servoMIN, servoMAX);
}

//Set a frequency to 100kHz

//  Initializing motor speeds
  for(int i=0; i<4; i++) motorSpeed[i] = 0;
  
  controlSignal = 0;
  FR = 0;
  BR = 0;
  FL = 0;
  FL = 0;
  TH = 0;
  ST = 0;
  Reserved_1 = 0;
  Reserved_2 = 0;
  Reserved_3 = 0;

// Set PWM parameters 
  analogWriteFreq(PWM_frequency);     //  Set PWM frequency to 500Hz
  analogWriteRange(PWM_Range);        //  Set PWM range(duty cycle), 0 - 2000
}

void loop() {
  /*Modbus Loop*/
  // Modbus start
  mb.task();
  // Read the 16bit control signal word from Labview
  controlSignal = mb.Hreg(controlReg);
  //Deviding the word into two bytes
  uint8_t controlSignal_low = controlSignal & 0xff;    //Other
  uint8_t controlSignal_high = (controlSignal >> 8);   //Throttle
  
  // Modbus control
  TH = controlSignal_low;                        //Throttle (Go Up/Down)
  FR = bitRead(controlSignal_high, 0);           //Go Forward
  FL = bitRead(controlSignal_high, 1);           //Go Back
  BR = bitRead(controlSignal_high, 2);           //Go Left
  FL = bitRead(controlSignal_high, 3);           //Go Right
  ST = bitRead(controlSignal_high, 4);           //StaFL or Stop bit
  
  //  Reserved for future use
  Reserved_1 = bitRead(controlSignal_high, 5);
  Reserved_2 = bitRead(controlSignal_high, 6);
  Reserved_3 = bitRead(controlSignal_high, 7);

  
  
  /* Magic here */
  // Control movement 
  // The +/- angle deviation for all pitch roll and yaw
  // Roll Controll
  // BR&RG != 1 means eithe of the two is 1 but not both
  if ((BR == 1)&&(FL == 0)){
     rollSetpoint = rollSetpoint + 1;
     if(rollSetpoint>=10)rollSetpoint = 10;  // Remove the +ve proFLem
      }
  if ((BR == 0)&&(FL == 1)){
      rollSetpoint = rollSetpoint - 1;
      if(rollSetpoint<=-10)rollSetpoint = -10;  // Remove the -ve proFLem
      }
  if ((BR == 0)&&(FL == 0)) rollSetpoint = rollSetpoint;
  if ((BR == 1)&&(FL == 1)) rollSetpoint = rollSetpoint;

   // Pitch Controll
  // FR&FL != 1 means eithe of the two is 1 but not both
  if ((FR == 1)&&(FL == 0)){
     pitchSetpoint = pitchSetpoint + 1;
     if(pitchSetpoint>=10)pitchSetpoint = 10;  // Remove the +ve proFLem
      }
  if ((FR == 0)&&(FL == 1)){
      pitchSetpoint = pitchSetpoint - 1;
      if(pitchSetpoint<=-10)pitchSetpoint = -10;  // Remove the -ve proFLem
      }
  if ((FR == 0)&&(FL == 0)) pitchSetpoint = pitchSetpoint;
  if ((FR == 1)&&(FL == 1)) pitchSetpoint = pitchSetpoint;


  /*PID Loop*/
  yawPID.Compute();
  rollPID.Compute();
  pitchPID.Compute();


// Since there is no YAW movenent, we can not do both pitch and yaw 

/*Motors Speed Control*/
  // FR(12), BR(13), FL(14), BL(15) in that order
  // {FR, BR, FL, BL}
  int TH_Map = map(TH, 0, 255, 0, PWM_Range-PID_offset);
  // The configuration is online based, need to be investigated
  motorSpeed[0] = 0;
  motorSpeed[1] = 0;
  motorSpeed[2] = 0;
  motorSpeed[3] = 0;

 // Check motor speed if is in PWM_range
  for(int i=0; i<4; i++) {
    if(motorSpeed[i]< 0) motorSpeed[i] = 0;
    if(motorSpeed[i]> PWM_Range) motorSpeed[i] = PWM_Range;   
  }
//  Change the motor speed according the the PWM range\
2000 being OFF and 0 Completely FULL THROTTLE

  for(int i; i<4; i++){
    // The transistor works as a not gate\
    0, FET is ON, PWM_Range, FET is OFF
    
    motorSpeed[i] = PWM_Range - motorSpeed[i]; 
  }


  if(ST == 1){
    // Give Motors kick


  int total_delay = 1;
  
  for(int i=0; i<4; i++){
    total_delay = total_delay + motorSpeed[i];
  }
  
//    Serial.println(total_delay);
//  // (No motor movemen) Turn off all  FET
//  for(int i=0; i<4; i++){
//    analogWrite(pins[i], PWM_Range); 
//  }

  // turn ON FET, one by one with respective duty cycle and pin
  for(int i=0; i<4; i++){
    analogWrite(pins[i],motorSpeed[i]);
//    digitalWrite(pins[i], LOW);   // Motor movement
    delay(1000);
//    digitalWrite(pins[i], HIGH);  // No motor movement 
    analogWrite(pins[i], PWM_Range);  
  }

    
    for(int i=0; i<4; i++) {
      Serial.print("STATE: ");
      Serial.print("ARMED |");
      Serial.print("PIN:");
      Serial.print(pins[i]);
      Serial.print("  |");
      Serial.print("Speed:");
      Serial.print(motorSpeed[i]);
      Serial.println(" | ");
      }
      
  }
  
  if(ST == 0){
     for(int i=0; i<4; i++) {
      analogWrite(pins[i], motorSpeed[i]);
      Serial.print("STATE: ");
      Serial.print("OFF |");
      Serial.print("PIN:");
      Serial.print(pins[i]);
      Serial.print("  |");
      Serial.print("Speed:");
      Serial.print(motorSpeed[i]);
      Serial.println(" | ");    
    
  }
  }

  // DEBUGING
  Serial.print("INPUT : ");
  Serial.print("Yaw: ");
  Serial.print(yawInput);
  Serial.print(" | Roll: ");
  Serial.print(rollInput);
  Serial.print(" | Pitch: ");
  Serial.println(pitchInput);
  
  Serial.print("OUTPUT: ");
  Serial.print("Yaw: ");
  Serial.print(yawOutput);
  
  Serial.print(" | Roll: ");
  Serial.print(rollOutput);
  Serial.print(" | Pitch: ");
  Serial.println(pitchOutput);

  Serial.print("LABVIEW:  ");
  Serial.print("Yaw: ");
  Serial.print(yawSetpoint);
  Serial.print(" | RollSET: ");
  Serial.print(rollSetpoint);
  Serial.print(" | PitchSET: ");
  Serial.println(pitchSetpoint);

  Serial.print("MOTORS: ");
  Serial.print(" |M5: ");
  Serial.print(motorSpeed[0]);
  Serial.print(" |M6: ");
  Serial.print(motorSpeed[1]);
  Serial.print(" |M7: ");
  Serial.print(motorSpeed[2]);
  Serial.print(" |M8: ");
  Serial.println(motorSpeed[3]);
  Serial.print("SIGNALS: ");
  Serial.print(" |FL: ");
  Serial.print(FL);
  Serial.print(" |FL: ");
  Serial.print(FL);
  Serial.print(" |BR: ");
  Serial.print(BR);
  Serial.print(" |FL ");
  Serial.print(FL);
  Serial.print(" |TH ");
  Serial.println(TH_Map);
  Serial.println("_______________________________");

 
  
   // Wait to full timeStep period
  delay(200);

}

void pulsout (int pin, int duration) {
int a = map(duration, 0, 2000, 0 , 1024);
analogWrite(pin, a);
}

// Set PWM range and frequency
void PWM_calibrate(int range, int){
  
}

// Calbrate Motors to get Maxmum and Minimum


// The ppwer issues: The power is very small to run all the motors\
We aim to make each motor to run at once during the cycle to increase the torque on \
th motors (As the total current will flow thruough one motor at a time)

void motors_pwm(float duty_cycle_array, int pin_number_array, int pwm_range = 2000){
  // Four Motors, because of the transistor, they work on reverse
  // 0 for output HIGH and 3.3V for output LOW\
  hight for the period of duty_cycle...

  // Find the fraction of delay
}
