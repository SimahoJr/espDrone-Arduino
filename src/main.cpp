#include <Arduino.h>
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
int pins[] = {14, 12, 13, 15};

// Motor speed
// attaching Motors
// Change default frequccy to 500kHz
int PWM_frequency = 30000;
int PWM_Range = 1023;   // 2000 == OFF, 0 == Maximum, 1024 id default

float motorSpeed[4];
float duty_0, duty_1, duty_2, duty_3, duty_total;


/* Modbus Definitions */

//ModbusIP object
ModbusIP mb;

// VariaFLes
const int offsetReg = 100;
unsigned int controlSignal;
bool FR, BR, BL, FL, ST, Reserved_1, Reserved_2, Reserved_3;
int TH;

/*MPU Definitions*/
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
VectorFloat gravity;    // [x, y, z]            gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
    
int SCL_PIN = 5;   //  Pin D0
int SDA_PIN = 4;   //  Pin D1

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

////  VariaFLes
double yawSetpoint, yawInput, yawOutput;
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;
double throttle;
int on_off;
float pid_yaw, pid_pitch, pid_roll;

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
int PID_offset = 45;

// functions
#include <Ticker.h>
Ticker motor0;
Ticker motor1;
Ticker motor2;
Ticker motor3;
Ticker pwm;
Ticker main_task;
void main_window();


int t = 0;
int t0 = 1;
int t1 = 2;
int t2 = 3;
int t3 = 4;
int period = 100;


void pwm_phase_0(){
  // if(t == 0){
  digitalWrite(pins[3], 0);
  digitalWrite(pins[2], 0);
  digitalWrite(pins[1], 0);
  digitalWrite(pins[0], 1);
  Serial.println("  phase_0");
  // t = 1;
  // }
  // else{
  //   digitalWrite(pins[3], 0);
  //   digitalWrite(pins[2], 0);
  //   digitalWrite(pins[1], 0);
  //   digitalWrite(pins[0], 0);
  // }
  // Serial.print(t0);
  // Serial.println("  phase_0");
  // t++;
  
}

void pwm_phase_1(){
  // if(t == 1){
  digitalWrite(pins[3], 0);
  digitalWrite(pins[2], 0);
  digitalWrite(pins[1], 1);
  digitalWrite(pins[0], 0);
  Serial.println("  phase_1");
  // t = 2;
  // }
  // // else{
  //   digitalWrite(pins[3], 0);
  //   digitalWrite(pins[2], 0);
  //   digitalWrite(pins[1], 0);
  //   digitalWrite(pins[0], 0);
  // }
  // // Serial.print(t1);
  // // Serial.println("  phase_1");
  // t++;

}

void pwm_phase_2(){
  // if(t == 2){
  digitalWrite(pins[3], 0);
  digitalWrite(pins[2], 1);
  digitalWrite(pins[1], 0);
  digitalWrite(pins[0], 0);
  Serial.println("  phase_2");
  // t = 3;
  // }
  // else{
  //   digitalWrite(pins[3], 0);
  //   digitalWrite(pins[2], 0);
  //   digitalWrite(pins[1], 0);
  //   digitalWrite(pins[0], 0);
  // }
  // // Serial.print(t2);
  // // Serial.println("  phase_2");
  // t++;

}

void pwm_phase_3(){
  // if(t == 3){
  digitalWrite(pins[3], 1);
  digitalWrite(pins[2], 0);
  digitalWrite(pins[1], 0);
  digitalWrite(pins[0], 0);
  Serial.println("  phase_3");
  // t = 0;
  // }
  // else{
  //   digitalWrite(pins[3], 0);
  //   digitalWrite(pins[2], 0);
  //   digitalWrite(pins[1], 0);
  //   digitalWrite(pins[0], 0);
  // }
  // // Serial.print(t3);
  // // Serial.println("  phase_3");
  // t++;
  
}

void pwm_motors();


void setup() {

    //  Initialization
    Serial.begin(38400);
    // Conect to TCP/IP
    WiFi.softAP("Crazy Engineer's Drone", "Alita:Battle Angel");
    Serial.print("Access Point \"");
    Serial.println("\" started");

    Serial.print("IP address:\t");
    Serial.println(WiFi.softAPIP());         // Send the IP address of the ESP8266 to the computer

    Serial.println(WiFi.localIP());

    // Initialize MPU6050
      // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // initialize MPU device
    mpu.initialize();
    
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    // wait for ready
    //  while (Serial.available() && Serial.read()); // empty buffer

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // reset offsets
    mpu.setXGyroOffset(-25);
    mpu.setYGyroOffset(45);
    mpu.setZGyroOffset(-45);
    mpu.setZAccelOffset(943); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
          }

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
    pid_yaw = 0;
    pid_pitch = 0;
    pid_roll = 0;
    yawPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);
    pitchPID.SetMode(AUTOMATIC);

    for(int i=0; i<=200; i++){
    mb.addHreg(offsetReg + i, 0, 1);
    }
    mb.slave();    //StaFL Modbus IP
    for(int i = 0; i<4; i++) pinMode(pins[i], OUTPUT);

    pwm.attach_ms(period, pwm_motors);
    
  }

void loop() {
    /*Modbus Loop*/
    // Modbus start
    mb.task();

    /*MPU Loop*/
    // Read normalized values
    // if programming failed, don't try to do anything
    // if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {// Get the Latest packet 
                // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // Assign  Pitch, Roll and Yaw
            yawInput = ypr[0] * 180/3.1415926535;
            pitchInput = ypr[1] * 180/3.1415926535;
            rollInput = ypr[2] * 180/3.1415926535;
    }

    /*PID SETPOINTS AND PARAMETERS
    --->from the app
    sent_list = [int(pid_values[i] * 100) for i in range(0, len(pid_values))]
    sent_list.extend([self.left_or_right, self.forward_or_back, int(self.throttle)])

                for m in range(0, len(sent_list)):
                c.write_single_register(110+m, sent_list[m])
    <---

    */
    // Devide the PID parameters by 100 as they were multiplied before send
    yawKi = mb.Hreg(110)/100;
    yawKd = mb.Hreg(111)/100;
    yawKp = mb.Hreg(112)/100;
    pitchKi = mb.Hreg(113)/100;
    pitchKd = mb.Hreg(114)/100; 
    pitchKp = mb.Hreg(115)/100;
    rollKi = mb.Hreg(116)/100;
    rollKd = mb.Hreg(117)/100;
    rollKp = mb.Hreg(118)/100;
    // yawSetpoint to be updated latter
    yawSetpoint = 0;
    // Check the negative inputs from Modbus
    pitchSetpoint = mb.Hreg(120);
    if(pitchSetpoint>9900) pitchSetpoint = 9900 - pitchSetpoint;
    rollSetpoint = mb.Hreg(119);
    if(rollSetpoint>9900) rollSetpoint = 9900 - rollSetpoint;
    throttle = mb.Hreg(121);
    on_off = mb.Hreg(122);

    /*PID Loop*/
    yawPID.Compute();
    rollPID.Compute();
    pitchPID.Compute();

     /*PID OUTPUTS
            -Remeber to control angles and not PWM
            -Create the return of duty cycles
            -Remember to constrain the values and not to map
            -Add PID values to modbus to send the values should be the Angle values
            -Find a way to send negative numbers
            -Find a way to send Float Numbers
            */
    
    motorSpeed[0] = throttle + rollOutput - pitchOutput - yawOutput;
    motorSpeed[1] = throttle - rollOutput - pitchOutput + yawOutput;
    motorSpeed[2] = throttle - rollOutput + pitchOutput - yawOutput;
    motorSpeed[3] = throttle + rollOutput + pitchOutput + yawOutput;

    pid_yaw = yawInput + yawOutput;
    pid_pitch = pitchInput + pitchOutput;
    pid_roll = rollInput + rollOutput;

    duty_total = motorSpeed[0] + motorSpeed[1] + motorSpeed[2] + motorSpeed[3];

    duty_0 = (motorSpeed[0]/ duty_total) * period;
    duty_1 = (motorSpeed[1]/ duty_total) * period;
    duty_2 = (motorSpeed[2]/ duty_total) * period;
    duty_3 = (motorSpeed[3]/ duty_total) * period;

    // Sending to App
    if(yawInput < 0) yawInput = 9900 - yawInput; 
    if(pitchInput < 0) pitchInput = 9900 - pitchInput;
    if(rollInput < 0) rollInput = 9900 - rollInput;
    if(pid_yaw < 0)  pid_yaw = 9900 - pid_yaw;
    if(pid_pitch < 0) pid_pitch = 9900 - pid_pitch;
    if(pid_roll < 0) pid_roll = 9900 - pid_roll;

    mb.Hreg(108, yawInput);
    mb.Hreg(107, pitchInput);
    mb.Hreg(106, rollInput);
    mb.Hreg(105, pid_yaw);
    mb.Hreg(104, pid_pitch);
    mb.Hreg(103, pid_roll);
    mb.Hreg(102, yawSetpoint);
    mb.Hreg(101, pitchSetpoint);
    mb.Hreg(100, rollSetpoint);

    mb.Hreg(109, 0);

    Serial.println(yawInput);
    Serial.println(pitchInput);
    Serial.println(rollInput);
    Serial.println(pid_yaw);
    Serial.println(pid_pitch);
    Serial.println(pid_roll);
    
  }

  void mainWindow(){
    
  }


  void pwm_motors(){
  motor0.once_ms(duty_0, pwm_phase_0);
  motor1.once_ms(duty_1 + duty_0, pwm_phase_1);
  motor2.once_ms(duty_2 + duty_1 + duty_0, pwm_phase_2);
  motor3.once_ms(duty_3 + duty_2 + duty_1 + duty_0, pwm_phase_3);
  }
  