#include <SPI.h>
#include <AS5047P.h>
#include <Encoder.h>
#include <rcc.h>

// breakout board with SPI
AS5047P as5047p(10);

// left built-in encoder
#define ENC_CHANA 20
#define ENC_CHANB 21

// right built-in encoder
#define ENC_CHANC 22
#define ENC_CHAND 23

// motor L (left)
#define AIN1 17
#define AIN2 16
#define PWMA 14

// motor R (right)
#define BIN1 18
#define BIN2 19
#define PWMB 15

// LEDs
#define GREEN_LED 2
#define YELLOW_LED 3
#define RED_LED 4

//buttons
#define ON_BUTTON 7
#define OFF_BUTTON 8
#define RESET_BUTTON 6
#define QUICK_BUTTON 5

struct DataFrame {
  unsigned long time;
  double motorSpeedA;
  double motorSpeedB;
  double pwmA;
  double pwmB;
  double uffA;
  double uffB;
};

//(LARGE MOTORS) built in encoder variables (eventually remove)
volatile int count = 0;       //LEFT
double inputAngle = 0;        //input gearbox shaft angle (motor shaft angle)
double outputAngle = 0;       //output gearbox shaft angle
volatile int count2 = 0;      //RIGHT
double inputAngle2 = 0;
double outputAngle2 = 0;
double N = 31.5;              //gear ratio
int CPR = 28;                 //counts of relative magnetic encoder per rotation of output shaft

//BUTTONS, related timing variables & flags
int buttonStatus = 1;
int buttonStop = 1;
int buttonReset = 1;
int buttonQuick =1;
bool resetFlag = false;             //when the reset button hit to turn off lights and motors (force shut down)
bool quickRun = false;             //skip the lights, and go straight into the PID adjustment
bool buttonTriggered = false;       //if start hit to run entire simulation
bool motorsOn = false;       //for when the motors started for 
bool positionControl =false; //for when PID kicks in so when button gets pressed or after 1s running when its going into square wave mode
uint32_t deltatime = 0;             // time since main button press
uint32_t startTime = 0;             // actual start of motion after 1 second delay, also when the yellow light comes on

//LED flags 
bool red = false;                   //light flags to make sure they don't spam
bool yellow = false;
bool green = false;
bool lightsOn =false;               //flag for when the position PID should run

//MAIN MOTOR LOOP setup
float frequency = 4;                  //input rev/s for position control reference 
float amplitude = 3.1415926535/2;     //90 deg sin wave amplitude, bounds of motion
int dir;                              //directionality (1=forward, 0=backward)
int u_pos;                            //PID error
int goal;                             //storage for variable position control calls
float tuning;                         
float angleRad;
float angleRadBound;
float ref;
float angleDeg;
float angleBound;
float pwra = 100;                     //variables to power the motors
float pwrb = 100;
unsigned long cur, prev;              //timing for data analysis & PID timing
double derivative;                    //variable to hold the calculation for velocity control (for later but super necessary)
double derivativeB;

//PID CONSTANTS---------------------------------------------------------------------------------------------------------------------------------------------------
// VELOCITY constants and mike's library (velocity PID outputting motor power)
double kpa=0;
double kia=0;
double kda=0;

double kpb=0;
double kib=0;
double kdb=0;

// POSITION control constants (for my function that takes position and outputs velocity)
double Kp=30; 
double Ki=0; //0.02
double Kd=0;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//mike's library setup
float sigma = 0.05;
float dt = 0.01; //determines the sample rate basically (but must stay this?)
PID_control ctrl(kpa, kia, kda, 0, 170, sigma, dt);
PID_control ctrlb(kpb, kib, kdb, 0, 170, sigma, dt);
Differentiator diff(sigma, dt);
Differentiator diffb(sigma, dt);

//timing and errors in the PID function I made
float totalError=0;                 //holds total past error for integral term
float previousError=0;              //holds onto the prior error value to use in derivative term calculation
unsigned long currentTime=0;        //used in calculating the actual time period of the control for 
unsigned long previousTime=0; 
unsigned long loop_timer=0;         // to know how long the PID has been going (after 1 sec do square wave)

void setup() {
  prev=micros();                    //give dt a starting value (this gets updated later too)
  ctrl.antiWindupEnabled = true;
  //  ctrlb.antiWindupEnabled = true;
  Serial.begin(115200);
  //while (!Serial)                 //wait for USB serial connection

  //motors
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  //left built in encoder
  pinMode(ENC_CHANA, INPUT_PULLUP);
  pinMode(ENC_CHANB, INPUT_PULLUP);
  attachInterrupt(ENC_CHANA, chanA_ISR, CHANGE);
  attachInterrupt(ENC_CHANB, chanB_ISR, CHANGE);
  //right built in encoder
  pinMode(ENC_CHANC, INPUT_PULLUP);
  pinMode(ENC_CHAND, INPUT_PULLUP);
  attachInterrupt(ENC_CHANC, chanC_ISR, CHANGE);
  attachInterrupt(ENC_CHAND, chanD_ISR, CHANGE);
  //buttons
  pinMode(ON_BUTTON, INPUT_PULLUP);                   //on green wire
  pinMode(OFF_BUTTON, INPUT_PULLUP);                  //off orange wire
  pinMode(RESET_BUTTON, INPUT_PULLUP);                //resets position to zero (white)
  pinMode(QUICK_BUTTON, INPUT_PULLUP);                //skips all the delays and timing straight to control(purple)
  //LEDS
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  //encoder
  SPI.begin();                                        //for initializing AS5047 magnetic encoder chip
  as5047p.initSPI();
}

void loop() {
  //TIMERS AND LIGHTS
  int startValue = digitalRead(ON_BUTTON);         //last reading of the button, high(1) or low(0)
  int stopValue = digitalRead(OFF_BUTTON);         //starts high pulled to gnd
  int resetValue = digitalRead(RESET_BUTTON);      //for moving back to 0
  int quickValue = digitalRead(QUICK_BUTTON);      //for skipping the lights and timing delay going straight into the PID

  //(LARGE MOTORS) Quadrature built in: conversion from back mounted encoder angles to the motor shaft
  inputAngle = 360.0 * (count) / CPR;
  outputAngle = inputAngle / N;
  outputAngle = fmod(outputAngle, 360.0);
  inputAngle2 = 360.0 * (count2) / CPR;
  outputAngle2 = inputAngle2 / N;
  outputAngle2 = fmod(outputAngle2, 360.0);

  //for debugging and characterizing the quality of the SPI read with large motors
  float encoderDifference=angleDeg-outputAngle;

  // START BUTTON (and red light to show waiting)
  if (buttonStatus != startValue) {                 //check if change from last reading
    buttonStatus = startValue;                      //update to new state
    if (buttonStatus == 0) {                        //if the button gets pressed
      reset();                                      //clean up any old flags/settings for fresh start
      buttonTriggered = true;
      startTime = micros();                         //timer starts right at button press
      motorsOn = false;                             //make sure motors are off 
    }
  }

  // STOP BUTTON
  if (buttonStop != stopValue) {                  //if stop (orange) button pressed 
    buttonStop = stopValue;
    if (buttonStop == 0) {
      count = 0;                                  //reset relative encoders for beginning calibration before comparison (remove eventually)
      count2 = 0;
      reset();
      return;                                     // exit everything early to force stop motor
      buttonStop=false;                           //trigger flag that been done already
    }
  }

  //RESET BUTTON: moves the motor back to target to reset for next trial of data without manually twisting motor
  if (buttonReset != resetValue) {                              //if the reset button gets pressed
    buttonReset = resetValue;
    if (buttonReset == 0){
      resetFlag=true;
    }
  }
  if (resetFlag){                                               
      float target=0;                                           //manually fill in setpoint to change for where the motor aims to go [radians]
      //3.14159265358979;
      uint16_t angle = as5047p.readAngleRaw();                  //pull the sensor angle data
      float angleRad = angle * 2 * 3.14159265358979 / 16384.0;  //convert from bits to radians
      float delta_x = target - angleRad;                        //error checking constantly how far angular position actually is from goal
      
      if (abs(delta_x) <= .1) {
        setMotor(0,0,0,0);                                      //if the current angular distance from the setpoint is within abt 5 deg turn off the motors
        resetFlag=false;                                        //and stop running the loop after the motor turns off
      }

      else{
      float u_pos=PID(target, angleRadBound);                        //use position PID function to find power proportion to move back to 0 degrees (180 adjusted)
      setPower(u_pos);                                          //use power function to set pwr and direction
      setMotor(pwra,dir,pwrb,dir);                              //use motor function to give motor controller values

      Serial.println(delta_x);                                  //debug print to check angle
      Serial.print(angleRad);
      }
  }
  
  if (buttonQuick != quickValue) { //if the quick button ever gets pressed, flip the flag 
    buttonQuick=quickValue;
    if (buttonQuick ==0) {
      buttonTriggered=true;               
      lightsOn=true;                      //skip thru all the layers of the loop straight to the PID
      quickRun = true;
      startTime=micros();                   //for controller clock
    }
  }

  // GREEN LED ON during the position control loop square wave
  if (!green && positionControl ) {
    digitalWrite(GREEN_LED, HIGH);
    green = true;
  }

//TIMING--------------------------------------------------------------------------------------------------------------------------------------------------

  //if start button gets pressed
  if (buttonTriggered) {
    deltatime = micros() - startTime;                     //time elapsed since starting new run
    
    // RED LED ON at timer start
    if (!red) { 
      digitalWrite(RED_LED, HIGH);
      red = true;
    }
    //log before motor starts (during wait)
    //if (buttonTriggered && !motorsOn) {
      //float angleA=
      //float angleB=
      //Serial.print()
    //}

    // YELLOW LED ON + trigger motors to run after 1 second since start button 
      if (deltatime > 1000 && !yellow && !positionControl) {    //do it as long as yellow isn't already on or the position control is happening
      digitalWrite(YELLOW_LED, HIGH);
      analogWriteResolution(8);                                 //changes the PWM resolution to 8 bits
      yellow = true;
      lightsOn = true;
    }

    //MAIN ANALYSIS and control stuffs [radians]-----------------------------------------------------------------------------------------------------------------------------

    if (lightsOn || quickRun) {                                 //if it's been a second since timed start button press, or right when the quickrun button is pressed
      if(deltatime>=0 && deltatime < 10000000.0){                //stay within boundary until 10 seconds max
        cur=micros();                                           //start a timer for the time inside the main loop
        
        //MAIN LOOP (data taken at each dt)
        if((cur-prev)>= dt/1000000.0)                           //is dt microsecond? so it turns from .01 to smol fraction of a second?? but isnt cur-prev microseconds?
        {                   
          float loop_timer = micros()-startTime;                //current time within the controlling loop
          //CONTROL, the PID goal can be changed depending on time (mostly for square wave)
          if (loop_timer<=3000000.0){                           //if it's been less than 1 second
            int goal= 0;                                       //angular position setpoint [degrees]
            moveAngle(goal);                                    //pull the current angle and do all the conversions for the current actual and setpoint angles
            /*float u_pos=PID(tuning, angleRadBound);                  //sub either PID(tuning, angleRadBound) or (refAngle, angleBound)
            setPower(u_pos);                                    //apply direction, constrain, and set the motor power based on the control input
            setMotor(pwra,dir,pwrb,dir);*/                        //use the error proportion to set the motor voltage
            positionControl = true;                             //green light shows when controls are active
            setMotor(0,0,0,0);                                  //for square wave nice start
          }
          else {                                                //if it's been less than 1 second
            int goal= 100;                                      //new angular position setpoint [degrees]
            moveAngle(goal);                                    //pull the current angle and do all the conversions for the current actual and setpoint angles
            float u_pos=PID(tuning, angleRadBound);                  //sub either PID(tuning, angleRadBound) or (refAngle, angleBound)
            setPower(u_pos);                                    //apply direction, constrain, and set the motor power based on the control input
            setMotor(pwra,dir,pwrb,dir);                        //use the error proportion to set the motor voltage
            positionControl = true;                             //green light shows when controls are active
          }
          
          //PLOTTER
          Serial.print(deltatime);
          Serial.print(",");
          Serial.println(angleDeg);

          positionControl=false;                //reset the flag so that the green light flashing indicates it entering or exiting a control loop
          prev=cur;                             //update sampling time chunk for next run thru
        }
      }
      else{ //timeout for if it runs too long
        reset();
        motorsOn=false;
      }
    }
  }
  //delay(50);                                              // slight delay for readability if using serial monitor
}

//MAIN CONTROL LOOP-----------------------------------------------------------------------------------------------------------------------------------
float PID(float setpoint, float input){                     //setpoint=degrees, input=degrees, motor rated max 330 rpm, 5.5 rev/s or 1980 deg/sec
  currentTime=micros();                                     //gets actual delta time between each dt sampling (maybe redundant?)
  float elapsedTime=(currentTime-previousTime)/1000000.0;   //find the time that's passed since the last control iteration and then convert to seconds
  //if(elapsedTime<=0) elapsedTime=0.001f; //prevent divide by 0 error

  //euler's approx for integrals & derivatives
  float error= setpoint-input;                              //subtract the two values fed into the function (setpoint=goal, input=sensor read)
  totalError += previousError*elapsedTime;                 //add onto the counter for integral calculation
  float instError = (error-previousError)/elapsedTime;      //using the current and last points find instantaneous slope

  //PID calculation 
  float output=Kp* error + Ki* totalError + Kd * instError;

  previousError=error;                                      //update to hold the values from this loop before next time thru
  previousTime= currentTime;
  return output;                                            //set u_pos to output, or proportion of motor power to correct
}

//function to refresh & change all the variables for each iteration of the control easily
void moveAngle(int goal){
  // SPI encoder chip read
  uint16_t angle = as5047p.readAngleRaw();              //pull the sensor angle data
  
  angleDeg = angle * 360.0 / 16384.0;             //sensor read angle conversions
  angleRad = angle * 2 * 3.14159265358979 / 16384.0;
  angleBound=angleDeg-180;                        //bounded -180 to 180 to have + and - for control motion
  angleRadBound=angleRad-3.14159265358979;        //bounding converted to radians

  tuning= goal*2*3.14159265358979/360;            //converts deg to rad         
  ref = fmod((360.0*cur/1000000.0),360);          //constantly increasing circle?
  //sin(2*3.14159265359*frequency*cur/1000000.0);       //reference position outputs value btwn -1 and 1
  float refAngle= amplitude*ref;                        //converts proportion to degree
}

//POWER SET & DIRECTION CONTROL
int setPower(int u_pos) {
  if(u_pos >= 0){                                          
    dir=1;                        //if error is positive, set direction forward
    pwra=(int)u_pos;
  }
  else{                           //if error is negative, set direction backward
    dir=0;
    pwra=(int)(-u_pos);
  }

  pwra=constrain(pwra,0,170);    //limit so it doesnt blow motors
  return pwra;
}

//MOTOR function to dedicate power easily with forward and backward
void setMotor(int pwma, int dira, int pwmb, int dirb) {
  if (dira) { // forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {    // backward
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  analogWrite(PWMA, pwma);

  if (dirb) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  analogWrite(PWMB, pwmb);
}

//COMPLETE STOP to turn off whole circuit, runs when stop button pressed to reset all the lights and flags
void reset() {
  digitalWrite(RED_LED, LOW);       //reset button states
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  red = false;                      //reset flags for lights/timing
  yellow = false;
  green = false;
  buttonTriggered = false;          //reset button flags
  quickRun = false;
  buttonTriggered = false;
  totalError = 0;                   //reset counter to zero for a fresh control
}


//ABI encoder edge readings (delete if switch to SPI)
void chanA_ISR() {
  int a = digitalRead(ENC_CHANA);
  int b = digitalRead(ENC_CHANB);
  if ((a && b) || (!a && !b)) {
    count++;
  } else if ((!a && b) || (a && !b)) {
    count--;
  }
}

void chanB_ISR() {
  int a = digitalRead(ENC_CHANA);
  int b = digitalRead(ENC_CHANB);
  if ((!a && b) || (a && !b)) {
    count++;
  } else if ((a && b) || (!a && !b)) {
    count--;
  }
}

void chanC_ISR() {
  int a = digitalRead(ENC_CHANC);
  int b = digitalRead(ENC_CHAND);
  if ((a && b) || (!a && !b)) {
    count2++;
  } else {
    count2--;
  }
}

void chanD_ISR() {
  int a = digitalRead(ENC_CHANC);
  int b = digitalRead(ENC_CHAND);
  if ((!a && b) || (a && !b)) {
    count2++;
  } else {
    count2--;
  }
}
