#include <SPI.h>
#include <AS5047P.h>
#include <Encoder.h>
#include <rcc.h>

// breakout board with SPI [labels are on the bottom]
AS5047P as5047p(10);        //right
AS5047P as5047pb(9);         //left

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

//BUTTONS, related timing variables & flags
int buttonStatus = 1;
int buttonStop = 1;
int buttonReset = 1;
int buttonQuick = 1;
bool resetFlag = false;        //when the reset button hit to turn off lights and motors (force shut down)
bool quickRun = false;         //skip the lights, and go straight into the PID adjustment
bool buttonTriggered = false;  //if start hit to run entire simulation
bool motorsOn = false;         //for when the motors started for
bool positionControl = false;  //for when PID kicks in so when button gets pressed or after 1s running when its going into square wave mode
uint32_t deltatime = 0;        // time since main button press
uint32_t startTime = 0;        // actual start of motion after 1 second delay, also when the yellow light comes on

//LED flags
bool red = false;  //light flags to make sure they don't spam
bool yellow = false;
bool green = false;
bool lightsOn = false;  //flag for when the position PID should run

//MAIN MOTOR LOOP setup
float frequency = 1;                     //input rev/s for position control reference
float theta_last;                        //to store the initial value for each iteration
float omega_last = (360.0 * frequency);  //set to a constant value for now, change if add acceleration

float amplitude = 3.1415926535 / 2;  //90 deg sin wave amplitude, bounds of motion
int dira;                             //directionality (1=forward, 0=backward)
int dirb;
float u_pos;                           //PID error
float u_posb;
int goal;                            //storage for variable position control calls
float angle;
float angleb;
float tuning;
float angleRad;
float angleRadb;
float angleRadBound;
float angleRadBoundb;
float ref;
float angleDeg;
float angleDegb;
float angleBound;
float angleBoundb;
float pwra = 100;  //variables to power the motors
float pwrb = 100;
unsigned long cur, prev;  //timing for data analysis & PID timing
double derivative;        //variable to hold the calculation for velocity control (for later but super necessary)
double derivativeb;

//PID CONSTANTS---------------------------------------------------------------------------------------------------------------------------------------------------
// VELOCITY constants and mike's library (velocity PID outputting motor power)
double kpa = 10; //left
double kia = 0.25;
double kda = 0.0025;

double kpb = 5; //right
double kib = 0.01;
double kdb = 0.001;

// POSITION control constants (for my function that takes position and outputs velocity)
double Kpa = 2;
double Kia = 0.;  //0.02
double Kda = 0.00;

double Kpb = 2;
double Kib = 0.;  //0.02
double Kdb = 0.00;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//mike's library setup
float sigma = 0.05;
float dt = 0.001;  //determines the sample rate basically (but must stay this?)
PID_control ctrl(kpa, kia, kda, -250, 250, sigma, dt);
PID_control ctrlb(kpb, kib, kdb, -250, 250, sigma, dt);
Differentiator diff(sigma, dt);
Differentiator diffb(sigma, dt);

//timing and errors in the PID function I made
float pos_setpoint;
float totalError = 0;           //holds total past error for integral term
float totalErrorb = 0;
float previousError = 0;        //holds onto the prior error value to use in derivative term calculation
float previousErrorb = 0;
unsigned long currentTime = 0;  //used in calculating the actual time period of the control for
unsigned long previousTime = 0;
unsigned long previousTimeb = 0;
unsigned long loop_timer = 0;  // to know how long the PID has been going (after 1 sec do square wave)

void setup() {
  prev = micros();  //give dt a starting value (this gets updated later too)
  ctrl.antiWindupEnabled = true;
  //  ctrlb.antiWindupEnabled = true;
  Serial.begin(4000000);
  //while (!Serial)                 //wait for USB serial connection
  analogWriteResolution(8);  //changes the PWM resolution to 8 bits
  //motors
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  //buttons
  pinMode(ON_BUTTON, INPUT_PULLUP);     //on green wire
  pinMode(OFF_BUTTON, INPUT_PULLUP);    //off orange wire
  pinMode(RESET_BUTTON, INPUT_PULLUP);  //resets position to zero (white)
  pinMode(QUICK_BUTTON, INPUT_PULLUP);  //skips all the delays and timing straight to control(purple)
  //LEDS
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  //encoder
  SPI.begin();  //for initializing AS5047 magnetic encoder chip
  as5047p.initSPI();
  as5047pb.initSPI();
}

void loop() {
  // put your main code here, to run repeatedly:
  cur = micros();  //start a timer for the time inside the main loop

  //MAIN LOOP (data taken at each dt)
  if ((cur - prev) >= (dt * 1000000.0))  //is dt microsecond? so it turns from .01 to smol fraction of a second?? but isnt cur-prev microseconds?
  {
                                                       //if it's been less than 1 second
      // float theta = -90;                                           //theta_last + (omega_last*dt);    //main angular position setpoint [degrees]
      // float omega = omega_last;                                     //constant for now, change this later
      pullAngle();                                                   //pull the current angle and do all the conversions for the current actual and setpoint angles
      pos_setpoint=60;
      // float bound_theta = fmod(theta, 360.0);                       //wrap into a circle
      u_pos = PIDa(pos_setpoint, angleBound);                           //amount of velocity needed to travel desired distance
      u_posb = PIDb(pos_setpoint, angleBoundb);
      float vel_setpoint = u_pos;  //+omega for trajectory          //total error with feedforward trajectory, or anticipated velocity at that point in time with adjustment
      float vel_setpointb = u_posb; 
      derivative = diff.differentiate(angleRad);             //converts measured position to current speed [rev/s]*360=[deg/s]
      derivativeb = diff.differentiate(angleRadb);
      float vel_setpoint_rads = vel_setpoint * 2 * 3.14159265358979 / 360.;  //converts deg to rad
      float vel_setpoint_radsb= vel_setpointb* 2 * 3.14159265358979 / 360.;
      float u_vel = ctrl.pid(vel_setpoint_rads, derivative);            //outputs motor pwr proportion [deg/s]
      float u_velb = ctrlb.pid(vel_setpoint_radsb, derivativeb);
      setPower(-1*u_vel);                                              //apply direction, constrain, and set the motor power based on the control input
      setPowerb(-1*u_velb);
      setMotors(0, dira, pwrb, dirb);                               //use the error proportion to set the motor voltage
      // positionControl = true;                                       //green light shows when controls are active
      // theta_last = theta;
      // omega_last = omega;
      Serial.print(cur-prev);
      Serial.print(";");
      Serial.print(pos_setpoint);
      Serial.print(',');
      Serial.print(angle);
      Serial.print(',');
      Serial.print(u_pos);
      Serial.print(',');
      Serial.print(vel_setpoint);
      Serial.print(';');
      Serial.print(derivative);
      Serial.print(';');
      Serial.print(u_vel);
      Serial.println();
      prev = cur;
    }

    // positionControl = false;  //reset the flag so that the green light flashing indicates it entering or exiting a control loop 
  }

  //MAIN CONTROL LOOP-----------------------------------------------------------------------------------------------------------------------------------
  float PIDa(float setpoint, float input) {                         //setpoint=degrees, input=degrees, motor rated max 330 rpm, 5.5 rev/s or 1980 deg/sec
    currentTime = micros();                                        //gets actual delta time between each dt sampling (maybe redundant?)
    float elapsedTime = (currentTime - previousTime) / 1000000.0;  //find the time that's passed since the last control iteration and then convert to seconds
    //if(elapsedTime<=0) elapsedTime=0.001f; //prevent divide by 0 error

    //euler's approx for integrals & derivatives
    float error = setpoint - input;                           //subtract the two values fed into the function (setpoint=goal, input=sensor read)
    totalError += error * elapsedTime;                        //add onto the counter for integral calculation
    float instError = (error - previousError) / elapsedTime;  //using the current and last points find instantaneous slope

    //PID calculation
    float output = (Kpa * error) + (Kia * totalError) + (Kda * instError);

    previousError = error;  //update to hold the values from this loop before next time thru
    previousTime = currentTime;
    return output;  //set u_pos to output, or proportion of motor power to correct
  }
  float PIDb(float setpoint, float input) {                         //setpoint=degrees, input=degrees, motor rated max 330 rpm, 5.5 rev/s or 1980 deg/sec
    currentTime = micros();                                        //gets actual delta time between each dt sampling (maybe redundant?)
    float elapsedTime = (currentTime - previousTimeb) / 1000000.0;  //find the time that's passed since the last control iteration and then convert to seconds
    //if(elapsedTime<=0) elapsedTime=0.001f; //prevent divide by 0 error

    //euler's approx for integrals & derivatives
    float error = setpoint - input;                           //subtract the two values fed into the function (setpoint=goal, input=sensor read)
    totalErrorb += error * elapsedTime;                        //add onto the counter for integral calculation
    float instError = (error - previousErrorb) / elapsedTime;  //using the current and last points find instantaneous slope

    //PID calculation
    float output = (Kpb * error) + (Kib * totalError) + (Kdb * instError);

    previousErrorb = error;  //update to hold the values from this loop before next time thru
    previousTimeb = currentTime;
    return output;  //set u_pos to output, or proportion of motor power to correct
  }

  //function to refresh & change all the variables for each iteration of the control easily
  void pullAngle() {
    // SPI encoder chip read
    uint16_t angle = as5047p.readAngleRaw();  //pull the sensor angle data
    uint16_t angleb = as5047pb.readAngleRaw();

    angleDeg = angle * 360.0 / 16384.0;  //sensor read angle conversions
    angleRad = angle * 2 * 3.14159265358979 / 16384.0;
    angleBound = angleDeg - 180;                  //bounded -180 to 180 to have + and - for control motion
    angleRadBound = angleRad - 3.14159265358979;  //bounding converted to radians

    angleDegb = angleb * 360.0 / 16384.0;  //sensor read angle conversions
    angleRadb = angleb * 2 * 3.14159265358979 / 16384.0;
    angleBoundb = angleDegb - 180;                  //bounded -180 to 180 to have + and - for control motion
    angleRadBoundb = angleRadb - 3.14159265358979;  //bounding converted to radians


    tuning = goal * 2 * 3.14159265358979 / 360;  //converts deg to rad
    ref = fmod((360.0 * cur / 1000000.0), 360);  //constantly increasing circle?
    //sin(2*3.14159265359*frequency*cur/1000000.0);       //reference position outputs value btwn -1 and 1
    float refAngle = amplitude * ref;  //converts proportion to degree
  }

  //POWER SET & DIRECTION CONTROL
  int setPower(float u) {
    if (u >= 0) {
      dira = 1;  //if error is positive, set direction forward
      pwra = abs(u);
    } else {  //if error is negative, set direction backward
      dira = 0;
      pwra = abs(u);
    }

    pwra = constrain(pwra, 0, 250);  //limit so it doesnt blow motors
    return int(pwra);
  }

    int setPowerb(float u) {
    if (u >= 0) {
      dirb = 1;  //if error is positive, set direction forward
      pwrb = abs(u);
    } else {  //if error is negative, set direction backward
      dirb = 0;
      pwrb = abs(u);
    }

    pwrb = constrain(pwrb, 0, 250);  //limit so it doesnt blow motors
    return int(pwrb);
  }

  //MOTOR function to dedicate power easily with forward and backward
  void setMotors(int pwma, int dira, int pwmb, int dirb) {
    if (dira) {  // forward
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    } else {  // backward
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
    digitalWrite(RED_LED, LOW);  //reset button states
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    red = false;  //reset flags for lights/timing
    yellow = false;
    green = false;
    buttonTriggered = false;  //reset button flags
    quickRun = false;
    buttonTriggered = false;
    totalError = 0;  //reset counter to zero for a fresh control
  }