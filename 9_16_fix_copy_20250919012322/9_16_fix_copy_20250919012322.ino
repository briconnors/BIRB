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
volatile int count = 0;   //LEFT
double inputAngle = 0;    //input gearbox shaft angle (motor shaft angle)
double outputAngle = 0;   //output gearbox shaft angle
volatile int count2 = 0;  //RIGHT
double inputAngle2 = 0;
double outputAngle2 = 0;
double N = 31.5;  //gear ratio
int CPR = 28;     //counts of relative magnetic encoder per rotation of output shaft

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
int dir;                             //directionality (1=forward, 0=backward)
float u_pos;                           //PID error
int goal;                            //storage for variable position control calls
int angle;
float tuning;
float angleRad;
float angleRadBound;
float ref;
float angleDeg;
float angleBound;
float pwra = 100;  //variables to power the motors
float pwrb = 100;
unsigned long cur, prev;  //timing for data analysis & PID timing
double derivative;        //variable to hold the calculation for velocity control (for later but super necessary)
double derivativeB;

//PID CONSTANTS---------------------------------------------------------------------------------------------------------------------------------------------------
// VELOCITY constants and mike's library (velocity PID outputting motor power)
double kpa = 10;
double kia = 0;
double kda = 0;

double kpb = 0.25;
double kib = 0.1;
double kdb = 0;

// POSITION control constants (for my function that takes position and outputs velocity)
double Kp = 4;
double Ki = 0;  //0.02
double Kd = 0.00;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//mike's library setup
float sigma = 0.05;
float dt = 0.001;  //determines the sample rate basically (but must stay this?)
PID_control ctrl(kpa, kia, kda, -250, 250, sigma, dt);
PID_control ctrlb(kpb, kib, kdb, -250, 250, sigma, dt);
Differentiator diff(sigma, dt);
Differentiator diffb(sigma, dt);

//timing and errors in the PID function I made
float totalError = 0;           //holds total past error for integral term
float previousError = 0;        //holds onto the prior error value to use in derivative term calculation
unsigned long currentTime = 0;  //used in calculating the actual time period of the control for
unsigned long previousTime = 0;
unsigned long loop_timer = 0;  // to know how long the PID has been going (after 1 sec do square wave)

float theta_actual = 1.0;

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
      setAngle();                                                   //pull the current angle and do all the conversions for the current actual and setpoint angles
      // float bound_theta = fmod(theta, 360.0);                       //wrap into a circle
      u_pos = ctrl.pid(theta_actual, angleRad);                           //amount of velocity needed to travel desired distance
      float vel_setpoint = u_pos;                                         //+omega for trajectory                    //total error with feedforward trajectory, or anticipated velocity at that point in time with adjustment
      derivative = diff.differentiate(angleRad);             //converts measured position to current speed [rev/s]*360=[deg/s]
      float vel_setpoint_rads = vel_setpoint * 2 * 3.14159265358979 / 360.;  //converts deg to rad
      float u_vel = ctrlb.pid(vel_setpoint, derivative);            //outputs motor pwr proportion [deg/s]
      setPower(-1*u_vel);                                              //apply direction, constrain, and set the motor power based on the control input
      setMotor(pwra, dir, pwrb, dir);                               //use the error proportion to set the motor voltage
      // positionControl = true;                                       //green light shows when controls are active
      // theta_last = theta;
      // omega_last = omega;
      Serial.print(cur-prev);
      Serial.print(";");
      Serial.print(theta_actual);
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
  float PID(float setpoint, float input) {                         //setpoint=degrees, input=degrees, motor rated max 330 rpm, 5.5 rev/s or 1980 deg/sec
    currentTime = micros();                                        //gets actual delta time between each dt sampling (maybe redundant?)
    float elapsedTime = (currentTime - previousTime) / 1000000.0;  //find the time that's passed since the last control iteration and then convert to seconds
    //if(elapsedTime<=0) elapsedTime=0.001f; //prevent divide by 0 error

    //euler's approx for integrals & derivatives
    float error = setpoint - input;                           //subtract the two values fed into the function (setpoint=goal, input=sensor read)
    totalError += error * elapsedTime;                        //add onto the counter for integral calculation
    float instError = (error - previousError) / elapsedTime;  //using the current and last points find instantaneous slope

    //PID calculation
    float output = (Kp * error) + (Ki * totalError) + (Kd * instError);

    previousError = error;  //update to hold the values from this loop before next time thru
    previousTime = currentTime;
    return output;  //set u_pos to output, or proportion of motor power to correct
  }

  //function to refresh & change all the variables for each iteration of the control easily
  void setAngle() {
    // SPI encoder chip read
    uint16_t angle = as5047p.readAngleRaw();  //pull the sensor angle data

    angleDeg = angle * 360.0 / 16384.0;  //sensor read angle conversions
    angleRad = angle * 2 * 3.14159265358979 / 16384.0;
    angleBound = angleDeg - 180;                  //bounded -180 to 180 to have + and - for control motion
    angleRadBound = angleRad - 3.14159265358979;  //bounding converted to radians

    tuning = goal * 2 * 3.14159265358979 / 360;  //converts deg to rad
    ref = fmod((360.0 * cur / 1000000.0), 360);  //constantly increasing circle?
    //sin(2*3.14159265359*frequency*cur/1000000.0);       //reference position outputs value btwn -1 and 1
    float refAngle = amplitude * ref;  //converts proportion to degree
  }

  //POWER SET & DIRECTION CONTROL
  int setPower(float u) {
    if (u >= 0) {
      dir = 1;  //if error is positive, set direction forward
      pwra = abs(u);
    } else {  //if error is negative, set direction backward
      dir = 0;
      pwra = abs(u);
    }

    pwra = constrain(pwra, 0, 250);  //limit so it doesnt blow motors
    return int(pwra);
  }

  //MOTOR function to dedicate power easily with forward and backward
  void setMotor(int pwma, int dira, int pwmb, int dirb) {
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