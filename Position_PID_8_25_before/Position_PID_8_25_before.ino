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

struct DataFrame {
  unsigned long time;
  double motorSpeedA;
  double motorSpeedB;
  double pwmA;
  double pwmB;
  double uffA;
  double uffB;
};

//built in encoder variables
volatile int count = 0;  //LEFT
double inputAngle = 0;   //Input gearbox shaft angle (motor shaft angle)
double outputAngle = 0;  //Output gearbox shaft angle

volatile int count2 = 0;  //RIGHT
double inputAngle2 = 0;
double outputAngle2 = 0;

double N = 31.5;  //Gear ratio
int CPR = 28;     //Counts per rotation of relative magnetic encoder

//button and timing variables
int buttonStatus = 1;
int buttonStop = 1;
bool buttonTriggered = false; //flag for if start hit
uint32_t startTime = 0;
bool motorsOn = false; //flag for when the motors started
bool positionControl =false; //flag for when PID kicks in
uint32_t deltatime = 0;

//LED flags 
uint16_t lastAngle = 0;
bool red = false;
bool yellow = false;
bool green = false;
bool lightsOn =false; //flag for when the position PID should run

//motor power setup
float frequency = 3; //input for position control reference angle
float amplitude = 3.1415926535/2;
//90; //sin wave amplitude, bounds of motion (-60 to 60)

float pwra = 100; 
float pwrb = 100;
float sigma = 0.05;
float dt = 1;
Differentiator diff(sigma, (dt / 1000000.0));
Differentiator diffb(sigma, (dt / 1000000.0));

//variables setup
unsigned long cur, prev; //timing for data analysis loop
double derivative;
double derivativeB;

//PID constants and mike's library (velocity PID outputting motor power)
//POSITION control constants
double kpa=0;
double kia=0;
double kda=0;

double kpb=0;
double kib=0;
double kdb=0;
PID_control ctrl(kpa, kia, kda, 0, 170, sigma, dt/1e6);
PID_control ctrlb(kpb, kib, kdb, 0, 170, sigma, dt/1e6);

//LARGE control constants
double Kp=9; 
double Ki=1;
double Kd=1;

float totalError=0;
float previousError=0;
unsigned long currentTime=0; //timing for PID function
unsigned long previousTime=0;


//int rep = 0;
//float a = 0;
//float b = 0;
// int buttonStatus = 1;
// int buttonStop = 1;
int t = -1;

void setup() {
  //give
  prev=micros(); //give dt a starting value
  ctrl.antiWindupEnabled = true;
  //  ctrlb.antiWindupEnabled = true;
  Serial.begin(115200);
  //while (!Serial)
    //wait for USB serial connection

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
  pinMode(ON_BUTTON, INPUT_PULLUP);  //on green wire (right)
  pinMode(OFF_BUTTON, INPUT_PULLUP);  //off orange wire (left)

  //LEDS
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);

  SPI.begin();
  as5047p.initSPI();
}

void loop() {
  int startValue = digitalRead(ON_BUTTON); //last reading of the button, high(1) or low(0)
  int stopValue = digitalRead(OFF_BUTTON); //starts high pulled to gnd

  // START BUTTON (and red light to show waiting)
  if (buttonStatus != startValue) { //check if change from last reading
    buttonStatus = startValue; //update to new state
    //if the button gets pressed
    if (buttonStatus == 0) { 
      reset();  //clean up for fresh start
      buttonTriggered = true;
      startTime = millis();  //timer starts right at button press
      motorsOn = false; //make sure motors are off 
      t = -1; 
      lastAngle = as5047p.readAngleRaw();  // capture baseline
    }
  }

  // STOP BUTTON
  if (buttonStop != stopValue) {
    buttonStop = stopValue;
    if (buttonStop == 0) {
      count = 0;  //reset relative encoders
      count2 = 0;
      t = 100000; //raise counter to break loops
      reset();
      return;  // exit early to prevent motor
    }
  }

  // SPI encoder angle
  uint16_t angle = as5047p.readAngleRaw();
  float angleDeg = angle * 360.0 / 16384.0;
  float angleRad = angle * 2 * 3.14159265358979 / 16384.0;
  float angleBound=angleDeg-180;
  float angleRadBound=angleRad-3.14159265358979;


  //Quadrature built in encoder
  inputAngle = 360.0 * (count) / CPR;
  outputAngle = inputAngle / N;
  outputAngle = fmod(outputAngle, 360.0);

  inputAngle2 = 360.0 * (count2) / CPR;
  outputAngle2 = inputAngle2 / N;
  outputAngle2 = fmod(outputAngle2, 360.0);

  float encoderDifference=angleDeg-outputAngle;


//for print
  //Serial.print("SPI read: ");
  //Serial.println(angle);
  //Serial.print("    deg: ");
  //Serial.println(angleDeg);
  //Serial.println(angleBound);
  //Serial.print("A/B: ");
  //Serial.println(outputAngle);
  //Serial.print("difference:");
  //Serial.println(encoder_difference);
  //Serial.println(" ");
  //Serial.print("Right: ");
  //Serial.println(outputAngle2);
  

  //for serial monitor
  //Serial.print(count);
  //Serial.print(" ");
  //Serial.print(count2);
  //Serial.print(" ");
  //Serial.print(angleDeg);
  //Serial.println("\t");
  //Serial.println(angleBound);
  //Serial.print(",");
  //Serial.println(outputAngle);
  //Serial.print(" ");
  //Serial.println(outputAngle2);

  if (buttonTriggered) {
    deltatime = millis() - startTime; //time elapsed since start
    

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

    // YELLOW LED ON + run motors after 1 second
      if (deltatime > 1000 && !yellow && !positionControl) {
      digitalWrite(YELLOW_LED, HIGH);
      analogWriteResolution(8); //changes the PWM resolution to 8 bits
      // Run motors
      //digitalWrite(AIN1, HIGH);
      //digitalWrite(AIN2, LOW);
      //analogWrite(PWMA, 20); //120 for smol
      //digitalWrite(BIN1, HIGH);
      //digitalWrite(BIN2, LOW);
      //analogWrite(PWMB, 20);
      t = 0; //start counter for buttons & preventing motors from running too long
      //motorsOn = true;
      yellow = true;
      lightsOn = true;
    }

    //analysis and control stuffs DEGREES FOR NOW
    if (lightsOn) {
      if(deltatime>=0 && deltatime < 10000){ //run until 10 seconds max
        cur=micros();
        //data taken at each dt
        if((cur-prev)>= dt)
        { 
          int dir;
          int pwra;
          positionControl = true;
          unsigned long now=deltatime-1000;
          float ref= sin(2*3.14159265359*frequency*((now/1000.0))); //reference position outputs value btwn -1 and 1
          float refAngle= amplitude*ref; //converts proportion to degrees
          //Serial.print("DATA HIT THIS POINT INSIDE THE LOOP");
          //angular position control that outputs a desired velocity for inner loop to match
          float u_pos= PID(refAngle, angleRadBound);

            if(u_pos >= 0){
              dir=1; //forward
              pwra=(int)u_pos;
            }
            else{
              dir=0;
              pwra=(int)(-u_pos);
            }
          //float velocitySetpointB= PID(refAngle, radians(outputAngle2));
          //prevent errors from blowing motors (170 for small)
          //u_pos=constrain(u_pos,0,1);
          //u_velb=constrain(u_velb,0,170);
          //set power of motors based on outputs
          //pwra = (int)u_pos;
          //pwrb = (int)u_velb;
          pwra=constrain(u_pos,0,10);
          //Serial.println(u_pos);
          

          
          setMotor(pwra,dir,pwrb,dir);
          //setMotor(40,1,40,1);

          prev=cur; //update sampling time chunk
          //for plotter
          //Serial.print(deltatime);
          //Serial.print(now);
          //Serial.print(",");
          Serial.println(angleDeg);
          //Serial.print(",");
          //Serial.print(outputAngle);
          //Serial.print(",");
          //Serial.println(encoderDifference);


        }
        t++; // counter for control iterations instead of time (unnecessary?)
      }
      else{ //timeout for if it runs too long
        reset();
        motorsOn=false;
        t=-1;
      }
    }

    // GREEN LED ON if encoder value has changed (first tick)
    if (!green && angle != lastAngle) {
      digitalWrite(GREEN_LED, HIGH);
      green = true;
    }
  }

  //delay(50);  // slight delay for readability
}

float PID(float setpoint, float input){ 
  //setpoint=degrees, input=degrees, motor rated max 330 rpm, 5.5 rev/s or 1980 deg/sec
  //gets actual delta time between each dt sampling (maybe redundant?)
  currentTime=millis();
  float elapsedTime=(currentTime-previousTime)/1000.0; //convert to seconds
  if(elapsedTime<=0) elapsedTime=0.001f; //prevent divide by 0 error

  //euler's approx for integrals & derivatives
  float error= setpoint-input;
  totalError += error*elapsedTime;
  float instError = (error-previousError)/elapsedTime;

  //PID calculation 
  float output=Kp* error + Ki* totalError + Kd * instError;
  //convert output from radians to rad/s
  //float velocitySetpoint=output/(elapsedTime);

  previousError=error; //update before next loop thru
  previousTime= currentTime;
  return output;
}

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


void reset() {
  //turn off whole circuit
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  //reset flags for lights/timing
  red = false;
  yellow = false;
  green = false;
  buttonTriggered = false;
}


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
