//Libraries
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include <TouchScreen.h>
#include <BBInverseKinematics.h>

//Step Constants
#define oneStep 2
#define twoStep 3
#define threeStep 4

//Direction Constants
#define oneDir 5
#define twoDir 6
#define threeDir 7

//Stepper Motors
AccelStepper stepper1(1, oneStep, oneDir);
AccelStepper stepper2(1, twoStep, twoDir);
AccelStepper stepper3(1, threeStep, threeDir);
MultiStepper steppers;

// Touch Screen class
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0 ); //touch screen pins (XGND, YGND, X5V, Y5V)

// Robot IK class
Robot robot(2.5, 4, 2, 3.75);

// Touch screen important variables
double Xoffset = 500;         //X offset for the center position of the touchpad
double Yoffset = 500;         //Y offset for the center position of the touchpad

//PID variables
double xval,yval;
double kp = 3E-4, ki = 1E-6, kd = 8.5E-3;                                                       //PID constants
double error[2] = { 0, 0 }, errorPrev[2], integr[2] = { 0, 0 }, deriv[2] = { 0, 0 }, out[2];  //PID terms for X and Y directions
long timeI;                                                                           //variables to capture initial times


//
double angToStep = 17.78;     // Angle to step conversion factor (steps per degree) for 16 microsteps or 3200 steps/rev
bool detected = 0;            // This value is 1 when the ball is detected and the value is 0 when the ball in not detected
bool initTap = 0;             // This value is default to 0 everytime the arduino is reset and will turn 1 when the intial tap has been counted


//Motor Variables
long tarAng[3] = { 180, 180, 180};
long tarPos[3] = { 0, 0, 0};
double speed[3] = { 0, 0, 0}, speedPrev[3], ks = 50, ka = 100; 
long curPos[3] = { 0, 0, 0};
double origPos[3] = {0, 0, 0};
double origAng = 205;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  motorSetup();
  
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:

  balance(4.25);
}

void motorSetup(){
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);

  for (int i = 0; i < 3; i++){
    origPos[i]=origAng*angToStep;
    tarPos[i]  = tarAng[i] * angToStep; //sets the target position in a value that the stepper motor understands instead of angle
  }

  stepper1.setCurrentPosition(origPos[0]);
  stepper2.setCurrentPosition(origPos[1]);
  stepper3.setCurrentPosition(origPos[2]);

  stepper1.setMaxSpeed(speed[0]);
  stepper2.setMaxSpeed(speed[1]);
  stepper3.setMaxSpeed(speed[2]);
}

void PID (double xVal, double yVal, double setpointX, double setpointY){
  for (int i = 0; i < 2; i++) {
      errorPrev[i] = error[i];                                                                     //sets previous error
      error[i] = (i == 0) * (Xoffset - xVal - setpointX) + (i == 1) * (Yoffset - yVal - setpointY);  //sets error aka X or Y ball position
      integr[i] += error[i] + errorPrev[i];                                                        //calculates the integral of the error (proportional but not equal to the true integral of the error)
      deriv[i] = error[i] - errorPrev[i];                                                          //calcuates the derivative of the error (proportional but not equal to the true derivative of the error)
      deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];                                //checks if the derivative is a real number or infinite
      out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i];                                     //sets output
      out[i] = constrain(out[i], -0.25, 0.25);                                                     //contrains output to have a magnitude of 0.25
    }
}

void balance(double hz){
  TSPoint p = ts.getPoint();  //measure X and Y positions
  //Serial.println((String) "X OUT = " + (p.x) + "   Y OUT = " + (p.y));  //print X and Y outputs
  if (p.x != 0) {
    detected = 1;
    Serial.println("BALL DETECTED");
    PID(p.x, p.y, 0, 0);
    //converts px from a range of 0 to 1000 to a value from 0 to 180
    for (int i = 0; i < 3; i++){
      tarAng[i]= robot.angle(i, hz, out[0], out[1]); // determines target angle
      tarPos[i]  = tarAng[i] * angToStep; //sets the target position in a value that the stepper motor understands instead of angle
    }
    
  }
  else {
    //double check that there is no ball
    delay(10);                  //10 millis delay before another reading
    TSPoint p = ts.getPoint();  //measure X and Y positions again to confirm no ball
    if (p.x == 0) {             //if the ball is still not detected
      Serial.println("BALL NOT DETECTED");
      detected = 0;
    }
  }

  timeI = millis();
  while (millis() - timeI < 20) {
    moveTo(hz, out[0], out[1]);  //moves the platfrom
  }
}

void moveTo(double hz, double nx, double ny){
  if (detected) {
    for (int i=0; i<3;i++){
      //tarPos[i]= robot.angle(i, hz, 0, 0) * angToStep;
    }
  }
  else{
    for (int i=0; i<3;i++){
      tarAng[i]= robot.angle(i, hz, 0, 0);
      tarPos[i]  = tarAng[i] * angToStep;
      //tarPos[i]= (robot.angle(i, hz, 0, 0)) * angToStep);
    }
  }


  for (int i = 0; i < 3; i++) {
      speedPrev[i] = speed[i];                                                                                                              //sets previous speed
      curPos[i] = (i == 0) * stepper1.currentPosition() + (i == 1) * stepper2.currentPosition() + (i == 2) * stepper3.currentPosition();    //sets current position
      speed[i] = abs(curPos[i] - tarPos[i]) * ks;                                                                                           //calculates the error in the current position and target position
      speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200);                                                               //filters speed by preventing it from beign over 100 away from last speed
      speed[i] = constrain(speed[i], 0, 1000);                                                                                              //constrains sped from 0 to 1000
    }
  //run motors
  stepper1.setMaxSpeed(speed[0]);
  stepper2.setMaxSpeed(speed[1]);
  stepper3.setMaxSpeed(speed[2]);
  //sets acceleration to be proportional to speed
  stepper1.setAcceleration(speed[0]*ka);
  stepper2.setAcceleration(speed[1]*ka);
  stepper3.setAcceleration(speed[2]*ka);
  //sets target positions
  stepper1.moveTo(tarPos[0]);
  stepper2.moveTo(tarPos[1]);
  stepper3.moveTo(tarPos[2]);
  //runs stepper to target position (increments at most 1 step per call)
  stepper1.run();
  stepper2.run();
  stepper3.run();

  //Calculate motor speeds
  
}
