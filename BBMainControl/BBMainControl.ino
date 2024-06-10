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

//Stepper Motor Objects
AccelStepper stepper1(1, oneStep, oneDir);
AccelStepper stepper2(1, twoStep, twoDir);
AccelStepper stepper3(1, threeStep, threeDir);
MultiStepper steppers;

//Motor Variables
long tarAng[3] = { 180, 180, 180};
long tarPos[3] = { 0, 0, 0};
double speed[3] = { 0, 0, 0}, speedPrev[3], ks = 25, ka = 50; 
long curPos[3] = { 0, 0, 0};
double origPos[3] = {0, 0, 0};
double origAng = 205;

//Touch Screen Object
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0 );

//Touch screen center variables
double Xoffset = 500; 
double Yoffset = 500;  

//Inverse Kinematics Object
Robot robot(2.5, 4, 2, 3.75);

//PID variables
double error[2] = { 0, 0 }, errorPrev[2], integr[2] = { 0, 0 }, deriv[2] = { 0, 0 }, out[2];   
double xval,yval; //ball x and y position
double kp = 3E-4, ki = 0.75E-6, kd = 7.5E-3;
long timeI;  

//Other Variables
double angToStep = 17.78;
bool detected = 0; 
//bool initTap = 0; 
bool centered = false;
bool triangle[3] = {false, false, false};
bool rectangle[4] = {false, false, false, false};
long startTime = millis();

void setup() {
  Serial.begin(115200);
  motorSetup();
  delay(2000);
  
}

void motorSetup(){
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);

  for (int i = 0; i < 3; i++){
    origPos[i]=origAng*angToStep;
    tarPos[i]  = tarAng[i] * angToStep;
  }

  stepper1.setCurrentPosition(origPos[0]);
  stepper2.setCurrentPosition(origPos[1]);
  stepper3.setCurrentPosition(origPos[2]);

  stepper1.setMaxSpeed(speed[0]);
  stepper2.setMaxSpeed(speed[1]);
  stepper3.setMaxSpeed(speed[2]);
}

void loop() {
  
  centerBalance();
  trianglePattern();
  centerBalance();
  rectanglePattern();
  Serial.println("Main Loop");
}

void PID (double xVal, double yVal, double tarX, double tarY){
  for (int i = 0; i < 2; i++) {
      errorPrev[i] = error[i];                                                                    
      error[i] = (i == 0) * (Xoffset - xVal - tarX) + (i == 1) * (Yoffset - yVal - tarY);
      integr[i] += error[i] + errorPrev[i];                                                        
      deriv[i] = error[i] - errorPrev[i];                                                          
      deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];                               
      out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i];                                     
      out[i] = constrain(out[i], -0.25, 0.25);                                                     
    }
}

void moveTo(double hz, double nx, double ny){
  if (detected) {
    for (int i=0; i<3;i++){
    }
  }
  else{
    for (int i=0; i<3;i++){
      tarAng[i]= robot.angle(i, hz, 0, 0);
      tarPos[i]  = tarAng[i] * angToStep;
    }
  }

  for (int i = 0; i < 3; i++) {
      speedPrev[i] = speed[i];
      curPos[i] = (i == 0) * stepper1.currentPosition() + (i == 1) * stepper2.currentPosition() + (i == 2) * stepper3.currentPosition();
      speed[i] = abs(curPos[i] - tarPos[i]) * ks;
      speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200);
      speed[i] = constrain(speed[i], 0, 1000);
    }
  stepper1.setMaxSpeed(speed[0]);
  stepper2.setMaxSpeed(speed[1]);
  stepper3.setMaxSpeed(speed[2]);
  
  stepper1.setAcceleration(speed[0]*ka);
  stepper2.setAcceleration(speed[1]*ka);
  stepper3.setAcceleration(speed[2]*ka);
 
  stepper1.moveTo(tarPos[0]);
  stepper2.moveTo(tarPos[1]);
  stepper3.moveTo(tarPos[2]);
  
  stepper1.run();
  stepper2.run();
  stepper3.run();  
}

void balance(double hz, double xtar, double ytar){
  TSPoint p = ts.getPoint();  
  if (p.x != 0) {
    detected = 1;
    PID(p.x, p.y, xtar, ytar);
    for (int i = 0; i < 3; i++) {
      tarAng[i] = robot.angle(i, hz, out[0], out[1]); 
      tarPos[i] = tarAng[i] * angToStep; 
    }
  } else {
    delay(10);                  
    TSPoint p = ts.getPoint();  
    if (p.x == 0) {    
      detected = 0;
      for (int i = 0; i < 3; i++) {
        tarAng[i] = robot.angle(i, hz, 0, 0);
        tarPos[i] = tarAng[i] * angToStep;
      }
    }
  }

  timeI = millis();
  while ((millis() - timeI < 20)) {
    moveTo(hz, out[0], out[1]);  
  }
}

bool balanceToTarget(double xtar, double ytar, long waitTime, double kb) {
  double bound = 25*kb;
  balance(4.25, xtar, ytar);
  if (detected) {
    if ((error[0] > -bound && error[0] < bound) && (error[1] > -bound && error[1] < bound)) {
      if (millis() - startTime >= waitTime) {
        return true;
      }
    } else {
      startTime = millis();
    }
  } else {
    startTime = millis();
  }
  return false;
}

void centerBalance(){
  centered = false;
  startTime = millis();
  while (!centered){
    centered=balanceToTarget(0, 0, 2000, 1.5);
    Serial.println("Center Loop");
  }
}

void trianglePattern(){
  for (int i = 0; i < 2; i++){
    triangle [0] = false;
    startTime = millis();
    while (!triangle[0]){
      triangle[0] = balanceToTarget(0, 250, 1500, 2);
      Serial.println("Tri1 Loop");
    }
    triangle [1] = false;
    startTime = millis();
    while (!triangle[1]){
      triangle[1] = balanceToTarget(-216.5, -125, 1500, 2);
      Serial.println("Tri2 Loop");
    }
    triangle [2] = false;
    startTime = millis();
    while (!triangle[2]){
      triangle[2] = balanceToTarget(216.5, -125, 1500, 2);
      Serial.println("Tri3 Loop");
    }
  }
}

void rectanglePattern(){
  for (int i = 0; i < 3; i++){
    rectangle [0] = false;
    startTime = millis();
    while (!rectangle[0]){
      rectangle[0] = balanceToTarget(-250, 200, 1500, 2);
      Serial.println("Rect1 Loop");
    }
    rectangle [1] = false;
    startTime = millis();
    while (!rectangle[1]){
      rectangle[1] = balanceToTarget(-250, -200, 1500, 2);
      Serial.println("Rect2 Loop");
    }
    rectangle [2] = false;
    startTime = millis();
    while (!rectangle[2]){
      rectangle[2] = balanceToTarget(250, -200, 1500, 2);
      Serial.println("Rect3 Loop");
    }
    rectangle [3] = false;
    startTime = millis();
    while (!rectangle[3]){
      rectangle[3] = balanceToTarget(250, 200, 1500, 2);
      Serial.println("Rect4 Loop");
    }
  }
}
