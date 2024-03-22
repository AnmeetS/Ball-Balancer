//Libraries
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

//Step Constants
#define oneStep 2
#define twoStep 3
#define threeStep 4

//Direction Constants
#define oneDir 5
#define twoDir 6
#define threeDir 7

//Setting up the classes
AccelStepper stepper1(1, oneStep, oneDir);
AccelStepper stepper2(1, twoStep, twoDir);
AccelStepper stepper3(1, threeStep, threeDir);
MultiStepper steppers;

double angToStep = 17.78;
long tarAngle[3]={ 90, 90, 90 };
long posTar[3]={ 0, 0, 0};
double speed[3] = { 0, 0, 0 }, speedPrev[3], ks = 10; 
long posCur[3]={ 0,0,0 };
long posOrig[3]={0, 0, 0};

void setup() {
  // Adding the steppers to the steppersControl instance for multi stepper control
  Serial.begin(115200);
  // Adding the steppers to the steppersControl instance for multi stepper control
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);

  for (int i = 0; i < 3; i++){
    posOrig[i]=posOrig[i]*angToStep;
  }

  stepper1.setCurrentPosition(posOrig[0]);
  stepper2.setCurrentPosition(posOrig[1]);
  stepper3.setCurrentPosition(posOrig[2]);
  
  Serial.println((String) "pos 1 "+stepper1.currentPosition()+"pos2 "+stepper2.currentPosition() +"pos3 "+ stepper3.currentPosition());

  delay(5000); 

  stepper1.setMaxSpeed(speed[0]);
  stepper2.setMaxSpeed(speed[1]);
  stepper3.setMaxSpeed(speed[2]);  

  for (int i = 0; i < 3; i++) {
      speedPrev[i] = speed[i];    
      posTar[i]  = tarAngle[i] * angToStep;                                                                                                  //sets previous speed
      posCur[i] = (i == 0) * stepper1.currentPosition() + (i == 1) * stepper2.currentPosition() + (i == 2) * stepper3.currentPosition();     //sets current position
      speed[i] = abs(posCur[i] - posTar[i]) * ks;                                                                                            //calculates the error in the current position and target position
      //speed[i] = constrain(speed[i], 0, 1000); //constrains sped from 0 to 1000                                                                                   
    }
}

void loop() {
  // put your main code here, to run repeatedly:
  moveTo(); 
}

void moveTo(){
  
  stepper1.setMaxSpeed(speed[0]);
  stepper2.setMaxSpeed(speed[1]);
  stepper3.setMaxSpeed(speed[2]);
  //sets acceleration to be proportional to speed
  stepper1.setAcceleration(speed[0]*50);
  stepper2.setAcceleration(speed[1]*50);
  stepper3.setAcceleration(speed[2]*50);
  //sets target positions
  stepper1.moveTo(posTar[0]);
  stepper2.moveTo(posTar[1]);
  stepper3.moveTo(posTar[2]);
  //runs stepper to target position (increments at most 1 step per call)
  Serial.println((String) "speed 1 "+ stepper1.speed());
  stepper1.run();
  stepper2.run();
  stepper3.run();
}