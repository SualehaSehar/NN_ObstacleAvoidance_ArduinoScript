#include <Neurona.h>
#include "model.h"

// Motor A
const int LeftMotorBackward = 2;
const int LeftMotorForward = 3;
const int enableMotorA = 10;

// Motor B
const int RightMotorForward = 4;
const int RightMotorBackward = 5;
const int enableMotorB = 11;

// Front Sensor
const int frontEchoPin = 8;
const int frontTriggerPin = 9;

// left Sensor
const int leftEchoPin = 6;
const int leftTriggerPin = 7;

// Right Sensor 
const int rightEchoPin = 12;
const int rightTriggerPin = 13 ;

float frontDuration, frontDistanceCm, leftDuration, leftDistanceCm, rightDuration, rightDistanceCm;
#define DEFAULT_SPEED 120
//# define backwardEchoPin A0
//# define backwardTriggerPin A1

MLP mlp(NET_INPUTS, NET_OUTPUTS, layerSizes, MLP::LOGISTIC, initW, true);
void setup() {
  pinMode(frontTriggerPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(leftTriggerPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(rightTriggerPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  // motors
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(enableMotorA, OUTPUT);
  pinMode(enableMotorB, OUTPUT);
  Serial.begin(19200);
}
float sensL = 0.0;
float sensF = 0.0;
float sensR = 0.0;

void loop() {
  reset();
//int val = (sens1 << 3) | (sens2 << 2) | (sens3 << 1) | sens4;
//int val = (sensL << 2) | (sensF << 1) | sensR ;
//Serial.print("value: ");
//Serial.println(val);


// Fill the input buffer
  netInput[1] = sensL;
  netInput[2] = sensF;
  netInput[3] = sensR;

// Run the inferencing
  int mlpClass = mlp.getActivation(netInput);
  Serial.print("Index: ");
  Serial.println(mlpClass);
//  mlpClass = Class[index];

switch (mlpClass) {
    // no obstacles 000
    case 0:
      straightForward();
      break;
    // obstacle to the right 001
    case 1:
      turnLeft(30);
      break;
    // obstacle in front  010
    case 2:
      turnLeft(90);
      break;
    // obstacle to the left  100
    case 3:
      turnRight(30);
      break;
    // obstacles left and right 101
    case 4:
      turnLeft(180);
      break
  }
  delay(100); 
}

void reset()
{  
  if(checkFrontDistance()<25){
    sensF = 1.0;
    } 
  else{sensF=0.0;}
  if(checkLeftDistance()<25){
    sensL = 1.0;
    } 
  else{sensL=0.0;}
  if(checkRightDistance()<25){
    sensR = 1.0;
    } 
  else{sensR=0.0;} 
}


float checkFrontDistance() {
  digitalWrite(frontTriggerPin, LOW);  
  delayMicroseconds(4);
  digitalWrite(frontTriggerPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(frontTriggerPin, LOW);
  frontDuration = pulseIn(frontEchoPin, HIGH);  
  frontDistanceCm = frontDuration * 10 / 292 / 2;  
  Serial.print("Distance: ");
  Serial.print(frontDistanceCm);
  Serial.println(" cm");
  return frontDistanceCm;
}
float checkLeftDistance() {
  digitalWrite(leftTriggerPin, LOW);  
  delayMicroseconds(4);
  digitalWrite(leftTriggerPin, HIGH);  
  delayMicroseconds(10);
  digitalWrite(leftTriggerPin, LOW);
  leftDuration = pulseIn(leftEchoPin, HIGH);  
  leftDistanceCm = leftDuration * 10 / 292 / 2; 
  Serial.print("Left distance: ");
  Serial.print(leftDistanceCm);
  Serial.println(" cm");
  return leftDistanceCm;
}
float checkRightDistance() {
  digitalWrite(rightTriggerPin, LOW);  
  delayMicroseconds(4);
  digitalWrite(rightTriggerPin, HIGH);  
  delayMicroseconds(10);
  digitalWrite(rightTriggerPin, LOW);
  rightDuration = pulseIn(rightEchoPin, HIGH);  
  rightDistanceCm = rightDuration * 10 / 292 / 2;  
  Serial.print("Right distance: ");
  Serial.print(rightDistanceCm);
  Serial.println(" cm");
  return rightDistanceCm;
}
void setLeftForward() {
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
}

void setRightForward() {
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
}

void setBothForward() {
  setLeftForward();
  setRightForward();
}

void setLeftBackward() {
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
}

void setRightBackward() {
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
}

void setBothBackward() {
  setRightBackward();
  setLeftBackward();
}

void setLeftSpeed(int speed) {
  analogWrite(enableMotorA, speed);
}

void setRightSpeed(int speed) {
  analogWrite(enableMotorB, speed);
}

void setBothSpeeds(int speed) {
  setLeftSpeed(speed);
  setRightSpeed(speed);
}

// sets direction of both motors to forward and sets both speeds
// to default speed
void straightForward() {
  setBothForward();
  setBothSpeeds(DEFAULT_SPEED);
}

// makes a turn by stopping one motor
// accepts an int, 0 or 1 for left or right turn respectively
void turnLeft(int deg) {
  setBothSpeeds(0);
  delay(100);
  setLeftBackward(); // set left motor to run backward
  setBothSpeeds(DEFAULT_SPEED); // set speeds to 1/2 default
  delay(10 * deg); // allow time for the bot to turn
                   // turn time is approx 5ms per degree
  straightForward(); // resume driving forward at default speed
}

void turnRight(int deg) {
  setBothSpeeds(0);
  delay(100);
  setRightBackward(); // set right motor to run backward
  setBothSpeeds(DEFAULT_SPEED); // set speeds to 1/2 default
  delay(10 * deg); // allow time for the bot to turn
                      // turn time is approx 5ms per degree
  straightForward(); // resume driving forward at default speed
}
