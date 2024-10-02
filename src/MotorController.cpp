#include "MotorController.h"

MotorController::MotorController(Motor& leftMotor, Motor& rightMotor, float leftMotorCalibration, float rightMotorCalibration)
  : leftMotor(leftMotor), rightMotor(rightMotor), leftMotorCalibration(leftMotorCalibration), rightMotorCalibration(rightMotorCalibration) {}

void MotorController::setup(int maxVelocity, int minVelocity) {
  this->maxVelocity = maxVelocity;
  this->minVelocity = minVelocity;

  leftMotor.setup();
  rightMotor.setup();
}

void MotorController::moveForward(int speed) {
  speed = checkSpeed(speed);

  int leftSpeed = speed * leftMotorCalibration;
  int rightSpeed = speed * rightMotorCalibration;
  leftMotor.velocity(leftSpeed);
  rightMotor.velocity(rightSpeed);
  leftMotor.direction(Motor::FORWARD);
  rightMotor.direction(Motor::FORWARD);
}

void MotorController::moveBackward(int speed) {
  speed = checkSpeed(speed);

  int leftSpeed = speed * leftMotorCalibration;
  int rightSpeed = speed * rightMotorCalibration;
  leftMotor.velocity(leftSpeed);
  rightMotor.velocity(rightSpeed);
  leftMotor.direction(Motor::BACKWARD);
  rightMotor.direction(Motor::BACKWARD);
}

void MotorController::stop() {
  leftMotor.direction(Motor::STOP);
  rightMotor.direction(Motor::STOP);
}

void MotorController::dynamicTurn(int leftSpeed, int rightSpeed) {
  leftSpeed = checkSpeed(leftSpeed);
  rightSpeed = checkSpeed(rightSpeed);

  leftMotor.velocity(leftSpeed * leftMotorCalibration);
  rightMotor.velocity(rightSpeed * rightMotorCalibration);
  leftMotor.direction(Motor::FORWARD);
  rightMotor.direction(Motor::FORWARD);
}

void MotorController::spinInPlaceRight(int speed) {
  speed = checkSpeed(speed);

  leftMotor.velocity(speed);
  rightMotor.velocity(speed);
  leftMotor.direction(Motor::FORWARD);
  rightMotor.direction(Motor::BACKWARD);
}

void MotorController::spinInPlaceLeft(int speed) {
  speed = checkSpeed(speed);

  leftMotor.velocity(speed);
  rightMotor.velocity(speed);
  leftMotor.direction(Motor::BACKWARD);
  rightMotor.direction(Motor::FORWARD);
}

void MotorController::turnRightWithObstacleDistance(int distance, int minDistance, int maxDistance) {
  int adjustedVelocity = constrain(map(distance, minDistance, maxDistance, minVelocity, maxVelocity), minVelocity, maxVelocity);
  dynamicTurn(maxVelocity, adjustedVelocity);
}

void MotorController::turnLeftWithObstacleDistance(int distance, int minDistance, int maxDistance) {
  int adjustedVelocity = constrain(map(distance, minDistance, maxDistance, minVelocity, maxVelocity), minVelocity, maxVelocity);
  dynamicTurn(adjustedVelocity, maxVelocity);
}

void MotorController::setCalibration(float leftCal, float rightCal) {
  leftMotorCalibration = leftCal;
  rightMotorCalibration = rightCal;
}

int MotorController::checkSpeed(int speed){
  return constrain(speed, minVelocity, maxVelocity);
}
