#include "MotorController.h"

MotorController::MotorController(Motor& leftMotor, Motor& rightMotor, float leftMotorCalibration, float rightMotorCalibration)
  : leftMotor(leftMotor), rightMotor(rightMotor), leftMotorCalibration(leftMotorCalibration), rightMotorCalibration(rightMotorCalibration) {}

void MotorController::setup() {
  leftMotor.setup();
  rightMotor.setup();
}

void MotorController::moveForward(int speed) {
  int leftSpeed = speed * leftMotorCalibration;
  int rightSpeed = speed * rightMotorCalibration;
  leftMotor.velocity(leftSpeed);
  rightMotor.velocity(rightSpeed);
  leftMotor.direction(Motor::FORWARD);
  rightMotor.direction(Motor::FORWARD);
}

void MotorController::moveBackward(int speed) {
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
  leftMotor.velocity(leftSpeed * leftMotorCalibration);
  rightMotor.velocity(rightSpeed * rightMotorCalibration);
  leftMotor.direction(Motor::FORWARD);
  rightMotor.direction(Motor::FORWARD);
}

void MotorController::spinInPlaceRight(int speed) {
  leftMotor.velocity(speed);
  rightMotor.velocity(speed);
  leftMotor.direction(Motor::FORWARD);
  rightMotor.direction(Motor::BACKWARD);
}

void MotorController::spinInPlaceLeft(int speed) {
  leftMotor.velocity(speed);
  rightMotor.velocity(speed);
  leftMotor.direction(Motor::BACKWARD);
  rightMotor.direction(Motor::FORWARD);
}

void MotorController::setCalibration(float leftCal, float rightCal) {
  leftMotorCalibration = leftCal;
  rightMotorCalibration = rightCal;
}
