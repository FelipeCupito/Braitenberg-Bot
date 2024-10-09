#include "MotorController.h"

#define MAX_SPEED 255
#define MIN_SPEED 0

MotorController::MotorController(Motor& leftMotor, Motor& rightMotor): leftMotor(leftMotor), rightMotor(rightMotor) {}

void MotorController::setup() {
  leftMotor.setup();
  rightMotor.setup();
}

/**********************
 ** Movement methods **
 **********************/ 

void MotorController::moveForward(int speed) {
  // If any motor is moving backwards, stop it before moving forward
  safeDirectionChangeFrom(Motor::BACKWARD);

  // Now we can safely change the direction to move forward
  leftMotor.moveForward(speed);
  rightMotor.moveForward(speed);
}

void MotorController::moveBackward(int speed) {
  // If any motor is moving forward, stop it before moving backward
  safeDirectionChangeFrom(Motor::FORWARD);

  // Now we can safely change the direction to move backward
  leftMotor.moveBackward(speed);
  rightMotor.moveBackward(speed);
}

void MotorController::stop() {
  leftMotor.fullStop();
  rightMotor.fullStop();
}

void MotorController::dynamicTurn(int leftSpeed, int rightSpeed) {
  // If any motor is moving backwards, stop it before moving forward
  safeDirectionChangeFrom(Motor::BACKWARD);

  // Now we can safely change the direction to move forward
  rightMotor.moveForward(rightSpeed);
  leftMotor.moveForward(leftSpeed);
}

void MotorController::spinInPlaceRight(int speed) {
  // If any motor is moving in the opposite direction, stop it before moving in the correct direction
  bool delayFlag = false;
  if(leftMotor.getState() == Motor::BACKWARD){
    leftMotor.fullStop();
    delayFlag = true;
  }
  if(rightMotor.getState() == Motor::FORWARD){
    rightMotor.fullStop();
    delayFlag = true;
  }
  if (delayFlag) {
    delay(100);
  }

  // Move motors in opposite directions to spin in place
  leftMotor.moveForward(speed);
  rightMotor.moveBackward(speed);
}

void MotorController::spinInPlaceLeft(int speed) {
  // If any motor is moving in the opposite direction, stop it before moving in the correct direction
  bool delayFlag = false;
  if(leftMotor.getState() == Motor::FORWARD){
    leftMotor.fullStop();
    delayFlag = true;
  }
  if(rightMotor.getState() == Motor::BACKWARD){
    rightMotor.fullStop();
    delayFlag = true;
  }
  if (delayFlag) {
    delay(100);
  }

  // Move motors in opposite directions to spin in place
  leftMotor.moveBackward(speed);
  rightMotor.moveForward(speed);
}

void MotorController::adjustSpeedsToApproach(int leftValue, int rightValue, int minValue, int maxValue, AdjustmentMethod method) {
  adjustSpeeds(leftValue, rightValue, minValue, maxValue, method, true);
}

void MotorController::adjustSpeedsToAvoid(int leftValue, int rightValue, int minValue, int maxValue, AdjustmentMethod method) {
  adjustSpeeds(leftValue, rightValue, minValue, maxValue, method, false);
}

void MotorController::safeDirectionChangeFrom(Motor::State fromState){
  bool delayFlag = false;
  if(leftMotor.getState() == fromState){
    leftMotor.fullStop();
    delayFlag = true;
  }
  if(rightMotor.getState() == fromState){
    rightMotor.fullStop();
    delayFlag = true;
  }
  if (delayFlag) {
    delay(100);
  }
}

void MotorController::adjustSpeeds(int leftValue, int rightValue, int minValue, int maxValue, AdjustmentMethod method, bool isApproaching) {
  int leftSpeed = MAX_SPEED;
  int rightSpeed = MAX_SPEED;

  // Calculate difference and normalize
  int valueDifference = rightValue - leftValue;
  float normalizedDifference = float(valueDifference) / float(maxValue - minValue);
  normalizedDifference = constrain(normalizedDifference, -1.0, 1.0);

  // Apply scaling function
  float scalingFactor = 0.0;
  switch (method) {
    case LINEAR_DIFFERENCE:
      scalingFactor = linearScaling(normalizedDifference);
      break;
    case QUADRATIC_DIFFERENCE:
      scalingFactor = quadraticScaling(normalizedDifference);
      break;
    case EXPONENTIAL_DIFFERENCE:
      scalingFactor = exponentialScaling(normalizedDifference);
      break;
    case SIGMOID_DIFFERENCE:
      scalingFactor = sigmoidScaling(normalizedDifference);
      break;
    default:
      scalingFactor = linearScaling(normalizedDifference);
      break;
  }

  int baseSpeed = (MAX_SPEED + MIN_SPEED) / 2;
  int maxDifferential = (MAX_SPEED - MIN_SPEED) / 2;
  int differentialSpeed = scalingFactor * maxDifferential;

  if (isApproaching) {
    // Move towards the higher value
    leftSpeed = baseSpeed - differentialSpeed;
    rightSpeed = baseSpeed + differentialSpeed;
  } else {
    // Move away from the higher value
    leftSpeed = baseSpeed + differentialSpeed;
    rightSpeed = baseSpeed - differentialSpeed;
  }

  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  dynamicTurn(leftSpeed, rightSpeed);
}

/*************************
 **  Ajustment methods  **
 *************************/
float MotorController::linearScaling(float value) {
    return value;
}

float MotorController::quadraticScaling(float value) {
    float scalingFactor = value * value;
    return (value < 0) ? -scalingFactor : scalingFactor;
}

float MotorController::exponentialScaling(float value) {
    float scalingFactor = pow(abs(value), 3);
    return (value < 0) ? -scalingFactor : scalingFactor;
}

float MotorController::sigmoidScaling(float value) {
    return tanh(3.0 * value); // Ajusta el multiplicador según sea necesario
}