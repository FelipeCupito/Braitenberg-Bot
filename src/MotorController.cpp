#include "MotorController.h"

MotorController::MotorController(Motor& leftMotor, Motor& rightMotor)
    : leftMotor(leftMotor), rightMotor(rightMotor), maxSpeed(255), minSpeed(0) {}

void MotorController::setup() {
  leftMotor.setup();
  rightMotor.setup();
}

void MotorController::setMaxSpeed(int maxSpeed) {
  this->maxSpeed = constrain(maxSpeed, 0, 255);
}

void MotorController::setMinSpeed(int minSpeed) {
  this->minSpeed = constrain(minSpeed, 0, 255);
}

int MotorController::getMaxSpeed() const {
  return maxSpeed;
}

int MotorController::getMinSpeed() const {
  return minSpeed;
}

/**********************
 ** Movement methods **
 **********************/ 

void MotorController::moveForward(int speed) {
  safeDirectionChangeFrom(Motor::BACKWARD);

  speed = constrain(speed, minSpeed, maxSpeed);
  leftMotor.moveForward(speed);
  rightMotor.moveForward(speed);
}

void MotorController::moveBackward(int speed) {
  safeDirectionChangeFrom(Motor::FORWARD);

  speed = constrain(speed, minSpeed, maxSpeed);
  leftMotor.moveBackward(speed);
  rightMotor.moveBackward(speed);
}

void MotorController::stop() {
  leftMotor.fullStop();
  rightMotor.fullStop();
}

void MotorController::dynamicTurn(int leftSpeed, int rightSpeed) {
  safeDirectionChangeFrom(Motor::BACKWARD);

  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  leftMotor.moveForward(leftSpeed);
  rightMotor.moveForward(rightSpeed);
}

void MotorController::spinInPlaceRight(int speed) {
  speed = constrain(speed, minSpeed, maxSpeed);

  leftMotor.moveForward(speed);
  rightMotor.moveBackward(speed);
}

void MotorController::spinInPlaceLeft(int speed) {
  speed = constrain(speed, minSpeed, maxSpeed);

  leftMotor.moveBackward(speed);
  rightMotor.moveForward(speed);
}

void MotorController::adjustSpeedsToApproach(int leftValue, int rightValue, int minValue, int maxValue, AdjustmentMethod method) {
  adjustSpeeds(leftValue, rightValue, minValue, maxValue, method, false);
}

void MotorController::adjustSpeedsToAvoid(int leftValue, int rightValue, int minValue, int maxValue, AdjustmentMethod method) {
  adjustSpeeds(leftValue, rightValue, minValue, maxValue, method, true);
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
  int leftSpeed = maxSpeed;
  int rightSpeed = maxSpeed;

  // Calcular diferencia y normalizar
  int valueDifference = rightValue - leftValue;
  float normalizedDifference = float(valueDifference) / float(maxValue - minValue);
  normalizedDifference = constrain(normalizedDifference, -1.0, 1.0);

  // Aplicar función de escalado
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

  int baseSpeed = (maxSpeed + minSpeed) / 2;
  int maxDifferential = (maxSpeed - minSpeed) / 2;
  int differentialSpeed = scalingFactor * maxDifferential;

  if (isApproaching) {
    // Moverse hacia el valor más alto
    leftSpeed = baseSpeed - differentialSpeed;
    rightSpeed = baseSpeed + differentialSpeed;
  } else {
    // Alejarse del valor más alto
    leftSpeed = baseSpeed + differentialSpeed;
    rightSpeed = baseSpeed - differentialSpeed;
  }

  // Limitar velocidades al rango válido
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

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
  //return tanh(4.0 * value);
  return tanh(8.0 * value);
}
