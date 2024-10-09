#include "MotorController.h"

#define MAX_SPEED 255
#define MIN_SPEED 0

MotorController::MotorController(Motor& leftMotor, Motor& rightMotor): leftMotor(leftMotor), rightMotor(rightMotor) {}

void MotorController::setup() {
  leftMotor.setup();
  rightMotor.setup();
}

void MotorController::setAdjustmentMethod(AdjustmentMethod method) {
    adjustmentMethod = method;
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

void MotorController::adjustSpeedsBasedOnObstacleDistances(int leftDistance, int rightDistance, int minDistance, int maxDistance) {
  int leftSpeed = MAX_SPEED;
  int rightSpeed = MAX_SPEED;

  // Cálculo de la diferencia de distancia y normalización
  int distanceDifference = rightDistance - leftDistance;
  float normalizedDifference = float(distanceDifference) / float(maxDistance - minDistance);
  normalizedDifference = constrain(normalizedDifference, -1.0, 1.0);

  // Aplicar la función de escalado
  float scalingFactor = 0.0;
  switch (adjustmentMethod) {
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

  leftSpeed = baseSpeed - differentialSpeed;
  rightSpeed = baseSpeed + differentialSpeed;

  Serial.print("Left Distance: "); Serial.print(leftDistance);
  Serial.print(" | Right Distance: "); Serial.print(rightDistance);
  Serial.print(" | Left Speed: "); Serial.print(leftSpeed);
  Serial.print(" | Right Speed: "); Serial.println(rightSpeed);

  dynamicTurn(leftSpeed, rightSpeed);
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

// void MotorController::turnRightWithObstacleDistance(int distance, int minDistance, int maxDistance) {
//   int rightSpeed = map(distance, minDistance, maxDistance, 0, 100);
//   int leftSpeed = map(distance, minDistance, maxDistance, 150, 255);

//     // Hacemos el cambio más brusco aplicando una transformación cuadrática
//   //adjustedVelocity = constrain((baseVelocity - minVelocity) * (baseVelocity - minVelocity) / (maxVelocity - minVelocity) + minVelocity, minVelocity, maxVelocity);
  
//   dynamicTurn(leftSpeed, rightSpeed);
// }

// void MotorController::turnLeftWithObstacleDistance(int distance, int minDistance, int maxDistance) {
//   // int adjustedVelocity = map(distance, minDistance, maxDistance, minVelocity, maxVelocity);
//   int rightSpeed = map(distance, minDistance, maxDistance, 150, 255);
//   int leftSpeed = map(distance, minDistance, maxDistance, 0, 100);

//   dynamicTurn(leftSpeed, rightSpeed);
// }

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