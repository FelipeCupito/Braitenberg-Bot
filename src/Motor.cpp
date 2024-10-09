#include "Motor.h"

#define MAX_SPEED 255
#define MIN_SPEED 0
#define MIN_CALIBRATION_FACTOR 0.0f
#define MAX_CALIBRATION_FACTOR 1.0f


Motor::Motor(int pinENA, int pinIN1, int pinIN2, Mode mode, float calibrationFactor = 1.0f) : EN(pinENA), IN1(pinIN1), IN2(pinIN2), mode(mode), state(STOP), speed(MIN_SPEED) {
  this->calibrationFactor = constrain(calibrationFactor, MIN_CALIBRATION_FACTOR, MAX_CALIBRATION_FACTOR);
}

void Motor::setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);
}

bool Motor::isMoving() {
  return state != STOP;
}

Motor::State Motor::getState() {
  return state;
}

int Motor::getSpeed() {
  return speed;
}

void Motor::setVelocity(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  float adjustedSpeed = speed * calibrationFactor;
  this->speed = (int)constrain(adjustedSpeed, MIN_SPEED, MAX_SPEED);
  
  analogWrite(EN, this->speed);
}



void Motor::setDirection(State state) {
  switch(state) {
    case FORWARD: 
      if(mode == INVERTED) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
      } else {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
      }
      this->state = FORWARD; 
      break;
    case BACKWARD: 
      if(mode == INVERTED) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
      } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
      }
      this->state = BACKWARD; 
      break;
    case STOP:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      this->state = STOP;
      break;
  }
}

void Motor::moveForward(int speed) {
  setDirection(FORWARD);
  setVelocity(speed);
}

void Motor::moveBackward(int speed) {
  setDirection(BACKWARD);
  setVelocity(speed);
}

void Motor::fullStop() {
  setDirection(STOP);
  setVelocity(MIN_SPEED);
}

