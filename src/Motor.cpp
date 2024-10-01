#include "Motor.h"

Motor::Motor(int pinENA, int pinIN1, int pinIN2, Mode mode) : EN(pinENA), IN1(pinIN1), IN2(pinIN2), mode(mode), state(STOP), speed(0) {}

void Motor::setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);
}

void Motor::velocity(int speed) {
  analogWrite(EN, speed);
  this->speed = speed;
}

void Motor::direction(State state) {
  switch(state) {
    case FORWARD: moveForward(speed); break;
    case BACKWARD: moveBackward(speed); break;
    case STOP: fullStop(); break;
  }
}

void Motor::moveForward(int speed) {
  if(mode == INVERTED) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  this->state = FORWARD;
}

void Motor::moveBackward(int speed) {
  if(mode == INVERTED) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  this->state = BACKWARD;
}

void Motor::fullStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(EN, 0);
  this->speed = 0;
  this->state = STOP;
}
