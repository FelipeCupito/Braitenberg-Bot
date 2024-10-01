#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  public:
    enum Mode { INVERTED = 0, NORMAL = 1 };
    enum State { STOP = 0, FORWARD = 1, BACKWARD = 2 };

    Motor(int pinENA, int pinIN1, int pinIN2, Mode mode);
    void setup();
    void velocity(int speed);
    void direction(State state);

  private:
    int EN, IN1, IN2;
    Mode mode;
    State state;
    int speed;

    void moveForward(int speed);
    void moveBackward(int speed);
    void fullStop();
};

#endif
