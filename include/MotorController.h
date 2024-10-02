#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "Motor.h"

class MotorController {
  public:
    MotorController(Motor& leftMotor, Motor& rightMotor, float leftMotorCalibration = 1.0, float rightMotorCalibration = 1.0);
    // Configuration
    void setup(int maxVelocity, int minVelocity);
    void setCalibration(float leftCal, float rightCal);

    // Movement
    void moveForward(int speed);
    void moveBackward(int speed);
    void stop();
    void dynamicTurn(int leftSpeed, int rightSpeed);
    void spinInPlaceRight(int speed);
    void spinInPlaceLeft(int speed);
    void turnRightWithObstacleDistance(int distance, int minDistance, int maxDistance);
    void turnLeftWithObstacleDistance(int distance, int minDistance, int maxDistance);


  private:
    Motor& leftMotor;
    Motor& rightMotor;
    float leftMotorCalibration;
    float rightMotorCalibration;
    int maxVelocity;
    int minVelocity;

    int checkSpeed(int speed);
};

#endif
