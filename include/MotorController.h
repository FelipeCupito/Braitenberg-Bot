#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "Motor.h"
#include <math.h>

class MotorController {
  public:
    enum AdjustmentMethod {
      LINEAR_DIFFERENCE,
      QUADRATIC_DIFFERENCE,
      EXPONENTIAL_DIFFERENCE,
      SIGMOID_DIFFERENCE,
    };

    MotorController(Motor& leftMotor, Motor& rightMotor);

    // Configuration
    void setup();
    void setMaxSpeed(int maxSpeed);
    void setMinSpeed(int minSpeed);
    int getMaxSpeed() const;
    int getMinSpeed() const;

    // Movement
    void moveForward(int speed);
    void moveBackward(int speed);
    void stop();
    void dynamicTurn(int leftSpeed, int rightSpeed);
    void spinInPlaceRight(int speed);
    void spinInPlaceLeft(int speed);
    void adjustSpeedsToApproach(int leftValue, int rightValue, int minValue, int maxValue, AdjustmentMethod method = LINEAR_DIFFERENCE);
    void adjustSpeedsToAvoid(int leftValue, int rightValue, int minValue, int maxValue, AdjustmentMethod method= LINEAR_DIFFERENCE);

  private:
    Motor& leftMotor;
    Motor& rightMotor;
    int maxSpeed;
    int minSpeed;

    // Funciones privadas para cada método de ajuste
    float linearScaling(float value);
    float quadraticScaling(float value);
    float exponentialScaling(float value);
    float sigmoidScaling(float value);

    // Funciones auxiliares
    void safeDirectionChangeFrom(Motor::State fromState);
    void adjustSpeeds(int leftValue, int rightValue, int minValue, int maxValue, AdjustmentMethod method, bool isApproaching);
};

#endif
