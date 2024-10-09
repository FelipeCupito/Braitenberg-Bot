#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "Motor.h"
#include <math.h>


class MotorController {
  public:
    enum AdjustmentMethod {
      LINEAR_DIFFERENCE,        // Ajuste simple y proporcional. Bueno para respuestas lineales.
      QUADRATIC_DIFFERENCE,     // Ajuste más agresivo, aplicando una curva cuadrática para reacciones rápidas.
      EXPONENTIAL_DIFFERENCE,   // El ajuste más agresivo, útil para respuestas críticas
      SIGMOID_DIFFERENCE,       // Ajuste controlado y suave, ideal para evitar movimientos bruscos.  
    };

    MotorController(Motor& leftMotor, Motor& rightMotor);
    // Configuration
    void setup();
    void setAdjustmentMethod(AdjustmentMethod method);

    // Movement
    void moveForward(int speed);
    void moveBackward(int speed);
    void stop();
    void dynamicTurn(int leftSpeed, int rightSpeed);
    void spinInPlaceRight(int speed);
    void spinInPlaceLeft(int speed);
    //void turnRightWithObstacleDistance(int distance, int minDistance, int maxDistance);
    //void turnLeftWithObstacleDistance(int distance, int minDistance, int maxDistance);
    void adjustSpeedsBasedOnObstacleDistances(int leftDistance, int rightDistance, int minDistance, int maxDistance);


  private:
    AdjustmentMethod adjustmentMethod;
    Motor& leftMotor;
    Motor& rightMotor;

    // Funciones privadas para cada método de ajuste
    float linearScaling(float value);
    float quadraticScaling(float value);
    float exponentialScaling(float value);
    float sigmoidScaling(float value);

    // Funciones auxiliares
    void safeDirectionChangeFrom(Motor::State fromState);
};

#endif


/*
void MotorController::combinedMapping(int leftDistance, int rightDistance, int minDistance, int maxDistance, int& leftSpeed, int& rightSpeed) {
    int minDetectedDistance = min(leftDistance, rightDistance);

    // Calcular factor basado en la distancia mínima
    float distanceFactor = float(minDetectedDistance - minDistance) / float(maxDistance - minDistance);
    distanceFactor = constrain(distanceFactor, 0.0, 1.0);

    // Calcular factor basado en la diferencia de distancias
    int distanceDifference = rightDistance - leftDistance;
    float differenceFactor = float(distanceDifference) / float(maxDistance - minDistance);
    differenceFactor = constrain(differenceFactor, -1.0, 1.0);

    // Combinar factores (puedes ajustar los pesos)
    float combinedFactor = (0.7 * distanceFactor) + (0.3 * differenceFactor);

    int speedRange = maxVelocity - minVelocity;
    int adjustedSpeed = minVelocity + combinedFactor * speedRange;

    // Ajustar velocidades de los motores
    leftSpeed = adjustedSpeed;
    rightSpeed = adjustedSpeed;

    // Puedes añadir ajustes adicionales basados en la diferencia
    leftSpeed -= differenceFactor * speedRange / 2;
    rightSpeed += differenceFactor * speedRange / 2;

    leftSpeed = constrain(leftSpeed, minVelocity, maxVelocity);
    rightSpeed = constrain(rightSpeed, minVelocity, maxVelocity);
}
 */