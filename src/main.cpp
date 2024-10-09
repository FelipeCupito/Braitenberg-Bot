#include <Arduino.h>
#include "Motor.h"
#include "MotorController.h"
#include "Sonar.h"

// Se perdio PWM de los pines 3 y 11 por usar NewPing non-blocking
//  Motores
#define MOTOR_PIN_ENA 6  // Azul
#define MOTOR_PIN_IN1 7  // Rojo
#define MOTOR_PIN_IN2 8  // Gris
#define MOTOR_PIN_IN3 9  // Amarillo
#define MOTOR_PIN_IN4 10 // Naraja
#define MOTOR_PIN_ENB 5 // Blanco

// Calibración de motores
#define MOTOR_CALIBRATION_LEFT 0.8
#define MOTOR_CALIBRATION_RIGHT 1.0

// Ultrasonico
#define ULTRA_SENSOR_RIGHT A0  // Amarillo
#define ULTRA_SENSOR_LEFT A1   // Marrón 
#define MAX_DISTANCE 100
#define MIN_DISTANCE 0 

// Distancia
#define CRITICAL_DISTANCE 20
#define AVOID_DISTANCE 60     

// Velocidades
#define SPIN_SPEED 100      // Velocidad para girar sobre sí mismo
#define BACKWARD_SPEED 100  // Velocidad al retroceder
#define MAX_VELOCITY 255   // Velocidad máxima
#define MIN_VELOCITY 100     // Velocidad mínima para giros


Motor motorLeft(MOTOR_PIN_ENA, MOTOR_PIN_IN1, MOTOR_PIN_IN2, Motor::INVERTED, MOTOR_CALIBRATION_LEFT);
Motor motorRight(MOTOR_PIN_ENB, MOTOR_PIN_IN3, MOTOR_PIN_IN4, Motor::NORMAL, MOTOR_CALIBRATION_RIGHT);
MotorController motorController(motorLeft, motorRight);

Sonar sonar(ULTRA_SENSOR_RIGHT, ULTRA_SENSOR_LEFT, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);
  
  motorController.setup();
  motorController.setAdjustmentMethod(MotorController::SIGMOID_DIFFERENCE);

  sonar.setup();
}

void loop() {
  sonar.update();
  
  int distance = sonar.getDistance();
  // int newMaxVelocity = constrain(map(distance, MIN_DISTANCE, MAX_DISTANCE, MIN_VELOCITY, MAX_VELOCITY), MIN_VELOCITY, MAX_VELOCITY);
  // motorController.setMaxVelocity(newMaxVelocity);


  if (distance > 0 && distance <= CRITICAL_DISTANCE) {
    Serial.println("Obstáculo muy cerca, girando en el lugar");
    motorController.stop();
    if (Sonar::CloserSide::RIGHT == sonar.getCloserSide()) {
        motorController.spinInPlaceLeft(SPIN_SPEED);
    } else {
        motorController.spinInPlaceRight(SPIN_SPEED);
    }
    delay(1000);
  } 
  else if (distance > CRITICAL_DISTANCE && distance <= AVOID_DISTANCE) {
    Serial.println("Obstáculo detectado, evitando");
    motorController.adjustSpeedsBasedOnObstacleDistances(sonar.getLeftDistance(), sonar.getRightDistance(), CRITICAL_DISTANCE, AVOID_DISTANCE);

    // if(Sonar::CloserSide::RIGHT == sonar.getCloserSide()){

    //   Serial.print("Girando a la izquierda, distancia: ");
    //   Serial.println(distance);
    //   motorController.turnLeftWithObstacleDistance(distance, CRITICAL_DISTANCE, AVOID_DISTANCE);
    // } else {
    //   Serial.print("Girando a la derecha, distancia: ");
    //   Serial.println(distance);
    //   motorController.turnRightWithObstacleDistance(distance, CRITICAL_DISTANCE, AVOID_DISTANCE);
    // }
  } 
  else {
    Serial.println("No hay obstáculo, avanzando");
    motorController.moveForward(MAX_VELOCITY); 
  }   
}