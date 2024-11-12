#include <Arduino.h>
#include "Motor.h"
#include "MotorController.h"
#include "Sonar.h"
#include "LightSensor.h"

// Se perdio PWM de los pines 3 y 11 por usar NewPing non-blocking
//  Motores
#define MOTOR_PIN_ENA 6  // Azul
#define MOTOR_PIN_IN1 7  // Rojo
#define MOTOR_PIN_IN2 8  // Gris
#define MOTOR_PIN_IN3 9  // Amarillo
#define MOTOR_PIN_IN4 10 // Naraja
#define MOTOR_PIN_ENB 5  // Blanco

// Calibración de motores
#define MOTOR_CALIBRATION_LEFT 0.9
#define MOTOR_CALIBRATION_RIGHT 1.0

// Ultrasonico
#define ULTRA_SENSOR_RIGHT A0  // Amarillo
#define ULTRA_SENSOR_LEFT A1   // Marrón 
#define MAX_DISTANCE 40

// Light Sensors (Analog Pins)
const int LIGHT_SENSOR_PINS[LightSensor::NUM_SENSORS] = {
    A2,   // FRONT_LEFT - Naranja
    A3,   // FRONT_RIGHT - Rojo
    //A4, // LEFT - Azul
    //A5, // RIGHT - Blanco
};

// Light Sensor Calibration
#define MAX_LIGHT_INTENSITY 1023
#define MIN_LIGHT_INTENSITY 0

// Distancia
#define CRITICAL_DISTANCE 40
#define AVOID_DISTANCE 60     

// Velocidades
#define SPIN_SPEED 150      // Velocidad para girar sobre sí mismo
#define BACKWARD_SPEED 150  // Velocidad al retroceder
#define MAX_VELOCITY 255    // Velocidad máxima
#define MIN_VELOCITY 150    // Velocidad mínima para giros

// Motors Instances
Motor motorLeft(MOTOR_PIN_ENA, MOTOR_PIN_IN1, MOTOR_PIN_IN2, Motor::INVERTED, MOTOR_CALIBRATION_LEFT);
Motor motorRight(MOTOR_PIN_ENB, MOTOR_PIN_IN3, MOTOR_PIN_IN4, Motor::NORMAL, MOTOR_CALIBRATION_RIGHT);
MotorController motorController(motorLeft, motorRight);

// Sensors Instances
Sonar sonar(ULTRA_SENSOR_RIGHT, ULTRA_SENSOR_LEFT, MAX_DISTANCE);
LightSensor lightSensor(LIGHT_SENSOR_PINS);

//private functions
float sigmoidMapping(int distance, int minDistance, int maxDistance, int minSpeed, int maxSpeed);

//Global varibles
int maxIntensity = 0;

void setup() {
  Serial.begin(9600);
  
  motorController.setup();
  sonar.setup();
  lightSensor.setup();
  
  maxIntensity = MAX_LIGHT_INTENSITY - lightSensor.getGlobalBaselineValue();
}

void loop() {

  sonar.update();
  lightSensor.update();
  
  // Obtener distancia del sensor ultrasonico
  int distance = sonar.getDistance();

  Serial.println("Detected distance: " + String(distance));
  int speed = sigmoidMapping(distance, 0, MAX_DISTANCE, MIN_VELOCITY, MAX_VELOCITY);
  motorController.setMaxSpeed(speed);
  Serial.println("Setting speed to: " + String(speed));

  // Obtener intensidades de los sensores frontales
  int leftIntensity = lightSensor.getLightIntensity(LightSensor::FRONT_LEFT);
  int rightIntensity = lightSensor.getLightIntensity(LightSensor::FRONT_RIGHT);

  // if (distance > 0 && distance <= MAX_DISTANCE) {
  //   // motorController.stop();
  //   // if (Sonar::CloserSide::RIGHT == sonar.getCloserSide()) {
  //   //     motorController.spinInPlaceLeft(SPIN_SPEED);
  //   // } else {
  //   //     motorController.spinInPlaceRight(SPIN_SPEED);
  //   // }
  //   // delay(600);
  // }
  // else {
    motorController.adjustSpeedsToApproach(
      leftIntensity, rightIntensity, 
      MIN_LIGHT_INTENSITY, maxIntensity,  // En realidad no porque tendria que usar el globalBaselineValue como minimo.
      MotorController::SIGMOID_DIFFERENCE
    );

  // }   
}


float sigmoidMapping(int distance, int minDistance, int maxDistance, int minSpeed, int maxSpeed) {
  // Normalizar la distancia al rango [0, 1], invertido
  float normalizedDistance = (float)(maxDistance - distance) / (maxDistance - minDistance);
  normalizedDistance = constrain(normalizedDistance, 0.0, 1.0);

  // Aplicar la función sigmoide usando tanh
  float sigmoidValue = tanh(8.0 * normalizedDistance);


  // Mapear el valor sigmoide al rango de velocidades
  int mappedSpeed = minSpeed + (maxSpeed - minSpeed) * sigmoidValue;
  return constrain(mappedSpeed, minSpeed, maxSpeed);
}



/***** EL OJO ******/

// #include <Arduino.h>
// #include <Servo.h>
// #include <Servo.h>

// //define name of the servo m
// Servo upDownServo;
// Servo rightLeftServo;

// //define position name and v
// #define left 60
// #define right 120
// #define middle 90
// #define closed 60
// #define fullOpen 160
// #define halfOpen 120

// #define waitTime 750

// void setup(){
//   //define pin numbers of the servo motors
//   upDownServo.attach (5);
//   rightLeftServo.attach(4);
  
//   //stacting position of the servo motorg
//   delay (10);
//   upDownServo.write (halfOpen);
//   rightLeftServo.write (middle);
// }

// void loop() {

// delay (1000);
// upDownServo.write(halfOpen);
// delay (waitTime);
// rightLeftServo.write(right);
// delay (waitTime);
// rightLeftServo.write (left);
// delay (waitTime);
// rightLeftServo.write(middle);

// delay (1000);
// upDownServo.write(fullOpen);
// delay (waitTime);
// rightLeftServo.write(right);
// delay (waitTime);
// rightLeftServo.write (left);
// delay (waitTime);
// rightLeftServo.write(middle);

// }