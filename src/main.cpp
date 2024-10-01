#include <Arduino.h>
//#include "AsyncSonarLib.h"
#include "Motor.h"
#include "MotorController.h"
#include "Sonar.h"

/******************************************************* MOTOR ********************************************************/
// class Motor{
//     /***** Tabla de control de motores *****
//                     Adelante    Atrás   Freno
//     IN1 (o IN3)	    HIGH	    LOW	    LOW
//     IN2 (o IN4)	    LOW	        HIGH    LOW
//     ***************************************/

//   public:
//     enum Mode{
//       INVERTED = 0,
//       NORMAL = 1
//     };

//     enum State{
//       STOP = 0,
//       FORWARD = 1,
//       BACKWARD = 2
//     };

//     Motor(int pinENA, int pinIN1, int pinIN2, Mode mode){
//       this->EN = pinENA;
//       this->IN1 = pinIN1;
//       this->IN2 = pinIN2;
//       this->mode = mode;
//       this->state = STOP;
//       this->speed = 0;
//     };
    
//     void setup(){
//       pinMode(IN1, OUTPUT);
//       pinMode(IN2, OUTPUT);
//       pinMode(EN, OUTPUT);
//     }

//     void velocity(int speed){
//       analogWrite(EN, speed);
//       this->speed = speed;
//     }

//     void direction(State state){
//       switch(state){
//         case FORWARD:
//           moveForward(speed);
//           break;
//         case BACKWARD:
//           moveBackward(speed);
//           break;
//         case STOP:
//           fullStop();
//           break;
//       }
//     }


//   private:
//     int EN;
//     int IN1;
//     int IN2;
//     Mode mode;
//     State state;
//     int speed;

//     void moveForward(int speed){
//       if(mode == INVERTED){
//         digitalWrite(IN1, LOW);
//         digitalWrite(IN2, HIGH);
//       }else{
//         digitalWrite(IN1, HIGH);
//         digitalWrite(IN2, LOW);
//       }
//       state = FORWARD;
//     }

//     void moveBackward(int speed){
//       if(mode == INVERTED){
//         digitalWrite(IN1, HIGH);
//         digitalWrite(IN2, LOW);
//       }else{
//         digitalWrite(IN1, LOW);
//         digitalWrite(IN2, HIGH);
//       }
//       state = BACKWARD;
//     }

//     void fullStop(){
//       digitalWrite(IN1, LOW);
//       digitalWrite(IN2, LOW);
//       analogWrite(EN, 0);
//       speed = 0;
//       state = STOP;
//     }
// };


/******************************************************* MotorController ********************************************************/
// class MotorController {
//   public:
//     MotorController(Motor& leftMotor, Motor& rightMotor, float leftMotorCalibration = 1.0, float rightMotorCalibration = 1.0)
//       : leftMotor(leftMotor), rightMotor(rightMotor), leftMotorCalibration(leftMotorCalibration), rightMotorCalibration(rightMotorCalibration) {}

//     void setup() {
//       leftMotor.setup();
//       rightMotor.setup();
//     }

//     // Movimiento hacia adelante con calibración
//     void moveForward(int speed) {
//       int leftSpeed = speed * leftMotorCalibration;
//       int rightSpeed = speed * rightMotorCalibration;
//       leftMotor.velocity(leftSpeed);
//       rightMotor.velocity(rightSpeed);
//       leftMotor.direction(Motor::FORWARD);
//       rightMotor.direction(Motor::FORWARD);
//     }

//     // Movimiento hacia atrás con calibración
//     void moveBackward(int speed) {
//       int leftSpeed = speed * leftMotorCalibration;
//       int rightSpeed = speed * rightMotorCalibration;
//       leftMotor.velocity(leftSpeed);
//       rightMotor.velocity(rightSpeed);
//       leftMotor.direction(Motor::BACKWARD);
//       rightMotor.direction(Motor::BACKWARD);
//     }

//     // Detener ambos motores
//     void stop() {
//       leftMotor.direction(Motor::STOP);
//       rightMotor.direction(Motor::STOP);
//     }

//     // Control dinámico para giros con calibración
//     void dynamicTurn(int leftSpeed, int rightSpeed) {
//       leftMotor.velocity(leftSpeed * leftMotorCalibration);
//       rightMotor.velocity(rightSpeed * rightMotorCalibration);
//       leftMotor.direction(Motor::FORWARD);
//       rightMotor.direction(Motor::FORWARD);
//     }

//      void spinInPlace(int speed) {
//       leftMotor.velocity(speed);
//       rightMotor.velocity(speed);
//       leftMotor.direction(Motor::FORWARD);  // Avanzar con el motor izquierdo
//       rightMotor.direction(Motor::BACKWARD);  // Retroceder con el motor derecho
//     }

//     // Establecer calibración
//     void setCalibration(float leftCal, float rightCal) {
//       leftMotorCalibration = leftCal;
//       rightMotorCalibration = rightCal;
//     }

//   private:
//     Motor& leftMotor;
//     Motor& rightMotor;
//     float leftMotorCalibration;
//     float rightMotorCalibration;
// };


/******************************************************* main ********************************************************/
// Se perdio PWM de los pines 3 y 11 por usar NewPing non-blocking
// motores
#define MOTOR_PIN_ENA 6  //Azul
#define MOTOR_PIN_IN1 7  //Rojo
#define MOTOR_PIN_IN2 8  //Gris
#define MOTOR_PIN_IN3 9  //Amarillo
#define MOTOR_PIN_IN4 10 //Naraja
#define MOTOR_PIN_ENB 11 //Blanco

#define MOTOR_CALIBRATION_LEFT 1.0
#define MOTOR_CALIBRATION_RIGHT 1.0

// ultrasonico
#define ULTRA_SENSOR_RIGHT A0  //amarillo
#define ULTRA_SENSOR_LEFT A1   //maron 
#define MAX_DISTANCE 100
#define MIN_DISTANCE 0 

// Constantes de distancia
#define CRITICAL_DISTANCE 20   // Si la distancia es menor
#define AVOID_DISTANCE 40      // Si la distancia está entre CRITICAL y AVOID, gira

// Velocidades
#define SPIN_SPEED 100         // Velocidad para girar sobre sí mismo
#define BACKWARD_SPEED 100     // Velocidad al retroceder
#define MAX_VELOCITY 200       // Velocidad máxima
#define MIN_VELOCITY 50       // Velocidad mínima para giros


Motor motorLeft(MOTOR_PIN_ENA, MOTOR_PIN_IN1, MOTOR_PIN_IN2, Motor::INVERTED);
Motor motorRight(MOTOR_PIN_ENB, MOTOR_PIN_IN3, MOTOR_PIN_IN4, Motor::NORMAL);
MotorController motorController(motorLeft, motorRight, MOTOR_CALIBRATION_LEFT, MOTOR_CALIBRATION_RIGHT);

Sonar sonar(ULTRA_SENSOR_RIGHT, ULTRA_SENSOR_LEFT, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);
  motorController.setup();
  sonar.setup();
}

void loop() {
  sonar.update();
  
  int distance = sonar.getDistance();
  if (distance > 0 && distance <= CRITICAL_DISTANCE) {
    // Obstáculo muy cerca, gira sobre sí mismo
    Serial.println("Obstáculo muy cerca, girando");
    if(Sonar::CloserSide::RIGHT == sonar.getCloserSide()){
      motorController.spinInPlaceRight(SPIN_SPEED);
    } else {
      motorController.spinInPlaceLeft(SPIN_SPEED);
    }
  } 
  else if (distance > CRITICAL_DISTANCE && distance <= AVOID_DISTANCE) {
    // Obstáculo cerca, intenatr evitarlo
    Serial.println("Obstáculo detectado, evitando");
    if(Sonar::CloserSide::RIGHT == sonar.getCloserSide()){
      //TODO: si funciona bien esto pasar a una funcion del motorController
      int leftMotorVelocity = constrain(map(distance, CRITICAL_DISTANCE, AVOID_DISTANCE, MIN_VELOCITY, MAX_VELOCITY), MIN_VELOCITY, MAX_VELOCITY);
      motorController.dynamicTurn(leftMotorVelocity, MAX_VELOCITY);
    } else {
      int rightMotorVelocity = constrain(map(distance, CRITICAL_DISTANCE, AVOID_DISTANCE, MIN_VELOCITY, MAX_VELOCITY), MIN_VELOCITY, MAX_VELOCITY);
      motorController.dynamicTurn(MAX_VELOCITY, rightMotorVelocity);
    }
  } 
  else {
    // No hay obstáculo, avanza
    Serial.println("No hay obstáculo, avanzando");
    motorController.moveForward(MAX_VELOCITY); 
  }
    
}

// Motor motorLeft(MOTOR_PIN_ENA, MOTOR_PIN_IN1, MOTOR_PIN_IN2, Motor::INVERTED);
// Motor motorRight(MOTOR_PIN_ENB, MOTOR_PIN_IN3, MOTOR_PIN_IN4, Motor::NORMAL);
// MotorController motorController(motorLeft, motorRight, MOTOR_CALIBRATION_LEFT, MOTOR_CALIBRATION_RIGHT);

// void TimeOut(AsyncSonar& sonar);
// void PingRecieved(AsyncSonar& sonar);

// AsyncSonar sonar_right(ULTRA_SENSOR_RIGHT, PingRecieved, TimeOut);
// AsyncSonar sonar_left(ULTRA_SENSOR_LEFT, PingRecieved, TimeOut);

// volatile int rightDistance = MAX_DISTANCE + 1;
// volatile int leftDistance = MAX_DISTANCE + 1;

// void setup() {
//   Serial.begin(9600);
//   motorController.setup();

//   sonar_right.SetTimeOutDistance(MAX_DISTANCE*10);
//   sonar_left.SetTimeOutDistance(MAX_DISTANCE*10);

//   sonar_left.Start();
//   sonar_right.Start();
// }

// void loop() {

//   //sendPingIfReady(); 
// 	sonar_right.Update(&sonar_right);
// 	sonar_left.Update(&sonar_left);

//   //por ahora toma la distancia mas larga
//   int currentDistance =  rightDistance < leftDistance ? rightDistance : leftDistance;



// }

// void TimeOut(AsyncSonar& sonar){
//   if(&sonar == &sonar_right){
//     rightDistance = MAX_DISTANCE + 1;
//   } else {
//     leftDistance = MAX_DISTANCE + 1;
//   }
// }

// void PingRecieved(AsyncSonar& sonar){
//   if(&sonar == &sonar_right){
//     rightDistance = sonar.GetRawMM()/10;
//   } else {
//     leftDistance = sonar.GetRawMM()/10;
//   }
// }