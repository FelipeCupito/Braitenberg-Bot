#include "Sonar.h"

// Inicializamos las distancias estáticas
volatile int Sonar::rightDistance = 0;
volatile int Sonar::leftDistance = 0;

// Inicializamos la instancia estática
Sonar* Sonar::instance = nullptr;

Sonar::Sonar(int triggerPinRight, int triggerPinLeft, int maxDistance) 
    : triggerPinRight(triggerPinRight), triggerPinLeft(triggerPinLeft), maxDistance(maxDistance),
      sonarRight(triggerPinRight, PingRecieved, TimeOut), sonarLeft(triggerPinLeft, PingRecieved, TimeOut) {

  instance = this;
}

void Sonar::setup() {
  sonarRight.SetTimeOutDistance(maxDistance * 10);
  sonarLeft.SetTimeOutDistance(maxDistance * 10);
  
  sonarRight.Start();
  sonarLeft.Start();
}

void Sonar::update() {
  sonarRight.Update(&sonarRight);
  sonarLeft.Update(&sonarLeft);
}

int Sonar::getRightDistance() {
  return rightDistance;
}

int Sonar::getLeftDistance() {
  return leftDistance;
}

bool Sonar::isObstacleClose() {
  return (rightDistance <= maxDistance || leftDistance <= maxDistance);
}

int Sonar::getDistance() {
  return rightDistance < leftDistance ? rightDistance : leftDistance;
}

//funcion que devulva el proceso de la distancia
int Sonar::getProcessDistance() {
  return  (rightDistance + leftDistance) / 2;
}

Sonar::CloserSide Sonar::getCloserSide() {
  if (rightDistance < leftDistance) {
    return RIGHT;
  } else if (leftDistance < rightDistance) {
    return LEFT;
  } else if (rightDistance == leftDistance && rightDistance <= maxDistance) {
    return EQUAL;
  } else {
    return NONE;
  }
}

// Función estática llamada cuando un ping es recibido
void Sonar::PingRecieved(AsyncSonar& sonar) {
  if (&sonar == &instance->sonarRight) {
    instance->rightDistance = sonar.GetRawMM() / 10;

  } else if (&sonar == &instance->sonarLeft) {
    instance->leftDistance = sonar.GetRawMM() / 10;
  }
}

// Función estática llamada en caso de timeout
void Sonar::TimeOut(AsyncSonar& sonar) {
  if (&sonar == &instance->sonarRight) {
    instance->rightDistance = instance->maxDistance + 1;

  } else if (&sonar == &instance->sonarLeft) {
    instance->leftDistance = instance->maxDistance + 1;
  }
}
