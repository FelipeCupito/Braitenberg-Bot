#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>
#include "AsyncSonarLib.h"

class Sonar {
  public:
    enum CloserSide {
      RIGHT,
      LEFT,
      EQUAL,
      NONE
    };

    Sonar(int triggerPinRight, int triggerPinLeft, int maxDistance);
    void setup();
    void update();
    int getRightDistance();
    int getLeftDistance();
    bool isObstacleClose();
    int getDistance();
    int getProcessDistance();
    CloserSide getCloserSide();

  private:
    int triggerPinRight;
    int triggerPinLeft;
    int maxDistance;
    static volatile int rightDistance;
    static volatile int leftDistance;
    AsyncSonar sonarRight;
    AsyncSonar sonarLeft;

    static void TimeOut(AsyncSonar& sonar);
    static void PingRecieved(AsyncSonar& sonar);

    static Sonar* instance;
};

#endif
