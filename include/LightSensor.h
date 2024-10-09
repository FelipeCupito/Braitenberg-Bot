#ifndef LIGHTSENSOR_H
#define LIGHTSENSOR_H

#include <Arduino.h>

class LightSensor {
  public:
    // Enum to define the positions of the sensors
    enum SensorPosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        LEFT,
        RIGHT,
        REAR,
        NONE,
        NUM_SENSORS = NONE // Represents the total number of sensors
    };

    // Constructor that initializes the sensor pins
    LightSensor(const int sensorPins[NUM_SENSORS]);

    // Public methods
    void setup();                       // Initializes sensor pins and calibration
    void update();                      // Updates sensor readings and determines direction
    int getLightIntensity(SensorPosition position);  // Returns the intensity of light for a specific sensor
    SensorPosition getStrongestLightSensor(); // Returns the strongest light direction based on current readings
    void calibrate();                   // Calibrates the sensors based on ambient light

  private:
    // Private attributes
    int sensorPins[NUM_SENSORS];        // Array of sensor pins
    int sensorValues[NUM_SENSORS];      // Array of raw sensor values
    int globalBaselineValue;            // Baseline value for ambient light across all sensors
    
    // Memory for past sensor states
    SensorPosition lastDirection;           // Last known direction based on the strongest light
    unsigned long directionChangeStartTime; // Timestamp when the direction was last changed
    int lastIntensity;                      // Last known light intensity (not used)
    
    // Private methods
    void readSensors();                 // Reads the raw values from each sensor
    void applyCalibration();            // Adjusts sensor values based on the global baseline
    void decideDirection();             // Determines the direction based on sensor data
};

#endif