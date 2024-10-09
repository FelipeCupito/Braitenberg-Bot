#include "LightSensor.h"

// Constructor initializes sensor pins and sets initial values for attributes
LightSensor::LightSensor(const int sensorPins[NUM_SENSORS]) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        this->sensorPins[i] = sensorPins[i];
        sensorValues[i] = 0;        
    }

    globalBaselineValue = 0;

    lastDirection = NONE; // No initial direction
    lastIntensity = 0;
    directionChangeStartTime = 0;
}

// Sets up the sensor pins as input and performs calibration
void LightSensor::setup() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
    calibrate();
}

// Calibrates the sensors by taking the average ambient light value across all sensors
void LightSensor::calibrate() {
    const int calibrationSamples = 100; // Number of samples for calibration
    unsigned long totalSum = 0;

    for (int i = 0; i < calibrationSamples; i++) {
        readSensors();
        for (int j = 0; j < NUM_SENSORS; j++) {
            totalSum += sensorValues[j];
        }
        delay(10);// Small delay between samples
    }

    globalBaselineValue = totalSum / (calibrationSamples * NUM_SENSORS);
}

// Updates sensor readings, applies calibration, and decides the current direction
void LightSensor::update() {
    readSensors();
    applyCalibration();
    decideDirection();
}

// Reads the analog value from each sensor and stores it in sensorValues array
void LightSensor::readSensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);
    }
}

// Adjusts the sensor values based on the global baseline
void LightSensor::applyCalibration() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        int calibratedValue = globalBaselineValue - sensorValues[i];
        // Ensure the value is not negative
        if (calibratedValue < 0) {
            calibratedValue = 0;
        }
        sensorValues[i] = calibratedValue;
    }
}

// Returns the light intensity value for a specific sensor position
int LightSensor::getLightIntensity(SensorPosition position) {
    if (position >= 0 && position < NUM_SENSORS) {
        return (int)sensorValues[position];
    } else {
        return -1; // Invalid position
    }
}

// Determines the direction based on the highest intensity detected among sensors
void LightSensor::decideDirection() {
    // Retrieve smoothed values for each sensor
    int frontLeftIntensity = (int)sensorValues[FRONT_LEFT];
    int frontRightIntensity = (int)sensorValues[FRONT_RIGHT];
    int leftIntensity = (int)sensorValues[LEFT];
    int rightIntensity = (int)sensorValues[RIGHT];
    int rearIntensity = (int)sensorValues[REAR];

    // Calculate the average intensity of the front sensors
    int frontAverageIntensity = (frontLeftIntensity + frontRightIntensity) / 2;

    // Default to assuming the strongest light source is in front
    int maxIntensity = frontAverageIntensity;
    SensorPosition maxPosition = NONE;  // Default to no change in direction

    // Constants defining thresholds for intensity and persistence
    const int absoluteIntensityThreshold = 100; // Minimum intensity to consider a significant source
    const int relativeIntensityThreshold = 50;  // Minimum difference from front to consider a direction change
    const unsigned long persistenceTime = 500;  // Minimum time (ms) to confirm a direction change

    // Array of other sensors to check
    struct SensorData {
        SensorPosition position;
        int intensity;
    } otherSensors[] = {
        {LEFT, leftIntensity},
        {RIGHT, rightIntensity},
        {REAR, rearIntensity}
    };

    // Check if any side or rear sensor has a significantly higher intensity than the front average
    for (int i = 0; i < 3; i++) {
        int intensityDifference = otherSensors[i].intensity - frontAverageIntensity;

        if (otherSensors[i].intensity >= absoluteIntensityThreshold && intensityDifference >= relativeIntensityThreshold) {
            if (otherSensors[i].intensity > maxIntensity) {
                maxIntensity = otherSensors[i].intensity;
                maxPosition = otherSensors[i].position;
            }
        }
    }

     unsigned long currentTime = millis();

    if (maxPosition != lastDirection) {
        // Potential new direction detected
        if (directionChangeStartTime == 0) {
            directionChangeStartTime = currentTime; // Start timing
        } else {
            if (currentTime - directionChangeStartTime >= persistenceTime) {
                // New direction has persisted long enough
                lastDirection = maxPosition;
                directionChangeStartTime = 0; // Reset timer
            }
            // Else, continue waiting
        }
    } else {
        // Direction hasn't changed
        directionChangeStartTime = 0; // Reset timer
    }
}

// Returns the strongest light sensor direction based on the internal logic
LightSensor::SensorPosition LightSensor::getStrongestLightSensor() {
    return lastDirection;
}
