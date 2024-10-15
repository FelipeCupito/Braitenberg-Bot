#include "LightSensor.h"

#define CALIBRATION_SAMPLES 200 // Number of samples for calibration

// Constructor initializes sensor pins and sets initial values for attributes
LightSensor::LightSensor(const int sensorPins[NUM_SENSORS]) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        this->sensorPins[i] = sensorPins[i];
        sensorValues[i] = 0;        
    }

    globalBaselineValue = 0;        // No initial calibration
    lastDirection = NONE;           // No initial direction
    lastIntensity = 0;              // No initial intensity
    directionChangeStartTime = 0;   // No initial direction change
}

int LightSensor::getGlobalBaselineValue(){
    return globalBaselineValue;
}

// Sets up the sensor pins as input and performs calibration
void LightSensor::setup() {
    Serial.println("Iniciando configuracion de LightSensor");
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
    calibrate();
}

// Calibrates the sensors by taking the average ambient light value across all sensors
void LightSensor::calibrate() {
    const int calibrationSamples = CALIBRATION_SAMPLES;
    unsigned long totalSum = 0;

    Serial.println("Iniciando calibración de sensores...");

    for (int i = 0; i < calibrationSamples; i++) {
        readSensors();
        for (int j = 0; j < NUM_SENSORS; j++) {
            totalSum += sensorValues[j];
        }
        delay(10);
        Serial.print("Right: ");
        Serial.print(sensorValues[FRONT_RIGHT]);
        Serial.print("         LEft : ");
        Serial.println(sensorValues[FRONT_LEFT]);
    }

    globalBaselineValue = totalSum / (calibrationSamples * NUM_SENSORS);
    globalBaselineValue = globalBaselineValue/2;
    
    Serial.print("Calibración completada. Valor base global: ");
    Serial.println(globalBaselineValue);
}

// Updates sensor readings, applies calibration, and decides the current direction
void LightSensor::update() {
    readSensors();
    applyCalibration();
    decideDirection();
    
    Serial.print("Left: ");
    Serial.print(sensorValues[FRONT_LEFT]);
    Serial.print("          Right: ");
    Serial.println(sensorValues[FRONT_RIGHT]);
}

 int LightSensor::getLightIntensity(SensorPosition position){
    return sensorValues[position];
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
        int calibratedValue = sensorValues[i] - globalBaselineValue;

        if (calibratedValue < 0) {
            calibratedValue = 0;
        }
        sensorValues[i] = calibratedValue;
    }
}

// Determines the direction based on the highest intensity detected among sensors
void LightSensor::decideDirection() {
    // // Leer los valores actuales de los sensores
    // int frontLeftIntensity = (int)sensorValues[FRONT_LEFT];
    // int frontRightIntensity = (int)sensorValues[FRONT_RIGHT];
    // //int leftIntensity = (int)sensorValues[LEFT];
    // //int rightIntensity = (int)sensorValues[RIGHT];
    // //int rearIntensity = (int)sensorValues[REAR];

    // // Calcular la intensidad promedio de los sensores delanteros
    // int frontAverageIntensity = (frontLeftIntensity + frontRightIntensity) / 2;

    // // Por defecto, se considera la luz más fuerte al frente
    // int maxIntensity = frontAverageIntensity;
    // SensorPosition maxPosition = NONE;

    // // Umbrales ajustados para filtrar lecturas erróneas y asegurar un seguimiento a luz intensa
    // const int absoluteIntensityThreshold = 300;  // Luz intensa como una linterna
    // const int relativeIntensityThreshold = 100;  // Diferencia significativa respecto al frente
    // const unsigned long persistenceTime = 500;   // Tiempo de persistencia para confirmar un cambio

    // // Estructura para verificar los sensores laterales y trasero
    // struct SensorData {
    //     SensorPosition position;
    //     int intensity;
    // } otherSensors[] = {
    //     //{LEFT, leftIntensity},
    //     //{RIGHT, rightIntensity},
    //     //{REAR, rearIntensity}
    // };

    // // Verificar si alguno de los sensores laterales o trasero detecta una luz más fuerte que la del frente
    // for (int i = 0; i < 3; i++) {
    //     int intensityDifference = otherSensors[i].intensity - frontAverageIntensity;

    //     // La luz detectada por este sensor debe ser significativamente más fuerte que la del frente
    //     if (otherSensors[i].intensity >= absoluteIntensityThreshold && intensityDifference >= relativeIntensityThreshold) {
    //         if (otherSensors[i].intensity > maxIntensity) {
    //             maxIntensity = otherSensors[i].intensity;
    //             maxPosition = otherSensors[i].position;
    //         }
    //     }
    // }

    // // Si la nueva dirección es la frontal, cambia inmediatamente sin esperar el tiempo de persistencia
    // if (maxPosition == NONE) {
    //     // Detectamos que el frente tiene la luz más fuerte, cambiar de inmediato
    //     if (lastDirection != NONE) {
    //         lastDirection = NONE;  // Restablecer la dirección al frente
    //         directionChangeStartTime = 0;  // Resetear el temporizador porque estamos en dirección al frente
    //         Serial.println("Cambio directo hacia el frente.");
    //     }
    //     return;  // No continuar, ya que no hay necesidad de persistencia
    // }

    // unsigned long currentTime = millis();

    // // Verificar si la dirección ha cambiado
    // if (maxPosition != lastDirection) {
    //     // Posible cambio de dirección, iniciar temporizador
    //    if (currentTime - directionChangeStartTime >= persistenceTime) {
    //         // La nueva dirección ha persistido el tiempo suficiente para considerarla válida
    //         lastDirection = maxPosition;  // Cambiar la dirección
    //         directionChangeStartTime = currentTime;  // Actualizar el tiempo de cambio
    //         Serial.print("Cambio de dirección confirmado a: ");
    //         Serial.println(maxPosition);
    //     }
    // }
}


// Returns the strongest light sensor direction based on the internal logic
LightSensor::SensorPosition LightSensor::getStrongestLightSensor() {
    return lastDirection;
}
