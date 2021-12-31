// External libraries

#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

// Global Variables

#define TURN_SPEED 100
#define CALIBRATE_SPEED 200
#define MOVE_SPEED 150

#define LED 13
#define HALT 0
#define NO_SENSORS 5

//setup library class objects
ZumoMotors motors;
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);
ZumoBuzzer buzzer;

//setup general variables
int robotMode;
int roomNumber = 0;
int calibrateData[NO_SENSORS];
char incomingByte;

void setup() {
    Serial.begin(9600);
    incomingByte = Serial.read();
    Serial.println("Zumo Ready To Go!");
    while (incomingByte != 'c')
    {
        incomingByte = (char) Serial.read();
    }

    robotMode = 0;
}

void loop() {

    switch(robotMode) {
        case 0:
            manualControl();
            break;
        case 1:
            autoMode();
            break;
    }
}

void manualControl() {

};

void autoMode() {

};

void calibrateZumo() {
    Serial.println("Calibrating sensors...");
    sensors.init();
    
    for (int i = 0; i < 160; i++)
    {
        // Developed from the maze solver, however uses higher range.
        if ((i > 10 && i <= 30) || (i > 50 && i <= 70) || (i > 90 && i <= 110) ||  (i > 130 && i <= 150) )
        {
            // Turn Zumo Right
             motors.setSpeeds(-CALIBRATE_SPEED, CALIBRATE_SPEED);

        }
        else
        {
            // Turn Zumo Left
            motors.setSpeeds(CALIBRATE_SPEED, -CALIBRATE_SPEED);
        }

        sensors.calibrate();
        delay(10);
    }

    for (int i = 0; i < NO_SENSORS; i++)
    {
        calibrateData[i] = sensors.calibratedMaximumOn[i];
    }
    motors.setSpeeds(HALT, HALT);
    buzzer.play(">g32>>c32");
    Serial.println("Calibration has completed");

};