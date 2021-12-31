// External libraries

#include <Zumo32U4Motors.h>
#include <ZumoBuzzer.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

// Global Variables

#define TURN_SPEED 100
#define CALIBRATE_SPEED 150
#define MOVE_SPEED 200

#define LED 13
#define HALT 0
#define NO_SENSORS 5

//setup library class objects
Zumo32U4Motors motors;
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
    calibrateZumo();
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
    incomingByte = Serial.read();

    // Check what the user wants to do.
    switch (incomingByte) {
        case 'w':
            Serial.println("Zumo Moving Forward...");
            motors.setSpeeds(MOVE_SPEED, MOVE_SPEED);
            delay(150);
            motors.setSpeeds(HALT, HALT);
            break;
        case 'a':
            Serial.println("Zumo Turning Left...");
            // Defining each motor to make it clearer which motor is moving forward, and which is moving backwards.
            motors.setLeftSpeed(-MOVE_SPEED);
            motors.setRightSpeed(MOVE_SPEED);
            delay(150);
            motors.setSpeeds(HALT, HALT);
            break;
        
        case 'd':
            // Defining each motor to make it clearer which motor is moving forward, and which is moving backwards.
            Serial.println("Zumo Turning Right...");
            motors.setLeftSpeed(MOVE_SPEED);
            motors.setRightSpeed(-MOVE_SPEED);
            delay(150);
            motors.setSpeeds(HALT, HALT);
            break;
        
        case 's':
            Serial.println("Zumo Reversing...");
            motors.setSpeeds(-MOVE_SPEED, -MOVE_SPEED);
            delay(150);
            motors.setSpeeds(HALT, HALT);
            break;
        case 'e':
            Serial.println("Zumo turning right 90 degrees...");
            motors.setSpeeds(MOVE_SPEED, -TURN_SPEED);
            delay(1050);
            motors.setSpeeds(HALT, HALT);
            break;
        case 'q':
            Serial.println("Zumo turning left 90 degrees...");
            motors.setSpeeds(-TURN_SPEED, MOVE_SPEED);
            delay(1050);
            motors.setSpeeds(HALT, HALT);
            break;
        case 'h':
            Serial.println("Tooting The Horn!");
            buzzer.playNote(NOTE_A_SHARP(5), 250, 90);
            break;
        case 'z':
            Serial.println("Emergency Stop Initiated!");
            motors.setSpeeds(HALT, HALT);
            break;
        case '1':
            Serial.println("Auto pilot activated");
            buzzer.play(">g32>>c32");
            robotMode = 1;
            break;

    }
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