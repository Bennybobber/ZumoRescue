// External libraries

#include <Zumo32U4Motors.h>
#include <ZumoBuzzer.h>
#include <QTRSensors.h>
#include <Zumo32U4ProximitySensors.h>
#include <ZumoReflectanceSensorArray.h>

// Global Variables

#define TURN_SPEED 100
#define CALIBRATE_SPEED 100
#define MOVE_SPEED 200
#define AUTO_SPEED 75

#define LED 13
#define HALT 0
#define NO_SENSORS 3
#define MAX_SONAR_DISTANCE 50
#define LEFT 0
#define RIGHT 1
#define IR_THRESHOLD = 1

//setup library class objects
Zumo32U4Motors motors;
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);
ZumoBuzzer buzzer;
Zumo32U4ProximitySensors proxSensors;

//setup general variables
unsigned int line_sensor_values[NO_SENSORS];
int robotMode;
int noOfRooms = 0;
unsigned int calibrateData[NO_SENSORS];
int leftValue = 0; // left value for the proximity sensor
int rightValue = 0; // right value for the promixty sensor
char incomingByte;
int leftProximityCount;
int rightProximityCount;
String rooms[10];
String room;
bool sensDir = RIGHT; // last indication of the direction of an object

void setup() {
    Serial.begin(9600);
    incomingByte = Serial.read();
    Serial.println("Zumo Ready To Go!");
    while (incomingByte != 'c')
    {
        incomingByte = (char) Serial.read();
    }
    proxSensors.initFrontSensor();
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
        case 'r':
            logRoom();
            break;
        case 'l':
            // Run the search room function
            searchRoom();
            break;
        case '1':
            // Change to auto mode and reset the proximity counts.
            leftProximityCount = 0;
            rightProximityCount = 0;
            Serial.println("Auto pilot activated");
            buzzer.play(">g32>>c32");
            robotMode = 1;
            break;

    }
};
void searchRoom() {
    Serial.println("Searching Room...");
    // Move zumo into the room.
    motors.setSpeeds(MOVE_SPEED, MOVE_SPEED);
    delay(250);
    motors.setSpeeds(HALT, HALT);

    for (int i = 0; i < 160; i++)
    {
        // Spin zumo around, and scan using the IR sensors
        proxSensors.read();
        
        uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
        uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
        Serial.println(leftValue);
        Serial.println(rightValue);
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
        //If a person is found, it's time to stop moving and inform the user
        bool objectSeen = leftValue >= 5 || rightValue >= 5;
        if (objectSeen){
            Serial.println("We have found a PERSON");
            motors.setSpeeds(HALT, HALT);
            return; 
        }
    }
    //Stop the motors if there is nothing found
    Serial.println("No PERSON found");
    motors.setSpeeds(HALT, HALT);


}
void autoMode() {
    incomingByte = Serial.read();
    sensors.read(line_sensor_values);
    Serial.println("Line data: ");
    Serial.println(line_sensor_values[0]);
    Serial.println(line_sensor_values[1]);
    Serial.println(line_sensor_values[2]);
    Serial.println("Calibrate dataZ: ");
    Serial.println(calibrateData[0]);
    Serial.println(calibrateData[1]);
    Serial.println(calibrateData[2]);
    // Check if a stop command is being sent
    if ( incomingByte == 'z' )
    {
        motors.setSpeeds(HALT, HALT);
        robotMode = 0;
    }
    else if ( incomingByte == 'r' )
    {
        logRoom();
    }
    else if ((line_sensor_values[0] > calibrateData[0] ) && (line_sensor_values[1] > calibrateData[1] + 200 ) || (line_sensor_values[1] > calibrateData[1] + 200 ) && (line_sensor_values[2] > calibrateData[2] )) {

    // if the middle sensors detect line, stop
    motors.setSpeeds(HALT, HALT);
    Serial.println("Wall Detected! 1");
    Serial.println("Manual Mode activated");
    robotMode = 0;
    }

    else if (line_sensor_values[2] >= calibrateData[2]+150)
    {
        // if rightmost sensor detects line, reverse and turn to the left
        // motors.setSpeeds(-MOVE_SPEED, -MOVE_SPEED);
        Serial.println("Correcting left...");
        delay(200);
        motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        delay(150);
        motors.setSpeeds(AUTO_SPEED, AUTO_SPEED);
        // leftProximityCount++;
        if (leftProximityCount > 3)
        {
            Serial.println("Wall Detected! 2");
            Serial.println("Manual Mode activated");
            robotMode = 0;
            motors.setSpeeds(HALT, HALT);
        }
        robotMode = 1;
    }
    else if (line_sensor_values[0] >= calibrateData[0]+150) {

        // if leftmost sensor detects line, reverse and turn to the right
        // motors.setSpeeds(-MOVE_SPEED, -MOVE_SPEED);
        Serial.println("Correcting right...");
        delay(200);
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        delay(150);
        motors.setSpeeds(AUTO_SPEED, AUTO_SPEED);
    // rightProximityCount++;
        if (rightProximityCount > 3)
        {
        Serial.println("Wall Detected! 3");
        Serial.println("Manual Mode activated");
        motors.setSpeeds(HALT, HALT);
        robotMode = 0;
        }
        robotMode = 1;
    }
    else
    {
        // otherwise, go straight
        Serial.println("Made it here...");
        motors.setSpeeds(AUTO_SPEED, AUTO_SPEED);
        robotMode = 1;
    }
    // 

};

void calibrateZumo() {
    Serial.println("Calibrating sensors...");
    byte pins[] = {A0, A3, 12};
    sensors.init(pins, 3);
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
        delay(5);
    }

    for (int i = 0; i < NO_SENSORS; i++)
    {
        calibrateData[i] = sensors.calibratedMaximumOn[i];
    }
    motors.setSpeeds(HALT, HALT);
    buzzer.play(">g32>>c32");
    Serial.println("Calibrate data: ");
    Serial.println(calibrateData[0]);
    Serial.println(calibrateData[1]);
    Serial.println(calibrateData[2]);
    // Serial.println(calibrateData[4]);
    // Serial.println(calibrateData[5]);
    // Serial.println(calibrateData[6]);
    Serial.println("Calibration has completed");

};


void logRoom() {
    Serial.println("Stopping for room...");
    delay(100);
    Serial.println("Please indicate position of room (L or R)");
    motors.setSpeeds(HALT, HALT);
    incomingByte = ' ';
    while ((incomingByte != 'l') && (incomingByte != 'r'))
    {
        incomingByte = (char) Serial.read();
    }
    // Increase the room counter by 1 then log which side the room is on.
    noOfRooms++;
    if (incomingByte == 'l')
    {
        rooms[noOfRooms] = "left";
    }
    else
    {
        rooms[noOfRooms] = "right";
    }
    Serial.print("Room Number: ");
    Serial.print(noOfRooms);
    Serial.print(" is on the ");
    Serial.println(rooms[noOfRooms]);
};