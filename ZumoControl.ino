// External libraries

#include <Zumo32U4Motors.h>
#include <Zumo32U4Buzzer.h>
#include <Zumo32U4ProximitySensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <Zumo32U4IMU.h>

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
Zumo32U4Buzzer buzzer;
Zumo32U4IMU gyro;

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
int stopCount;
String rooms[10];
String people[10];
String room;
bool sensDir = RIGHT; // last indication of the direction of an object

void setup() {
    Serial1.begin(9600);
    incomingByte = Serial1.read();
    Serial1.println("Zumo Ready To Go!");
    while (incomingByte != 'c')
    {
        incomingByte = (char) Serial1.read();
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
    incomingByte = Serial1.read();

    // Check what the user wants to do.
    switch (incomingByte) {
        case 'w':
            Serial1.println("Zumo Moving Forward...");
            motors.setSpeeds(MOVE_SPEED, MOVE_SPEED);
            delay(50);
            motors.setSpeeds(HALT, HALT);
            break;
        case 'a':
            Serial1.println("Zumo Turning Left...");
            // Defining each motor to make it clearer which motor is moving forward, and which is moving backwards.
            motors.setLeftSpeed(-MOVE_SPEED);
            motors.setRightSpeed(MOVE_SPEED);
            delay(50);
            motors.setSpeeds(HALT, HALT);
            break;
        
        case 'd':
            // Defining each motor to make it clearer which motor is moving forward, and which is moving backwards.
            Serial1.println("Zumo Turning Right...");
            motors.setLeftSpeed(MOVE_SPEED);
            motors.setRightSpeed(-MOVE_SPEED);
            delay(50);
            motors.setSpeeds(HALT, HALT);
            break;
        
        case 's':
            Serial1.println("Zumo Reversing...");
            motors.setSpeeds(-MOVE_SPEED, -MOVE_SPEED);
            delay(50);
            motors.setSpeeds(HALT, HALT);
            break;
        case 'e':
            Serial1.println("Zumo turning right 90 degrees...");
            turn90(TURN_SPEED, -TURN_SPEED);
            break;
        case 'q':
            Serial1.println("Zumo turning left 90 degrees...");
            turn90(-TURN_SPEED, TURN_SPEED);
            break;
        case 'h':
            Serial1.println("Tooting The Horn!");
            buzzer.playNote(NOTE_A_SHARP(5), 250, 90);
            break;
        case 'z':
            Serial1.println("Emergency Stop Initiated!");
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
            Serial1.println("Auto pilot activated");
            buzzer.play(">g32>>c32");
            robotMode = 1;
            break;

    }
};
void searchRoom() {
    Serial1.println("Searching Room...");
    Zumo32U4ProximitySensors proxSensors;
    proxSensors.initFrontSensor(SENSOR_NO_PIN);
    bool objectSeen = false;
    // make the turn for the room
    if (rooms[noOfRooms] == "left")
    {
        turn90(-TURN_SPEED, TURN_SPEED);
    }
    else
    {
        turn90(TURN_SPEED, -TURN_SPEED);
    }
    // Move zumo into the room.
    motors.setSpeeds(AUTO_SPEED, AUTO_SPEED);
    delay(700);
    motors.setSpeeds(HALT, HALT);
    for (int i = 0; i < 160; i++)
    {
        // Spin zumo around, and scan using the IR sensors
        proxSensors.read();
        
        uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
        uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
        // uint8_t leftValue = 1;
        // uint8_t rightValue = 1;
        Serial1.println(leftValue);
        Serial1.println(rightValue);
        if ((i > 10 && i <= 30) || (i > 50 && i <= 70) || (i > 90 && i <= 110) ||  (i > 130 && i <= 150) )
        {
            // Turn Zumo Right
             motors.setSpeeds(-TURN_SPEED, TURN_SPEED);


        }
        else
        {
            // Turn Zumo Left
            motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        }
        delay(15);
        //If a person is found, it's time to stop moving and inform the user
        objectSeen = leftValue >= 5 || rightValue >= 5;
        
    }
    motors.setSpeeds(HALT, HALT);
    //Stop the motors if there is nothing found
    if (objectSeen)
    {
        Serial1.println("We have found a PERSON");
        people[noOfRooms] = "true";
    }
    else
    {
        Serial.println("No PERSON found");
        people[noOfRooms] = "false";
    }
    // Leave the room
    Serial.println("Returning to corridor");
    motors.setSpeeds(-AUTO_SPEED, -AUTO_SPEED);
    delay(500);
    motors.setSpeeds(HALT, HALT);
    if (rooms[noOfRooms] == "left")
    {
        turn90(TURN_SPEED, -TURN_SPEED);
    }
    else
    {
        turn90(-TURN_SPEED, TURN_SPEED);
    }  


}
void autoMode() {
    incomingByte = Serial1.read();
    sensors.read(line_sensor_values);
    if ( incomingByte == 'z' )
    {
        // Stop command recived so STOP and go back to manual control
        motors.setSpeeds(HALT, HALT);
        robotMode = 0;
    }
    else if ( incomingByte == 'r' )
    {   
        //Allow user to log the room before moving forward
        logRoom();
        searchRoom();
        robotMode = 1;
    }
    else if ((line_sensor_values[0] > calibrateData[0] ) && (line_sensor_values[1] > calibrateData[1] ) || (line_sensor_values[1] > calibrateData[1] ) && (line_sensor_values[2] > calibrateData[2] )) {

    // if the middle sensors detect line, reverse away then stop.
    motors.setSpeeds(-TURN_SPEED, -TURN_SPEED);
    delay(250);
    motors.setSpeeds(HALT, HALT);
    Serial1.println("Wall Detected!");
    Serial1.println("Manual Mode activated");
    // We have reached the end of a corridor, so increase the stop count by 1.
    stopCount++;
    // Enter junction to perform turn
    junction();
    }

    else if (line_sensor_values[2] >= calibrateData[2])
    {
        // If right most line sensor detects line move left.
        Serial1.println("Correcting left...");
        delay(200);
        motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        delay(150);
        motors.setSpeeds(AUTO_SPEED, AUTO_SPEED);
        robotMode = 1;
    }
    else if (line_sensor_values[0] >= calibrateData[0]) {

        // If left most line sensor detects line move right.
        Serial1.println("Correcting right...");
        delay(200);
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        delay(150);
        motors.setSpeeds(AUTO_SPEED, AUTO_SPEED);
        robotMode = 1;
    }
    else
    {
        // Go straight forward if not hitting any line
        Serial1.println("Going forward...");
        motors.setSpeeds(AUTO_SPEED, AUTO_SPEED);
        robotMode = 1;
    }

};

void calibrateZumo() {
    Serial1.println("Calibrating sensors...");
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
        //Get the maximums (lightest areas) then add 150 to add a margin of error.
        calibrateData[i] = sensors.calibratedMaximumOn[i] + 200;
    }
    motors.setSpeeds(HALT, HALT);
    buzzer.play(">g32>>c32");
    Serial.println("Calibration has completed");

};


void logRoom() {
    Serial1.println("Stopping for room...");
    delay(100);
    Serial1.println("Please indicate position of room (L or R)");
    motors.setSpeeds(HALT, HALT);
    incomingByte = ' ';
    while ((incomingByte != 'l') && (incomingByte != 'r'))
    {
        incomingByte = (char) Serial1.read();
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
    Serial1.print("Room Number: ");
    Serial1.print(noOfRooms);
    Serial1.print(" is on the ");
    Serial1.println(rooms[noOfRooms]);
};

void junction() {
    Serial1.println("Wall encountered!");
    incomingByte = ' ';
    if (stopCount == 1)
    {
        Serial1.println("Do you wish to go left, or right?");
        while ((incomingByte != 'l') && (incomingByte != 'r'))
        {
            incomingByte = (char) Serial1.read();
        }

        if (incomingByte == 'l')
        {
            Serial1.println("Turning left down the T junction...");
            turn90(-TURN_SPEED, TURN_SPEED);
        }
        else 
        {
            Serial1.println("Turning right down the T junction...");
            turn90(TURN_SPEED, -TURN_SPEED);
        }
        //Go back to auto mode
        Serial1.println("Continuing Search!");
        robotMode = 1;
    }
    else if (stopCount == 2)
    {
        // Wait until the B key is pressed
        Serial1.println("Press 'B' to turn 180 degrees and continue.");
        while ((incomingByte != 'b'))
        {
            incomingByte = (char) Serial1.read();
        }
        Serial.println("Starting 180 degree turn...");
        turn90(-TURN_SPEED, TURN_SPEED);
        turn90(-TURN_SPEED, TURN_SPEED);
        Serial1.println("Continuing Search!");
        //Go back to auto mode
        robotMode = 1;
    }
    else if (stopCount == 3) {
        Serial1.println("We've reached the end of the journey.");
        robotMode = 0;
    }

}

void turn90(int leftSpeed, int rightSpeed) {
    motors.setSpeeds(leftSpeed, rightSpeed);
    delay(1700);
    motors.setSpeeds(HALT, HALT);
}