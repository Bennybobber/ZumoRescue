/*
	ZumoControl.io

	Executes a search and rescue, as well as being able to control
    the movement of a Zumo device from the serial ports. There is a automatic
    and a manual mode.

    Manual mode: 
    Controls via WASD etc to move the Zumo around.
    Q, E Moves turn the robot approx 90 degrees left or right.
    B I ntiates an emergency stop.
    Z Stops the zumo.
    R Logs the room.
    L Intiates a search of the room.
    1 Switches to automatic mode.

    Automatic mode:
    The Zumo will navigate down a corridor until it reaches a wall and will stop.
    The user can then choose to go left or right (This assumes a T junction corridor)
    The Zumo will automatically move down the corridor after turning 90 degrees either left OR right.
    At the end of the corridor it will wait for the user to press 'b' and then turn approx 180 degrees.
    This will then continue to the end of the corridor and mark the end of it's journey. 

	Created 01 January 2022
	By Benjamin Todd Sinyard
	Modified 07 January 2022
	By Benjamin Todd Sinyard

*/

// External libraries

#include <Zumo32U4Motors.h>
#include <Zumo32U4Buzzer.h>
#include <Zumo32U4ProximitySensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <Zumo32U4Encoders.h>
#include <Wire.h>
// Global Variables

// Movement based variables
#define TURN_SPEED 250
#define CALIBRATE_SPEED 100
#define MOVE_SPEED 350
#define AUTO_SPEED 100
#define HALT 0

// Sesnsor variables
#define NO_SENSORS 3
#define LEFT false
#define RIGHT true
#define IR_THRESHOLD 6

// setup library class objects
Zumo32U4Motors motors;
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);
Zumo32U4Buzzer buzzer;
Zumo32U4Encoders encoders;

// setup general variables
unsigned int line_sensor_values[NO_SENSORS];
int robotMode;
int noOfRooms = 0;
int wallHitCount = 0;
unsigned int calibrateData[NO_SENSORS]; // calibration data for the line sensors
int stopCount;
int leftSpeed;
int rightSpeed;
char incomingByte;
// To keep track of number of rooms, their position and if a person is inside.
String rooms[10];
String people[10];
String room;
bool sensDir = RIGHT; // last indication of the direction of an object
bool listenMessages = true;

/*
	Setup the serial port we're going to use.
    Get the byte and then wait for the user to press the calibrate button.
    
    Then we switch to the default robot mode of 0 (Manual)
*/
void setup() {
    Serial1.begin(9600);
    incomingByte = Serial1.read();
    Serial1.println("Zumo Ready To Calibrate, press C to start");
    while (incomingByte != 'c')
    {
        incomingByte = (char) Serial1.read();
    }
    calibrateZumo();
    robotMode = 0;
}
/*
	Loops around, checking to see if the mode of the robot has changed.
*/
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
/*
	Uses a switch statement to check how the robot should react
    depending on the input given by the user.
*/
void manualControl() {
    //This will speed down the engine unless a command to speed up is received
    if(abs(rightSpeed)>0){rightSpeed=rightSpeed-rightSpeed/5;}
    if(abs(leftSpeed)>0){leftSpeed=leftSpeed-leftSpeed/5;}
    motors.setRightSpeed(rightSpeed);
    motors.setLeftSpeed(leftSpeed);
    incomingByte = Serial1.read();

    // Check what the user wants to do.
    switch (incomingByte) {
        case 'w':
            Serial1.println("Zumo Moving Forward...");
            leftSpeed = MOVE_SPEED;
            rightSpeed = MOVE_SPEED;
            break;
        case 'a':
            Serial1.println("Zumo Turning Left...");
            leftSpeed = -MOVE_SPEED;
            rightSpeed = MOVE_SPEED;
            break;
        
        case 'd':
            Serial1.println("Zumo Turning Right...");
            leftSpeed = MOVE_SPEED;
            rightSpeed = -MOVE_SPEED;
            break;
        
        case 's':
            Serial1.println("Zumo Reversing...");
            leftSpeed = -MOVE_SPEED;
            rightSpeed = -MOVE_SPEED;
            break;
        case 'e':
            Serial1.println("Zumo turning right 90 degrees...");
            turn90(RIGHT);
            break;
        case 'q':
            Serial1.println("Zumo turning left 90 degrees...");
            turn90(LEFT);
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
            // Run logRoom function
            logRoom();
            break;
        case 'l':
            // Run the search room function
            searchRoom();
            break;
        case '1':
            // Change to auto mode.
            Serial1.println("Auto pilot activated");
            buzzer.play(">g32>>c32");
            robotMode = 1;
            break;
    
    }
    delay(3);
};
/*
	Makes the Zumo turn and search a room depending on the last logged
    room location.

    Once the search is complete it reports back if anyone was found and the location
    that the search took place before moving back to the corridor to continue the search.
*/
void searchRoom() {
    Serial1.println("Searching Room...");
    Zumo32U4ProximitySensors proxSensors;
    proxSensors.initFrontSensor(SENSOR_NO_PIN);
    bool objectSeen = false;
    // make the turn for the room
    if (rooms[noOfRooms] == "left")
    {
        turn90(LEFT);
    }
    else
    {
        turn90(RIGHT);
    }
    // Move zumo into the room.
    motors.setSpeeds(AUTO_SPEED, AUTO_SPEED);
    delay(800);
    motors.setSpeeds(HALT, HALT);
    for (int i = 0; i < 40; i++)
    {
        // Spin zumo around, and scan using the IR sensors
        proxSensors.read();
        uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
        uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
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
        delay(60);
        //If a person is found, it's time to stop moving and inform the user
        if (leftValue == IR_THRESHOLD || rightValue == IR_THRESHOLD)
            objectSeen = true;
        
    }
    motors.setSpeeds(HALT, HALT);
    //Stop the motors if there is nothing found
    if (objectSeen)
    {
        Serial1.println(" ");
        Serial1.print("Person FOUND in room: ");
        Serial1.print(noOfRooms);
        Serial1.print(" on the ");
        Serial1.print(rooms[noOfRooms]);
        buzzer.playNote(NOTE_A_SHARP(5), 250, 90);
        people[noOfRooms] = "true";
    }
    else
    {
        Serial1.println("");
        Serial1.print("No PERSON in room: ");
        Serial1.print(noOfRooms);
        Serial1.print(" on the ");
        Serial1.print(rooms[noOfRooms]);
        people[noOfRooms] = "false";
    }
    // Wait 2 seconds to show message before leaving.
    delay(2000);
    Serial1.println("");
    Serial1.println("Returning to corridor");
    // Reverse out of the room.
    motors.setSpeeds(-AUTO_SPEED, -AUTO_SPEED);
    delay(750);
    motors.setSpeeds(HALT, HALT);
    if (rooms[noOfRooms] == "left")
    {
        turn90(RIGHT);
    }
    else
    {
         turn90(LEFT);
    }  


}
/*
	The auto mode for the Zumo, it will navigate down a corridor, avoiding all walls
    as well as allowing the user to emergency stop OR log a room. It will continously check
    the line sensors to see if:

    The left sensor is over the line, it will attempt to turn right

    The right sensor is over the line, it will attempt to turn left

    The middle sensor is over the line OR the side sensors repeatedly hit a line, it will
    cause the Zumo to stop and move into the junction function to check whether it needs to turn
    to the left or right, or 180 degrees to go to the other end of the corridor. 

    It will finally end the journey once it has completed all different junction parts.
*/
void autoMode() {
    
    incomingByte = Serial1.read();
    sensors.read(line_sensor_values);
    if ( incomingByte == 'z' )
    {
        Serial1.println("Emergency STOP!");
        // Stop command recived so STOP and go back to manual control
        motors.setSpeeds(HALT, HALT);
        robotMode = 0;
    }
    else if ( (incomingByte == 'r') && (listenMessages == true))
    {   Serial1.println("Room Located!");
        //Allow user to log the room before moving forward
        logRoom();
        searchRoom();
        robotMode = 1;
    }
    else if ( (incomingByte == 'p') && (listenMessages != true))
    {
        listenMessages = true;
    }
    // Check if the Zumo has hit a wall.
    else if ((wallHitCount >= 2) || (line_sensor_values[1] > calibrateData[1])) {
    // if the middle sensors detect line, reverse away then stop.
    motors.setSpeeds(-AUTO_SPEED, -AUTO_SPEED);
    delay(125);
    motors.setSpeeds(HALT, HALT);
    Serial1.println("Wall Detected!");
    wallHitCount = 0;
    // We have reached the end of a corridor, so increase the stop count by 1.
    stopCount++;
    // Enter junction to perform turn
    junction();
    }

    // Check if the right most sensor is over the line
    else if ((line_sensor_values[2] >= calibrateData[2]) && (line_sensor_values[1] < calibrateData[1]))
    {
        // If right most line sensor detects line move left.
        Serial1.println("Correcting left...");
        motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        delay(100);
        robotMode = 1;
        wallHitCount++;
        // Straighten the Zumo up since it's hit the far wall
        if (wallHitCount > 1) {
            motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
            Serial1.println("Straightening Up!");
            delay(200);
        }
    }

    // Check if the left most sensor is over the line
    else if ((line_sensor_values[0] >= calibrateData[0]) && (line_sensor_values[1] < calibrateData[1])) {
        // If left most line sensor detects line move right.
        // motors.setSpeeds(-TURN_SPEED, -TURN_SPEED);
        Serial1.println("Correcting right...");
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        delay(100);
        robotMode = 1;
        wallHitCount++;
        // Straighten the Zumo up since it's hit the far wall
        if (wallHitCount > 1) {
            motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
            Serial1.println("Straightening Up!");
            delay(200); 
        }
    }
    else
    {
        // Go straight forward if not hitting any line
        Serial1.println("Going forward...");
        motors.setSpeeds(AUTO_SPEED, AUTO_SPEED);
        robotMode = 1;
        wallHitCount = 0;
    }

};
/*
	Calibrates the zumos Line Sensors
*/
void calibrateZumo() {
    Serial1.println("Calibrating sensors...");
    // Intiate the sensors with the correct pins to use.
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
        //Get the maximums then add 150 to add a margin of error.
        calibrateData[i] = sensors.calibratedMaximumOn[i] + 200;
    }
    motors.setSpeeds(HALT, HALT);
    buzzer.play(">g32>>c32");
    Serial1.println("Calibration has completed");

};

/*
	Logs the side of the corridor where a room is from the Zumo.
*/
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
    Serial1.print(rooms[noOfRooms]);
};

/*
	junction function.
    This will check the amount of times the Zumo has encountered the end.
    Since it's a T-junction corridor it can only have a total of stops
    before the Zumo must be at the end. This allows for choices at:

    The T Junction, where you can turn left or right down the corridor.

    The end of the first turn, in which it is time to turn 180 degrees and go
    check the other end of the corridor.

    The final stop is at the end of the corridor and now the end of the available
    route.

*/
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
            turn90(LEFT);
        }
        else 
        {
            Serial1.println("Turning right down the T junction...");
            turn90(RIGHT);
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
        turn90(LEFT);
        turn90(LEFT);
        Serial1.println("Continuing Search!");
        robotMode = 1;
        // Ensure that the zumo can no longer search
        listenMessages = false;
    }
    else if (stopCount == 3) {
        Serial1.println("We've reached the end of the journey.");
        robotMode = 0;
        stopCount = 0;
    }

}
/*

 Uses the Zumo's compass to calculate a 90 degree turn from where the Zumo
 is currently facing.

 @PARAMS:
 Bool: direction. Tells the function which way the Zumo wants to turn 90 degrees. 

*/
void turn90(bool direction) {
    int countsLeft;
    int countsRight;

    // Reset the encoders 
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    countsLeft = 0;
    countsRight = 0;

    // Check the direction we want to turn
    if (direction){
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        while ((countsLeft <= 750)) {
            countsLeft = encoders.getCountsLeft();
            countsRight = encoders.getCountsRight();
        }
    }
    else{
       motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
       while ((countsRight < 600)) {
            countsLeft = encoders.getCountsLeft();
            countsRight = encoders.getCountsRight();
        }
    }
    countsLeft = encoders.getCountsAndResetLeft();
    countsRight = encoders.getCountsAndResetRight();
    motors.setSpeeds(HALT, HALT);

}