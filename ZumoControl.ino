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
#include <Zumo32U4IMU.h>
#include <Wire.h>
// Global Variables

#define TURN_SPEED 100
#define CALIBRATE_SPEED 200
#define MOVE_SPEED 200
#define AUTO_SPEED 75
#define CALIBRATION_SAMPLES 70 // Number of calibration samples to take for the compass
#define DEVIATION_THRESHOLD 5 // How far compass reading can deviate

#define LED 13
#define HALT 0
#define NO_SENSORS 3
#define MAX_SONAR_DISTANCE 50
#define LEFT false
#define RIGHT true
#define IR_THRESHOLD = 1

//setup library class objects
Zumo32U4Motors motors;
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);
Zumo32U4Buzzer buzzer;
Zumo32U4IMU imu;

//setup general variables
Zumo32U4IMU::vector<int16_t> m_max; // maximum magnetometer values, used for calibration
Zumo32U4IMU::vector<int16_t> m_min; // minimum magnetometer values, used for calibration
unsigned int line_sensor_values[NO_SENSORS];
int robotMode;
int noOfRooms = 0;
unsigned int calibrateData[NO_SENSORS]; // calibration data for the line sensors
int leftValue = 0; // left value for the proximity sensor
int rightValue = 0; // right value for the promixty sensor
char incomingByte;
int leftProximityCount;
int rightProximityCount;
int stopCount;
int leftSpeed;
int rightSpeed;
String rooms[10];
String people[10];
String room;
bool sensDir = RIGHT; // last indication of the direction of an object

void setup() {
    Serial1.begin(9600);
    incomingByte = Serial1.read();
    Serial1.println("Zumo Ready To Calibrate, press C to start");
    while (incomingByte != 'c')
    {
        incomingByte = (char) Serial1.read();
    }
    calibrateZumo();
    calibrateCompass();
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

            // motors.setLeftSpeed(MOVE_SPEED);
            // motors.setRightSpeed(MOVE_SPEED);
            leftSpeed = MOVE_SPEED;
            rightSpeed = MOVE_SPEED;
            break;
        case 'a':
            Serial1.println("Zumo Turning Left...");
            // Defining each motor to make it clearer which motor is moving forward, and which is moving backwards.
            // motors.setLeftSpeed(-MOVE_SPEED);
            // motors.setRightSpeed(MOVE_SPEED);
            leftSpeed = -MOVE_SPEED;
            rightSpeed = MOVE_SPEED;
            break;
        
        case 'd':
            // Defining each motor to make it clearer which motor is moving forward, and which is moving backwards.
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
    delay(5);
};
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
    delay(1100);
    motors.setSpeeds(HALT, HALT);
    for (int i = 0; i < 160; i++)
    {
        // Spin zumo around, and scan using the IR sensors
        proxSensors.read();
        
        uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
        uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
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
        delay(40);
        //If a person is found, it's time to stop moving and inform the user
        objectSeen = leftValue >= 5 || rightValue >= 5;
        
    }
    motors.setSpeeds(HALT, HALT);
    //Stop the motors if there is nothing found
    if (objectSeen)
    {
        Serial1.println("We have found a PERSON");
        buzzer.playNote(NOTE_A_SHARP(5), 250, 90);
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
    delay(900);
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
    else if ( incomingByte == 'r' )
    {   Serial1.println("Room Found!");
        //Allow user to log the room before moving forward
        logRoom();
        searchRoom();
        robotMode = 1;
    }
    else if ((line_sensor_values[1] > calibrateData[1] )) {

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

    else if ((line_sensor_values[2] >= calibrateData[2]) && (line_sensor_values[1] < calibrateData[1]))
    {
        // If right most line sensor detects line move left.
        // motors.setSpeeds(-TURN_SPEED, -TURN_SPEED);
        Serial1.println("Correcting left...");
        // delay(200);
        motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        delay(250);
        motors.setSpeeds(AUTO_SPEED, AUTO_SPEED);
        robotMode = 1;
    }
    else if ((line_sensor_values[0] >= calibrateData[0]) && (line_sensor_values[1] < calibrateData[1])) {

        // If left most line sensor detects line move right.
        // motors.setSpeeds(-TURN_SPEED, -TURN_SPEED);
        Serial1.println("Correcting right...");
        //delay(200);
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        delay(250);
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
/*
	Calibrates the zumos Line Sensors
*/
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
        delay(2.5);
    }

    for (int i = 0; i < NO_SENSORS; i++)
    {
        //Get the maximums (lightest areas) then add 150 to add a margin of error.
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
    Serial1.println(rooms[noOfRooms]);
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
        //Go back to auto mode
        robotMode = 1;
    }
    else if (stopCount == 3) {
        Serial1.println("We've reached the end of the journey.");
        robotMode = 0;
    }

}
/*

 Uses the Zumo's compass to calculate a 90 degree turn from where the Zumo
 is currently facing.

 :PARAMS:
 Bool: direction. Tells the function which way the Zumo wants to turn 90 degrees. 

*/
void turn90(bool direction) {
    float heading, relative_heading;
    int speed;
    heading = averageHeading();
    bool in_correct_pos = false;
    float target_heading;
    speed = 100;
    target_heading = fmod(averageHeading() + 90, 360);
    if (!direction)
        target_heading = (averageHeading() - 90);
        if (target_heading < 0)
            target_heading = 360 + target_heading;

    while (!in_correct_pos)
    {
        heading = averageHeading();
        relative_heading = relativeHeading(heading, target_heading);
        Serial1.print("TH: ");
        Serial1.print(target_heading);
        Serial1.print("CH: ");
        Serial1.print(heading);
        if (abs(relative_heading) < DEVIATION_THRESHOLD)
        {
            motors.setSpeeds(HALT, HALT);
            in_correct_pos = true;
            Serial1.println("Turn Complete... ");
        }
        else
        {
            // To avoid overshooting, the closer the Zumo gets to the target
            // heading, the slower it should turn. Set the motor speeds to a
            // minimum base amount plus an additional variable amount based
            // on the heading difference.

            speed = 180*relative_heading/180;
            Serial1.print("S: ");
            
            if (speed < 0)
                speed -= TURN_SPEED;
            else
                speed += TURN_SPEED;
            Serial1.print(speed);
            motors.setSpeeds(speed, -speed);
            Serial1.println();

  }

    }
}
/*

 Calibrates the Compass on the Zumo by rotating and getting the readings.
 They're stored in a vectors for the maximum and minimum readings from the sensors.

*/
void calibrateCompass() {
    unsigned char index;
    Zumo32U4IMU::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};
    Wire.begin();
    // Initialize IMU
    imu.init();

    // Enables accelerometer and magnetometer
    imu.enableDefault();
    imu.configureForTurnSensing();

    Serial.println("Calibrating Compass...");

    // To calibrate the magnetometer, the Zumo spins to find the max/min
    // magnetic vectors. This information is used to correct for offsets
     // in the magnetometer data.
    motors.setLeftSpeed(CALIBRATE_SPEED);
    motors.setRightSpeed(-CALIBRATE_SPEED);

    for(index = 0; index < CALIBRATION_SAMPLES; index ++)
    {
        // Take a reading of the magnetic vector and store it in compass.m
        imu.readMag();

        running_min.x = min(running_min.x, imu.m.x);
        running_min.y = min(running_min.y, imu.m.y);

        running_max.x = max(running_max.x, imu.m.x);
        running_max.y = max(running_max.y, imu.m.y);

        Serial.println(index);

        delay(50);
    }

    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);

    // Store calibrated values in m_max and m_min
    m_max.x = running_max.x;
    m_max.y = running_max.y;
    m_min.x = running_min.x;
    m_min.y = running_min.y;
}

// Converts x and y components of a vector to a heading in degrees.
// This calculation assumes that the Zumo is always level.
template <typename T> float heading(Zumo32U4IMU::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - m_min.x) / (m_max.x - m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - m_min.y) / (m_max.y - m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

// Gets the difference in degrees between each angle.
float relativeHeading(float heading_from, float heading_to)
{
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

/*

 Get the average of 10 different measurements to help bypass
 any motors magnetic interference on the magnetometer.

 RETURN: Float variable

*/
float averageHeading()
{
  Zumo32U4IMU::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    imu.readMag();
    avg.x += imu.m.x;
    avg.y += imu.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}