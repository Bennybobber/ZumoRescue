# Programming Things Zumo Search Individual Assignment

This is a project for the Zumo32u4 robot conducting a search and rescue down a make shift 2D map of a corridor with a T junction at the end.
The aim is that the Zumo can either be manually or automatically navigated along the corridor, while checking rooms for "people" to rescue.

The GUI is written in Python and is a basic interface, allowing for keyboard control of the ZUMO robot and being able to see the messages that
the robot is sending to the user over the XBee chips so that they can communicate wirelessly over the Serial port 9600. 

## Getting started

##Prequisites

1. Install Arduino Software from https://www.arduino.cc/en/Guide/Windows (This link is for windows but other options are available).
2. Install Python from https://www.python.org/downloads/ (Contains links to Linux, Windows and MAC OS).

## Install 

1. Download the project as a Zip File (Or Clone).
2. Install Arduino and open the Library install tab and ensure that Zumo32U4 library is installed)
3. Open the project in Arduino IDE.
4. Plug in your Zumo and upload the file to your Zumo robot.
5. Open Python, and then using PIP install (Serial and pyGame libraries)
6. Ensure your XBEE is setup to work on the Serial port 9600 and is paired.

This should allow you to then use the Zumo wireless via the Python prgoram and execute the program.

### ZUMO CONTROLS ###


Manual mode: 
    Controls via WASD etc to move the Zumo around.
    Q, E Moves turn the robot approx 90 degrees left or right.
    B I ntiates an emergency stop.
    Z Stops the zumo.
    R Logs the room.
    L Intiates a search of the room.
    1 Switches to automatic mode.
Auto Mode:
    Z Makes an emergency stop.
    R Stops the Zumo to check for a room. 
    There will be other options once you reach certain points in the junction, these will inform you on the 
    Python screen such as when to intiate the 180 degree turn.
    
    

## Authors
 ** Benjamin Sinyard ** 


