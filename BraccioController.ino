/*
 * First include all external dependencies
 * - The libraries for controlling the Braccio Robot Arm
 * - The inverse kinematic lib for the Braccio Arm (https://github.com/cgxeiji/CGx-InverseK)
 * - The lib for communication with the arduino (https://bitbucket.org/mobility_innovation/communicationlib/src/master/)
 */
#include <Braccio.h>
#include <Servo.h>
#include <InverseK.h>
#include "./arduinoserialirqcom/SerialIrqCom.h"

/*
 * Now include all other header files
 */
#include "InverseKinematics.h"


/*
 * Define some constants
 */
#define COORDINATE_SEPARATOR ','
#define STOP_COMMAND "stop"
#define MOVE_TO_COMMAND "moveTo:"

#define BRACCIO_STEP_DELAY 20
#define BRACCIO_SAVE_POS_M1 0
#define BRACCIO_SAVE_POS_M2 45
#define BRACCIO_SAVE_POS_M3 180
#define BRACCIO_SAVE_POS_M4 180
#define BRACCIO_SAVE_POS_M5 135
#define BRACCIO_SAVE_POS_M6 10
#define BRACCIO_DEFAULT_POS_M5 0
#define BRACCIO_DEFAULT_POS_M6 73

/*
 * The relevant variables for steering the braccio
 */
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

void setup() {
  // Setup the inverse kinematics header file
  InverseKinematics::setup();
  
  // Initialize the braccio
  Braccio.begin();

  // Finally, initialize the uart / serial communication, because we are not ready to receive commands
  SerialIrqCom::setup();
}

void loop() {
  // Receive the communication
  SerialIrqCom::ReturnType result = SerialIrqCom::receive();

  if (result == SerialIrqCom::NewCommandAvailable) {
    processMessage(SerialIrqCom::msg);
  }
}


void processMessage(const String &msg) {
  // Check whether the string starts with a command can call the correct function
  if(msg.indexOf(STOP_COMMAND) == 0) {
    // We received a stop command
    Serial.println(stopCmd());
  } else if(msg.indexOf(MOVE_TO_COMMAND) == 0) {
    // Call the moveTo command and return the result over the com interface
    Serial.println(moveTo(removeCmd(msg, MOVE_TO_COMMAND)));
  } else {
    // If it is none of the above commands, we reject it
    // Return a false value over the com interface
    Serial.println(false);
  }
}

/**
 * Remove the trailing command of a message
 */
String removeCmd(const String &msg, const String &cmd) {
  // First copy the string because we cannot alter the msg string
  String result = msg;

  // Now remove the command
  result.remove(0, cmd.length());

  return result;
}


/**
 * Stops the robot arm and brings it to a save position, triggered by STOP_COMMAND
 */
bool stopCmd() {
  Braccio.ServoMovement(BRACCIO_STEP_DELAY, BRACCIO_SAVE_POS_M1, BRACCIO_SAVE_POS_M2, BRACCIO_SAVE_POS_M3, BRACCIO_SAVE_POS_M4, BRACCIO_SAVE_POS_M5, BRACCIO_SAVE_POS_M6); 
  return true;
}


/**
 * Tries to move the braccio robot arm to a certain position, triggered by MOVE_TO_COMMAND
 */
bool moveTo(const String &msg) {
  // Get the coodinates from the remaining msg string
  float* coordinates = decodeCoordinates(msg, 4);

  // Set the servo variables
  float m1, m2, m3, m4;

  // Calculate the inverse values
  bool result = InverseKinematics::solve(coordinates, m1, m2, m3, m4);
  if(result) {
    // We were able to find rotations satisfying the given requirements
    // Perform the movement
    Braccio.ServoMovement(BRACCIO_STEP_DELAY, m1, m2, m3, m4, BRACCIO_DEFAULT_POS_M5, BRACCIO_DEFAULT_POS_M6); 
  }

  // Clean up to avoid memory leaks
  delete coordinates;

  // Return a the result value over the com interface
  return result;
}

float* decodeCoordinates(const String& msg, unsigned int numberOfCoordinates) {
  // Be carefull! Allocation on the heap
  float* result = new float[numberOfCoordinates];
  for(unsigned int i = 0; i<numberOfCoordinates; i++) {
    result[i] = getCoordinate(msg, i);
  }
  return result;
}

float getCoordinate(const String& msg, unsigned int index) {
  return atof(getValue(msg, COORDINATE_SEPARATOR, index).c_str());
}

/**
 * Function taken from https://stackoverflow.com/questions/9072320/split-string-into-string-array and sanitized to work better
 */
String getValue(const String& data, char separator, unsigned int index)
{
  unsigned int found = 0;
  int strIndex[] = {0, -1};
  unsigned int maxIndex = data.length()-1;

  for(unsigned int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
