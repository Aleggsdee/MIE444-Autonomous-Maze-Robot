#include <CircularBuffer.hpp>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// Servo Variables 
Servo grip_servo; // create servo object to control gripping action
Servo lift_servo; // create servo object to control lifting action

// servo pin assignments
int gripper = 44;  // grip servo
int lifter = 46;  // lift servo

// gripper servo positions
int openPos = 150;
int closePos = 0;

// lift servo positions
int upPos = 150;
int downPos = 5; 

// Define Stepper pin assignments (driver mode, step pin, direction pin)
AccelStepper stepper1(1, 2, 5); // Left Motor (X on driver shield)
AccelStepper stepper2(1, 3, 6); // Right Motor (Y on driver shield)
AccelStepper stepper3(1, 4, 7); // Back Motor (Z on driver shield)
MultiStepper steppers;  // Multistepper object (to hold stepper1,stepper2,stepper3)

// OPTIONAL: Power saving - turn steppers off when not in use 
const int ENABLE_PIN = 8;

// Define drive perameters 
#define BASE_STEPS_PER_ROTATION 200 // 200 steps * 1.8 degrees = 360 
#define MICROSTEP_RESOLUTION 4 // 1/16th microstep resolution
#define STEPS_PER_ROTATION (BASE_STEPS_PER_ROTATION * MICROSTEP_RESOLUTION) //800 steps per rotation for 1/4 microsteps

long targetSteps[3] = {0,0,0};  // Target steps for each stepper 
long currentSteps[3] = {0,0,0}; // Current steps for each stepper
long lastSteps[3] = {0,0,0};    // Last steps (from last sensor data call) for each stepper

// Define robot geometry
const float WHEEL_RADIUS = 0.029;
const float ROBOT_RADIUS = 0.0753;  
const float WHEEL_CIRCUMFERENCE = PI * 2 * WHEEL_RADIUS;
const float COUNTS_PER_METER = STEPS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

#define DEG_TO_RAD (PI / 180.0)

// Variables for recieving sensor data
#define SENSOR_COUNT 8
#define START_BYTE 0xFF
#define FLOAT_SIZE sizeof(float)
#define TOTAL_BYTES (FLOAT_SIZE * SENSOR_COUNT)

uint8_t buffer[TOTAL_BYTES];
float sensor_data[SENSOR_COUNT];
bool receivingData = false;
unsigned int bytesReceived = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);    // Initialize serial monitor for bluetooth communication [boost baud rate if needed]
  Serial2.begin(115200); // Initialize serial monitor for slave-master communication 
  // TODO: Uncomment serial2 begin line
  initialize_steppers();  // Setup stepper settings (speed)

  grip_servo.attach(gripper); // attaches grip servo to pin 9
  lift_servo.attach(lifter);  // attaches lift servo to pin 10
  
  // Drop any block in gripper
  lift_servo.write(downPos); // CLOSE GRIPPER
  delay(500); // WAIT FOR GRIPPER TO CLOSE
  grip_servo.write(openPos);  // LIFT GRIPPER
  delay(3000);
  // Set gripper to stowed position
  grip_servo.write(closePos);
  delay(500);  // Wait for gripper to close (to clear front standoffs)
  lift_servo.write(upPos);

  pinMode(ENABLE_PIN, OUTPUT);  // Initialize power saving pin
  digitalWrite(ENABLE_PIN, HIGH);  // Supply power to steppers
}

// Main loop variables for reading commands 
String cmdStr;
bool commandReadComplete = false;
float msg_val = 0;
float drive_val = 0;
String currentCommand = "";
char driveCommandType = 'a';

CircularBuffer<String, 10> queue; // need to make sure this is big enough

bool isDriving = false;  // Flag to indicate driving state

void loop() {
    
  if (isDriving) {
    
    // Check if target distance is reached
    // TODO: consider breaking into 3 if statements one for each motor
    //if (abs(encoderCount1) >= abs(targetCounts[0]) && abs(encoderCount2) >= abs(targetCounts[1]) && abs(encoderCount3) >= abs(targetCounts[2])) { // TODO: change this to equal when we use PID

    if (!steppers.run()){
      // TODO: check if this is ok
      isDriving = false;
      
      // Turns steppers off to save power (uncomment line below)
      digitalWrite(ENABLE_PIN, HIGH);

    }

  }
  else if (!queue.isEmpty()) {
    // Process command type and drive value from queue
    currentCommand = queue.shift();
    driveCommandType = currentCommand.charAt(1);
    currentCommand.remove(currentCommand.length() - 1);
    currentCommand.remove(0,4);
    drive_val = currentCommand.toFloat();

    // Determine target steps for each stepper motor
    calculateTargetSteps(driveCommandType, drive_val);  // Calculates and steps
    steppers.moveTo(targetSteps);  //Updates target steps
    // Set lastSteps to 0 
    lastSteps[0] = stepper1.currentPosition();
    lastSteps[1] = stepper2.currentPosition();
    lastSteps[2] = stepper3.currentPosition();  

    // TODO: RUNMOTORS(s1,s2,s3); FOR TARGET STEPS s1,s2,s3
    isDriving = true;
    // Turns steppers on to save power
    digitalWrite(ENABLE_PIN, LOW); 
  }
  while (Serial.available() > 0 && commandReadComplete == false) {
    // If there is something in the serial monitor, start reading
    char incomingChar = Serial.read();
    if (incomingChar == '[') {
      cmdStr = "[";
    }
    else if (incomingChar == ']') {
      cmdStr += incomingChar;
      commandReadComplete = true;
    }
    else {
      cmdStr += incomingChar;
    }
  }

  if (commandReadComplete) {
    if(cmdStr.charAt(0) == '[') { // for handling bad messages
      if(cmdStr == "[xx]") {
        // Stop the motors
        stop_steppers();  // Stops motors, sets targetSteps = CurrentSteps
        // TODO: Stop motors
        // TODO: Reset target step counts to running step counts
        queue.clear();
        Serial.println("[xx]");
      }
      else if(cmdStr.charAt(1) == 't') {
        
        // TODO: for 't', send SNESOR and STEP data (reset count to zero)
        // TODO: use for loop to send all sensor and step data

        // Sending Sensor Data
        Serial.print("[");  // Send initial square bracket
        sendSensorAndStepData();
        Serial.print("]");  // Send final square bracket

        // Resetting Step counts
      }

      else if(cmdStr.charAt(1) == 'm') {
        
        // TODO: for 'm', send STEP data (and reset count to zero)
        Serial.print("[");  // Send initial square bracket
        sendStepData();
        Serial.print("]");  // Send final square bracket
          
      }

      else if (cmdStr.charAt(1) == 'w' || cmdStr.charAt(1) == 'd' ||cmdStr.charAt(1) == 'r') {
        if (!queue.isFull()) {
          queue.push(cmdStr); // only adds to queue if the queue is empty, otherwise it dumps the current cmdStr so you don't block other commands from being read like [xx] or sensor
        }
        Serial.println("]");
      }

      else if (cmdStr.charAt(1) == 'c') {
        // Close and lift gripper
        grip_servo.write(closePos); // CLOSE GRIPPER
        delay(500); // WAIT FOR GRIPPER TO CLOSE
        lift_servo.write(upPos);  // LIFT GRIPPER
        Serial.println("]");
      }
        else if (cmdStr.charAt(1) == 'o') {
        // Lower and open gripper
        lift_servo.write(downPos); // CLOSE GRIPPER
        delay(500); // WAIT FOR GRIPPER TO CLOSE
        grip_servo.write(openPos);  // LIFT GRIPPER
        Serial.println("]");
      }

      else if (cmdStr.charAt(1) == 'k') {
        // Enable holding torque
        digitalWrite(ENABLE_PIN, LOW);
        Serial.println("]");
      }
      else if (cmdStr.charAt(1) == 'l') {
        // Disable holding torque
        digitalWrite(ENABLE_PIN, HIGH);
        Serial.println("]");
      }
      else {
        Serial.println("]");
      }
      // else: do nothing just dump cmdStr because it is not a valid command
    }
    cmdStr = "";
    commandReadComplete = false;
  }  
}


void calculateTargetSteps(char commandType, float driveValue){
  // TODO: Set target steps for each motor, given the distance 
  //steppers.moveto(long 	absolute[]);
  float driveValueMeters = driveValue / 39.3701;

  if (commandType == 'w') {
    // Forward command
  targetSteps[0] += (COUNTS_PER_METER * driveValueMeters) * cos(30*DEG_TO_RAD);
  targetSteps[1] += (COUNTS_PER_METER * -driveValueMeters) * cos(30*DEG_TO_RAD);
  targetSteps[2] += 0;

  stepper1.setMaxSpeed(600);
  stepper2.setMaxSpeed(600);
  stepper3.setMaxSpeed(600);

  if (abs(driveValue) <= 1){
    stepper1.setMaxSpeed(200);
    stepper2.setMaxSpeed(200);
    stepper3.setMaxSpeed(200);
  }

  // Serial.println("target steps w set");
  // Serial.println(COUNTS_PER_METER);
  // Serial.println(driveValue);
  // Serial.println(driveValueMeters);
  // Serial.println(targetSteps[0]);
  // Serial.println(targetSteps[1]);
  // Serial.println(targetSteps[2]);


  }
  else if (commandType == 'd') {
    // Sideways command
  targetSteps[0] += (COUNTS_PER_METER * driveValueMeters) * cos(60*DEG_TO_RAD);
  targetSteps[1] += (COUNTS_PER_METER * driveValueMeters) * cos(60*DEG_TO_RAD);
  targetSteps[2] += COUNTS_PER_METER * -driveValueMeters;

  stepper1.setMaxSpeed(150);
  stepper2.setMaxSpeed(150);
  stepper3.setMaxSpeed(150);

  }
  else if (commandType == 'r') {
    // Rotate command
  int targetRotateSteps = (driveValue * ROBOT_RADIUS * 800) / (WHEEL_RADIUS * 360);  // See Raphael's Ipad equation

  targetSteps[0] += targetRotateSteps;
  targetSteps[1] += targetRotateSteps;
  targetSteps[2] += targetRotateSteps;

  stepper1.setMaxSpeed(600);
  stepper2.setMaxSpeed(600);
  stepper3.setMaxSpeed(600);  //TODO: Unfuck this technical debt

    if (abs(driveValue) <= 5){
      stepper1.setMaxSpeed(100);
      stepper2.setMaxSpeed(100);
      stepper3.setMaxSpeed(100);
      }

  }
  else {
    // Do nothing
  }

}

void stop_steppers() {
  // Sets targetSteps to currentPosition
  targetSteps[0] = stepper1.currentPosition();
  targetSteps[1] = stepper2.currentPosition();
  targetSteps[2] = stepper3.currentPosition();
  isDriving = false;
  digitalWrite(ENABLE_PIN, HIGH);
  //TODO: make sure run only triggers when isDriving is true
}

void initialize_steppers() {
  // Configure max speeds (in steps per second)
  stepper1.setMaxSpeed(600);
  stepper2.setMaxSpeed(600);
  stepper3.setMaxSpeed(600);

  // Then give stepper objects to MultiStepper to manage simultaneously
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
}

void sendSensorAndStepData(){
  for (int i = 0; i<SENSOR_COUNT; i++){
    Serial.print("t");
    Serial.print(i);
    Serial.print(":");
    Serial.print(sensor_data[i]);  // TODO: Round to 2 decimals
    Serial.print(",");
    // Only print a comma if this is not the last element
  }

  // Sending Step Data

  // Fetch current step counts, put into array 
  currentSteps[0] = stepper1.currentPosition();
  currentSteps[1] = stepper2.currentPosition();
  currentSteps[2] = stepper3.currentPosition();  

  for (int i = 0; i<3;i++){
    Serial.print("m");
    Serial.print(i+1);
    Serial.print(":");
    Serial.print(currentSteps[i]-lastSteps[i]);  // Current count
    lastSteps[i] = currentSteps[i];
    if (i < 2) {
        Serial.print(",");
    }          
  }
}

void sendStepData(){
  // Sending Step Data

  // Fetch current step counts, put into array 
  currentSteps[0] = stepper1.currentPosition();
  currentSteps[1] = stepper2.currentPosition();
  currentSteps[2] = stepper3.currentPosition();  

  for (int i = 0; i<3;i++){
    Serial.print("m");
    Serial.print(i+1);
    Serial.print(":");
    Serial.print(currentSteps[i]-lastSteps[i]);  // Current count
    lastSteps[i] = currentSteps[i];
    if (i < 2) {
        Serial.print(",");
    }          
  }
}

void serialEvent2() {
  while (Serial2.available()) {
    uint8_t incomingByte = Serial2.read();

    if (!receivingData) {
      if (incomingByte == START_BYTE) {
        receivingData = true;
        bytesReceived = 0;
        memset(buffer, 0, TOTAL_BYTES);
      }
    } else {
      buffer[bytesReceived++] = incomingByte;

      if (bytesReceived >= TOTAL_BYTES) {
        receivingData = false;

        memcpy(sensor_data, buffer, TOTAL_BYTES);

        // Print received data
        // Serial.print("Received Data: ");
        // for (int i = 0; i < SENSOR_COUNT; i++) {
        //   Serial.print("Sensor ");
        //   Serial.print(i + 1);
        //   Serial.print(": ");
        //   Serial.print(sensor_data[i], 2);
        //   if (i < SENSOR_COUNT - 1) Serial.print(", ");
        // }
        // Serial.println();
      }
    }
  }
}