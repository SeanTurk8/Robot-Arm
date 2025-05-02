//All libraries required to run this
#include <Wire.h>
#include <AccelStepper.h>
#include <math.h>
#include <SoftwareWire.h>
#include <LiquidCrystal_SoftI2C.h> //this one was downloaded off github
#include <Servo.h>

// Motor driver type
#define MOTOR_INTERFACE_TYPE AccelStepper::DRIVER

// Motor pins
//Base
#define STEP_PIN1 3
#define DIR_PIN1  2

//SHoulder
#define STEP_PIN2 5
#define DIR_PIN2  6

//Elbow
#define STEP_PIN3 8
#define DIR_PIN3  9

//Wrist
//#define STEP_PIN4 11
//#define DIR_PIN4  12

// Create stepper objects
AccelStepper motor1(MOTOR_INTERFACE_TYPE, STEP_PIN1, DIR_PIN1); // Base
AccelStepper motor2(MOTOR_INTERFACE_TYPE, STEP_PIN2, DIR_PIN2); // Shoulder
AccelStepper motor3(MOTOR_INTERFACE_TYPE, STEP_PIN3, DIR_PIN3); // Elbow
//AccelStepper motor4(MOTOR_INTERFACE_TYPE, STEP_PIN4, DIR_PIN4); // Wrist

// Limit switch pins
#define LIMIT_SWITCH_PIN1 50 //base
#define LIMIT_SWITCH_PIN2 48 //Shoulder
#define LIMIT_SWITCH_PIN3 46 //Elbow
//#define LIMIT_SWITCH_PIN4 44 //wrist

// Constants
const int BASE_GEAR_RATIO = 3.0;
const int SHOULDER_GEAR_RATIO = 40.0;
const int ELBOW_GEAR_RATIO = 40.0;
const int STEPS_PER_REV = 400.0; // Full steps per rev
const int dcmotor1 = 36; //
const int dcmotor2 = 35;

struct JointAngles {
  int base;
  int shoulder;
  int elbow;
  int wrist;
};

// Speeds for each motor
const int maxSpeed1 = 800.0;
const int maxSpeed2 = 600.0;
const int maxSpeed3 = 3200.0;
const int maxSpeed4 = 500.0;

// Acceleration for each motor
const int acceleration1 = 600.0;
const int acceleration2 = 600.0;
const int acceleration3 = 1000.0;
const int acceleration4 = 500.0;

bool surrendered = false;
bool playerlose = false;
bool playerwin = false;

// 8x8 matrix containing all the joint angles
JointAngles angleMap[8][8];

// I2C-related variables
int old_I, old_J, new_I, new_J, jump_I, jump_J;
bool newMoveReceived = false;
bool pieceJumped = false;
bool jumpconfirm = false;
String moveType = ""; // To distinguish between Player and Computer move

// Software I2C and LCD Display Setup
SoftwareWire myWire(A4, A5);
LiquidCrystal_I2C lcd(0x27, 16, 2, &myWire);
//unsigned long lastLCDUpdate = 0;
bool GameStart = false;


//All Checks used for the Jumping is Mandatory Prompts
bool jumpPrompt = false;
bool jumpwait = false;
unsigned long lastjump = 0;
int jumpphase = 0;
bool showJumpStatus = false;
bool jumpIsMandatory = false;
bool jumpStatusDisplayed = false;
unsigned long jumpDisplayStart = 0;

// These two strings store the last message shown on each line of the LCD
String lastLine1 = "";
String lastLine2 = "";

// updates the LCD display
// Used for some but not all because didn't want to change too much and it causes issues
void updateLCD(const String& line1, const String& line2) {
  if (line1 != lastLine1 || line2 != lastLine2) {  // only update the LCD if at least one of these lines has changed
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1); // Print the new line1
    lcd.setCursor(0, 1); 
    lcd.print(line2); // Print the new line2
    // Save the newly printed lines as lastline values
    lastLine1 = line1;
    lastLine2 = line2;
  }
} 



void initializeAngleMap() {
  //Manually recorded aangle values for all 32 squares on the checkers board
  //if adjustments are made to offsets, all the squares would need to be re-recorded
  //record these values with the button program (buttons.ino)

  angleMap[0][1] = {-77.10, -33.82, -2.88, 0.00};//latest gripper in middle
  angleMap[0][3] = {-116.70, -21.58, 0.00, 0.00};
  angleMap[0][5] = {-153.3, -25.45, -3.69, 0.00};
  angleMap[0][7] = {-166.80, -39.17, -10.53, 0.00};//latest gripper in middle
  angleMap[1][0] = {-68.10, -46.93, -22.14, 0.00};//latest gripper in middle
  angleMap[1][2] = {-96.00, -37.73, -7.47, 0.00};//latest gripper in middle
  angleMap[1][4] = {-125.10, -34.36, -5.04, 0.00};//latest gripper in middle
  angleMap[1][6] = {-145.20, -42.08, -13.97, 0.00};//latest gripper in middle
  angleMap[2][1] = {-83.70, -48.33, -25.63, 0.00};//latest gripper in middle
  angleMap[2][3] = {-108.00, -44.73, -18.07, 0.00};//latest gripper in middle
  angleMap[2][5] = {-129.60, -45.16, -20.79, 0.00};//latest gripper in middle 
  angleMap[2][7] = {-144.60, -52.02, -33.77, 0.00};//latest gripper in middle
  angleMap[3][0] = {-75.00, -58.36, -45.00, 0.00};//latest gripper in middle
  angleMap[3][2] = {-93.90, -52.45, -32.92, 0.00};//latest gripper in middle
  angleMap[3][4] = {-113.40, -51.77, -32.29, 0.00};//latest gripper in middle
  angleMap[3][6] = {-130.50, -54.99, -38.45, 0.00};//latest gripper in middle
  angleMap[4][1] = {-85.50, -62.01, -52.31, 0.00};//latest gripper in middle
  angleMap[4][3] = {-101.40, -60.48, -48.56, 0.00};//latest gripper in middle 
  angleMap[4][5] = {-117.30, -58.84, -48.69, 0.00};//latest gripper in middle
  angleMap[4][7] = {0.00, 0.00, 0.00, 0.00};
  angleMap[5][0] = {-78.30, -72.43, -79.13, 0.00};//latest gripper in middle
  angleMap[5][2] = {-92.40, -66.13, -65.43, 0.00};//latest gripper in middle
  angleMap[5][4] = {-107.00, -66.76, -65.34, 0.00};//latest gripper in middle
  angleMap[5][6] = {-120.30, -70.65, -73.08, 0.00};//latest gripper in middle 
  angleMap[6][1] = {-85.80, -80.89, -95.80, 0.00};//latest gripper in middle
  angleMap[6][3] = {-99.00, -77.47, -88.54, 0.00};//latest gripper in middle
  angleMap[6][5] = {0.00, 0.00, 0.00, 0.00};
  angleMap[6][7] = {-121.50, -88.74, -111.82, 0.00};
  angleMap[7][0] = {0.00, 0.00, 0.00, 0.00};
  angleMap[7][2] = {0.00, 0.00, 0.00, 0.00};
  angleMap[7][4] = {0.00, 0.00, 0.00, 0.00};
  angleMap[7][6] = {0.00, 0.00, 0.00, 0.00};
}

// takes angle and gear ratio as input arguments 
// return the steps calculated for the specified angle
int angleToSteps(float angle, int gear_ratio) {
  return round(angle * gear_ratio * (STEPS_PER_REV / 360.0));
}

//  Move to square specified by input arguments
void moveToSquare(int i, int j) {
  //traverses the anglemap matirx and retrieves the 
  // list of angles setting it to target variable 
  JointAngles target = angleMap[i][j];

  //calls angle step function with each motor's determined angle and gear ratio
  int baseSteps = angleToSteps(target.base, BASE_GEAR_RATIO);
  int shoulderSteps = angleToSteps(target.shoulder, SHOULDER_GEAR_RATIO);
  int elbowSteps = angleToSteps(target.elbow, ELBOW_GEAR_RATIO);

  //debug statements
  //Serial.print("Base steps: "); Serial.println(baseSteps);
  //Serial.print("Shoulder steps: "); Serial.println(shoulderSteps);
  //Serial.print("Elbow steps: "); Serial.println(elbowSteps);

  // run motor1 (base motor) to calculated steps
  motor1.moveTo(baseSteps);
  while (motor1.isRunning()) {
    motor1.run();
  }

  motor2.moveTo(shoulderSteps);
  motor3.moveTo(elbowSteps);

  //Allows both motor 2 (shoulder) and motor 3 (elbow) move at the same time
  //This cuts down on the time it takes for the robot to make a move
  while (motor2.isRunning() || motor3.isRunning()){
    motor2.run();
    motor3.run();
  }
}
void setup() {
  Serial.begin(9600); //initialize the serial monitor
  Wire.begin(0x08); // Initialize I2C with address 0x08
  Wire.onReceive(receiveEvent); // Register the receive event
  //clears any previous messages on the LCD
  lcd.begin();
  lcd.clear();

  //displays the press start button message as soon as game starts
  updateLCD("Press Start", "Button");
  
  // Limit switch pins as INPUT
  // or use INPUT_PULLUP if wired differently
  pinMode(LIMIT_SWITCH_PIN1, INPUT);
  pinMode(LIMIT_SWITCH_PIN2, INPUT);
  pinMode(LIMIT_SWITCH_PIN3, INPUT);
  /*pinMode(LIMIT_SWITCH_PIN4, INPUT);*/

  pinMode(dcmotor1, OUTPUT);
  pinMode(dcmotor2, OUTPUT);

  digitalWrite(dcmotor1, LOW);
  digitalWrite(dcmotor2, LOW);
  // Initialize current position at the home position
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  motor3.setCurrentPosition(0);
  //motor4.setCurrentPosition(0);

  //initializing motor speeds and acceleration
  motor1.setMaxSpeed(maxSpeed1);
  motor1.setAcceleration(acceleration1);

  motor2.setMaxSpeed(maxSpeed2); 
  motor2.setAcceleration(acceleration2);  
  
  motor3.setMaxSpeed(maxSpeed3);
  motor3.setAcceleration(acceleration3);

  //motor4.setMaxSpeed(maxSpeed4);
  //motor4.setAcceleration(acceleration4);

  // Initialize the angle map 
  initializeAngleMap();

}

void loop() {
  // Handle 3-phase jump prompt
  if (jumpPrompt) {
    unsigned long now = millis();
    //displays each of these messages for a second until the last awaiting input message
    if (jumpphase == 0 && now - lastjump > 1000) {
      updateLCD("Press Once", "For Yes");
      lastjump = now;
      jumpphase++;
    }
    else if (jumpphase == 1 && now - lastjump > 1000) {
      updateLCD("Press Twice", "For No");
      lastjump = now;
      jumpphase++;
    }
    //remains on this message until enter button is pressed
    else if (jumpphase == 2 && now - lastjump > 1000) {
      updateLCD("Awaiting Input", "From Player...");
      jumpPrompt = false;
    }
    return;
  }

  //checks if the player has surrendered
  if (surrendered) {
    // Display message
    //lastLCDUpdate = 0;
    //updates the lcd 
    updateLCD("You Surrendered!", "");
    delay(6000);

    // Clear LCD for the updateLCD message
    lastLine1 = "";
    lastLine2 = "";

    // Final screen
    updateLCD("Press START", "To Play Again");
  
    // Reset game state
    GameStart = false;
    jumpPrompt = false;
    jumpwait = false;
    newMoveReceived = false;
    surrendered = false;  // Clear flag
    return;
  }

  //checks if the player has lost the game
  if (playerlose){
    // Display message
    //lastLCDUpdate = 0;

    //update the display for the lose message
    updateLCD("YOU", "LOSE!");
    delay(3000);

    // Clear LCD for the updateLCD message
    lastLine1 = "";
    lastLine2 = "";

    //Final screen
    updateLCD("Press START", "To Play Again");
  
    // Reset game state
    GameStart = false;
    jumpPrompt = false;
    jumpwait = false;
    newMoveReceived = false;
    playerlose = false;  // Clear player lose
    return;
  }

  //checks if the player has won
  //this hypothetically, shoould work since the player lose one did
  //we just havent gotten someone to win yet to display this message
  if (playerwin){
    // Display message
    //lastLCDUpdate = 0;
    //updates display to show win message
    updateLCD("YOU", "WIN!");
    delay(3000);

    // Clear LCD for the updateLCD message
    lastLine1 = "";
    lastLine2 = "";

    // Final screen
    updateLCD("Press START", "To Play Again");
  
    // Reset game state
    GameStart = false;
    jumpPrompt = false;
    jumpwait = false;
    newMoveReceived = false;
    playerwin = false;  // Clear player win
    return;
  }

  //looks at if the player has decided for jumping to be mandatory is true or not
  if (showJumpStatus) {
    //
    if (!jumpStatusDisplayed) {
      if (jumpIsMandatory) {
        updateLCD("Jumping IS", "Mandatory");
      } else {
        updateLCD("Jumping is", "NOT Mandatory");
      }

      jumpDisplayStart = millis();
      jumpStatusDisplayed = true;
      return;
    }

    if (millis() - jumpDisplayStart >= 1000) {
      updateLCD("Player's Turn!", "");
      GameStart = true;
      jumpwait = false;
      showJumpStatus = false;
    }
    return;
  }

  // This is to ensure that player's turn begins after every computer turn sequence
  if (!newMoveReceived && GameStart && !jumpwait) {
    updateLCD("Player's", "Turn!");
  }

  // Calls handlelimitswitch function for each of the motors
  handleLimitSwitch(motor1, LIMIT_SWITCH_PIN1, acceleration1);
  handleLimitSwitch(motor2, LIMIT_SWITCH_PIN2, acceleration2);
  handleLimitSwitch(motor3, LIMIT_SWITCH_PIN3, acceleration3);
  //handleLimitSwitch(motor4, LIMIT_SWITCH_PIN4, acceleration4);*/

  // waits for the new move to come from the PI
  if (newMoveReceived) {
    //if that new move is from the computer, not the player, we'll process it
    if (moveType == "Computer") {

      lcd.setCursor(0, 0);
      lcd.print("Computer Moving");
      lcd.setCursor(0, 1);
      lcd.print("(" + String(old_I) + "," + String(old_J) +") -->" + "(" + String(new_I) + "," + String(new_J) +")");
  
      // old_I first, old_J seccond, new_I third, new_J fourth
      int oldi = old_I; // Example input
      int oldj = old_J;

      int newi = new_I; // Example input
      int newj = new_J;
      //Serial.println(old_I);
      //Serial.println(old_J);
      //Serial.println(new_I);
      //Serial.println(new_J);
      // Call moveToSquare(i, j) to test a position
      moveToSquare(oldi, oldj); // Example: move to square (5,2)
      delay(2000);
      
      //close grabber
      digitalWrite(dcmotor1, HIGH);
      digitalWrite(dcmotor2, LOW);
      delay(1500);
      digitalWrite(dcmotor1, LOW);
      digitalWrite(dcmotor2, LOW);

      motor2.move(400);
      motor2.runToPosition();
      delay(500);

      moveToSquare(newi, newj);
      delay(2000);

      //open grabber
      digitalWrite(dcmotor1, LOW);
      digitalWrite(dcmotor2, HIGH);
      delay(1500);
      digitalWrite(dcmotor1, LOW);
      digitalWrite(dcmotor2, LOW);
      homeRobotArm(); // calls homing function for each joint to begin homing
      if(pieceJumped){
        String row1message = "Please Remove";
        String row2message = "Piece at (" + String(jump_I) + "," + String(jump_J) +")  ";
        for (int i = 0; i < 5; i++){
          lcd.clear();
          delay(600);
          lcd.setCursor(0,0);
          lcd.print(row1message);
          lcd.setCursor(0,1);
          lcd.print(row2message);
          delay(600);
      }
        lcd.setCursor(0,0);
        lcd.print("Press Enter     ");
        lcd.setCursor(0,1);
        lcd.print("to Confirm jump ");
        while (!jumpconfirm){
          delay(10);
        }
        // Reset LCD cache to force reprint
        lastLine1 = "";
        lastLine2 = "";
        pieceJumped = false; 
        jumpconfirm = false;
        // resets piece jumped flag to false
      }
    }
    //Serial.println("New Move Received");
    newMoveReceived = false; // Reset flag
  }
}

void handleLimitSwitch(AccelStepper &motor, int limitPin, float normalAccel) {
    if (digitalRead(limitPin) == HIGH) {
        // Stop motor when the limit switch is pressed
        motor.stop();
        motor.setCurrentPosition(0);  // Reset position to home
        motor.disableOutputs();  // Disable motor to avoid holding torque
    } else {
        // Restore motor settings only if it was previously disabled
        if (!motor.isRunning()) {
            motor.enableOutputs(); // Re-enable motor
            motor.setAcceleration(normalAccel); // Restore acceleration
        }
    }
}

void homeRobotArm() {
    //Serial.println("Homing robot arm...");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Homing");
    lcd.setCursor(0,1);
    lcd.print("Robot Arm...");

    homeMotor2and3(motor2, LIMIT_SWITCH_PIN2, motor3, LIMIT_SWITCH_PIN3);
    
    // Home motor 1 after motor 2 is done
    //Serial.println("Homing motor 1...");
    homeMotor1(motor1, LIMIT_SWITCH_PIN1);

    //Serial.println("Homing complete.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Homing");
    lcd.setCursor(0, 1);
    lcd.print("Completed!");
    delay(2000);
    
    // Reset the cache so updateLCD will reprint
    lastLine1 = "";
    lastLine2 = "";
    
    // Now return to Player's Turn screen
    updateLCD("Player's", "Turn!");
    GameStart = true;
}

void homeMotor1(AccelStepper &motor, int limitPin) {
    //Serial.print("Homing motor on limit switch: ");
    //Serial.println(limitPin);

    motor.setMaxSpeed(200);  // Safe speed for homing for motor 1
    motor.setAcceleration(800);
    motor.move(10000);  // Move in direction of limit switch

    // Run this motor only until its limit switch is triggered
    while (digitalRead(limitPin) == LOW) {
        motor.run();
    }
    //Serial.println("Limit switch hit!");
    
    motor.stop();
    delay(200);

    motor.setCurrentPosition(0);  // Set home position to zero
    motor.enableOutputs();  // Ensure motor is enabled after homing
    motor.setAcceleration(acceleration1);  // Restore normal acceleration

    //Serial.println("Motor homed.");
}

//make shoulder and elbow home almost in sync
void homeMotor2and3(AccelStepper &motor2, int limitPin2, AccelStepper &motor3, int limitPin3) {
    bool shocomplete = false;
    bool elbbegin = false;
    bool elbfinish = false;
    unsigned long startTime = millis();
    
    motor2.setMaxSpeed(800);  // Safe speed for homing shoulder
    motor2.setAcceleration(1000);

    motor3.setMaxSpeed(800);  // Safe speed for homing shoulder
    motor3.setAcceleration(1000);

    motor2.move(10000);

    while (!shocomplete || !elbfinish){
      unsigned long timer = millis();

      if(!elbbegin && timer - startTime >= 1000){
        motor3.move(10000);
        elbbegin = true;
        //Serial.println("Are we starting elbow home");
      }

      if(!shocomplete){
        if(digitalRead(limitPin2) == LOW){
          motor2.run();
        }
        else{
          motor2.stop();
          motor2.setCurrentPosition(0);  // Set home position to zero
          motor2.enableOutputs();  // Ensure motor is enabled after homing
          motor2.setAcceleration(acceleration2);  // Restore normal acceleration
          shocomplete = true;
        }
      }
      
      if(elbbegin && !elbfinish) {
        if(digitalRead(limitPin3) == LOW){
          motor3.run();
        }
        else{
          motor3.stop();
          motor3.setCurrentPosition(0);  // Set home position to zero
          motor3.enableOutputs();  // Ensure motor is enabled after homing
          motor3.setAcceleration(acceleration2);  // Restore normal acceleration
          elbfinish = true;
        }
      }
    }
}


// ============================================================
// I2C Receive Event
// ============================================================

void receiveEvent(int howMany) {
  //doesnt seem to be working? dont want to remove it in case it causes more problems
  delay(2); // small delay to try and help prevent timing issues

  String receivedData = "";
  // Read all bytes from I2C
  while (Wire.available()) {
    char c = Wire.read();
    if (c != '\0' && c != '\n') { // Ignore null and newline characters
      receivedData += c;
    }
  }
  // Clean up the string
  receivedData.trim(); // Remove leading/trailing spaces


  if (receivedData == "Game ended!") {
    //Serial.println("Game ended!");
    return;
  }

  if (receivedData.startsWith("remove jumped piece")) {
    //Serial.println("jump confirmed");
    jumpconfirm = true;
  }

  if (receivedData.startsWith("YOU LOSE")) {
    //Serial.println("Game Over!");
    playerlose = true;
    return;
  }

  if (receivedData == "Result: Player Wins") {
  //Serial.println("Game Over!");
  playerwin = true;
  return;
  }

  if (receivedData.startsWith("You surrendered")) {
    //Serial.println(receivedData);
    surrendered = true; // Let loop() handle LCD + reset
    return;
  }

  // Handle generic game outcomes
  if (receivedData.startsWith("Result:") || receivedData.startsWith("You have no moves left")) {
    //Serial.println(receivedData); // Print ALL game results (win, lose, draw, no moves)
    return;
  }
  if (receivedData.startsWith("JUMPED:")) {
      int x, y;
      //Reads the jumped coordinates setting them to the X and Y values
      if (sscanf(receivedData.c_str(), "JUMPED: (%d,%d)", &x, &y) == 2) {
          jump_I = x;
          jump_J = y;
          // sets the piecejumped variable to true
          pieceJumped = true;
      }
      return; // We’ve handled this message, so exit early
  }
  if (receivedData.startsWith("Is Jumping")) {
    if(!jumpPrompt && !jumpwait){
      jumpPrompt = true;
      jumpwait = true;
      lastjump = millis();
      jumpphase = 0;
      updateLCD("Is Jumping", "Mandatory?");
   }
  return;
  }
  if (receivedData.startsWith("Yes Jumping") && jumpwait) {
    jumpIsMandatory = true;
    showJumpStatus = true;
    jumpStatusDisplayed = false;  // Ensure timing starts
    return;
  } 
  else if (receivedData.startsWith("No Jumping") && jumpwait) {
    jumpIsMandatory = false;
    showJumpStatus = true;
    jumpStatusDisplayed = false;
    return;
  }
  if (receivedData == "Game ended!") {
    //Serial.println("Game ended!");
    updateLCD("Game Over", "");
    GameStart = false;
    return;
  }
  // Check for move messages
  if (receivedData.startsWith("Player move:")) {
    moveType = "Player";
    receivedData.replace("Player move:", "");
    receivedData.trim();
  } else if (receivedData.startsWith("Computer move:")) {
    moveType = "Computer";
    receivedData.replace("Computer move:", "");
    receivedData.trim();
  } else {
    // **Print all unrecognized messages**
    //Serial.print("Game Message: ");
    //Serial.println(receivedData);
    return; // Exit if the message is not a move
  }
  // Parse the coordinates
  if (sscanf(receivedData.c_str(), "(%d,%d) -> (%d,%d)", &old_I, &old_J, &new_I, &new_J) == 4) {
    newMoveReceived = true; // Parsing successful
  }
}
