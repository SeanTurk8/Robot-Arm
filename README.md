This robot arm was built as part of a senior capstone project at UMass Dartmouth. It’s designed to physically play a full game of checkers against a human on a real 8x8 board. The robot arm uses stepper motors to control movement across three joints—base, shoulder, and elbow—plus a DC motor claw for grabbing pieces. A Raspberry Pi runs the game logic and sends move commands to an Arduino Mega using I2C. Each board position has a set of pre-recorded angles to ensure accurate movement. The system uses limit switches for consistent homing and has an LCD screen to display turn updates and game status. Players remove jumped pieces by hand and press the ‘Enter’ button to confirm. The robot supports full checkers gameplay, including win/loss detection and surrender options.

Features
- Full physical checkers game on an 8x8 board
- Robotic arm with a base, shoulder, elbow + 3-prong DC gripper
- Piece detection using a Raspberry Pi camera
- LCD screen shows turn prompts and game outcomes
- Buttons for Start, Surrender, and Enter
- Manual piece removal with button confirmation
- I2C communication between Pi and Arduino Mega

Hardware
- Arduino Mega 2560
- Raspberry Pi 4 (AI)
- NEMA 17 stepper motors x3
- DC motor for claw gripper
- Limit switches (base, shoulder, elbow)
- I2C 16x2 LCD (LiquidCrystal_SoftI2C)

Software
- Arduino firmware written in C++
- Uses:
    - AccelStepper for motor control
    - LiquidCrystal_SoftI2C for LCD
    - Wire and SoftwareWire for I2C
- Python code on Raspberry Pi:
    - Detects moves with camera
    - Runs game logic and sends moves to Arduino via I2C
    - Handles mandatory jump rule 
  
Operation Flow
- On boot, LCD shows "Press START to play"
- Game asks if jumping is mandatory (1 press = Yes, 2 presses = No)
- Player moves are detected via Pi camera
- Jumped pieces are manually removed with player confirmation
- Game ends with result shown on LCD

Setup
- Define joint angles per board square in angleMap[8][8]
- Upload Arduino firmware
- Connect Pi and Arduino via I2C (address 0x08)
- Run the Pi's main Python script to start the game

Notes
- Homing is done using limit switches
- Each square's angle was manually tuned
- Piece confirmation is required to reduce camera error impact
