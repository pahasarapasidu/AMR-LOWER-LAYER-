/*
 * Arduino Leonardo Sketch for Dual CL57T(V4.1) Closed Loop Stepper Driver Control
 * Controls left and right motors for a differential drive robot
 * Full Step Mode, Closed Loop Operation
 * 
 * INVERTED SIGNAL CONFIGURATION:
 * LEFT MOTOR:
 * - Arduino D6 -> PUL- (Step pulse)
 * - Arduino D12 -> DIR- (Direction)
 * - Arduino D5 -> ENA- (Enable)
 * - 5V -> PUL+, DIR+, ENA+
 * 
 * RIGHT MOTOR:
 * - Arduino D9 -> PUL- (Step pulse)
 * - Arduino D8 -> DIR- (Direction)
 * - Arduino A0 -> ENA- (Enable)
 * - 5V -> PUL+, DIR+, ENA+
 * 
 * Make sure to set DIP switches on both drivers correctly:
 * - SW1-SW4: ON-ON-ON-ON (Full step mode)
 * - SW5: Set for desired direction
 * - SW6: OFF (Closed loop control)
 * - SW7: OFF (PUL/DIR mode)
 * - SW8: Your preference (pulse filter time)
 * 
 * S3 Switch: Set to 5V for Arduino control
 */

// Pin definitions
// Left motor pins
const int LEFT_PULSE_PIN = 6;   // Connect to PUL- of left motor
const int LEFT_DIR_PIN = 12;    // Connect to DIR- of left motor
const int LEFT_ENA_PIN = 5;     // Connect to ENA- of left motor

// Right motor pins
const int RIGHT_PULSE_PIN = 9;  // Connect to PUL- of right motor
const int RIGHT_DIR_PIN = 8;    // Connect to DIR- of right motor
const int RIGHT_ENA_PIN = A0;   // Connect to ENA- of right motor (using analog pin as digital)

// Motion parameters
const int PULSE_PER_REV = 200;  // For full step mode (1.8° motor)
int motorSpeed = 200;           // Speed in RPM
int motorAccel = 50;            // Acceleration in RPM per second

// Motor-specific parameters
int leftTargetSteps = 0;    // Target position for left motor
int rightTargetSteps = 0;   // Target position for right motor
int leftDirection = LOW;    // Direction flag for left motor - INVERTED - LOW means forward
int rightDirection = LOW;   // Direction flag for right motor - INVERTED - LOW means forward

// Timing variables
unsigned long leftLastStepTime = 0;
unsigned long rightLastStepTime = 0;
unsigned long stepInterval = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ;  // Wait for serial port to connect (needed for Leonardo only)
  }

  // Configure control pins for left motor
  pinMode(LEFT_PULSE_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(LEFT_ENA_PIN, OUTPUT);

  // Configure control pins for right motor
  pinMode(RIGHT_PULSE_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_ENA_PIN, OUTPUT);

  // Initialize pin states
  // PUL- pins should be HIGH when not pulsing (inverted logic)
  digitalWrite(LEFT_PULSE_PIN, HIGH);
  digitalWrite(RIGHT_PULSE_PIN, HIGH);
  
  // DIR- pins - set direction (inverted logic)
  digitalWrite(LEFT_DIR_PIN, leftDirection);
  digitalWrite(RIGHT_DIR_PIN, rightDirection);
  
  // ENA- pins - HIGH to enable the drivers (inverted logic)
  digitalWrite(LEFT_ENA_PIN, HIGH);   // 
  digitalWrite(RIGHT_ENA_PIN, HIGH);  // As the controller works without enable signal, this gotta be inverted in terms of optocoupler logic ig compared to other signals. motor works without the led of the coupler not turned on. 

  // Calculate initial step interval
  setSpeed(motorSpeed);

  Serial.println("Dual CL57T(V4.1) Stepper Driver Control for Robot");
  Serial.println("Commands:");
  Serial.println("  S<rpm> - Set speed for both motors in RPM");
  Serial.println("  A<rpm> - Set acceleration in RPM/sec");
  Serial.println("  ML<steps> - Move left motor specified steps (+ forward, - backward)");
  Serial.println("  MR<steps> - Move right motor specified steps (+ forward, - backward)");
  Serial.println("  MB<steps> - Move both motors the same number of steps (+ forward, - backward)");
  Serial.println("  F<steps> - Move forward specified steps");
  Serial.println("  B<steps> - Move backward specified steps");
  Serial.println("  R<steps> - Turn right (rotate clockwise)");
  Serial.println("  L<steps> - Turn left (rotate counterclockwise)");
  Serial.println("  E0/E1 - Disable/Enable both motors");
  Serial.println("  EL0/EL1 - Disable/Enable left motor");
  Serial.println("  ER0/ER1 - Disable/Enable right motor");
  Serial.println("  STOP - Immediately stop all movement");
}

void loop() {
  // Check for serial commands
  checkSerialCommand();

  // Handle left motor movement
  if (leftTargetSteps != 0) {
    unsigned long currentTime = micros();

    // Check if it's time for the next step
    if (currentTime - leftLastStepTime >= stepInterval) {
      leftLastStepTime = currentTime;

      // Generate a pulse for left motor - INVERTED LOGIC
      digitalWrite(LEFT_PULSE_PIN, LOW);   // Active pulse (LOW for inverted logic)
      delayMicroseconds(5);                // 5µs pulse width (must be > 1µs per manual)
      digitalWrite(LEFT_PULSE_PIN, HIGH);  // Inactive state (HIGH for inverted logic)

      // Update remaining steps
      if (leftTargetSteps > 0) {
        leftTargetSteps--;
      } else {
        leftTargetSteps++;
      }
    }
  }

  // Handle right motor movement
  if (rightTargetSteps != 0) {
    unsigned long currentTime = micros();

    // Check if it's time for the next step
    if (currentTime - rightLastStepTime >= stepInterval) {
      rightLastStepTime = currentTime;

      // Generate a pulse for right motor - INVERTED LOGIC
      digitalWrite(RIGHT_PULSE_PIN, LOW);   // Active pulse (LOW for inverted logic)
      delayMicroseconds(5);                 // 5µs pulse width (must be > 1µs per manual)
      digitalWrite(RIGHT_PULSE_PIN, HIGH);  // Inactive state (HIGH for inverted logic)

      // Update remaining steps
      if (rightTargetSteps > 0) {
        rightTargetSteps--;
      } else {
        rightTargetSteps++;
      }
    }
  }
}

void checkSerialCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'S':
      case 's':
        // Set speed for both motors
        motorSpeed = Serial.parseInt();
        setSpeed(motorSpeed);
        Serial.print("Speed set to ");
        Serial.println(motorSpeed);
        break;

      case 'A':
      case 'a':
        // Set acceleration for both motors
        motorAccel = Serial.parseInt();
        Serial.print("Acceleration set to ");
        Serial.println(motorAccel);
        break;

      case 'M':
      case 'm':
        // Check the next character to determine which motor to move
        if (Serial.available() > 0) {
          char motorSelect = Serial.read();

          if (motorSelect == 'L' || motorSelect == 'l') {
            // Move left motor
            leftTargetSteps = Serial.parseInt();
            setLeftDirection(leftTargetSteps);
            Serial.print("Moving left motor ");
            Serial.print(abs(leftTargetSteps));
            Serial.println(leftDirection == LOW ? " steps forward" : " steps backward");  // INVERTED LOGIC
          } else if (motorSelect == 'R' || motorSelect == 'r') {
            // Move right motor
            rightTargetSteps = Serial.parseInt();
            setRightDirection(rightTargetSteps);
            Serial.print("Moving right motor ");
            Serial.print(abs(rightTargetSteps));
            Serial.println(rightDirection == LOW ? " steps forward" : " steps backward");  // INVERTED LOGIC
          } else if (motorSelect == 'B' || motorSelect == 'b') {
            // Move both motors
            int steps = Serial.parseInt();
            leftTargetSteps = steps;
            rightTargetSteps = steps;
            setLeftDirection(-leftTargetSteps);
            setRightDirection(rightTargetSteps);
            Serial.print("Moving both motors ");
            Serial.print(abs(steps));
            Serial.println(steps > 0 ? " steps forward" : " steps backward");
          }
        }
        break;

      case 'F':
      case 'f':
        // Move forward (both motors forward)
        {
          int steps = Serial.parseInt();
          if (steps < 0) steps = -steps;  // Ensure positive for forward

          leftTargetSteps = steps;
          rightTargetSteps = steps;
          setLeftDirection(leftTargetSteps);
          setRightDirection(rightTargetSteps);

          Serial.print("Moving forward ");
          Serial.print(steps);
          Serial.println(" steps");
        }
        break;

      case 'B':
      case 'b':
        // Move backward (both motors backward)
        {
          int steps = Serial.parseInt();
          if (steps < 0) steps = -steps;  // Ensure positive then make negative for backward

          leftTargetSteps = -steps;
          rightTargetSteps = -steps;
          setLeftDirection(leftTargetSteps);
          setRightDirection(rightTargetSteps);

          Serial.print("Moving backward ");
          Serial.print(steps);
          Serial.println(" steps");
        }
        break;

      case 'R':
      case 'r':
        // Turn right (rotate clockwise)
        {
          int steps = Serial.parseInt();
          if (steps < 0) steps = -steps;  // Ensure positive

          leftTargetSteps = steps;
          rightTargetSteps = -steps;
          setLeftDirection(leftTargetSteps);
          setRightDirection(rightTargetSteps);

          Serial.print("Turning right ");
          Serial.print(steps);
          Serial.println(" steps");
        }
        break;

      case 'L':
      case 'l':
        // Turn left (rotate counterclockwise)
        {
          int steps = Serial.parseInt();
          if (steps < 0) steps = -steps;  // Ensure positive

          leftTargetSteps = -steps;
          rightTargetSteps = steps;
          setLeftDirection(leftTargetSteps);
          setRightDirection(rightTargetSteps);

          Serial.print("Turning left ");
          Serial.print(steps);
          Serial.println(" steps");
        }
        break;

      case 'E':
      case 'e':
        // Enable/disable motors based on the next character
        if (Serial.available() > 0) {
          char motorSelect = Serial.read();

          if (motorSelect == 'L' || motorSelect == 'l') {
            // Left motor control
            int state = Serial.parseInt();
            digitalWrite(LEFT_ENA_PIN, state == 0 ? HIGH : LOW);  // INVERTED: HIGH = enabled, LOW = disabled
            Serial.println(state == 0 ? "Left motor enabled" : "Left motor disabled");
          } else if (motorSelect == 'R' || motorSelect == 'r') {
            // Right motor control
            int state = Serial.parseInt();
            digitalWrite(RIGHT_ENA_PIN, state == 0 ? HIGH : LOW);  // INVERTED: HIGH = enabled, LOW = disabled
            Serial.println(state == 0 ? "Right motor enabled" : "Right motor disabled");
          } else {
            // Both motors
            int state = Serial.parseInt();
            digitalWrite(LEFT_ENA_PIN, state == 0 ? HIGH : LOW);   // INVERTED: HIGH = enabled, LOW = disabled
            digitalWrite(RIGHT_ENA_PIN, state == 0 ? HIGH : LOW);  // INVERTED: HIGH = enabled, LOW = disabled
            Serial.println(state == 0 ? "Both motors enabled" : "Both motors disabled");
          }
        }
        break;

      case 'T':
      case 't':
        // Check for STOP command
        if (Serial.available() > 0) {
          char nextChar = Serial.read();
          if (nextChar == 'O' || nextChar == 'o') {
            // If the command is starting with "TO" or "STOP"
            if (Serial.available() > 0 && Serial.read() == 'P') {
              emergencyStop();
            }
          }
        }
        break;
    }

    // Clear remaining characters
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}

void setSpeed(int rpm) {
  // Calculate step interval in microseconds
  // Step interval = (60 seconds * 1,000,000 microseconds) / (rpm * steps per revolution)
  long stepsPerMinute = (long)rpm * PULSE_PER_REV;
  stepInterval = 60000000L / stepsPerMinute;
}

void setLeftDirection(int steps) {
  if (steps > 0) {
    // Forward direction - INVERTED LOGIC
    digitalWrite(LEFT_DIR_PIN, LOW);   // CHANGED: LOW for forward with inverted logic
    leftDirection = LOW;
  } else {
    // Backward direction - INVERTED LOGIC
    digitalWrite(LEFT_DIR_PIN, HIGH);  // CHANGED: HIGH for backward with inverted logic
    leftDirection = HIGH;
  }
}

void setRightDirection(int steps) {
  if (steps > 0) {
    // Forward direction - INVERTED LOGIC
    digitalWrite(RIGHT_DIR_PIN, LOW);   // CHANGED: LOW for forward with inverted logic
    rightDirection = LOW;
  } else {
    // Backward direction - INVERTED LOGIC
    digitalWrite(RIGHT_DIR_PIN, HIGH);  // CHANGED: HIGH for backward with inverted logic
    rightDirection = HIGH;
  }
}

void emergencyStop() {
  // Immediately stop all movement
  leftTargetSteps = 0;
  rightTargetSteps = 0;
  Serial.println("EMERGENCY STOP - All motors stopped");
}