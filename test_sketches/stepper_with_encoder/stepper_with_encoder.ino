/*
 * Arduino Leonardo Sketch for Dual CL57T(V4.1) Closed Loop Stepper Driver Control
 * Controls left and right motors for a differential drive robot
 * Full Step Mode, Closed Loop Operation with Encoder Reading
 * 
 * INVERTED SIGNAL CONFIGURATION:
 * LEFT MOTOR:
 * - Arduino D11 -> PUL- (Step pulse) = PB7 / OC0A
 * - Arduino D12 -> DIR- (Direction)  = PD6
 * - Arduino D5  -> ENA- (Enable)     = PC6
 * - 5V -> PUL+, DIR+, ENA+
 * 
 * RIGHT MOTOR:
 * - Arduino D9  -> PUL- (Step pulse) = PB5 / OC1A
 * - Arduino D8  -> DIR- (Direction)  = PB4
 * - Arduino D10 -> ENA- (Enable)     = PB0 (Currently not terminated at header)
 * - 5V -> PUL+, DIR+, ENA+
 * 
 * ENCODER CONFIGURATION:
 * LEFT MOTOR:
 * - Encoder Pin A -> D7
 * - Encoder Pin B -> D6
 * 
 * RIGHT MOTOR:
 * - Currently not connected
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
const int LEFT_PULSE_PIN = 6;  // Connect to PUL- of left motor (PB7/OC0A)
const int LEFT_DIR_PIN = 12;    // Connect to DIR- of left motor (PD6)
const int LEFT_ENA_PIN = 5;     // Connect to ENA- of left motor (PC6)

// Right motor pins
const int RIGHT_PULSE_PIN = 9;  // Connect to PUL- of right motor (PB5/OC1A)
const int RIGHT_DIR_PIN = 8;    // Connect to DIR- of right motor (PB4)
const int RIGHT_ENA_PIN = A0;   // Connect to ENA- of right motor (PB0) - Not terminated at header currently

// Encoder pins
const int LEFT_ENCODER_PIN_A = 1;  // Left motor encoder channel A
const int LEFT_ENCODER_PIN_B = 0;  // Left motor encoder channel B

// Motion parameters
const int PULSE_PER_REV = 200;  // For full step mode (1.8° motor)
int motorSpeed = 200;           // Speed in RPM
int motorAccel = 50;            // Acceleration in RPM per second

// Motor-specific parameters
int leftTargetSteps = 0;   // Target position for left motor
int rightTargetSteps = 0;  // Target position for right motor
int leftDirection = LOW;   // Direction flag for left motor - INVERTED - LOW means forward
int rightDirection = LOW;  // Direction flag for right motor - INVERTED - LOW means forward

// Timing variables
unsigned long leftLastStepTime = 0;
unsigned long rightLastStepTime = 0;
unsigned long stepInterval = 0;
unsigned long lastEncoderPrintTime = 0;  // For continuous encoder monitoring

// Monitoring flags
bool continuousMonitoring = false;  // Flag for continuous display of encoder values

// Encoder variables
volatile long leftEncoderCounter = 0;  // Encoder rotation count for left motor
volatile long lastEncodedLeft = 0;     // Last encoded value for left motor

// Buffer for reading commands
String commandBuffer = "";
bool commandReady = false;

// Called when left encoder changes state (ISR)
void leftEncoderISR() {
  int MSB = digitalRead(LEFT_ENCODER_PIN_A);  // Most Significant Bit (A)
  int LSB = digitalRead(LEFT_ENCODER_PIN_B);  // Least Significant Bit (B)

  int encoded = (MSB << 1) | LSB;              // Create a 2-bit value from A and B
  int sum = (lastEncodedLeft << 2) | encoded;  // Combine current and previous states

  // Update position based on the transition
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    leftEncoderCounter--;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    leftEncoderCounter++;
  }

  lastEncodedLeft = encoded;  // Save the current state
}

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

  // Configure encoder pins with pull-up resistors
  pinMode(LEFT_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN_B, INPUT_PULLUP);

  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_B), leftEncoderISR, CHANGE);

  // Initialize pin states
  // PUL- pins should be HIGH when not pulsing (inverted logic)
  digitalWrite(LEFT_PULSE_PIN, HIGH);
  digitalWrite(RIGHT_PULSE_PIN, HIGH);

  // DIR- pins - set direction (inverted logic)
  digitalWrite(LEFT_DIR_PIN, leftDirection);
  digitalWrite(RIGHT_DIR_PIN, rightDirection);

  // ENA- pins - HIGH to enable the drivers (inverted logic)
  digitalWrite(LEFT_ENA_PIN, HIGH);   // Enable motor (HIGH for inverted logic)
  digitalWrite(RIGHT_ENA_PIN, HIGH);  // Enable motor (HIGH for inverted logic)

  // Calculate initial step interval
  setSpeed(motorSpeed);

  Serial.println("Dual CL57T(V4.1) Stepper Driver Control with Encoder Feedback");
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
  Serial.println("  HALT - Immediately stop all movement");
  Serial.println("  P - Print encoder values once");
  Serial.println("  Z - Monitor encoder values continuously");
  Serial.println("  X - Stop continuous monitoring");
  Serial.println("  C - Clear/reset encoder values");
}

void loop() {
  // Read and buffer incoming serial data
  readSerialData();
  
  // Process commands when ready
  if (commandReady) {
    processCommand();
    commandReady = false;
    commandBuffer = "";
  }

  // Handle continuous encoder value monitoring if enabled
  if (continuousMonitoring) {
    unsigned long currentTime = millis();
    // Print encoder values every 100ms to keep the serial port readable
    if (currentTime - lastEncoderPrintTime >= 100) {
      lastEncoderPrintTime = currentTime;
      printEncoderValues();
    }
  }

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

void readSerialData() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Check for end of command (newline or carriage return)
    if (inChar == '\n' || inChar == '\r') {
      if (commandBuffer.length() > 0) {
        commandReady = true;
        return;
      }
    } else {
      commandBuffer += inChar;
    }
  }
}

void processCommand() {
  // Convert command to uppercase for easier comparison
  commandBuffer.trim();
  
  // Handle special commands first
  if (commandBuffer.equalsIgnoreCase("HALT")) {
    emergencyStop();
    return;
  } else if (commandBuffer.equalsIgnoreCase("P")) {
    printEncoderValues();
    return;
  } else if (commandBuffer.equalsIgnoreCase("Z")) {
    continuousMonitoring = true;
    lastEncoderPrintTime = millis();
    Serial.println("Starting continuous encoder monitoring");
    Serial.println("Send X to end monitoring");
    return;
  } else if (commandBuffer.equalsIgnoreCase("X")) {
    continuousMonitoring = false;
    Serial.println("Stopped continuous monitoring");
    return;
  } else if (commandBuffer.equalsIgnoreCase("C")) {
    leftEncoderCounter = 0;
    Serial.println("Encoder values cleared");
    return;
  }
  
  // Handle commands with parameters
  if (commandBuffer.length() >= 2) {
    char cmd = commandBuffer.charAt(0);
    
    switch (cmd) {
      case 'S':
      case 's':
        // Set speed for both motors
        {
          String valueStr = commandBuffer.substring(1);
          int value = valueStr.toInt();
          if (value > 0) {
            motorSpeed = value;
            setSpeed(motorSpeed);
            Serial.print("Speed set to ");
            Serial.println(motorSpeed);
          }
        }
        break;
        
      case 'A':
      case 'a':
        // Set acceleration for both motors
        {
          String valueStr = commandBuffer.substring(1);
          int value = valueStr.toInt();
          if (value > 0) {
            motorAccel = value;
            Serial.print("Acceleration set to ");
            Serial.println(motorAccel);
          }
        }
        break;
        
      case 'M':
      case 'm':
        // Motion commands for specific motors
        if (commandBuffer.length() >= 3) {
          char motorSelect = commandBuffer.charAt(1);
          String valueStr = commandBuffer.substring(2);
          int steps = valueStr.toInt();
          
          if (motorSelect == 'L' || motorSelect == 'l') {
            // Move left motor
            leftTargetSteps = steps;
            setLeftDirection(leftTargetSteps);
            Serial.print("Moving left motor ");
            Serial.print(abs(leftTargetSteps));
            Serial.println(leftDirection == LOW ? " steps forward" : " steps backward");  // INVERTED LOGIC
          } else if (motorSelect == 'R' || motorSelect == 'r') {
            // Move right motor
            rightTargetSteps = steps;
            setRightDirection(rightTargetSteps);
            Serial.print("Moving right motor ");
            Serial.print(abs(rightTargetSteps));
            Serial.println(rightDirection == LOW ? " steps forward" : " steps backward");  // INVERTED LOGIC
          } else if (motorSelect == 'B' || motorSelect == 'b') {
            // Move both motors
            leftTargetSteps = steps;
            rightTargetSteps = steps;
            setLeftDirection(leftTargetSteps);
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
          String valueStr = commandBuffer.substring(1);
          int steps = valueStr.toInt();
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
          String valueStr = commandBuffer.substring(1);
          int steps = valueStr.toInt();
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
          String valueStr = commandBuffer.substring(1);
          int steps = valueStr.toInt();
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
          String valueStr = commandBuffer.substring(1);
          int steps = valueStr.toInt();
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
        // Enable/disable motors based on the second character
        if (commandBuffer.length() >= 3) {
          char motorSelect = commandBuffer.charAt(1);
          String valueStr = commandBuffer.substring(2);
          int state = valueStr.toInt();
          
          if (motorSelect == 'L' || motorSelect == 'l') {
            // Left motor control
            digitalWrite(LEFT_ENA_PIN, state == 0 ? LOW : HIGH);  // INVERTED: HIGH = enabled, LOW = disabled
            Serial.println(state == 0 ? "Left motor disabled" : "Left motor enabled");
          } else if (motorSelect == 'R' || motorSelect == 'r') {
            // Right motor control
            digitalWrite(RIGHT_ENA_PIN, state == 0 ? LOW : HIGH);  // INVERTED: HIGH = enabled, LOW = disabled
            Serial.println(state == 0 ? "Right motor disabled" : "Right motor enabled");
          } else {
            // Both motors
            digitalWrite(LEFT_ENA_PIN, state == 0 ? LOW : HIGH);   // INVERTED: HIGH = enabled, LOW = disabled
            digitalWrite(RIGHT_ENA_PIN, state == 0 ? LOW : HIGH);  // INVERTED: HIGH = enabled, LOW = disabled
            Serial.println(state == 0 ? "Both motors disabled" : "Both motors enabled");
          }
        }
        break;
        
      default:
        // Unknown command
        Serial.print("Unknown command: ");
        Serial.println(commandBuffer);
        break;
    }
  } else {
    // Invalid command (too short)
    Serial.print("Invalid command: ");
    Serial.println(commandBuffer);
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
    digitalWrite(LEFT_DIR_PIN, LOW);  // INVERTED: LOW for forward with inverted logic
    leftDirection = LOW;
  } else {
    // Backward direction - INVERTED LOGIC
    digitalWrite(LEFT_DIR_PIN, HIGH);  // INVERTED: HIGH for backward with inverted logic
    leftDirection = HIGH;
  }
}

void setRightDirection(int steps) {
  if (steps > 0) {
    // Forward direction - INVERTED LOGIC
    digitalWrite(RIGHT_DIR_PIN, LOW);  // INVERTED: LOW for forward with inverted logic
    rightDirection = LOW;
  } else {
    // Backward direction - INVERTED LOGIC
    digitalWrite(RIGHT_DIR_PIN, HIGH);  // INVERTED: HIGH for backward with inverted logic
    rightDirection = HIGH;
  }
}

void emergencyStop() {
  // Immediately stop all movement
  leftTargetSteps = 0;
  rightTargetSteps = 0;

  // Also stop monitoring if it's active
  if (continuousMonitoring) {
    continuousMonitoring = false;
    Serial.println("EMERGENCY STOP - Motors stopped and monitoring ended");
  } else {
    Serial.println("EMERGENCY STOP - All motors stopped");
  }
}

void printEncoderValues() {
  if (continuousMonitoring) {
    // Simplified output for continuous monitoring to reduce serial traffic
    Serial.print("L:");
    Serial.println(leftEncoderCounter);
  } else {
    // Detailed output for one-time display
    Serial.println("-------- Encoder Values --------");
    Serial.print("Left encoder: ");
    Serial.println(leftEncoderCounter);
    Serial.println("-------------------------------");
  }
}