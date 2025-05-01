/*
 * Arduino Leonardo Sketch for CL57T(V4.1) Closed Loop Stepper Driver Control
 * Full Step Mode, Closed Loop Operation
 * 
 * Connections:
 * - Arduino D9 -> PUL+ (Step pulse)
 * - Arduino D8 -> DIR+ (Direction)
 * - Arduino D7 -> ENA+ (Enable, optional)
 * - GND -> PUL-, DIR-, ENA-
 * 
 * Make sure to set DIP switches correctly:
 * - SW1-SW4: ON-ON-ON-ON (Full step mode)
 * - SW5: Set for desired direction
 * - SW6: OFF (Closed loop control)
 * - SW7: OFF (PUL/DIR mode)
 * - SW8: Your preference (pulse filter time)
 * 
 * S3 Switch: Set to 5V for Arduino control
 */

// Pin definitions
const int PULSE_PIN = 9;  // Connect to PUL+
const int DIR_PIN = 8;    // Connect to DIR+
const int ENA_PIN = 7;    // Connect to ENA+ (optional)

// Motion parameters
const int PULSE_PER_REV = 200;  // For full step mode (1.8° motor)
int motorSpeed = 200;           // Speed in RPM
int motorAccel = 50;            // Acceleration in RPM per second
int targetSteps = 0;            // Target position in steps
boolean newCommand = false;     // Flag for new command from serial

// Timing variables
unsigned long lastStepTime = 0;
unsigned long stepInterval = 0;
int currentDirection = HIGH;    // HIGH = CW, LOW = CCW

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for Leonardo only)
  }
  
  // Configure control pins
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  
  // Initialize pin states
  digitalWrite(PULSE_PIN, LOW);
  digitalWrite(DIR_PIN, currentDirection);
  digitalWrite(ENA_PIN, LOW);  // Enable driver (active LOW)
  
  // Calculate initial step interval
  setSpeed(motorSpeed);
  
  Serial.println("CL57T(V4.1) Stepper Driver Control");
  Serial.println("Commands:");
  Serial.println("  S<rpm> - Set speed in RPM");
  Serial.println("  A<rpm> - Set acceleration in RPM/sec");
  Serial.println("  M<steps> - Move specified number of steps (+ CW, - CCW)");
  Serial.println("  E0/E1 - Disable/Enable motor");
}

void loop() {
  // Check for serial commands
  checkSerialCommand();
  
  // Execute motion if there are steps to take
  if (targetSteps != 0) {
    unsigned long currentTime = micros();
    
    // Check if it's time for the next step
    if (currentTime - lastStepTime >= stepInterval) {
      lastStepTime = currentTime;
      
      // Generate a pulse
      digitalWrite(PULSE_PIN, HIGH);
      delayMicroseconds(5);  // 5µs pulse width (must be > 1µs per manual)
      digitalWrite(PULSE_PIN, LOW);
      
      // Update remaining steps
      if (targetSteps > 0) {
        targetSteps--;
      } else {
        targetSteps++;
      }
    }
  }
}

void checkSerialCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'S': case 's':
        // Set speed
        motorSpeed = Serial.parseInt();
        setSpeed(motorSpeed);
        Serial.print("Speed set to ");
        Serial.println(motorSpeed);
        break;
        
      case 'A': case 'a':
        // Set acceleration
        motorAccel = Serial.parseInt();
        Serial.print("Acceleration set to ");
        Serial.println(motorAccel);
        break;
        
      case 'M': case 'm':
        // Move specified steps
        targetSteps = Serial.parseInt();
        if (targetSteps > 0) {
          digitalWrite(DIR_PIN, HIGH);  // CW direction
          currentDirection = HIGH;
        } else {
          digitalWrite(DIR_PIN, LOW);   // CCW direction
          currentDirection = LOW;
        }
        Serial.print("Moving ");
        Serial.print(abs(targetSteps));
        Serial.println(currentDirection == HIGH ? " steps CW" : " steps CCW");
        break;
        
      case 'E': case 'e':
        // Enable/disable motor
        int state = Serial.parseInt();
        digitalWrite(ENA_PIN, state == 0);  // Active LOW enable
        Serial.println(state == 0 ? "Motor enabled" : "Motor disabled");
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