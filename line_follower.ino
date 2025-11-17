// =====================================
// Line Following Robot with Serial Control
// Compatible with Arduino UNO R4 WiFi
// Uses 3 line sensors: Left, Middle, Right
// @Krzysztof Kozyra
// =====================================

const int LEFT_SENSOR_PIN = 7;
const int MIDDLE_SENSOR_PIN = 8;
const int RIGHT_SENSOR_PIN = 9;

const int LEFT_MOTOR_PWM = 10;
const int RIGHT_MOTOR_PWM = 11;
const int LEFT_MOTOR_DIR_A = 2;
const int LEFT_MOTOR_DIR_B = 3;
const int RIGHT_MOTOR_DIR_A = 4;
const int RIGHT_MOTOR_DIR_B = 5;

const int MODE_SWITCH_PIN = 12;  // Pin for physical mode switch
// Connect switch: one side to pin 12, other side to GND
// When switch is LOW (connected to GND) = AUTO mode
// When switch is HIGH (pulled up) = MANUAL mode

const bool LEFT_MOTOR_REVERSED = false;  // Change to true if left motor is reversed
const bool RIGHT_MOTOR_REVERSED = true;  // Change to true if right motor is reversed

bool isManualMode = true;  // Start in AUTO mode by default (for standalone operation)
bool hasCommandToProcess = false;
bool isParsingSpeed = false;
bool usePhysicalSwitch = true;  // Set to false to ignore physical switch

int manualModeSpeed = 0;
int autoModeBaseSpeed = 225;  // Base speed for auto mode (0-255)

// === PID CONTROL VARIABLES ===
int currentError = 0;
int previousError = 0;
int errorIntegral = 0;
int lastKnownError = 0;  // Last error when line was detected
unsigned long lineDetectionTime = 0;
unsigned long lineLostTime = 0;
bool isLineDetected = true;

// PID Constants
float proportionalGain = 4.0;
float integralGain = 0.0001;
float derivativeGain = 0.0001;

// === LINE LOSS RECOVERY SETTINGS ===
const unsigned long LINE_LOST_TIMEOUT = 300;  // Time in ms before entering recovery mode
const int RECOVERY_SEARCH_SPEED = 140;        // Speed when searching for line

int parsedCommand = 0;
int parsedSpeed = 0;

void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_A, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_B, OUTPUT);

  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(MIDDLE_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);

  if (usePhysicalSwitch) {
    checkModeSwitch();
  }

  Serial.println("=================================");
  Serial.println("Line Follower Robot Ready!");
  if (isManualMode) {
    Serial.println("Mode: MANUAL");
  } else {
    Serial.println("Mode: AUTO (Line Following)");
    Serial.print("Base Speed: ");
    Serial.println(autoModeBaseSpeed);
  }
  Serial.println("Type 40 for help");
  Serial.println("=================================");
}


int calculateLinePosition() {
  // Read sensors (inverted logic - LOW means line detected)
  int leftValue = !digitalRead(LEFT_SENSOR_PIN) * 10;
  int middleValue = !digitalRead(MIDDLE_SENSOR_PIN) * 100;
  int rightValue = !digitalRead(RIGHT_SENSOR_PIN) * 1000;

  int sensorSum = leftValue + middleValue + rightValue;

  // Check if line is detected
  if (sensorSum == 0) {
    isLineDetected = false;
    if (lineLostTime == 0) {
      lineLostTime = millis();
    }
  } else {
    isLineDetected = true;
    lineDetectionTime = millis();
    lineLostTime = 0;
  }

  // Return error value based on sensor combination
  switch (sensorSum) {
    case 10: lastKnownError = 50; return 50;      // Only left sensor - line is far left
    case 110: lastKnownError = 25; return 25;     // Left + middle - line slightly left
    case 100: lastKnownError = 0; return 0;       // Only middle - line is centered
    case 1100: lastKnownError = -25; return -25;  // Middle + right - line slightly right
    case 1000: lastKnownError = -50; return -50;  // Only right - line is far right
    case 1110: lastKnownError = 0; return 0;      // All three - wide line, treat as centered
    case 0: return lastKnownError;                // No line - use last known position
    default: return lastKnownError;
  }
}

void followLine() {

  currentError = calculateLinePosition();

  // LINE RECOVERY MODE - if line is lost for too long
  if (!isLineDetected && lineLostTime > 0 && (millis() - lineLostTime) > LINE_LOST_TIMEOUT) {

    Serial.println("LINE LOST! Recovery mode...");

    // Continue turning in the direction of last known error
    if (lastKnownError > 0) {
      // Line was on the left - turn left sharply
      setMotorSpeeds(RECOVERY_SEARCH_SPEED, RECOVERY_SEARCH_SPEED, false, true);
      Serial.println("Searching left...");
    } else if (lastKnownError < 0) {
      // Line was on the right - turn right sharply
      setMotorSpeeds(RECOVERY_SEARCH_SPEED, RECOVERY_SEARCH_SPEED, true, false);
      Serial.println("Searching right...");
    } else {
      // Unknown direction - spin in place to search
      setMotorSpeeds(RECOVERY_SEARCH_SPEED, RECOVERY_SEARCH_SPEED, false, true);
      Serial.println("Searching...");
    }

    // Reset integral to avoid windup during recovery
    errorIntegral = 0;
    return;
  }

  // NORMAL PID CONTROL
  // Update integral with anti-windup
  errorIntegral += currentError;
  errorIntegral = constrain(errorIntegral, -10000, 10000);

  int errorDerivative = currentError - previousError;

  float pidOutput = (proportionalGain * currentError) + (integralGain * errorIntegral) + (derivativeGain * errorDerivative);

  previousError = currentError;
  pidOutput = constrain(pidOutput, -512, 512);

  // Apply PID correction to motor speeds
  int leftMotorSpeed = autoModeBaseSpeed;
  int rightMotorSpeed = autoModeBaseSpeed;

  if (pidOutput <= 0 && pidOutput > -256) {
    // Slight left correction - slow down right motor
    rightMotorSpeed = constrain(autoModeBaseSpeed + pidOutput, 0, 255);
    setMotorSpeeds(leftMotorSpeed, rightMotorSpeed, true, true);
  } else if (pidOutput > 0 && pidOutput < 256) {
    // Slight right correction - slow down left motor
    leftMotorSpeed = constrain(autoModeBaseSpeed - pidOutput, 0, 255);
    setMotorSpeeds(leftMotorSpeed, rightMotorSpeed, true, true);
  } else if (pidOutput < -255) {
    // Sharp left turn - reverse right motor
    rightMotorSpeed = constrain(-pidOutput - 256, 0, 255);
    setMotorSpeeds(leftMotorSpeed, rightMotorSpeed, true, false);
  } else if (pidOutput > 255) {
    // Sharp right turn - reverse left motor
    leftMotorSpeed = constrain(pidOutput - 256, 0, 255);
    setMotorSpeeds(leftMotorSpeed, rightMotorSpeed, false, true);
  }

  Serial.print("PID=");
  Serial.print(pidOutput);
  Serial.print(" | Left=");
  Serial.print(leftMotorSpeed);
  Serial.print(" | Right=");
  Serial.println(rightMotorSpeed);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed, bool leftForward, bool rightForward) {

  // Apply direction correction if motors are reversed
  if (LEFT_MOTOR_REVERSED) {
    leftForward = !leftForward;
  }
  if (RIGHT_MOTOR_REVERSED) {
    rightForward = !rightForward;
  }

  // Control left motor direction
  digitalWrite(LEFT_MOTOR_DIR_A, leftForward ? LOW : HIGH);
  digitalWrite(LEFT_MOTOR_DIR_B, leftForward ? HIGH : LOW);
  analogWrite(LEFT_MOTOR_PWM, leftSpeed);

  // Control right motor direction
  digitalWrite(RIGHT_MOTOR_DIR_A, rightForward ? LOW : HIGH);
  digitalWrite(RIGHT_MOTOR_DIR_B, rightForward ? HIGH : LOW);
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_DIR_A, LOW);
  digitalWrite(LEFT_MOTOR_DIR_B, LOW);
  digitalWrite(RIGHT_MOTOR_DIR_A, LOW);
  digitalWrite(RIGHT_MOTOR_DIR_B, LOW);
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

void moveForward(int speed) {
  setMotorSpeeds(speed, speed, true, true);
}

void moveBackward(int speed) {
  setMotorSpeeds(speed, speed, false, false);
}

void turnLeft(int speed) {
  setMotorSpeeds(speed, speed, false, true);
}

void turnRight(int speed) {
  setMotorSpeeds(speed, speed, true, false);
}

void checkModeSwitch() {
  static bool lastSwitchState = HIGH;
  bool currentSwitchState = digitalRead(MODE_SWITCH_PIN);

  if (currentSwitchState != lastSwitchState) {
    delay(50);  // Debounce delay
    currentSwitchState = digitalRead(MODE_SWITCH_PIN);

    if (currentSwitchState != lastSwitchState) {
      if (currentSwitchState == LOW) {
        // Switch connected to GND = AUTO mode
        if (isManualMode) {
          isManualMode = false;
          Serial.println("Switch: AUTO mode activated");
        }
      } else {
        // Switch disconnected (pulled HIGH) = MANUAL mode
        if (!isManualMode) {
          isManualMode = true;
          stopMotors();
          Serial.println("Switch: MANUAL mode activated");
        }
      }
      lastSwitchState = currentSwitchState;
    }
  }
}

// =====================================
// SERIAL COMMAND PARSING
// =====================================
void parseSerialInput() {
  if (Serial.available()) {
    char inputChar = Serial.read();

    if (inputChar == '\n' || inputChar == '\r') {
      hasCommandToProcess = true;
    } else if (inputChar == '_') {
      isParsingSpeed = true;
      parsedSpeed = 0;
    } else if (inputChar == ' ') {
      isParsingSpeed = false;
    } else if (isdigit(inputChar)) {
      int digit = inputChar - '0';
      if (isParsingSpeed) {
        parsedSpeed = parsedSpeed * 10 + digit;
      } else {
        parsedCommand = parsedCommand * 10 + digit;
      }
    }
  }
}

void executeCommand() {
  switch (parsedCommand) {
    case 1:  // Forward
      if (isManualMode) {
        moveForward(parsedSpeed);
        Serial.println("Moving forward");
      } else {
        Serial.println("ERROR: Robot is in AUTO mode. Switch to MANUAL (command: 10)");
      }
      break;

    case 2:  // Backward
      if (isManualMode) {
        moveBackward(parsedSpeed);
        Serial.println("Moving backward");
      } else {
        Serial.println("ERROR: Robot is in AUTO mode. Switch to MANUAL (command: 10)");
      }
      break;

    case 3:  // Left
      if (isManualMode) {
        turnLeft(parsedSpeed);
        Serial.println("Turning left");
      } else {
        Serial.println("ERROR: Robot is in AUTO mode. Switch to MANUAL (command: 10)");
      }
      break;

    case 4:  // Right
      if (isManualMode) {
        turnRight(parsedSpeed);
        Serial.println("Turning right");
      } else {
        Serial.println("ERROR: Robot is in AUTO mode. Switch to MANUAL (command: 10)");
      }
      break;

    case 0:  // Stop
      stopMotors();
      isManualMode = true;
      Serial.println("STOPPED - Switched to MANUAL mode");
      break;

    case 10:  // Switch to MANUAL mode
      isManualMode = true;
      stopMotors();
      Serial.println("MANUAL mode activated");
      break;

    case 11:  // Switch to AUTO mode
      isManualMode = false;
      Serial.print("AUTO mode activated (line following). Base speed: ");
      Serial.println(autoModeBaseSpeed);
      break;

    case 40:  // Help
      printHelp();
      break;

    default:
      // Check if this is a speed setting command
      if (parsedCommand > 0 && isParsingSpeed) {
        autoModeBaseSpeed = constrain(parsedSpeed, 0, 255);
        Serial.print("Auto mode base speed set to: ");
        Serial.println(autoModeBaseSpeed);
      } else {
        Serial.println("Unknown command. Type 40 for help.");
      }
      break;
  }
}

// =====================================
// HELP DISPLAY
// =====================================
void printHelp() {
  Serial.println("=====================================");
  Serial.println("ROBOT COMMANDS:");
  Serial.println("-------------------------------------");
  Serial.println("Movement (MANUAL mode only):");
  Serial.println("  1_<speed> - Move forward");
  Serial.println("  2_<speed> - Move backward");
  Serial.println("  3_<speed> - Turn left");
  Serial.println("  4_<speed> - Turn right");
  Serial.println("  0         - STOP");
  Serial.println("-------------------------------------");
  Serial.println("Mode Control:");
  Serial.println("  10 - Switch to MANUAL mode");
  Serial.println("  11 - Switch to AUTO mode (line following)");
  Serial.println("-------------------------------------");
  Serial.println("Settings:");
  Serial.println("  <value>_<speed> - Set auto mode base speed");
  Serial.println("  Example: 50_170 sets base speed to 170");
  Serial.println("-------------------------------------");
  Serial.println("Speed range: 0-255");
  Serial.println("=====================================");
}

void loop() {

  if (usePhysicalSwitch) {
    checkModeSwitch();
  }

  parseSerialInput();

  if (!isManualMode) {
    followLine();
  }

  if (hasCommandToProcess) {
    hasCommandToProcess = false;
    executeCommand();

    parsedCommand = 0;
    parsedSpeed = 0;
    isParsingSpeed = false;
  }
}