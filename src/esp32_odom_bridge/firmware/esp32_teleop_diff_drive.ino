/*
  esp32_teleop_diff_drive.ino (v3.2)
  =============================================================================
  Universal Dual-Mode ESP32 Firmware for Differential-Drive Robot (IBT-2 + Encoders)
  
  FEATURES:
  1. Full ROS 2 compatibility (odom_bridge_node forwards /cmd_vel as "lin,ang\n")
  2. Direct Keyboard Teleop (i=fwd, ,=rev, j=left, l=right, k=stop, t=test)
  3. Interactive Pin & Direction Toggling on the fly (no re-upload needed!):
     - 'x' : Toggle Left Motor Invert
     - 'y' : Toggle Right Motor Invert
     - 's' : Toggle Left/Right Motor Swap
     - '1' : Test Left Motor Forward
     - '2' : Test Left Motor Reverse
     - '3' : Test Right Motor Forward
     - '4' : Test Right Motor Reverse
     - 't' : Full 2-wheel automated test
  =============================================================================
*/

#include <Arduino.h>

// =============================================================================
// 1. PIN CONFIGURATION
// =============================================================================
// Preset 1: Standard clean_camera / mobBOT wiring
// If your Left reverse was wired to pin 26 instead of 14, change LEFT_LPWM to 26!
#define LEFT_RPWM   27
#define LEFT_LPWM   14   // Change to 26 if your Left reverse is pin 26
#define LEFT_REN    4
#define LEFT_LEN    5

#define RIGHT_RPWM  17   // Verified working!
#define RIGHT_LPWM  16   // Verified working!
#define RIGHT_REN   25
#define RIGHT_LEN   26

#define LEFT_ENC_A  34
#define LEFT_ENC_B  35
#define RIGHT_ENC_A 18
#define RIGHT_ENC_B 19

// LEDC PWM Channels (ESP32)
#define CH_L_RPWM 0
#define CH_L_LPWM 1
#define CH_R_RPWM 2
#define CH_R_LPWM 3

// =============================================================================
// 2. RUNTIME DIRECTION CONFIGURATION (Adjustable live via serial!)
// =============================================================================
bool invertLeft   = false;  // Start false so Pin 27 fires for forward
bool invertRight  = false;
bool swapLR       = false;

const float WHEELBASE_M   = 0.56f;
const float MAX_MPS       = 0.60f;
const int   MIN_PWM       = 130;    // Minimum PWM to overcome IG52 gearbox static friction
const float TELEOP_SPEED  = 0.25f;  // Teleop test speed (m/s)

// =============================================================================
// 3. ENCODER STATE
// =============================================================================
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

void IRAM_ATTR leftISR() {
  leftTicks += (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) ? 1 : -1;
}

void IRAM_ATTR rightISR() {
  rightTicks += (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) ? 1 : -1;
}

// =============================================================================
// 4. MOTOR DRIVE IMPLEMENTATION
// =============================================================================
void driveSingleMotor(int chFwd, int chRev, float mps, bool invert) {
  if (invert) mps = -mps;

  if (fabs(mps) < 0.005f) {
    ledcWrite(chFwd, 0);
    ledcWrite(chRev, 0);
    return;
  }

  float mag = constrain(fabs(mps) / MAX_MPS, 0.0f, 1.0f);
  int pwm = MIN_PWM + (int)(mag * (255 - MIN_PWM));
  pwm = constrain(pwm, 0, 255);

  if (mps >= 0.0f) {
    ledcWrite(chFwd, pwm);
    ledcWrite(chRev, 0);
  } else {
    ledcWrite(chFwd, 0);
    ledcWrite(chRev, pwm);
  }
}

void driveBase(float leftMps, float rightMps) {
  if (swapLR) {
    driveSingleMotor(CH_R_RPWM, CH_R_LPWM, leftMps,  invertRight);
    driveSingleMotor(CH_L_RPWM, CH_L_LPWM, rightMps, invertLeft);
  } else {
    driveSingleMotor(CH_L_RPWM, CH_L_LPWM, leftMps,  invertLeft);
    driveSingleMotor(CH_R_RPWM, CH_R_LPWM, rightMps, invertRight);
  }
}

void stopMotors() {
  ledcWrite(CH_L_RPWM, 0); ledcWrite(CH_L_LPWM, 0);
  ledcWrite(CH_R_RPWM, 0); ledcWrite(CH_R_LPWM, 0);
}

void printStatus() {
  Serial.println("\n--- CURRENT CONFIGURATION ---");
  Serial.printf("  invertLeft:  %s  (Press 'x' to flip)\n", invertLeft ? "TRUE (Reverse)" : "FALSE (Normal)");
  Serial.printf("  invertRight: %s  (Press 'y' to flip)\n", invertRight ? "TRUE (Reverse)" : "FALSE (Normal)");
  Serial.printf("  swapLR:      %s  (Press 's' to flip)\n", swapLR ? "TRUE (Swapped)" : "FALSE (Normal)");
  Serial.printf("  Pins: Left RPWM=%d, LPWM=%d | Right RPWM=%d, LPWM=%d\n", LEFT_RPWM, LEFT_LPWM, RIGHT_RPWM, RIGHT_LPWM);
  Serial.println("-----------------------------\n");
}

// =============================================================================
// 5. SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);

  // Enable pins
  pinMode(LEFT_REN, OUTPUT);  digitalWrite(LEFT_REN, HIGH);
  pinMode(LEFT_LEN, OUTPUT);  digitalWrite(LEFT_LEN, HIGH);
  pinMode(RIGHT_REN, OUTPUT); digitalWrite(RIGHT_REN, HIGH);
  pinMode(RIGHT_LEN, OUTPUT); digitalWrite(RIGHT_LEN, HIGH);

  // PWM setup
  ledcSetup(CH_L_RPWM, 20000, 8); ledcAttachPin(LEFT_RPWM, CH_L_RPWM);
  ledcSetup(CH_L_LPWM, 20000, 8); ledcAttachPin(LEFT_LPWM, CH_L_LPWM);
  ledcSetup(CH_R_RPWM, 20000, 8); ledcAttachPin(RIGHT_RPWM, CH_R_RPWM);
  ledcSetup(CH_R_LPWM, 20000, 8); ledcAttachPin(RIGHT_LPWM, CH_R_LPWM);

  stopMotors();

  // Encoders
  pinMode(LEFT_ENC_A, INPUT);  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT); pinMode(RIGHT_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightISR, CHANGE);

  Serial.println("\n=============================================================");
  Serial.println("  ESP32 TELEOP & DIFF-DRIVE BRIDGE READY (v3.2)");
  Serial.println("=============================================================");
  printStatus();
  Serial.println("  DIAGNOSTIC KEYS (Work immediately in Serial Monitor):");
  Serial.println("    [1] Test Left Forward     [2] Test Left Reverse");
  Serial.println("    [3] Test Right Forward    [4] Test Right Reverse");
  Serial.println("    [t] 2-Wheel Test          [x] Flip Left Dir");
  Serial.println("    [y] Flip Right Dir        [s] Swap Left/Right");
  Serial.println("  TELEOP KEYS:");
  Serial.println("    [i] Forward     [,] Backward    [j] Left    [l] Right   [k] Stop");
  Serial.println("=============================================================\n");
}

// =============================================================================
// 6. MAIN LOOP & COMMAND DISPATCHER
// =============================================================================
String inputBuf = "";
unsigned long lastCmdTime = 0;
unsigned long lastOdoTime = 0;
bool activeMotion = false;

void handleSingleKey(char key) {
  lastCmdTime = millis();
  activeMotion = true;

  switch (key) {
    case '1':
      Serial.println("[TEST 1] LEFT Motor FORWARD (Pin 27) for 1.2s...");
      ledcWrite(CH_L_RPWM, 180); ledcWrite(CH_L_LPWM, 0);
      delay(1200); stopMotors();
      Serial.println("[TEST 1] Done.");
      activeMotion = false;
      break;

    case '2':
      Serial.println("[TEST 2] LEFT Motor REVERSE (Pin 14) for 1.2s...");
      ledcWrite(CH_L_RPWM, 0); ledcWrite(CH_L_LPWM, 180);
      delay(1200); stopMotors();
      Serial.println("[TEST 2] Done.");
      activeMotion = false;
      break;

    case '3':
      Serial.println("[TEST 3] RIGHT Motor FORWARD (Pin 32) for 1.2s...");
      ledcWrite(CH_R_RPWM, 180); ledcWrite(CH_R_LPWM, 0);
      delay(1200); stopMotors();
      Serial.println("[TEST 3] Done.");
      activeMotion = false;
      break;

    case '4':
      Serial.println("[TEST 4] RIGHT Motor REVERSE (Pin 33) for 1.2s...");
      ledcWrite(CH_R_RPWM, 0); ledcWrite(CH_R_LPWM, 180);
      delay(1200); stopMotors();
      Serial.println("[TEST 4] Done.");
      activeMotion = false;
      break;

    case 'x': case 'X':
      invertLeft = !invertLeft;
      Serial.printf(">>> [CONFIG] Left Motor Inverted: %s\n", invertLeft ? "YES" : "NO");
      activeMotion = false;
      break;

    case 'y': case 'Y':
      invertRight = !invertRight;
      Serial.printf(">>> [CONFIG] Right Motor Inverted: %s\n", invertRight ? "YES" : "NO");
      activeMotion = false;
      break;

    case 's': case 'S':
      swapLR = !swapLR;
      Serial.printf(">>> [CONFIG] Left/Right Swapped: %s\n", swapLR ? "YES" : "NO");
      activeMotion = false;
      break;

    case 'i': case 'I': // Forward
      driveBase(TELEOP_SPEED, TELEOP_SPEED);
      break;

    case ',': case '<': // Backward
      driveBase(-TELEOP_SPEED, -TELEOP_SPEED);
      break;

    case 'j': case 'J': // Pivot Left
      driveBase(-TELEOP_SPEED, TELEOP_SPEED);
      break;

    case 'l': case 'L': // Pivot Right
      driveBase(TELEOP_SPEED, -TELEOP_SPEED);
      break;

    case 'u': case 'U':
      driveBase(TELEOP_SPEED * 0.5f, TELEOP_SPEED);
      break;

    case 'o': case 'O':
      driveBase(TELEOP_SPEED, TELEOP_SPEED * 0.5f);
      break;

    case 'k': case 'K': case ' ': // Stop
      stopMotors();
      activeMotion = false;
      break;

    case 'r': case 'R': // Reset Odom
      noInterrupts();
      leftTicks = 0;
      rightTicks = 0;
      interrupts();
      Serial.println("RESET_OK");
      activeMotion = false;
      break;

    case 't': case 'T':
      Serial.println("\n>>> [SELF TEST] Running Left Motor (1.2s)...");
      driveBase(TELEOP_SPEED, 0.0f);
      delay(1200);
      stopMotors();
      delay(500);
      Serial.println(">>> [SELF TEST] Running Right Motor (1.2s)...");
      driveBase(0.0f, TELEOP_SPEED);
      delay(1200);
      stopMotors();
      Serial.println(">>> [SELF TEST] Complete!\n");
      activeMotion = false;
      break;

    case '?': case 'h': case 'H':
      printStatus();
      activeMotion = false;
      break;

    default:
      activeMotion = false;
      break;
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    // Instant single-key triggers
    if (inputBuf.length() == 0 && (c == '1' || c == '2' || c == '3' || c == '4' ||
                                  c == 'x' || c == 'X' || c == 'y' || c == 'Y' ||
                                  c == 's' || c == 'S' || c == 't' || c == 'T' ||
                                  c == 'i' || c == 'I' || c == 'j' || c == 'J' ||
                                  c == 'k' || c == 'K' || c == 'l' || c == 'L' ||
                                  c == ',' || c == '<' || c == ' ' || c == '?' ||
                                  c == 'u' || c == 'U' || c == 'o' || c == 'O')) {
      handleSingleKey(c);
      continue;
    }

    if (c == '\n' || c == '\r') {
      inputBuf.trim();
      if (inputBuf.length() > 0) {
        if (inputBuf.length() == 1) {
          handleSingleKey(inputBuf.charAt(0));
        } else if (inputBuf == "R") {
          noInterrupts();
          leftTicks = 0;
          rightTicks = 0;
          interrupts();
          Serial.println("RESET_OK");
        } else {
          // ROS 2 "linear,angular"
          int commaIdx = inputBuf.indexOf(',');
          if (commaIdx > 0) {
            float lin = inputBuf.substring(0, commaIdx).toFloat();
            float ang = inputBuf.substring(commaIdx + 1).toFloat();

            float ls = lin - (ang * WHEELBASE_M / 2.0f);
            float rs = lin + (ang * WHEELBASE_M / 2.0f);

            driveBase(ls, rs);
            lastCmdTime = millis();
            activeMotion = (fabs(lin) > 0.005f || fabs(ang) > 0.005f);
          }
        }
        inputBuf = "";
      }
    } else {
      inputBuf += c;
    }
  }

  // Safety Watchdog: Stop motors if no command received for 650ms during active motion
  if (activeMotion && (millis() - lastCmdTime > 650)) {
    stopMotors();
    activeMotion = false;
  }

  // 50Hz ODOM Telemetry Output
  unsigned long now = millis();
  if (now - lastOdoTime >= 20) {
    lastOdoTime = now;
    long lt, rt;
    noInterrupts();
    lt = leftTicks;
    rt = rightTicks;
    interrupts();

    Serial.print("ODO,");
    Serial.print(lt);
    Serial.print(",");
    Serial.print(rt);
    Serial.print(",");
    Serial.println(now);
  }
}
