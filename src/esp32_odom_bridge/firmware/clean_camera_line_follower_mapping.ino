/*
  clean_camera_line_follower_mapping.ino - ESP32 firmware for robot base.
  Base: clean_camera_line_follower.ino
  Change: added "R" serial command to zero encoder ticks on demand,
  so the ROS 2 side can reset odometry exactly at the start of a
  mapping/localization run.
*/

#include <Arduino.h>

// Motor pins (IBT-2 x2)
#define LEFT_RPWM   27
#define LEFT_LPWM   14
#define LEFT_REN    4
#define LEFT_LEN    5
#define RIGHT_RPWM  17   // Reassigned from 23 to 32
#define RIGHT_LPWM  16   // Reassigned from 22 to 33
#define RIGHT_REN   25
#define RIGHT_LEN   26

// Encoder pins
#define LEFT_ENC_A  34
#define LEFT_ENC_B  35
#define RIGHT_ENC_A 18
#define RIGHT_ENC_B 19

// LEDC channels
#define CH_LR 0
#define CH_LL 1
#define CH_RR 2
#define CH_RL 3

// Motor Direction Configuration (mirrored mounting on diff-drive chassis)
#define INVERT_LEFT_MOTOR   true   // true if left wheel spins backward for forward command
#define INVERT_RIGHT_MOTOR  false  // true if right wheel spins backward for forward command

const float WHEELBASE = 0.56f;
const float MAX_MPS   = 0.60f;
const int   MIN_PWM   = 135;  // Boosted from 95: IG52 gearboxes require ~130-150 PWM to overcome static friction!

volatile long leftTicks = 0, rightTicks = 0;

void IRAM_ATTR leftISR()  { leftTicks  += (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) ? 1 : -1; }
void IRAM_ATTR rightISR() { rightTicks += (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) ? 1 : -1; }

String buf = "";

void driveMotor(int chFwd, int chRev, float mps, bool invert) {
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

void stopAllMotors() {
  ledcWrite(CH_LR, 0); ledcWrite(CH_LL, 0);
  ledcWrite(CH_RR, 0); ledcWrite(CH_RL, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_REN, OUTPUT);  digitalWrite(LEFT_REN, HIGH);
  pinMode(LEFT_LEN, OUTPUT);  digitalWrite(LEFT_LEN, HIGH);
  pinMode(RIGHT_REN, OUTPUT); digitalWrite(RIGHT_REN, HIGH);
  pinMode(RIGHT_LEN, OUTPUT); digitalWrite(RIGHT_LEN, HIGH);

  ledcSetup(CH_LR, 20000, 8); ledcAttachPin(LEFT_RPWM, CH_LR);
  ledcSetup(CH_LL, 20000, 8); ledcAttachPin(LEFT_LPWM, CH_LL);
  ledcSetup(CH_RR, 20000, 8); ledcAttachPin(RIGHT_RPWM, CH_RR);
  ledcSetup(CH_RL, 20000, 8); ledcAttachPin(RIGHT_LPWM, CH_RL);

  stopAllMotors();

  pinMode(LEFT_ENC_A, INPUT);  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT); pinMode(RIGHT_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightISR, CHANGE);

  Serial.println("READY - clean_camera_line_follower_mapping v2");
  Serial.println("Commands: 'R'=reset, 'T'=test both wheels, 'F'=forward, 'B'=reverse, 'S'=stop, 'lin,ang'=drive");
}

unsigned long lastOdo = 0;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf.trim();
      if (buf.length() > 0) {
        if (buf == "R") {
          noInterrupts();
          leftTicks = 0;
          rightTicks = 0;
          interrupts();
          Serial.println("RESET_OK");
        } else if (buf == "S") {
          stopAllMotors();
          Serial.println("STOP_OK");
        } else if (buf == "T") {
          // Self-test: spins left wheel, then right wheel
          Serial.println("TEST: Running LEFT wheel for 1.2s...");
          driveMotor(CH_LR, CH_LL, 0.25f, INVERT_LEFT_MOTOR);
          delay(1200);
          stopAllMotors();
          delay(500);
          Serial.println("TEST: Running RIGHT wheel for 1.2s...");
          driveMotor(CH_RR, CH_RL, 0.25f, INVERT_RIGHT_MOTOR);
          delay(1200);
          stopAllMotors();
          Serial.println("TEST: Complete!");
        } else if (buf == "F") {
          Serial.println("DRIVE: Forward 1.2s...");
          driveMotor(CH_LR, CH_LL, 0.25f, INVERT_LEFT_MOTOR);
          driveMotor(CH_RR, CH_RL, 0.25f, INVERT_RIGHT_MOTOR);
          delay(1200);
          stopAllMotors();
        } else if (buf == "B") {
          Serial.println("DRIVE: Backward 1.2s...");
          driveMotor(CH_LR, CH_LL, -0.25f, INVERT_LEFT_MOTOR);
          driveMotor(CH_RR, CH_RL, -0.25f, INVERT_RIGHT_MOTOR);
          delay(1200);
          stopAllMotors();
        } else {
          int ci = buf.indexOf(',');
          if (ci > 0) {
            float lin = buf.substring(0, ci).toFloat();
            float ang = buf.substring(ci + 1).toFloat();

            float ls = lin - (ang * WHEELBASE / 2.0f);
            float rs = lin + (ang * WHEELBASE / 2.0f);

            driveMotor(CH_LR, CH_LL, ls, INVERT_LEFT_MOTOR);
            driveMotor(CH_RR, CH_RL, rs, INVERT_RIGHT_MOTOR);
          }
        }
      }
      buf = "";
    } else {
      buf += c;
    }
  }

  unsigned long now = millis();
  if (now - lastOdo >= 20) {
    lastOdo = now;
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
