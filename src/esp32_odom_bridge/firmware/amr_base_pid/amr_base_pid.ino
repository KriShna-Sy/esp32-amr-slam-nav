/*
  amr_base_pid.ino  -  ESP32 diff-drive base firmware (drop-in replacement for
                        clean_camera_line_follower_mapping.ino)

  Serial protocol unchanged. ODO ticks are NORMALISED: forward = DECREASING ticks on both
  wheels (what odom_bridge_node expects), whatever way each encoder is mounted.
  New motors (Nov 2026): ~47000 ticks per wheel revolution (hand-measured, refine later).
    ESP32 -> PC : "ODO,<left_ticks>,<right_ticks>,<millis>\n"   @ 50 Hz
    PC -> ESP32 : "<lin_mps>,<ang_radps>\n"   drive
                  "R"  reset ticks (+ clear fault)
                  "S"  stop
                  "F" / "B"  forward / backward 0.15 m/s for 1.2 s (non-blocking)
                  "C"  clear fault
                  "P"  toggle PID <-> open-loop (old behaviour)
                  "V"  toggle debug lines "DBG,tl,ml,pwl,tr,mr,pwr" (bridge ignores them)

  NEW:
   1. Serial watchdog: no drive command for 400 ms -> motors stop. (Old firmware had NONE:
      if the PC/bridge crashed or USB dropped, the robot kept driving forever.)
   2. Closed-loop wheel speed control (feedforward + PI on encoder speed).
      Old firmware: any |v| > 0.005 m/s jumped straight to PWM 135, so slow wheels in a
      curve ran far too fast and Nav2 small corrections were badly distorted.
   3. Encoder fault guards: wheel spinning the WRONG way, or full PWM with no encoder
      motion -> motors stop, "FAULT,<reason>" printed every 1 s until "C" or "R".
   4. Fixed-size serial buffer (old String buffer could grow without limit).
*/

#include <Arduino.h>

// ---------------- Pins ----------------
// Verified with dir_test 2026-11-02 (standing behind robot, camera = front).
#define LEFT_RPWM   27
#define LEFT_LPWM   14
#define LEFT_REN    4
#define LEFT_LEN    5
#define RIGHT_RPWM  32   // (GPIO12 avoided: boot strapping pin)
#define RIGHT_LPWM  13
#define RIGHT_REN   25
#define RIGHT_LEN   26

#define LEFT_ENC_A  35
#define LEFT_ENC_B  34
#define RIGHT_ENC_A 18
#define RIGHT_ENC_B 19

#define CH_LR 0
#define CH_LL 1
#define CH_RR 2
#define CH_RL 3

// ---------------- Direction config ----------------
#define INVERT_LEFT_MOTOR   false   // v3: both motors reversed vs v1 (v1 drove backward)
#define INVERT_RIGHT_MOTOR  true
// Forward travel produces NEGATIVE ticks on both wheels (bridge inverts them).
// If "FAULT,ENC_DIR_L/R" appears when driving forward, flip that wheel's sign here.
const int ENC_FWD_SIGN_L = 1;    // v3: flipped together with the motor direction
const int ENC_FWD_SIGN_R = 1;

// ---------------- Geometry ----------------
const float WHEELBASE     = 0.56f;
const float WHEEL_DIA     = 0.127f;   // 5 inch wheels
const float TICKS_PER_REV = 47000.0f;   // new motors, hand-measured 46.5k-48k
const float TICKS_PER_M   = TICKS_PER_REV / (PI * WHEEL_DIA);   // ~117800 with 47000 ticks/rev
const float MAX_MPS       = 0.60f;

// ---------------- Control tuning ----------------
const uint32_t LOOP_MS        = 20;      // 50 Hz control + telemetry
const uint32_t CMD_TIMEOUT_MS = 400;     // watchdog
const float    MIN_TARGET     = 0.010f;  // |v| below this = stop wheel

const int   PWM_STATIC  = 135;   // breakaway PWM (wheel not yet moving)
const int   PWM_KINETIC = 110;   // PWM to keep a slowly-moving wheel turning
const float K_FF        = 200.0f;  // PWM per m/s on top of PWM_KINETIC (old map: 120/0.6)
const float KP          = 350.0f;  // PWM per (m/s) error
const float KI          = 900.0f;  // PWM per (m/s * s) error
const float I_LIMIT     = 70.0f;   // max integrator contribution (PWM)
const float ACCEL_LIM   = 0.6f;    // m/s^2 ramp of wheel targets (speed-up)
const float DECEL_LIM   = 1.5f;    // m/s^2 ramp when slowing (stop command = instant)
const float VEL_LPF     = 0.5f;    // measured-speed low-pass (0..1, higher = less filtering)

// Fault guards
const uint32_t FAULT_DIR_MS   = 400;   // wrong direction this long -> fault
const uint32_t FAULT_STALL_MS = 1500;  // PWM >= 230 with no motion this long -> fault

// ---------------- State ----------------
volatile long leftTicks = 0, rightTicks = 0;
void IRAM_ATTR leftISR()  { leftTicks  += (digitalRead(LEFT_ENC_A)  == digitalRead(LEFT_ENC_B))  ? 1 : -1; }
void IRAM_ATTR rightISR() { rightTicks += (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) ? 1 : -1; }

struct Wheel {
  int chFwd, chRev;
  bool invert;
  int encSign;
  float cmd = 0;      // requested wheel speed (m/s)
  float target = 0;   // ramped target
  float meas = 0;     // filtered measured speed
  float integ = 0;
  int pwmOut = 0;     // signed
  long lastTicks = 0;
  uint32_t dirBadSince = 0, stallSince = 0;
};

Wheel W[2];            // 0 = left, 1 = right (functions take an index, not Wheel&,
Wheel& L = W[0];       //  because Arduino IDE auto-prototypes break struct params)
Wheel& R = W[1];
bool pidEnabled = true;
bool debugOut = false;
uint32_t lastCmdMs = 0;
uint32_t cmdTimeoutMs = CMD_TIMEOUT_MS;
uint32_t lastLoop = 0, lastFaultPrint = 0;
const char* fault = nullptr;

char rxBuf[48];
uint8_t rxLen = 0;

// ---------------- PWM helpers (ESP32 Arduino core 2.x and 3.x) ----------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  static const int CH_PIN[4] = {LEFT_RPWM, LEFT_LPWM, RIGHT_RPWM, RIGHT_LPWM};
  void pwmSetup(int ch, int pin) { ledcAttach(pin, 20000, 8); }
  void pwmWrite(int ch, int duty) { ledcWrite(CH_PIN[ch], duty); }
#else
  void pwmSetup(int ch, int pin) { ledcSetup(ch, 20000, 8); ledcAttachPin(pin, ch); }
  void pwmWrite(int ch, int duty) { ledcWrite(ch, duty); }
#endif

void writeMotor(int wi, int signedPwm) {
  Wheel& w = W[wi];
  w.pwmOut = signedPwm;
  int p = w.invert ? -signedPwm : signedPwm;
  int mag = constrain(abs(p), 0, 255);
  if (p > 0)      { pwmWrite(w.chFwd, mag); pwmWrite(w.chRev, 0); }
  else if (p < 0) { pwmWrite(w.chFwd, 0);   pwmWrite(w.chRev, mag); }
  else            { pwmWrite(w.chFwd, 0);   pwmWrite(w.chRev, 0); }
}

void resetWheel(int wi) {
  Wheel& w = W[wi];
  w.cmd = w.target = 0; w.integ = 0; w.dirBadSince = w.stallSince = 0;
  writeMotor(wi, 0);
}

void stopAll() { resetWheel(0); resetWheel(1); }

void setFault(const char* why) {
  if (!fault) { fault = why; stopAll(); Serial.print("FAULT,"); Serial.println(why); }
}

void setDrive(float lin, float ang, uint32_t timeoutMs) {
  if (fault) return;
  lin = constrain(lin, -MAX_MPS, MAX_MPS);
  L.cmd = constrain(lin - ang * WHEELBASE / 2.0f, -MAX_MPS, MAX_MPS);
  R.cmd = constrain(lin + ang * WHEELBASE / 2.0f, -MAX_MPS, MAX_MPS);
  lastCmdMs = millis();
  cmdTimeoutMs = timeoutMs;
}

// ---------------- Per-wheel control ----------------
void controlWheel(int wi, float dt, uint32_t now) {
  Wheel& w = W[wi];
  // 1. Ramp target toward command
  if (fabs(w.cmd) < MIN_TARGET) {
    w.target = 0;                                   // stop = instant
  } else {
    float lim = ((fabs(w.cmd) > fabs(w.target)) && (w.cmd * w.target >= 0) ? ACCEL_LIM : DECEL_LIM) * dt;
    w.target += constrain(w.cmd - w.target, -lim, lim);
  }

  if (fabs(w.target) < MIN_TARGET) {
    w.integ = 0; w.dirBadSince = w.stallSince = 0;
    writeMotor(wi, 0);
    return;
  }

  float dir = (w.target > 0) ? 1.0f : -1.0f;

  if (!pidEnabled) {   // old open-loop map, for comparison / fallback
    float mag = constrain(fabs(w.target) / MAX_MPS, 0.0f, 1.0f);
    writeMotor(wi, (int)(dir * (PWM_STATIC + mag * (255 - PWM_STATIC))));
    return;
  }

  // 2. Feedforward + PI (computed in the "direction of travel" frame)
  float tgtMag = fabs(w.target);
  float measAlong = w.meas * dir;            // >0 when moving the right way
  float err = tgtMag - measAlong;

  float ff = PWM_KINETIC + K_FF * tgtMag;
  float u = ff + KP * err + w.integ;

  bool saturated = (u >= 255.0f && err > 0) || (u <= 0.0f && err < 0);
  if (!saturated) w.integ = constrain(w.integ + KI * err * dt, -I_LIMIT, I_LIMIT);

  // Breakaway: wheel essentially stopped -> make sure we beat static friction
  if (fabs(w.meas) < 0.01f && u < PWM_STATIC) u = PWM_STATIC;

  // Never drive against the commanded direction (no reverse-braking oscillation)
  int pwm = (int)constrain(u, 0.0f, 255.0f);
  writeMotor(wi, (int)(dir * pwm));

  // 3. Fault guards
  if (measAlong < -0.05f) {                       // spinning the wrong way
    if (!w.dirBadSince) w.dirBadSince = now;
    else if (now - w.dirBadSince > FAULT_DIR_MS) setFault(wi == 0 ? "ENC_DIR_L" : "ENC_DIR_R");
  } else w.dirBadSince = 0;

  if (pwm >= 230 && fabs(w.meas) < 0.005f) {      // full power, encoder silent
    if (!w.stallSince) w.stallSince = now;
    else if (now - w.stallSince > FAULT_STALL_MS) setFault(wi == 0 ? "STALL_OR_NO_ENC_L" : "STALL_OR_NO_ENC_R");
  } else w.stallSince = 0;
}

// ---------------- Serial ----------------
void handleLine(char* s) {
  if (!strcmp(s, "R")) {
    noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts();
    L.lastTicks = R.lastTicks = 0; L.meas = R.meas = 0;
    fault = nullptr; stopAll();
    Serial.println("RESET_OK");
  } else if (!strcmp(s, "S")) { stopAll(); Serial.println("STOP_OK"); }
  else if (!strcmp(s, "C"))   { fault = nullptr; stopAll(); Serial.println("FAULT_CLEARED"); }
  else if (!strcmp(s, "P"))   { pidEnabled = !pidEnabled; stopAll(); Serial.println(pidEnabled ? "PID_ON" : "PID_OFF_OPENLOOP"); }
  else if (!strcmp(s, "V"))   { debugOut = !debugOut; Serial.println(debugOut ? "DEBUG_ON" : "DEBUG_OFF"); }
  else if (!strcmp(s, "F"))   { setDrive( 0.15f, 0, 1200); Serial.println("DRIVE: Forward 1.2s"); }
  else if (!strcmp(s, "B"))   { setDrive(-0.15f, 0, 1200); Serial.println("DRIVE: Backward 1.2s"); }
  else {
    char* comma = strchr(s, ',');
    if (comma && comma != s) {
      *comma = '\0';
      setDrive(atof(s), atof(comma + 1), CMD_TIMEOUT_MS);
    }
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen) { rxBuf[rxLen] = '\0'; handleLine(rxBuf); rxLen = 0; }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;   // overflow: drop garbage line
    }
  }
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_REN, OUTPUT);  digitalWrite(LEFT_REN, HIGH);
  pinMode(LEFT_LEN, OUTPUT);  digitalWrite(LEFT_LEN, HIGH);
  pinMode(RIGHT_REN, OUTPUT); digitalWrite(RIGHT_REN, HIGH);
  pinMode(RIGHT_LEN, OUTPUT); digitalWrite(RIGHT_LEN, HIGH);

  pwmSetup(CH_LR, LEFT_RPWM);  pwmSetup(CH_LL, LEFT_LPWM);
  pwmSetup(CH_RR, RIGHT_RPWM); pwmSetup(CH_RL, RIGHT_LPWM);

  L.chFwd = CH_LR; L.chRev = CH_LL; L.invert = INVERT_LEFT_MOTOR;  L.encSign = ENC_FWD_SIGN_L;
  R.chFwd = CH_RR; R.chRev = CH_RL; R.invert = INVERT_RIGHT_MOTOR; R.encSign = ENC_FWD_SIGN_R;
  stopAll();

  pinMode(LEFT_ENC_A, INPUT);  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT); pinMode(RIGHT_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightISR, CHANGE);

  Serial.println("READY - amr_base_pid v3 (dir fixed 2026-11-02)");
  lastLoop = millis();
}

void loop() {
  readSerial();

  uint32_t now = millis();
  if (now - lastLoop < LOOP_MS) return;
  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  long lt, rt;
  noInterrupts(); lt = leftTicks; rt = rightTicks; interrupts();

  // Measured wheel speeds (m/s, + = forward)
  float vl = L.encSign * (lt - L.lastTicks) / TICKS_PER_M / dt;
  float vr = R.encSign * (rt - R.lastTicks) / TICKS_PER_M / dt;
  L.lastTicks = lt; R.lastTicks = rt;
  L.meas += VEL_LPF * (vl - L.meas);
  R.meas += VEL_LPF * (vr - R.meas);

  // Watchdog
  if ((L.cmd != 0 || R.cmd != 0) && (now - lastCmdMs > cmdTimeoutMs)) {
    L.cmd = R.cmd = 0;
  }

  if (fault) {
    stopAll();
    if (now - lastFaultPrint > 1000) { lastFaultPrint = now; Serial.print("FAULT,"); Serial.println(fault); }
  } else {
    controlWheel(0, dt, now);
    controlWheel(1, dt, now);
  }

  // Telemetry (unchanged format)
  // normalise: forward travel -> decreasing ticks on BOTH wheels (bridge convention)
  Serial.print("ODO,"); Serial.print(-ENC_FWD_SIGN_L * lt); Serial.print(",");
  Serial.print(-ENC_FWD_SIGN_R * rt);
  Serial.print(",");    Serial.println(now);

  if (debugOut && (now / LOOP_MS) % 5 == 0) {
    Serial.printf("DBG,%.3f,%.3f,%d,%.3f,%.3f,%d\n",
                  L.target, L.meas, L.pwmOut, R.target, R.meas, R.pwmOut);
  }
}
