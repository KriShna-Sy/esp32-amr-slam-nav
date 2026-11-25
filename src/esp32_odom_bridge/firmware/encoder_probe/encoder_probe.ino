/*
  encoder_probe.ino - raw encoder diagnostic (NO motor drive: drivers disabled).

  Watches all 4 encoder wires separately and counts edges on EACH channel, with
  internal pull-ups enabled where the ESP32 has them (18/19; 34/35 have none).

  Upload, open Serial Monitor at 115200, turn each wheel slowly BY HAND:
    every 200 ms:  L A=<level> B=<level> eA=<edges> eB=<edges> | R A=.. B=.. eA=.. eB=..

  Reading the result for the LEFT wheel:
    eA and eB both increase  -> encoder OK (then the problem was elsewhere)
    only eB increases        -> channel A dead/broken wire (firmware counts ONLY on A!)
    only eA increases        -> channel B dead (direction wrong, counts still possible)
    neither, levels stuck 0  -> no power to encoder / GND missing / output needs pull-up
    neither, levels stuck 1  -> sensor dead or magnet disc loose

  Send 'P' to toggle pull-ups on 18/19 (test open-collector encoders), 'Z' to zero.
*/
#include <Arduino.h>

#define LEFT_ENC_A  34
#define LEFT_ENC_B  35
#define RIGHT_ENC_A 18
#define RIGHT_ENC_B 19

// motor driver enables (held LOW = drivers off, wheels free)
#define LEFT_REN  4
#define LEFT_LEN  5
#define RIGHT_REN 25
#define RIGHT_LEN 26

volatile unsigned long eLA = 0, eLB = 0, eRA = 0, eRB = 0;
void IRAM_ATTR isrLA() { eLA++; }
void IRAM_ATTR isrLB() { eLB++; }
void IRAM_ATTR isrRA() { eRA++; }
void IRAM_ATTR isrRB() { eRB++; }

bool pullups = true;

void configPins() {
  pinMode(LEFT_ENC_A, INPUT);            // 34/35: input-only, no internal pull-up exists
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, pullups ? INPUT_PULLUP : INPUT);
  pinMode(RIGHT_ENC_B, pullups ? INPUT_PULLUP : INPUT);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_REN, OUTPUT);  digitalWrite(LEFT_REN, LOW);
  pinMode(LEFT_LEN, OUTPUT);  digitalWrite(LEFT_LEN, LOW);
  pinMode(RIGHT_REN, OUTPUT); digitalWrite(RIGHT_REN, LOW);
  pinMode(RIGHT_LEN, OUTPUT); digitalWrite(RIGHT_LEN, LOW);

  configPins();
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  isrLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  isrLB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), isrRB, CHANGE);
  Serial.println("ENCODER PROBE ready - motors disabled. Turn wheels by hand. 'P'=pullups 18/19, 'Z'=zero");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'P' || c == 'p') {
      pullups = !pullups; configPins();
      Serial.println(pullups ? "PULLUPS 18/19 ON" : "PULLUPS 18/19 OFF");
    } else if (c == 'Z' || c == 'z') {
      noInterrupts(); eLA = eLB = eRA = eRB = 0; interrupts();
      Serial.println("ZEROED");
    }
  }
  static uint32_t last = 0;
  if (millis() - last >= 200) {
    last = millis();
    unsigned long la, lb, ra, rb;
    noInterrupts(); la = eLA; lb = eLB; ra = eRA; rb = eRB; interrupts();
    Serial.printf("L(34/35) A=%d B=%d eA=%lu eB=%lu | R(18/19) A=%d B=%d eA=%lu eB=%lu%s\n",
                  digitalRead(LEFT_ENC_A), digitalRead(LEFT_ENC_B), la, lb,
                  digitalRead(RIGHT_ENC_A), digitalRead(RIGHT_ENC_B), ra, rb,
                  pullups ? " [pu]" : "");
  }
}
