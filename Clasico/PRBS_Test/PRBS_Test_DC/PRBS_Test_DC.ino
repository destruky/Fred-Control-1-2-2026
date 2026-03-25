#include "PIN_MAP.h"
#include "MOTORDC.h"

// ── Temporización ──────────────────────────────────────
unsigned long prevTime   = 0;
unsigned long lastChange = 0;
unsigned long startTime  = 0;

const unsigned long SAMPLE_DT  = 100;   // ms
const unsigned long DWELL_TIME = 1500;  // ms entre cambios de PWM

// ── PRBS ───────────────────────────────────────────────
const int pwmLevels[] = {30, 60, 90, 120, 150};
const int N_LEVELS    = 5;
int currentPwm        = 60;

// ══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  pinMode(pinMotor, OUTPUT);
  pinMode(C1, INPUT_PULLUP);
  pinMode(C2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);

  Serial.println("t_ms,pwm,rpm");

  startTime  = millis();
  prevTime   = millis();
  lastChange = millis();
  lastTime   = millis();

  analogWrite(pinMotor, currentPwm);
}

// ══════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── Cambio de nivel PRBS ────────────────────────────
  if (now - lastChange >= DWELL_TIME) {
    lastChange = now;
    int idx;
    do {
      idx = random(N_LEVELS);
    } while (pwmLevels[idx] == currentPwm);

    currentPwm = pwmLevels[idx];
    analogWrite(pinMotor, currentPwm);
  }

  // ── Muestreo cada 100 ms ────────────────────────────
  if (now - prevTime >= SAMPLE_DT) {

    computeRpm();
    input = N;

    unsigned long t = now - startTime;

    Serial.print(t);
    Serial.print(',');
    Serial.print(currentPwm);
    Serial.print(',');
    Serial.println(N, 1);

    prevTime = now;
  }
}