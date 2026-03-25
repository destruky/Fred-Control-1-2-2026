#include "PIN_MAP.h"
#include "HOTEND.h"

// ── Temporización ──────────────────────────────────────
// NOTA: prevTime está declarado en HOTEND.h — NO redeclarar aquí
unsigned long prevTime_sample = 0;
unsigned long lastChange      = 0;
unsigned long startTime       = 0;

const unsigned long SAMPLE_DT  = 100;   // ms
const unsigned long DWELL_TIME = 5000;  // ms entre cambios (hotend lento, τ ≈ 10-30s)

// ── PRBS ───────────────────────────────────────────────
const int pwmLevels[] = {0, 60, 100, 140, 180, 220};
const int N_LEVELS    = 6;
int currentPwm        = 60;

// ══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  pinMode(pinHotend, OUTPUT);
  analogWrite(pinHotend, 0);

  Serial.println("t_ms,pwm_heater,temp_C");

  startTime       = millis();
  prevTime_sample = millis();
  lastChange      = millis();

  analogWrite(pinHotend, currentPwm);
}

// ══════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── Leer temperatura ────────────────────────────────
  float temp = thermistor(analogRead(termPin));

  // ── Seguridad (CRÍTICO — va antes de todo) ───────────
  if (temp > 255.0 || temp == -999) {
    analogWrite(pinHotend, 0);
    Serial.println("SHUTDOWN_SEGURIDAD");
    while (1);
  }

  // ── Cambio de nivel PRBS ────────────────────────────
  if (now - lastChange >= DWELL_TIME) {
    lastChange = now;
    int idx;
    do {
      idx = random(N_LEVELS);
    } while (pwmLevels[idx] == currentPwm);

    currentPwm = pwmLevels[idx];
    analogWrite(pinHotend, currentPwm);
  }

  // ── Muestreo cada 100 ms ────────────────────────────
  if (now - prevTime_sample >= SAMPLE_DT) {
    unsigned long t = now - startTime;

    Serial.print(t);
    Serial.print(',');
    Serial.print(currentPwm);
    Serial.print(',');
    Serial.println(temp, 2);

    prevTime_sample = now;
  }
}
