#include "PIN_MAP.h"
#include "HOTEND.h"

// ── Step Response: PWM constante por 15 min ───────────
const int STEP_PWM = 180;

// ── Temporización ─────────────────────────────────────
unsigned long prevTime_sample = 0;
unsigned long startTime       = 0;

const unsigned long SAMPLE_DT   = 100;      // 100ms
const unsigned long DURACION_MS = 900000UL; // 15 min

bool finished = false;

// ══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  pinMode(pinHotend, OUTPUT);
  analogWrite(pinHotend, 0);
  delay(2000);

  Serial.println("t_ms,pwm_heater,temp_C");

  analogWrite(pinHotend, STEP_PWM);
  startTime       = millis();
  prevTime_sample = millis();
}

// ══════════════════════════════════════════════════════
void loop() {
  if (finished) return;

  unsigned long now = millis();
  float temp = thermistor(analogRead(termPin));

  // ── Seguridad ───────────────────────────────────────
  if (temp > 250.0 || temp == -999) {
    analogWrite(pinHotend, 0);
    Serial.println("SHUTDOWN_SEGURIDAD");
    while (1);
  }

  // ── Muestreo cada 100 ms ───────────────────────────
  if (now - prevTime_sample >= SAMPLE_DT) {
    prevTime_sample = now;

    Serial.print(now - startTime);
    Serial.print(',');
    Serial.print(STEP_PWM);
    Serial.print(',');
    Serial.println(temp, 2);
  }

  // ── Apaga al terminar 15 min ───────────────────────
  if (now - startTime >= DURACION_MS) {
    analogWrite(pinHotend, 0);
    Serial.println("# Experimento completo");
    finished = true;
  }
}