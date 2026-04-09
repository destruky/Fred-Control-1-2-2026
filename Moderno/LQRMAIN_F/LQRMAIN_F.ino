#include "PIN_MAP.h"
#include "HOTEND.h"
#include "MOTORDC.h"

// Variables de Control Objetivo
double setpoint_H = 190.0; // °C
double setpoint_M = 35.0;  // RPM

// Variables de Tiempo Estricto para el LQR (Ts = 0.1s)
unsigned long prevMillisLQR = 0;
const unsigned long intervaloTs = 100; // 100ms (10Hz)

void setup() {
    Serial.begin(115200);

    // Configuración de pines
    pinMode(pinHotend, OUTPUT);
    pinMode(pinMotor, OUTPUT);
    pinMode(C1, INPUT_PULLUP);
    pinMode(C2, INPUT_PULLUP);

    // Interrupciones del Encoder (Pines 18 y 19)
    attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);

    // Inicializamos motor en reposo
    initLQRMotor();
    lastTime = millis();

    Serial.println("Control LQR Iniciado.");
}

void loop() {
    unsigned long currentMillis = millis();

    // Lazo de control discreto estricto a 10Hz
    if (currentMillis - prevMillisLQR >= intervaloTs) {
        // Compensación de deriva para mantener muestreo exacto
        prevMillisLQR += intervaloTs;

        // ==============================
        // 1. LAZO TÉRMICO (HOTEND)
        // ==============================
        int raw_temp = analogRead(termPin);
        float temp_real = thermistor(raw_temp);

        int pwm_hotend = (int)LQRHotend(temp_real, setpoint_H);
        analogWrite(pinHotend, pwm_hotend);

        // ==============================
        // 2. LAZO DE MOVIMIENTO (MOTOR DC)
        // ==============================
        computeRpm(); // Calcula N_rpm con los pulsos acumulados en los últimos 100ms

        int pwm_motor = (int)LQRMotor((float)N_rpm, setpoint_M);
        analogWrite(pinMotor, pwm_motor);

        // ==============================
        // 3. TELEMETRÍA (Para el sistema adaptativo en Python)
        // ==============================
        // Formato: TempReal,PWM_H,RPMReal,PWM_M
        Serial.print(temp_real); Serial.print(",");
        Serial.print(pwm_hotend); Serial.print(",");
        Serial.print(N_rpm); Serial.print(",");
        Serial.println(pwm_motor);
    }
}
