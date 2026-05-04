// ============================================================
// LQR_FIJO_MAIN.ino — Control LQR Fijo para FrED-TEC
// Ubicación: Moderno/LQR_FIJO_MAIN/
//
// COMPATIBILIDAD: AlFrED0_GUI.py (protocolo serial idéntico a MAIN_F)
// MUESTREO: Ts = 100ms (10Hz), lazo discreto estricto
// ESTADO: Funcional con matrices dummy — requiere K y Nbar reales
//
// ANTES DE USAR EN PRODUCCIÓN:
//   1. Correr scripts NN + MATLAB (ver TODO en LQR_HOTEND.h y LQR_MOTORDC.h)
//   2. Insertar K y Nbar reales en ambos .h
//   3. Verificar estabilidad: todos |eigenvalues| < 1
// ============================================================

#include "LQR_HOTEND.h"
#include "LQR_MOTORDC.h"
#include "PIN_MAP.h"
#include <AccelStepper.h>

// ============================================================
// VARIABLES GLOBALES
// ============================================================

// Estado de actuadores (recibido de la GUI via ACTUATE:XXXX)
// digits[0]=Motor DC, digits[1]=Fan, digits[2]=Extrusor, digits[3]=Hotend
String digits    = "0000";
String inputSerial = "";

// Setpoints (actualizados por GUI via TEMP: y DCSPEED:)
double setpoint_Hotend = 190.0;  // °C
double setpoint_Motor  = 20.0;   // RPM

// Motor extrusor stepper
AccelStepper motor2(AccelStepper::DRIVER, 26, 28);
const int enablePin2 = 24;
bool motor2Enabled = false;

// Ventilador
int fanPWM = 0;

// Estados para telemetría
int moto_m = 0, fan_m = 0, heater_m = 0, extruder_m = 0;

// Temporizadores
unsigned long prevMillisLQR    = 0;
unsigned long lastStatusUpdate = 0;

const unsigned long Ts_ms     = 100;   // Lazo LQR: 100ms (10Hz)
const unsigned long STATUS_ms = 1000;  // Telemetría GUI: 1s

// Salidas PWM actuales (para telemetría)
int pwmHotendActual = 0;
int pwmMotorActual  = 0;

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("LQR_FIJO_MAIN: Iniciado.");

    pinMode(pinFan,    OUTPUT);
    pinMode(pinHotend, OUTPUT);
    pinMode(pinMotor,  OUTPUT);
    pinMode(enablePin2, OUTPUT);
    pinMode(C1, INPUT);
    pinMode(C2, INPUT);

    digitalWrite(enablePin2, HIGH);
    digitalWrite(pinHotend, LOW);
    analogWrite(pinMotor, 0);

    motor2.setMaxSpeed(2000);
    motor2.setAcceleration(1000);

    attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);

    initLQRMotor();
    lastTime_M = millis();
}

// ============================================================
// LOOP PRINCIPAL
// ============================================================
void loop() {

    // --- 1. Leer comandos seriales de la GUI ---
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            processInput(inputSerial);
            inputSerial = "";
        } else {
            inputSerial += c;
        }
    }

    unsigned long now = millis();

    // --- 2. Lazo de control LQR a 10Hz (Ts = 100ms) ---
    // Se usa += para compensar deriva acumulada
    if (now - prevMillisLQR >= Ts_ms) {
        prevMillisLQR += Ts_ms;

        float tempActual = thermistor(analogRead(termPin));
        computeRpm();

        // -- HOTEND LQR --
        if (digits.length() >= 4 && digits[3] == '1') {
            if (tempActual == -999 || tempActual >= maxTemp_H) {
                analogWrite(pinHotend, 0);
                pwmHotendActual = 0;
                Serial.println("!SHUTDOWN HOTEND!");
            } else {
                pwmHotendActual = (int)LQRHotend(tempActual, setpoint_Hotend);
                analogWrite(pinHotend, pwmHotendActual);
            }
            heater_m = 1;
        } else {
            analogWrite(pinHotend, 0);
            pwmHotendActual = 0;
            heater_m = 0;
        }

        // -- MOTOR DC LQR --
        if (digits.length() >= 1 && digits[0] == '1') {
            pwmMotorActual = (int)LQRMotor((float)N_rpm, setpoint_Motor);
            analogWrite(pinMotor, pwmMotorActual);
            moto_m = 1;
        } else {
            analogWrite(pinMotor, 0);
            pwmMotorActual = 0;
            // Limpiar historial para arranque limpio
            initLQRMotor();
            moto_m = 0;
        }
    }

    // --- 3. Ventilador (no tiene lazo de control, control directo) ---
    if (digits.length() >= 2 && digits[1] == '1') {
        analogWrite(pinFan, fanPWM);
        fan_m = (fanPWM > 0) ? 1 : 0;
    } else {
        analogWrite(pinFan, 0);
        fan_m = 0;
    }

    // --- 4. Extrusor Stepper ---
    if (digits.length() >= 3 && digits[2] == '1') {
        if (!motor2Enabled) {
            digitalWrite(enablePin2, LOW);
            motor2Enabled = true;
        }
        motor2.runSpeed();
        extruder_m = 1;
    } else {
        if (motor2Enabled) {
            digitalWrite(enablePin2, HIGH);
            motor2Enabled = false;
        }
        extruder_m = 0;
    }

    // --- 5. Telemetría → GUI (cada 1s, mismo formato que MAIN_F) ---
    unsigned long curr = millis();
    if (curr - lastStatusUpdate >= STATUS_ms) {
        lastStatusUpdate = curr;

        float tempTele = thermistor(analogRead(termPin));
        Serial.print("Temp:");       Serial.println(tempTele);
        Serial.print("Motor DC RPM:"); Serial.println(N_rpm);
        Serial.println("--- Estado de componentes ---");
        Serial.print("Fan:        "); Serial.println(fan_m     ? "Encendido" : "Apagado");
        Serial.print("Heater:     "); Serial.println(heater_m  ? "Encendido" : "Apagado");
        Serial.print("Extruder:   "); Serial.println(extruder_m ? "Encendido" : "Apagado");
        Serial.println("-----------------------------");
    }
}

// ============================================================
// processInput — Protocolo serial compatible con AlFrED0_GUI.py
// Comandos soportados: ACTUATE, TEMP, DCSPEED, SPEED, FANSPEED
// Comandos ignorados:  PIDH, PIDM (no aplican para LQR fijo)
// ============================================================
void processInput(String command) {

    if (command.startsWith("ACTUATE:")) {
        digits = command.substring(8);
    }
    else if (command.startsWith("TEMP:")) {
        setpoint_Hotend = command.substring(5).toDouble();
        // Al cambiar setpoint, reiniciar historial del hotend para evitar
        // transitorio brusco por el término integral heredado
        lqr_H_ready = false;
        Serial.print("LQR SP Hotend: "); Serial.println(setpoint_Hotend);
    }
    else if (command.startsWith("DCSPEED:")) {
        setpoint_Motor = command.substring(8).toDouble();
        Serial.print("LQR SP Motor: "); Serial.println(setpoint_Motor);
    }
    else if (command.startsWith("SPEED:")) {
        int vel = command.substring(6).toInt();
        motor2.setSpeed(vel);
        Serial.print("Extrusor vel: "); Serial.println(vel);
    }
    else if (command.startsWith("FANSPEED:")) {
        int pct = command.substring(9).toInt();
        fanPWM = map(pct, 0, 100, 0, 255);
        Serial.print("Fan PWM: "); Serial.println(fanPWM);
    }
    // PIDH/PIDM — ignorados en LQR fijo (la GUI los envía pero aquí no aplican)
    else if (command.startsWith("PIDH:") || command.startsWith("PIDM:")) {
        Serial.println("INFO: PIDH/PIDM ignorados en modo LQR fijo.");
    }
}