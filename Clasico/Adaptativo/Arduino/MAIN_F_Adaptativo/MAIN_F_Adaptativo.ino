// ===============================================================
// MAIN_F_Adaptativo.ino
// Variante de MAIN_F.ino para soportar BO Adaptativo Clásico.
// Cambios respecto al original:
//   1. Emite Motor DC RPM, PWM Motor, Temp y PWM Hotend cada 100 ms (10 Hz)
//      en lugar de 1 Hz, para alimentar las ventanas del worker adaptativo.
//   2. Añade líneas "PWM Motor:<val>" y "PWM Hotend:<val>" — necesarias
//      como entrada u(t) para identify_arx() en el lado Python.
//   3. Comprime el bloque "--- Estado de componentes ---" a 2 Hz.
//
// El protocolo de comandos serial es idéntico al original:
//   ACTUATE:<digits>   (motor, fan, extrusor, hotend)
//   DCSPEED:<rpm>
//   TEMP:<celsius>
//   FANSPEED:<0-100>
//   PIDM:Kp,Ki,Kd
//   PIDH:Kp,Ki,Kd
// ===============================================================

#include "MOTORDC.h"
#include "HOTEND.h"
#include "Pin_map.h"
#include <AccelStepper.h>

// --- VARIABLES GLOBALES ---
String inputSerial = "";
String digits = "0000";

// --- MOTOR EXTRUSOR (STEPPER) ---
AccelStepper motor2(AccelStepper::DRIVER, 26, 28);
const int enablePin2 = 24;
bool motor2Enabled = false;

// --- CONTROL PID DE MOTOR DC ---
double setpoint_Motor = 20.0;
double Kp_M = 1.8, Ki_M = 0.9, Kd_M = 0.3;  // Motor DC manual (corregido)

// --- CONTROL PID DE HOTEND ---
double setpoint_Hotend = 190.0;
float Kp_H = 35.0, Ki_H = 4.0, Kd_H = 3.0;  // Hotend BO Fijo (gp_minimize ITAE+SSE)

// --- ESTADOS Y TEMPORIZADORES ---
int moto_m = 0, fan_m = 0, heater_m = 0, extruder_m = 0;
unsigned long lastStatusUpdate = 0;
unsigned long lastFastTelemetry = 0;

int fanPWM = 0;

// --- Variables de telemetría rápida (para BO Adaptativo) ---
int last_pwm_motor = 0;
int last_pwm_hotend = 0;
float last_temp = 0.0;
float temp_filtered = 0.0;   // termistor filtrado (EMA) — reduce ciclo límite

// --- CONFIGURACIÓN INICIAL ---
void setup() {
  Serial.begin(115200);
  Serial.println("Conexion establecida con MAIN_F_Adaptativo (10 Hz telemetria).");

  pinMode(pinFan, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(pinHotend, OUTPUT);
  pinMode(pinMotor, OUTPUT);
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  digitalWrite(enablePin2, HIGH);
  digitalWrite(pinHotend, LOW);
  analogWrite(pinMotor, 0);

  motor2.setMaxSpeed(2000);
  motor2.setAcceleration(1000);

  attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);

  prevTime = millis();
}

// --- BUCLE PRINCIPAL ---
void loop() {
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
  if (now - prevTime >= 100) {
    float dt = (now - prevTime) / 1000.0;
    prevTime = now;

    // Lectura cruda del termistor
    float temp_raw = thermistor(analogRead(termPin));
    // Filtro EMA paso-bajo: la cuantización del ADC mete ruido (~±0.5°C) que,
    // amplificado por Kp alto, causa el ciclo límite de ±1.5°C. alpha=0.80
    // suaviza ese ruido con lag despreciable (tau hotend ≈ 100 s >> 0.1 s).
    // Un fault (-999) pasa directo para no enmascarar la seguridad.
    if (temp_filtered < 1.0f || temp_raw == -999) {
      temp_filtered = temp_raw;
    } else {
      temp_filtered = 0.80f * temp_filtered + 0.20f * temp_raw;
    }
    last_temp = temp_filtered;
    computeRpm();

    // --- PID HOTEND ---
    if (digits.length() >= 4 && digits[3] == '1') {
      if (last_temp == -999 || last_temp > maxTemp || last_temp < 10) {
        digitalWrite(pinHotend, LOW);
        integral_H = 0;
        last_pwm_hotend = 0;
        Serial.println("¡SHUTDOWN DE SEGURIDAD!");
      } else {
        double pidPWM_Hotend = PIDHotend(last_temp, dt, setpoint_Hotend, Kp_H, Ki_H, Kd_H);
        last_pwm_hotend = constrain((int)pidPWM_Hotend, 0, 255);
        analogWrite(pinHotend, last_pwm_hotend);
      }
      heater_m = 1;
    } else {
      analogWrite(pinHotend, 0);
      last_pwm_hotend = 0;
      heater_m = 0;
    }

    // --- PID MOTOR DC ---
    if (digits.length() >= 1 && digits[0] == '1') {
      input = N;
      double pidPWM_Motor = PIDMotor(dt, setpoint_Motor, Kp_M, Ki_M, Kd_M);
      last_pwm_motor = constrain((int)pidPWM_Motor, 0, 255);
      analogWrite(pinMotor, last_pwm_motor);
      moto_m = 1;
    } else {
      analogWrite(pinMotor, 0);
      last_pwm_motor = 0;
      moto_m = 0;
    }

    // --- TELEMETRÍA RÁPIDA 10 Hz para BO Adaptativo ---
    Serial.print("Motor DC RPM:"); Serial.println(N);
    Serial.print("PWM Motor:"); Serial.println(last_pwm_motor);
    Serial.print("Temp:"); Serial.println(last_temp);
    Serial.print("PWM Hotend:"); Serial.println(last_pwm_hotend);
  }

  // --- FAN ---
  if (digits.length() >= 2 && digits[1] == '1') {
    analogWrite(pinFan, fanPWM);
    fan_m = (fanPWM > 0) ? 1 : 0;
  } else {
    analogWrite(pinFan, 0);
    fan_m = 0;
  }

  // --- EXTRUDER ---
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

  // --- ESTADO COMPLETO 2 Hz (informativo, no se parsea) ---
  if (millis() - lastStatusUpdate >= 500) {
    lastStatusUpdate = millis();
    Serial.print("Status: motor="); Serial.print(moto_m);
    Serial.print(" fan=");          Serial.print(fan_m);
    Serial.print(" heater=");       Serial.print(heater_m);
    Serial.print(" extruder=");     Serial.println(extruder_m);
  }
}

// ===============================================================
//   COMANDOS SERIALES (idéntico a MAIN_F.ino)
// ===============================================================
void processInput(String command) {
  if (command.startsWith("ACTUATE:")) {
    digits = command.substring(8);
  }
  else if (command.startsWith("SPEED:")) {
    int nuevaVel = command.substring(6).toInt();
    motor2.setSpeed(nuevaVel);
  }
  else if (command.startsWith("TEMP:")) {
    setpoint_Hotend = command.substring(5).toDouble();
  }
  else if (command.startsWith("DCSPEED:")) {
    setpoint_Motor = command.substring(8).toDouble();
  }
  else if (command.startsWith("FANSPEED:")) {
    int nuevaVelFan = command.substring(9).toInt();
    fanPWM = map(nuevaVelFan, 0, 100, 0, 255);
  }
  else if (command.startsWith("PIDH:")) {
    String values = command.substring(5);
    int firstComma = values.indexOf(',');
    int secondComma = values.indexOf(',', firstComma + 1);
    if (firstComma > 0 && secondComma > firstComma) {
      Kp_H = values.substring(0, firstComma).toFloat();
      Ki_H = values.substring(firstComma + 1, secondComma).toFloat();
      Kd_H = values.substring(secondComma + 1).toFloat();
      integral_H = 0;  // reset al cambiar ganancias (evita transitorio)
    }
  }
  else if (command.startsWith("PIDM:")) {
    String values = command.substring(5);
    int firstComma = values.indexOf(',');
    int secondComma = values.indexOf(',', firstComma + 1);
    if (firstComma > 0 && secondComma > firstComma) {
      Kp_M = values.substring(0, firstComma).toFloat();
      Ki_M = values.substring(firstComma + 1, secondComma).toFloat();
      Kd_M = values.substring(secondComma + 1).toFloat();
      integral_M = 0;
    }
  }
}
