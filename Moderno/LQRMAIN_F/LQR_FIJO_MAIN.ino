#include "LQR_MOTORDC.h"
#include "LQR_HOTEND.h"
#include "PIN_MAP.h"
#include <AccelStepper.h>

// --- VARIABLES GLOBALES ---
String inputSerial = "";
String digits = "0000"; // [Motor DC, Fan, Extrusor, Hotend]

// --- MOTOR EXTRUSOR (STEPPER) ---
AccelStepper motor2(AccelStepper::DRIVER, 26, 28);
const int enablePin2 = 24;
bool motor2Enabled = false;

// --- SETPOINTS ---
double setpoint_Motor = 20.0;
double setpoint_Hotend = 190.0;

// --- VARIABLES "FANTASMA" PARA QUE LA GUI PYTHON NO FALLE ---
// Como la GUI sigue mandando comandos "PIDH" y "PIDM", el Arduino
// los va a recibir y guardar aquí, pero el código JAMÁS los usa.
float Kp_H = 1.8, Ki_H = 0.9, Kd_H = 0.3;
double Kp_M = 25, Ki_M = 2.5, Kd_M = 1.5;

// --- ESTADOS Y TEMPORIZADORES ---
int moto_m = 0, fan_m = 0, heater_m = 0, extruder_m = 0;
unsigned long lastStatusUpdate = 0;
unsigned long prevTime_loop = 0;
int fanPWM = 0; 

void setup() {
  Serial.begin(115200);
  Serial.println(">>> CONTROL LQR FIJO ACTIVADO (Hotend y Motor DC) <<<");
  
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

  prevTime_loop = millis();
}

void loop() {
  // 1. LEER COMANDOS SERIALES DE LA GUI
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processInput(inputSerial);
      inputSerial = "";
    } else {
      inputSerial += c;
    }
  }

  // 2. CICLO DE CONTROL (Cada 100ms = Ts = 0.1s de tu modelo)
  unsigned long now = millis();
  if (now - prevTime_loop >= 100) {
    prevTime_loop = now;

    // Actualizar sensores
    float temp_actual = thermistor(analogRead(termPin));
    computeRpm(); 

    // --- APLICAR LQR: HOTEND ---
    if (digits.length() >= 4 && digits[3] == '1') {
      if (temp_actual == -999 || temp_actual > maxTemp || temp_actual < 10) {
        digitalWrite(pinHotend, LOW);
        Serial.println("¡SHUTDOWN DE SEGURIDAD TERMICA!");
      } else {
        int pwm_H = (int)LQRHotend(temp_actual, setpoint_Hotend);
        analogWrite(pinHotend, pwm_H);
      }
      heater_m = 1;
    } else {
      analogWrite(pinHotend, 0);
      heater_m = 0;
      // Mantener el historial estable si está apagado
      for(int i=0; i<6; i++) temp_history[i] = temp_actual;
    }

    // --- APLICAR LQR: MOTOR DC ---
    if (digits.length() >= 1 && digits[0] == '1') {
      int pwm_M = (int)LQRMotor(N_rpm, setpoint_Motor);
      analogWrite(pinMotor, pwm_M);
      moto_m = 1;
    } else {
      analogWrite(pinMotor, 0);
      moto_m = 0;
      // Mantener el historial estable si está apagado
      for(int i=0; i<N_ESTADOS_M; i++) rpm_history[i] = 0;
    }
  }

  // --- FAN Y EXTRUDER (Sin cambios) ---
  if (digits.length() >= 2 && digits[1] == '1') {
    analogWrite(pinFan, fanPWM);
    fan_m = (fanPWM > 0) ? 1 : 0;
  } else {
    analogWrite(pinFan, 0);
    fan_m = 0;
  }

  if (digits.length() >= 3 && digits[2] == '1') {
    if (!motor2Enabled) { digitalWrite(enablePin2, LOW); motor2Enabled = true; }
    motor2.runSpeed();
    extruder_m = 1;
  } else {
    if (motor2Enabled) { digitalWrite(enablePin2, HIGH); motor2Enabled = false; }
    extruder_m = 0;
  }

  // --- ENVÍO DE ESTADO A LA GUI ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastStatusUpdate >= 1000) {
    lastStatusUpdate = currentMillis;
    Serial.print("Temp:"); Serial.println(thermistor(analogRead(termPin)));
    Serial.print("Motor DC RPM:"); Serial.println(N_rpm);
    Serial.println("--- Estado de componentes ---");
    Serial.print("Fan:        "); Serial.println(fan_m ? "Encendido" : "Apagado");
    Serial.print("Heater:     "); Serial.println(heater_m ? "Encendido" : "Apagado");
    Serial.print("Extruder:   "); Serial.println(extruder_m ? "Encendido" : "Apagado");
    Serial.println("-----------------------------");
  }
}

// --- LECTURA DE COMANDOS DESDE PYTHON ---
void processInput(String command) {
  if (command.startsWith("ACTUATE:")) digits = command.substring(8);
  else if (command.startsWith("SPEED:")) motor2.setSpeed(command.substring(6).toInt());
  else if (command.startsWith("TEMP:")) setpoint_Hotend = command.substring(5).toDouble();
  else if (command.startsWith("DCSPEED:")) setpoint_Motor = command.substring(8).toDouble();
  else if (command.startsWith("FANSPEED:")) fanPWM = map(command.substring(9).toInt(), 0, 100, 0, 255);
  // Las variables PID se reciben para que la GUI no falle, pero el LQR las ignora.
  else if (command.startsWith("PIDH:")) {
    int f = command.indexOf(','), s = command.indexOf(',', f + 1);
    if (f > 0 && s > f) { Kp_H = command.substring(5, f).toFloat(); Ki_H = command.substring(f + 1, s).toFloat(); Kd_H = command.substring(s + 1).toFloat(); }
  }
  else if (command.startsWith("PIDM:")) {
    int f = command.indexOf(','), s = command.indexOf(',', f + 1);
    if (f > 0 && s > f) { Kp_M = command.substring(5, f).toFloat(); Ki_M = command.substring(f + 1, s).toFloat(); Kd_M = command.substring(s + 1).toFloat(); }
  }
  // Actualización LQR adaptativo desde Python: "LQRH:k0,k1,k2,k3,k4,k5,nbar"
  else if (command.startsWith("LQRH:")) {
    String vals = command.substring(5);
    int pos = 0;
    for (int i = 0; i < 6; i++) {
      int comma = vals.indexOf(',', pos);
      if (comma < 0) break;
      K_LQR_H[i] = vals.substring(pos, comma).toDouble();
      pos = comma + 1;
    }
    Nbar_H = vals.substring(pos).toDouble();
    Serial.println("LQR_H actualizado");
  }
  // Actualización LQR adaptativo desde Python: "LQRM:k0..k10,nbar" (11 estados)
  else if (command.startsWith("LQRM:")) {
    String vals = command.substring(5);
    int pos = 0;
    for (int i = 0; i < N_ESTADOS_M; i++) {
      int comma = vals.indexOf(',', pos);
      if (comma < 0) break;
      K_LQR_M[i] = vals.substring(pos, comma).toDouble();
      pos = comma + 1;
    }
    Nbar_M = vals.substring(pos).toDouble();
    Serial.println("LQR_M actualizado");
  }
}