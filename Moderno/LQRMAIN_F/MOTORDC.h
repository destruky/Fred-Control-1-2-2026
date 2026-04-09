#ifndef MOTORDC_h
#define MOTORDC_h

#include <Arduino.h>

// ==========================================
// VARIABLES DEL ENCODER Y RPM
// ==========================================
volatile int n = 0;
volatile byte ant = 0;
volatile byte act = 0;
double N_rpm = 0.0;
unsigned long lastTime = 0;
const int R = 4704;

// ==========================================
// SEGURIDAD DEL MOTOR
// ==========================================
// uMin_M = 30 por zona muerta: PWM < 30 no mueve el motor
float uMin_M = 30.0, uMax_M = 255.0;

// ==========================================
// PARÁMETROS LQR MOTOR (De tabla MATLAB)
// TODO: Reemplazar con ganancias reales de design_motor_lqr.m
// ==========================================
float K_M[6] = {1.230, 0.540, -0.210, 0.050, 0.010, 0.002};
float Nbar_M = 1.150;

// Memoria Histórica (Ventana W=5, Modelo de 6 estados)
float y_hist_M[3] = {0.0, 0.0, 0.0}; // RPM pasadas
float u_hist_M[3] = {0.0, 0.0, 0.0}; // PWM pasado

// Funciones del Encoder
void computeRpm(void) {
  unsigned long now = millis();
  unsigned long elapsed = now - lastTime;
  if (elapsed > 0) {
    N_rpm = (n * 60.0 * 1000.0) / (elapsed * R);
  } else {
    N_rpm = 0;
  }
  lastTime = now;
  n = 0;
}

void encoder(void) {
  ant = act;
  act = PIND & 12; // Pines 18 y 19 en el Mega

  if(ant==0  && act== 4)  n++;
  if(ant==4  && act==12)  n++;
  if(ant==8  && act== 0)  n++;
  if(ant==12 && act== 8)  n++;

  if(ant==0 && act==8)  n--;
  if(ant==4 && act==0)  n--;
  if(ant==8 && act==12) n--;
  if(ant==12 && act==4) n--;
}

// Inicialización a 0 RPM
void initLQRMotor() {
    for(int i = 0; i < 3; i++) {
        y_hist_M[i] = 0.0;
        u_hist_M[i] = 0.0;
    }
}

// Lazo de Control Principal LQR
double LQRMotor(float rpm_actual, double setpoint) {
    // Si el setpoint es 0, cortamos la energía directamente
    if (setpoint <= 0.1) return 0.0;

    // Construcción del vector de estados x(k)
    float x[6] = {rpm_actual, y_hist_M[0], y_hist_M[1], u_hist_M[0], u_hist_M[1], u_hist_M[2]};

    // Ecuación LQR
    double u_k = Nbar_M * setpoint;
    for(int i = 0; i < 6; i++) {
        u_k -= K_M[i] * x[i];
    }

    // Saturación física (respeta zona muerta: si setpoint>0, mínimo PWM=30)
    double u_salida = constrain(u_k, uMin_M, uMax_M);

    // Actualización de los buffers históricos
    y_hist_M[2] = y_hist_M[1]; y_hist_M[1] = y_hist_M[0]; y_hist_M[0] = rpm_actual;
    u_hist_M[2] = u_hist_M[1]; u_hist_M[1] = u_hist_M[0]; u_hist_M[0] = u_salida;

    return u_salida;
}

#endif
