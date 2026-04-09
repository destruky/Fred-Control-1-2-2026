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
// Modelo: companion form completa, W=5, n = 2*W+1 = 11 estados
// x(k) = [rpm(k), rpm(k-1)..rpm(k-5), pwm(k-1)..pwm(k-5)]
// TODO: Reemplazar K_M y Nbar_M con salida de design_motor_lqr.m
// ==========================================
float K_M[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float Nbar_M = 1.0;

// Historial: W=5 muestras pasadas de RPM y PWM
float y_hist_M[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // rpm(k-1)..rpm(k-5)
float u_hist_M[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // pwm(k-1)..pwm(k-5)

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
    for(int i = 0; i < 5; i++) {
        y_hist_M[i] = 0.0;
        u_hist_M[i] = 0.0;
    }
}

// Lazo de Control Principal LQR
double LQRMotor(float rpm_actual, double setpoint) {
    // Si el setpoint es 0, cortamos la energía directamente
    if (setpoint <= 0.1) return 0.0;

    // Construcción del vector de estados x(k) — companion form 11 estados
    // x = [rpm(k), rpm(k-1)..rpm(k-5), pwm(k-1)..pwm(k-5)]
    float x[11] = {
        rpm_actual,
        y_hist_M[0], y_hist_M[1], y_hist_M[2], y_hist_M[3], y_hist_M[4],
        u_hist_M[0], u_hist_M[1], u_hist_M[2], u_hist_M[3], u_hist_M[4]
    };

    // Ecuación LQR: u(k) = Nbar*r - K*x(k)
    double u_k = Nbar_M * setpoint;
    for(int i = 0; i < 11; i++) {
        u_k -= K_M[i] * x[i];
    }

    // Saturación física (respeta zona muerta: si setpoint>0, mínimo PWM=30)
    double u_salida = constrain(u_k, uMin_M, uMax_M);

    // Actualización de los buffers históricos (shift registers)
    for(int i = 4; i > 0; i--) y_hist_M[i] = y_hist_M[i-1];
    y_hist_M[0] = rpm_actual;
    for(int i = 4; i > 0; i--) u_hist_M[i] = u_hist_M[i-1];
    u_hist_M[0] = (float)u_salida;

    return u_salida;
}

#endif
