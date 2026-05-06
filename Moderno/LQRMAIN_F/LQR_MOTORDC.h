#ifndef LQR_MOTORDC_h
#define LQR_MOTORDC_h

#include <Arduino.h>

volatile int n = 0;
volatile byte ant = 0;
volatile byte act = 0;

double N_rpm = 0.0;             
unsigned long lastTime_motor = 0;
const int R_encoder = 4704; 

// =========================================================
// ⚠️ ¡REEMPLAZAR CON LAS MATRICES DEL MOTOR DC DE MATLAB!
// Usa las matrices que te salgan después de correr el script .py del motor
// =========================================================
#define N_ESTADOS_M 11
double K_LQR_M[N_ESTADOS_M] = {
     9.0774, -0.5882, -0.9874, -0.8549,  0.3925,
    -0.2623,  0.0915, -0.5752, -0.1625,  0.2766,  0.0802
};
double Nbar_M = 8.390320;

// Memoria 11 lecturas de RPM (companion form, W=5 → 2W+1=11)
double rpm_history[N_ESTADOS_M] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void computeRpm(void){
  unsigned long now = millis();
  unsigned long elapsed = now - lastTime_motor;
  if (elapsed > 0) {
    N_rpm = (n * 60.0 * 1000.0) / (elapsed * R_encoder);
  } else {
    N_rpm = 0;
  }
  lastTime_motor = now;
  n = 0;
} 

void encoder(void){
  ant = act;                         
  act = PIND & 12;             
  if(ant==0  && act== 4)  n++;
  if(ant==4  && act==12)  n++;
  if(ant==8  && act== 0)  n++;
  if(ant==12 && act== 8)  n++;
  if(ant==0 && act==8)  n--; 
  if(ant==4 && act==0)  n--;
  if(ant==8 && act==12) n--;
  if(ant==12 && act==4) n--;   
}

// CONTROL LQR - MOTOR DC
double LQRMotor(double rpm_actual, double setpoint) {
    // 1. Desplazar el historial
    for (int i = N_ESTADOS_M - 1; i > 0; i--) {
        rpm_history[i] = rpm_history[i-1];
    }
    rpm_history[0] = rpm_actual;

    // 2. Ley de control: u = Nbar*r - K*x
    double u_ff = Nbar_M * setpoint;
    double u_fb = 0;
    for (int i = 0; i < N_ESTADOS_M; i++) {
        u_fb += K_LQR_M[i] * rpm_history[i];
    }

    double control_u = u_ff - u_fb;

    // Si el objetivo es 0, forzamos apagado
    if (setpoint <= 0) return 0;

    // 3. Saturación física (Min PWM 10, Max 255)
    return constrain((int)control_u, 10, 255);
}

#endif