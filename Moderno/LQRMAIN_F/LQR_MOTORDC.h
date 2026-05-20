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
double Ki_LQR_M = 0.0;                    // ganancia integral del LQI (a actualizar con valor de MATLAB)
double integral_error_M = 0.0;            // acumulador del error
const double INT_WINDUP_LIMIT_M = 100.0;  // límite anti-windup

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

// CONTROL LQR - MOTOR DC (LQI: u = Nbar*r - K*x - Ki*∫error)
double LQRMotor(double rpm_actual, double setpoint) {
    // 1. Desplazar el historial
    for (int i = N_ESTADOS_M - 1; i > 0; i--) {
        rpm_history[i] = rpm_history[i-1];
    }
    rpm_history[0] = rpm_actual;

    // 2. Realimentación de estados
    double u_ff = Nbar_M * setpoint;
    double u_fb = 0;
    for (int i = 0; i < N_ESTADOS_M; i++) {
        u_fb += K_LQR_M[i] * rpm_history[i];
    }

    // 3. Acumular integral del error (solo si no saturado — anti-windup condicional)
    double error = setpoint - rpm_actual;
    double u_pre = u_ff - u_fb - Ki_LQR_M * integral_error_M;

    // Anti-windup: solo integrar si la salida no está saturada en la dirección del error
    bool saturated_high = (u_pre >= 255) && (error > 0);
    bool saturated_low  = (u_pre <= 30)  && (error < 0);
    if (!saturated_high && !saturated_low) {
        integral_error_M += error * 0.1;  // Ts = 0.1s
        // Clamp adicional
        if (integral_error_M >  INT_WINDUP_LIMIT_M) integral_error_M =  INT_WINDUP_LIMIT_M;
        if (integral_error_M < -INT_WINDUP_LIMIT_M) integral_error_M = -INT_WINDUP_LIMIT_M;
    }

    double control_u = u_ff - u_fb - Ki_LQR_M * integral_error_M;

    // 4. Setpoint = 0 → apagar
    if (setpoint <= 0) {
        integral_error_M = 0.0;  // reset integrador
        return 0;
    }

    // 5. Saturación con ZONA MUERTA correcta (PWM mín 30, máx 255)
    return constrain((int)control_u, 30, 255);
}

#endif