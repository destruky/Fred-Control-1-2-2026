#ifndef LQR_HOTEND_h
#define LQR_HOTEND_h

#include <Arduino.h>

const float Raux = 460; 
const float A_const = 1.1384e-03, B_const = 2.3245e-04, C_const = 9.489e-08;

// =========================================================
// ⚠️ ¡REEMPLAZAR CON LAS MATRICES DEL HOTEND DE MATLAB!
// =========================================================
double K_LQR_H[6] = {0.8143, 0.9357, 1.0759, 0.8190, -0.2589, 0.4493}; 
double Nbar_H = 5.5129; 

// Memoria de las últimas 6 lecturas de temperatura
double temp_history[6] = {0, 0, 0, 0, 0, 0};

// Seguridad
float maxTemp = 250.0;
float minTemp = 0.0;

float thermistor(int reading) {
    if (reading <= 10 || reading >= 1020) return -999;
    float R = Raux * ((float)reading / (1023.0 - (float)reading));
    float logR = log(R);
    float TempK = 1.0 / (A_const + B_const * logR + C_const * logR * logR * logR);
    return TempK - 273.15;
}

// CONTROL LQR - HOTEND
double LQRHotend(float temp_actual, double setpoint) {
    // 1. Desplazar el historial
    for (int i = 5; i > 0; i--) {
        temp_history[i] = temp_history[i-1];
    }
    temp_history[0] = temp_actual;

    // 2. Ley de control: u = Nbar*r - K*x
    double u_ff = Nbar_H * setpoint;
    double u_fb = 0;
    for (int i = 0; i < 6; i++) {
        u_fb += K_LQR_H[i] * temp_history[i];
    }

    double control_u = u_ff - u_fb;

    // 3. Saturación física (PWM de 0 a 255)
    return constrain(control_u, 0, 255);
}

#endif