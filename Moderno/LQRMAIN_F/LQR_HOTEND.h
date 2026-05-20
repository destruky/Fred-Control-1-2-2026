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
double Ki_LQR_H = 0.0;                    // ganancia integral del LQI (a actualizar con valor de MATLAB)
double integral_error_H = 0.0;            // acumulador del error
const double INT_WINDUP_LIMIT_H = 50.0;   // límite anti-windup

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

// CONTROL LQR - HOTEND (LQI: u = Nbar*r - K*x - Ki*∫error)
double LQRHotend(float temp_actual, double setpoint) {
    // 1. Desplazar el historial
    for (int i = 5; i > 0; i--) {
        temp_history[i] = temp_history[i-1];
    }
    temp_history[0] = temp_actual;

    // 2. Realimentación de estados
    double u_ff = Nbar_H * setpoint;
    double u_fb = 0;
    for (int i = 0; i < 6; i++) {
        u_fb += K_LQR_H[i] * temp_history[i];
    }

    // 3. Acumular integral del error (solo si no saturado — anti-windup condicional)
    double error = setpoint - temp_actual;
    double u_pre = u_ff - u_fb - Ki_LQR_H * integral_error_H;

    bool saturated_high = (u_pre >= 255) && (error > 0);
    bool saturated_low  = (u_pre <= 0)   && (error < 0);
    if (!saturated_high && !saturated_low) {
        integral_error_H += error * 0.1;  // Ts = 0.1s
        if (integral_error_H >  INT_WINDUP_LIMIT_H) integral_error_H =  INT_WINDUP_LIMIT_H;
        if (integral_error_H < -INT_WINDUP_LIMIT_H) integral_error_H = -INT_WINDUP_LIMIT_H;
    }

    double control_u = u_ff - u_fb - Ki_LQR_H * integral_error_H;

    // 4. Saturación física (PWM de 0 a 255)
    return constrain(control_u, 0, 255);
}

#endif