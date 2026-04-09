#ifndef HOTEND_h
#define HOTEND_h

#include <Arduino.h>

// ==========================================
// CONSTANTES DEL TERMISTOR Y SEGURIDAD
// ==========================================
const float Raux = 460;
const float A = 1.1384e-03, B = 2.3245e-04, C = 9.489e-08;
float maxTemp = 250.0;
float minTemp = 0.0;
float uMin_H = 0.0, uMax_H = 255.0;

// ==========================================
// PARÁMETROS LQR HOTEND (De tabla MATLAB)
// TODO: Reemplazar con ganancias reales de fred_lqr_hotend.m
// ==========================================
float K_H[6] = {0.456, -0.123, 0.089, 0.012, -0.005, 0.001};
float Nbar_H = 0.850;

// Memoria Histórica (Ventana W=5, Modelo de 6 estados)
float y_hist_H[3] = {25.0, 25.0, 25.0}; // Temperaturas pasadas
float u_hist_H[3] = {0.0, 0.0, 0.0};    // PWM pasado
bool lqr_H_initialized = false;

// Función del Termistor
float thermistor(int reading) {
    if (reading <= 10 || reading >= 1020) return -999;
    float R = Raux * ((float)reading / (1023.0 - (float)reading));
    float logR = log(R);
    float TempK = 1.0 / (A + B * logR + C * logR * logR * logR);
    return TempK - 273.15;
}

// Inicialización suave para evitar "saltos"
void initLQRHotend(float currentTemp) {
    for(int i = 0; i < 3; i++) {
        y_hist_H[i] = currentTemp;
        u_hist_H[i] = 0.0;
    }
    lqr_H_initialized = true;
}

// Lazo de Control Principal LQR
double LQRHotend(float temp_actual, double setpoint) {
    if (!lqr_H_initialized) initLQRHotend(temp_actual);

    // Seguridad térmica
    if (temp_actual >= maxTemp || temp_actual == -999) return 0.0;

    // Construcción del vector de estados x(k)
    float x[6] = {temp_actual, y_hist_H[0], y_hist_H[1], u_hist_H[0], u_hist_H[1], u_hist_H[2]};

    // Ecuación LQR
    double u_k = Nbar_H * setpoint;
    for(int i = 0; i < 6; i++) {
        u_k -= K_H[i] * x[i];
    }

    // Saturación física
    double u_salida = constrain(u_k, uMin_H, uMax_H);

    // Actualización de los buffers históricos
    y_hist_H[2] = y_hist_H[1]; y_hist_H[1] = y_hist_H[0]; y_hist_H[0] = temp_actual;
    u_hist_H[2] = u_hist_H[1]; u_hist_H[1] = u_hist_H[0]; u_hist_H[0] = u_salida;

    return u_salida;
}

#endif
