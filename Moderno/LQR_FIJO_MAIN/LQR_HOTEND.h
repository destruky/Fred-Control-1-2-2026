#ifndef LQR_HOTEND_h
#define LQR_HOTEND_h

#include <Arduino.h>

// ============================================================
// CONSTANTES FÍSICAS (idénticas a MAIN_F)
// ============================================================
const float Raux_H = 460;
const float A_SH   = 1.1384e-03, B_SH = 2.3245e-04, C_SH = 9.489e-08;
float maxTemp_H = 250.0;
float uMin_H = 0.0, uMax_H = 255.0;

// ============================================================
// GANANCIAS LQR — HOTEND
// Sistema reducido: 6 estados abstractos (minreal() en MATLAB)
// x(k) = [temp(k), temp(k-1), temp(k-2), pwm(k-1), pwm(k-2), pwm(k-3)]
//
// FLUJO PARA OBTENER ESTOS VALORES:
//   1. Correr: Moderno/NN/Hotend/hotend_sysid_nn.py
//              → genera state_space_hotend.mat
//   2. Correr: Moderno/NN/Hotend/fred_lqr_hotend_design.m
//              → imprime K (1x6) y Nbar en consola MATLAB
//   3. Copiar los 6 valores de K aquí abajo
//   4. Copiar el valor de Nbar aquí abajo
// ============================================================

// TODO: INSERTAR MATRIZ K AQUI — Reemplaza los 6 ceros con salida de fred_lqr_hotend_design.m
// Ejemplo de formato de salida MATLAB: K = [0.4563  -0.1234  0.0892  0.0115  -0.0051  0.0012]
float K_H[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// TODO: INSERTAR NBAR AQUI — Reemplaza el 0.0 con el Nbar que imprime fred_lqr_hotend_design.m
float Nbar_H = 0.0;

// ============================================================
// BUFFERS DE HISTORIAL
// ============================================================
float y_hist_H[3] = {25.0, 25.0, 25.0}; // temp(k-1), temp(k-2), temp(k-3) en °C real
float u_hist_H[3] = {0.0, 0.0, 0.0};    // pwm(k-1), pwm(k-2), pwm(k-3) en 0-255

bool lqr_H_ready = false;

// ============================================================
// FUNCIONES
// ============================================================

float thermistor(int reading) {
    if (reading <= 10 || reading >= 1020) return -999;
    float R     = Raux_H * ((float)reading / (1023.0 - (float)reading));
    float logR  = log(R);
    float TempK = 1.0 / (A_SH + B_SH * logR + C_SH * logR * logR * logR);
    return TempK - 273.15;
}

// Inicialización suave: llena los buffers con la temperatura actual
// para evitar salto brusco al arrancar
void initLQRHotend(float tempActual) {
    for (int i = 0; i < 3; i++) {
        y_hist_H[i] = tempActual;
        u_hist_H[i] = 0.0;
    }
    lqr_H_ready = true;
}

// Calcula el PWM del hotend mediante LQR
// Devuelve: PWM entero en [0, 255]
double LQRHotend(float tempActual, double setpoint) {

    // Seguridad térmica — corta energía si hay problema
    if (tempActual == -999 || tempActual >= maxTemp_H) return 0.0;

    // Inicialización diferida al primer llamado con temperatura válida
    if (!lqr_H_ready) initLQRHotend(tempActual);

    // Vector de estados: x = [temp(k), temp(k-1), temp(k-2), pwm(k-1), pwm(k-2), pwm(k-3)]
    float x[6] = {
        tempActual,
        y_hist_H[0], y_hist_H[1],
        u_hist_H[0], u_hist_H[1], u_hist_H[2]
    };

    // Ley de control: u(k) = Nbar * r(k) - K * x(k)
    double u_k = (double)Nbar_H * setpoint;
    for (int i = 0; i < 6; i++) {
        u_k -= (double)K_H[i] * x[i];
    }

    // Saturación PWM (0-255)
    double u_out = constrain(u_k, uMin_H, uMax_H);

    // Actualizar buffers históricos (shift register)
    y_hist_H[2] = y_hist_H[1];
    y_hist_H[1] = y_hist_H[0];
    y_hist_H[0] = tempActual;

    u_hist_H[2] = u_hist_H[1];
    u_hist_H[1] = u_hist_H[0];
    u_hist_H[0] = (float)u_out;

    return u_out;
}

#endif