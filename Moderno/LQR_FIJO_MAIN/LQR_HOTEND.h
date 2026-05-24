#ifndef LQR_HOTEND_h
#define LQR_HOTEND_h

#include <Arduino.h>

const float Raux = 460; 
const float A_const = 1.1384e-03, B_const = 2.3245e-04, C_const = 9.489e-08;

// =========================================================
// ⚠️ ¡REEMPLAZAR CON LAS MATRICES DEL HOTEND DE MATLAB!
// =========================================================
double K_LQR_H[158] = {-1.526300, -1.526725, -1.504316, -1.547340, -1.543246, -1.491131, -1.566722, -1.545952, -1.488213, -1.531965, -1.462296, -1.511897, -1.339096, -1.521953, -1.287061, -1.513663, -1.121330, -1.639386, -1.107157, -1.322276, -1.446480, -1.018416, -1.571325, -1.005094, -1.717184, -0.718688, -1.802931, -0.743616, -1.605680, -1.016520, -1.386702, -1.144245, -1.074673, -1.724026, -0.874425, -1.403157, -0.823872, -1.212965, -0.762071, -1.077708, -0.931257, -0.951136, -0.578902, -1.172871, -0.501623, -1.126148, -0.627346, -0.491081, -0.981453, -0.144460, -0.753218, -0.325079, -0.454078, -0.046821, -0.652068, -0.248927, 1.895296, -1.056958, -1.466507, -0.416159, -0.460758, -0.269485, -0.681136, 0.862485, 0.728449, 0.744754, 0.859824, -0.831577, -0.905752, -1.003680, 0.669955, 0.895365, 0.825942, -0.921721, -0.842666, -0.923088, 0.429441, 0.170798, 0.364708, -0.928499, 0.441956, 0.028088, 0.551664, 0.274584, -0.435553, 0.075814, 0.242582, -0.143509, 0.228975, 0.282863, 0.259521, -0.592583, -0.095035, 0.547035, -0.108076, -0.791863, 0.434541, 0.106755, 0.093484, 0.009180, -0.029593, 0.031824, -0.016326, -0.079284, 0.394113, -0.676131, -0.534771, 0.311365, 0.020560, -0.081148, 0.081077, 0.136043, -0.165588, -0.065364, -0.057245, 0.025423, 0.023931, 0.065952, -0.090531, -0.271416, 0.158715, -0.124208, 0.040490, -0.027578, 0.009350, -0.003221, -0.021435, -0.009072, -0.030882, 0.050258, -0.020036, -0.009962, -0.000484, 0.009534, -0.024722, 0.025540, 0.045647, -0.046467, 0.012743, 0.318690, 0.088745, 0.078614, 0.022755, 0.053297, 0.037865, -0.025750, 0.020695, -0.039985, -0.019264, 0.005809, 0.132876, 0.052367, -0.030550, 0.058580, 0.009872, 0.029611, 0.008999, 0.085190};
double Nbar_H = 8.928907;                     // ganancia de referencia del LQI (a actualizar con valor de MATLAB)
double Ki_LQR_H = -1.893344;                    // ganancia integral del LQI (a actualizar con valor de MATLAB)
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