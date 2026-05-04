#ifndef LQR_MOTORDC_h
#define LQR_MOTORDC_h

#include <Arduino.h>

// ============================================================
// ENCODER Y RPM (idéntico a MAIN_F)
// ============================================================
volatile int n_enc = 0;
volatile byte ant_enc = 0;
volatile byte act_enc = 0;
double N_rpm = 0.0;
unsigned long lastTime_M = 0;
const int R_ENC = 4704;  // Pulsos por revolución del encoder

// ============================================================
// SEGURIDAD DEL MOTOR
// Zona muerta: PWM < 30 no mueve físicamente el motor
// ============================================================
float uMin_M = 30.0, uMax_M = 255.0;

// ============================================================
// GANANCIAS LQR — MOTOR DC
// Sistema: companion form completa, W=5 → 2W+1 = 11 estados
// x(k) = [rpm(k), rpm(k-1)..rpm(k-5), pwm(k-1)..pwm(k-5)]
//
// FLUJO PARA OBTENER ESTOS VALORES:
//   1. Correr: Moderno/NN/Motor/motor_sysid_nn.py
//              → genera state_space_motor.mat
//   2. Correr: Moderno/MATLAB/Motor/design_motor_lqr.m
//              → imprime K_motor (1x11) y Nbar_motor en consola MATLAB
//   3. Copiar los 11 valores de K_motor aquí abajo
//   4. Copiar el valor de Nbar_motor aquí abajo
// ============================================================

// TODO: INSERTAR MATRIZ K AQUI — Reemplaza los 11 ceros con salida de design_motor_lqr.m
// Ejemplo de formato de salida MATLAB: K_motor = [k1 k2 k3 k4 k5 k6 k7 k8 k9 k10 k11]
float K_M[11] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// TODO: INSERTAR NBAR AQUI — Reemplaza el 0.0 con Nbar_motor de design_motor_lqr.m
float Nbar_M = 0.0;

// ============================================================
// BUFFERS DE HISTORIAL (W=5)
// ============================================================
float y_hist_M[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // rpm(k-1)..rpm(k-5)
float u_hist_M[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // pwm(k-1)..pwm(k-5)

// ============================================================
// FUNCIONES DEL ENCODER (idénticas a MAIN_F)
// ============================================================

void computeRpm(void) {
    unsigned long now = millis();
    unsigned long elapsed = now - lastTime_M;
    if (elapsed > 0) {
        N_rpm = (n_enc * 60.0 * 1000.0) / ((double)elapsed * R_ENC);
    } else {
        N_rpm = 0.0;
    }
    lastTime_M = now;
    n_enc = 0;
}

void encoder(void) {
    ant_enc = act_enc;
    act_enc = PIND & 12;

    if (ant_enc == 0  && act_enc ==  4)  n_enc++;
    if (ant_enc == 4  && act_enc == 12)  n_enc++;
    if (ant_enc == 8  && act_enc ==  0)  n_enc++;
    if (ant_enc == 12 && act_enc ==  8)  n_enc++;

    if (ant_enc == 0  && act_enc == 8)   n_enc--;
    if (ant_enc == 4  && act_enc == 0)   n_enc--;
    if (ant_enc == 8  && act_enc == 12)  n_enc--;
    if (ant_enc == 12 && act_enc == 4)   n_enc--;
}

// Inicialización a cero
void initLQRMotor() {
    for (int i = 0; i < 5; i++) {
        y_hist_M[i] = 0.0;
        u_hist_M[i] = 0.0;
    }
}

// Calcula el PWM del motor mediante LQR
// Devuelve: PWM entero en [30, 255] o 0 si setpoint=0
double LQRMotor(float rpmActual, double setpoint) {

    // Si se pide parar, cortar energía directamente
    if (setpoint <= 0.1) {
        // Limpiar historial para arranque limpio en el siguiente ciclo
        initLQRMotor();
        return 0.0;
    }

    // Vector de estados: companion form 11 estados
    // x = [rpm(k), rpm(k-1)..rpm(k-5), pwm(k-1)..pwm(k-5)]
    float x[11] = {
        rpmActual,
        y_hist_M[0], y_hist_M[1], y_hist_M[2], y_hist_M[3], y_hist_M[4],
        u_hist_M[0], u_hist_M[1], u_hist_M[2], u_hist_M[3], u_hist_M[4]
    };

    // Ley de control: u(k) = Nbar * r(k) - K * x(k)
    double u_k = (double)Nbar_M * setpoint;
    for (int i = 0; i < 11; i++) {
        u_k -= (double)K_M[i] * x[i];
    }

    // Saturación (respeta zona muerta: mínimo PWM=30 cuando hay setpoint)
    double u_out = constrain(u_k, uMin_M, uMax_M);

    // Actualizar shift registers
    for (int i = 4; i > 0; i--) y_hist_M[i] = y_hist_M[i - 1];
    y_hist_M[0] = rpmActual;

    for (int i = 4; i > 0; i--) u_hist_M[i] = u_hist_M[i - 1];
    u_hist_M[0] = (float)u_out;

    return u_out;
}

#endif