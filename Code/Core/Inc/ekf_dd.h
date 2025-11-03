#ifndef EKF_DD_H
#define EKF_DD_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bits de máscara para indicar qué mediciones están disponibles en z[]
#define EKF_MEAS_X      (1u << 0)   // z[0] = x
#define EKF_MEAS_Y      (1u << 1)   // z[1] = y
#define EKF_MEAS_THETA  (1u << 2)   // z[2] = theta

// Parámetros físicos (para predicción DINÁMICA)
typedef struct {
    float m, I, R, L, Kt, Ke, Ra, bv, bw;
} DDParams;

// Estado del EKF
typedef struct {
    float x[5];      // [x, y, theta, v, omega]
    float P[5][5];   // covarianza
    float Q[5][5];   // ruido de proceso (discreto)
    float R[3][3];   // ruido de medición diag([Rx,Ry,Rtheta])
} EKF_DD;

// --- Inicialización ---
void ekf_dd_init(EKF_DD* kf,
                 const float x0[5],
                 const float P0[5][5],
                 const float Qd[5][5],
                 const float Rmeas[3][3]);

// --- Predicción (elige UNA de las dos según tu caso) ---
// 1) Predicción CINEMÁTICA con entradas v (m/s) y omega (rad/s)
void ekf_dd_predict_kin(EKF_DD* kf, float v, float omega, float dt);

// 2) Predicción DINÁMICA con entradas u=[uR,uL] (voltios) y parámetros físicos
void ekf_dd_predict_dyn(EKF_DD* kf, const float u[2], float dt, const DDParams* p);

// --- Actualización de medición ---
// z = [x, y, theta] (solo coloca los canales presentes); usa mask con EKF_MEAS_*.
void ekf_dd_update(EKF_DD* kf, const float z[3], unsigned mask);

#ifdef __cplusplus
}
#endif
#endif // EKF_DD_H
