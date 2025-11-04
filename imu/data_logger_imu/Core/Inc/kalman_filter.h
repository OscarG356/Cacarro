#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct {
    float angle;      // Ángulo estimado
    float bias;       // Sesgo del giroscopio
    float rate;       // Velocidad angular medida
    float P[2][2];    // Matriz de covarianza
    float Q_angle;    // Ruido de proceso para el ángulo
    float Q_bias;     // Ruido de proceso para el sesgo
    float R_measure;  // Ruido de medición

    float velocity;   // Velocidad estimada (para posición)
    float position;   // Posición estimada
} KalmanFilter_t;

typedef struct {
    float offset[3];       // [offset_x, offset_y, offset_z]
    float softIron[3][3];  // Matriz 3x3 de corrección soft iron
} MagnetometerCalibration;

void KalmanFilter_Init(KalmanFilter_t* kf);
float KalmanFilter_Update(KalmanFilter_t* kf, float newAngle, float newRate);
void Position_Update(KalmanFilter_t* kf, float acceleration, float dt);

#endif // KALMAN_FILTER_H
