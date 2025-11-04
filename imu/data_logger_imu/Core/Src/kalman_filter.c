/*
 * kalman_filter.c
 *
 *  Created on: May , 2025
 *      Author: Mario Leal
 */
#include "kalman_filter.h"

void KalmanFilter_Init(KalmanFilter_t* kf) {
  kf->angle = 0.0f;
  kf->bias = 0.0f;
  kf->P[0][0] = 0.0f; kf->P[0][1] = 0.0f;
  kf->P[1][0] = 0.0f; kf->P[1][1] = 0.0f;
  kf->Q_angle = 0.001f;
  kf->Q_bias = 0.003f;
  kf->R_measure = 0.03f;
}

float KalmanFilter_Update(KalmanFilter_t* kf, float newAngle, float newRate) {
  float dt = 0.01f;

  // Predicción
  kf->rate = newRate - kf->bias;
  kf->angle += dt * kf->rate;

  // Covarianza predictiva
  kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
  kf->P[0][1] -= dt * kf->P[1][1];
  kf->P[1][0] -= dt * kf->P[1][1];
  kf->P[1][1] += kf->Q_bias * dt;

  // Corrección
  float y = newAngle - kf->angle;
  float S = kf->P[0][0] + kf->R_measure;
  float K[2];
  K[0] = kf->P[0][0] / S;
  K[1] = kf->P[1][0] / S;

  kf->angle += K[0] * y;
  kf->bias += K[1] * y;

  float P00_temp = kf->P[0][0];
  float P01_temp = kf->P[0][1];

  kf->P[0][0] -= K[0] * P00_temp;
  kf->P[0][1] -= K[0] * P01_temp;
  kf->P[1][0] -= K[1] * P00_temp;
  kf->P[1][1] -= K[1] * P01_temp;

  return kf->angle;
}

void calibrateMagnetometer(const float raw[3], float calibrated[3], const MagnetometerCalibration* calib) {
    float corrected[3];

    // Compensación hard iron
    corrected[0] = raw[0] - calib->offset[0];
    corrected[1] = raw[1] - calib->offset[1];
    corrected[2] = raw[2] - calib->offset[2];

    // Compensación soft iron (multiplicación matricial)
    for (int row = 0; row < 3; row++) {
        calibrated[row] = calib->softIron[row][0] * corrected[0] +
                          calib->softIron[row][1] * corrected[1] +
                          calib->softIron[row][2] * corrected[2];
    }
}


