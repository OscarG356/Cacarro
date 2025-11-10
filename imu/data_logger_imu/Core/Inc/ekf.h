#ifndef EKF_H
#define EKF_H

#include <stdint.h>

typedef struct {
    float x;
    float y;
    float phi;
    float v;
    float b_omega;
} ekf_state_t;

void ekf_init(float dt);
void ekf_set_process_noise(float q_pos, float q_vel, float q_phi, float q_v, float q_bomega);
void ekf_set_measurement_noise(float r_mag, float r_acc);

/// Predict step: provide raw gyro (rad/s) and accel longitudinal (m/s^2) and dt
void ekf_predict(float gyro_raw, float ax_body, float dt);

/// Update with magnetometer yaw measurement (radians, e.g., atan2(my,mx) after calibration)
void ekf_update_mag(float yaw_meas);

/// Update with accelerometer longitudinal measurement (ax_body, m/s^2)
void ekf_update_acc(float ax_meas);

/// Optional: update using gyro measurement to estimate bias (gyro_raw in rad/s)
void ekf_update_gyro_bias(float gyro_raw);

/// Get current state
void ekf_get_state(ekf_state_t *s);

#endif
