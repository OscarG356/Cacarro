#include "ekf.h"
#include <string.h>
#include <math.h>

// State vector: [x, y, phi, v, b_omega]
static float x_vec[5];
static float P[5][5];   // covariance
static float Q[5][5];   // process noise covariance
static float R_mag;     // measurement var for magnetometer (yaw)
static float R_acc;     // measurement var for accelerometer (ax)
static float dt_default = 0.001f; // default dt

// helpers
static void mat_zero5(float M[5][5]) { memset(M, 0, sizeof(float)*25); }
static void mat_copy5(float dst[5][5], float src[5][5]) { memcpy(dst, src, sizeof(float)*25); }

// compute P = F * P * F^T + Q, for 5x5
static void predict_covariance(const float F[5][5]) {
    float tmp[5][5]; mat_zero5(tmp);
    // tmp = F * P
    for (int i=0;i<5;i++)
        for (int j=0;j<5;j++)
            for (int k=0;k<5;k++)
                tmp[i][j] += F[i][k] * P[k][j];
    float newP[5][5]; mat_zero5(newP);
    // newP = tmp * F^T
    for (int i=0;i<5;i++)
        for (int j=0;j<5;j++)
            for (int k=0;k<5;k++)
                newP[i][j] += tmp[i][k] * F[j][k]; // note F^T indexing
    // add Q
    for (int i=0;i<5;i++)
        for (int j=0;j<5;j++)
            P[i][j] = newP[i][j] + Q[i][j];
}

// (I - K*H) * P update with H row vector (1x5) and scalar S
static void update_covariance_with_KH(const float K[5], const float H[5]) {
    float IminusKH[5][5];
    for (int i=0;i<5;i++)
        for (int j=0;j<5;j++)
            IminusKH[i][j] = (i==j ? 1.0f : 0.0f) - K[i]*H[j];
    // newP = IminusKH * P
    float newP[5][5]; memset(newP,0,sizeof(newP));
    for (int i=0;i<5;i++)
        for (int j=0;j<5;j++)
            for (int k=0;k<5;k++)
                newP[i][j] += IminusKH[i][k] * P[k][j];
    // copy back
    mat_copy5(P, newP);
}

void ekf_init(float dt) {
    dt_default = dt > 0.0f ? dt : 0.01f;
    // init state to zeros
    for (int i=0;i<5;i++) x_vec[i]=0.0f;
    // small initial velocity
    x_vec[3] = 0.0f;
    // covariances: P diag moderate
    mat_zero5(P);
    P[0][0] = 1.0f;  // x
    P[1][1] = 1.0f;  // y
    P[2][2] = 0.5f;  // phi
    P[3][3] = 1.0f;  // v
    P[4][4] = 0.1f;  // b_omega
    // default Q small
    mat_zero5(Q);
    Q[0][0] = 1e-4f;
    Q[1][1] = 1e-4f;
    Q[2][2] = 1e-5f;
    Q[3][3] = 1e-3f;
    Q[4][4] = 1e-6f;
    R_mag = (0.05f*0.05f); // rad^2, tune
    R_acc = (0.5f*0.5f);   // (m/s^2)^2, tune
}

void ekf_set_process_noise(float q_pos, float q_vel, float q_phi, float q_v, float q_bomega) {
    mat_zero5(Q);
    Q[0][0] = q_pos;
    Q[1][1] = q_pos;
    Q[2][2] = q_phi;
    Q[3][3] = q_v;
    Q[4][4] = q_bomega;
}

void ekf_set_measurement_noise(float r_mag, float r_acc) {
    R_mag = r_mag;
    R_acc = r_acc;
}

void ekf_predict(float gyro_raw, float ax_body, float dt) {
    if (dt <= 0.0f) dt = dt_default;
    // state indices: 0:x,1:y,2:phi,3:v,4:bw
    float x = x_vec[0], y = x_vec[1], phi = x_vec[2], v = x_vec[3], bomega = x_vec[4];

    // correct gyro with current bias estimate
    float omega = gyro_raw - bomega;

    // propagate state using simple Euler discretization
    float x_new = x + v * cosf(phi) * dt;
    float y_new = y + v * sinf(phi) * dt;
    float phi_new = phi + omega * dt;
    float v_new = v + ax_body * dt; // we assume ax_body is primary longitudinal accel
    float bomega_new = bomega; // random walk (no change)

    x_vec[0] = x_new;
    x_vec[1] = y_new;
    x_vec[2] = phi_new;
    x_vec[3] = v_new;
    x_vec[4] = bomega_new;

    // Build Jacobian F (5x5) as in derivation:
    float F[5][5] = {0};
    for (int i=0;i<5;i++) for (int j=0;j<5;j++) F[i][j]=0.0f;
    F[0][0] = 1.0f;
    F[0][2] = -v * sinf(phi) * dt;
    F[0][3] = cosf(phi) * dt;
    F[1][1] = 1.0f;
    F[1][2] = v * cosf(phi) * dt;
    F[1][3] = sinf(phi) * dt;
    F[2][2] = 1.0f;
    F[2][4] = -dt;
    F[3][3] = 1.0f;
    F[4][4] = 1.0f;

    // Predict covariance
    predict_covariance(F);
}

void ekf_update_mag(float yaw_meas) {
    // Measurement model: z = phi + noise
    // H = [0 0 1 0 0]  (row vector)
    float H[5] = {0,0,1,0,0};
    // Innovation y = z - H*x
    float y = yaw_meas - x_vec[2];
    // normalize angle innovation to [-pi,pi]
    while (y > M_PI) y -= 2.0f*M_PI;
    while (y < -M_PI) y += 2.0f*M_PI;

    // S = H*P*H^T + R  (scalar)
    float S = 0.0f;
    for (int i=0;i<5;i++)
        for (int j=0;j<5;j++)
            S += H[i] * P[i][j] * H[j];
    S += R_mag;

    // K = P * H^T / S   (5x1)
    float K[5];
    for (int i=0;i<5;i++) {
        float sum = 0.0f;
        for (int j=0;j<5;j++) sum += P[i][j] * H[j];
        K[i] = sum / S;
    }

    // x = x + K * y
    for (int i=0;i<5;i++) x_vec[i] += K[i] * y;

    // P = (I - K H) P
    update_covariance_with_KH(K, H);
}

void ekf_update_acc(float ax_meas) {
    // We treat accelerometer as measuring longitudinal acceleration approx equal to v_dot.
    // Our model: v_{k+1} = v_k + ax*dt  -> but we use instantaneous: z = a_meas = (v_next - v)/dt approx
    // For simplicity, use H = [0 0 0 1 0] mapping to v and compare expected ax = (v - v_prev)/dt is not available here.
    // Simpler approach: interpret acc measurement as direct (no derivative) info on v_dot. To keep it simple we update v:
    // We'll use a pseudo-measure: z_v = v (velocity) by integrating ax externally, but here we directly update v using ax_meas scaled.
    // Alternative: do an update that pulls v towards v_estimated_from_acc_integrated on your side.
    //
    // Here we'll do a pragmatic scalar update: treat z = v_dot_meas and relate to state by v_dot = unknown.
    // Simpler and stable: update v by trusting ax_meas over dt: we convert ax_meas -> delta_v and correct v.
    //
    // delta_v_meas = ax_meas * dt_default
    // H = [0 0 0 1 0] (we will compare delta_v_meas with predicted delta_v which is 0 in state form)
    float dt = dt_default;
    float H[5] = {0,0,0,1,0};
    float delta_v_meas = ax_meas * dt;
    // predicted delta_v (from state) = v_new - v_old ~ we don't keep v_old here. Instead compare with v directly:
    // We'll assume measurement z_v = v + alpha*(delta_v_meas) -- to avoid confusion, a practical simple update:
    // Build innovation y = (v + delta_v_meas) - v = delta_v_meas   -> so this reduces to pulling v by delta_v_meas
    float y = delta_v_meas; // pragmatic innovation

    // S = H P H^T + R_acc
    float S = 0.0f;
    for (int i=0;i<5;i++)
        for (int j=0;j<5;j++)
            S += H[i] * P[i][j] * H[j];
    S += R_acc;

    // K = P * H^T / S
    float K[5];
    for (int i=0;i<5;i++) {
        float sum = 0.0f;
        for (int j=0;j<5;j++) sum += P[i][j] * H[j];
        K[i] = sum / S;
    }
    // state update (this will primarily update v)
    for (int i=0;i<5;i++) x_vec[i] += K[i] * y;

    // covariance update
    update_covariance_with_KH(K, H);
}

void ekf_update_gyro_bias(float gyro_raw) {
    // Optional: measure gyro and update bias state. Model: z = omega_raw = phi_dot + b_omega + n
    // Approx phi_dot ~ (phi - phi_prev)/dt not stored; but we can use predicted phi derivative approx 0.
    // Simpler: use z - (phi_dot_est) ~ b_omega. For pragmatic use, apply H = [0 0 0 0 1]
    float H[5] = {0,0,0,0,1};
    float z = gyro_raw;
    float y = z - x_vec[4]; // innovation assumes phi_dot small, works as slow bias correction

    float S = 0.0f;
    for (int i=0;i<5;i++) for (int j=0;j<5;j++) S += H[i] * P[i][j] * H[j];
    float Rg = 1e-3f;
    S += Rg;

    float K[5];
    for (int i=0;i<5;i++) {
        float sum=0.0f;
        for (int j=0;j<5;j++) sum += P[i][j] * H[j];
        K[i] = sum / S;
    }
    for (int i=0;i<5;i++) x_vec[i] += K[i] * y;
    update_covariance_with_KH(K, H);
}

void ekf_get_state(ekf_state_t *s) {
    if (!s) return;
    s->x = x_vec[0];
    s->y = x_vec[1];
    s->phi = x_vec[2];
    s->v = x_vec[3];
    s->b_omega = x_vec[4];
}
