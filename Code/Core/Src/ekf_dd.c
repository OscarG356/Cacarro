#include "ekf_dd.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =================== Utilidades internas ===================
static inline float wrap_pi(float a){
    while (a >=  (float)M_PI) a -= 2.0f*(float)M_PI;
    while (a <  -(float)M_PI) a += 2.0f*(float)M_PI;
    return a;
}
static void eye5(float A[5][5]) {
    memset(A, 0, sizeof(float)*25);
    for (int i=0;i<5;i++) A[i][i]=1.0f;
}
static void mat5_add(float A[5][5], const float B[5][5]){
    for(int i=0;i<5;i++) for(int j=0;j<5;j++) A[i][j]+=B[i][j];
}
static void mat5_mul(const float A[5][5], const float B[5][5], float C[5][5]){
    for(int i=0;i<5;i++){
        for(int j=0;j<5;j++){
            float s=0.0f;
            for(int k=0;k<5;k++) s += A[i][k]*B[k][j];
            C[i][j]=s;
        }
    }
}
static void mat5_T(const float A[5][5], float AT[5][5]){
    for(int i=0;i<5;i++) for(int j=0;j<5;j++) AT[j][i]=A[i][j];
}
static bool inv1(const float A, float* Ain){ if(fabsf(A)<1e-9f) return false; *Ain = 1.0f/A; return true; }
static bool inv2(const float A[2][2], float Ai[2][2]){
    float d = A[0][0]*A[1][1]-A[0][1]*A[1][0];
    if(fabsf(d)<1e-9f) return false;
    float id=1.0f/d;
    Ai[0][0]= A[1][1]*id; Ai[0][1]=-A[0][1]*id;
    Ai[1][0]=-A[1][0]*id; Ai[1][1]= A[0][0]*id;
    return true;
}
static bool inv3(const float A[3][3], float Ai[3][3]){
    float a=A[0][0], b=A[0][1], c=A[0][2];
    float d=A[1][0], e=A[1][1], f=A[1][2];
    float g=A[2][0], h=A[2][1], i=A[2][2];
    float A11 =  (e*i - f*h), A12 = -(d*i - f*g), A13 =  (d*h - e*g);
    float A21 = -(b*i - c*h), A22 =  (a*i - c*g), A23 = -(a*h - b*g);
    float A31 =  (b*f - c*e), A32 = -(a*f - c*d), A33 =  (a*e - b*d);
    float det = a*A11 + b*A12 + c*A13;
    if(fabsf(det) < 1e-9f) return false;
    float id=1.0f/det;
    Ai[0][0]=A11*id; Ai[0][1]=A21*id; Ai[0][2]=A31*id;
    Ai[1][0]=A12*id; Ai[1][1]=A22*id; Ai[1][2]=A32*id;
    Ai[2][0]=A13*id; Ai[2][1]=A23*id; Ai[2][2]=A33*id;
    return true;
}

// =================== Predicción DINÁMICA (uR,uL) ===================
static void f_dyn(const float x[5], const float u[2], const DDParams* p, float xdot[5]){
    float th = x[2];
    float phiR = (x[3]/p->R) + (p->L/(2.0f*p->R))*x[4];
    float phiL = (x[3]/p->R) - (p->L/(2.0f*p->R))*x[4];
    float tauR = (p->Kt/p->Ra) * (u[0] - p->Ke*phiR);
    float tauL = (p->Kt/p->Ra) * (u[1] - p->Ke*phiL);

    xdot[0] = x[3] * cosf(th);
    xdot[1] = x[3] * sinf(th);
    xdot[2] = x[4];
    xdot[3] = ((tauR + tauL)/(p->R*p->m)) - (p->bv/p->m)*x[3];
    xdot[4] = ((p->L*(tauR - tauL))/(2.0f*p->R*p->I)) - (p->bw/p->I)*x[4];
}
static void jacobian_f_dyn(const float x[5], const DDParams* p, float F[5][5]){
    float th=x[2], v=x[3];
    float dphiR_dv = 1.0f/p->R, dphiR_dom =  p->L/(2.0f*p->R);
    float dphiL_dv = 1.0f/p->R, dphiL_dom = -p->L/(2.0f*p->R);

    float dtauR_dv  = -(p->Kt/p->Ra)*p->Ke*dphiR_dv;
    float dtauR_dom = -(p->Kt/p->Ra)*p->Ke*dphiR_dom;
    float dtauL_dv  = -(p->Kt/p->Ra)*p->Ke*dphiL_dv;
    float dtauL_dom = -(p->Kt/p->Ra)*p->Ke*dphiL_dom;

    float a_v  = (dtauR_dv + dtauL_dv)/(p->R*p->m) - (p->bv/p->m);
    float a_om = (dtauR_dom + dtauL_dom)/(p->R*p->m);
    float b_v  = (p->L*(dtauR_dv - dtauL_dv))/(2.0f*p->R*p->I);
    float b_om = (p->L*(dtauR_dom - dtauL_dom))/(2.0f*p->R*p->I) - (p->bw/p->I);

    for(int i=0;i<5;i++) for(int j=0;j<5;j++) F[i][j]=0.0f;
    F[0][2] = -v*sinf(th);  F[0][3] =  cosf(th);
    F[1][2] =  v*cosf(th);  F[1][3] =  sinf(th);
    F[2][4] = 1.0f;
    F[3][3] = a_v;          F[3][4] = a_om;
    F[4][3] = b_v;          F[4][4] = b_om;
}

// =================== Predicción CINEMÁTICA (v,omega) ===================
static void f_kin(float x[5], float v, float om, float xdot[5]){
    float th = x[2];
    xdot[0] = v * cosf(th);
    xdot[1] = v * sinf(th);
    xdot[2] = om;
    xdot[3] = 0.0f; // dv/dt modelado por Q (ruido de proceso)
    xdot[4] = 0.0f; // dom/dt modelado por Q
}
static void jacobian_f_kin(const float x[5], float v, float om, float F[5][5]){
    (void)om; // no se usa en derivadas aquí
    float th=x[2];
    for(int i=0;i<5;i++) for(int j=0;j<5;j++) F[i][j]=0.0f;
    F[0][2] = -v*sinf(th);  F[0][3] =  cosf(th);
    F[1][2] =  v*cosf(th);  F[1][3] =  sinf(th);
    F[2][4] = 1.0f;
    // F[3,*] y F[4,*] = 0 (modelo de aceleración absorbida en Q)
}

// =================== API pública ===================
void ekf_dd_init(EKF_DD* kf,
                 const float x0[5],
                 const float P0[5][5],
                 const float Qd[5][5],
                 const float Rmeas[3][3])
{
    memcpy(kf->x, x0, sizeof(float)*5);
    memcpy(kf->P, P0, sizeof(float)*25);
    memcpy(kf->Q, Qd, sizeof(float)*25);
    memcpy(kf->R, Rmeas, sizeof(float)*9);
    kf->x[2] = wrap_pi(kf->x[2]);
}

void ekf_dd_predict_dyn(EKF_DD* kf, const float u[2], float dt, const DDParams* p){
    float F[5][5], Fd[5][5], xdot[5], x_pred[5];
    jacobian_f_dyn(kf->x, p, F);
    for(int i=0;i<5;i++) for(int j=0;j<5;j++) Fd[i][j] = (i==j?1.0f:0.0f) + dt*F[i][j];
    f_dyn(kf->x, u, p, xdot);
    for(int i=0;i<5;i++) x_pred[i] = kf->x[i] + dt*xdot[i];

    float FP[5][5], FdT[5][5], P_pred[5][5];
    mat5_mul(Fd, kf->P, FP);
    mat5_T(Fd, FdT);
    mat5_mul(FP, FdT, P_pred);
    mat5_add(P_pred, kf->Q);

    memcpy(kf->x, x_pred, sizeof(float)*5);
    memcpy(kf->P, P_pred, sizeof(float)*25);
    kf->x[2] = wrap_pi(kf->x[2]);
}

void ekf_dd_predict_kin(EKF_DD* kf, float v, float omega, float dt){
    float F[5][5], Fd[5][5], xdot[5], x_pred[5];
    jacobian_f_kin(kf->x, v, omega, F);
    for(int i=0;i<5;i++) for(int j=0;j<5;j++) Fd[i][j] = (i==j?1.0f:0.0f) + dt*F[i][j];
    f_kin(kf->x, v, omega, xdot);
    for(int i=0;i<5;i++) x_pred[i] = kf->x[i] + dt*xdot[i];

    float FP[5][5], FdT[5][5], P_pred[5][5];
    mat5_mul(Fd, kf->P, FP);
    mat5_T(Fd, FdT);
    mat5_mul(FP, FdT, P_pred);
    mat5_add(P_pred, kf->Q);

    memcpy(kf->x, x_pred, sizeof(float)*5);
    memcpy(kf->P, P_pred, sizeof(float)*25);
    kf->x[2] = wrap_pi(kf->x[2]);
}

void ekf_dd_update(EKF_DD* kf, const float z[3], unsigned mask){
    // Construir H (Kx5), Rk (KxK), z_k (Kx1)
    float H[3][5]={{0}}, Rk[3][3]={{0}}, zk[3]={0};
    int K=0;

    if (mask & EKF_MEAS_X)     { H[K][0]=1.0f; zk[K]=z[0]; Rk[K][K]=kf->R[0][0]; K++; }
    if (mask & EKF_MEAS_Y)     { H[K][1]=1.0f; zk[K]=z[1]; Rk[K][K]=kf->R[1][1]; K++; }
    if (mask & EKF_MEAS_THETA) { H[K][2]=1.0f; zk[K]=z[2]; Rk[K][K]=kf->R[2][2]; K++; }

    if (K==0) return; // no hay mediciones

    // y = z - H x
    float zpred[3]={0}, y[3]={0};
    for(int i=0;i<K;i++){
        for(int j=0;j<5;j++) zpred[i] += H[i][j]*kf->x[j];
        y[i] = zk[i] - zpred[i];
    }
    // residual angular envuelto
    if (mask & EKF_MEAS_THETA){
        for(int i=0;i<K;i++){
            if (fabsf(H[i][2]-1.0f)<1e-6f) { y[i] = wrap_pi(y[i]); break; }
        }
    }

    // S = H P H' + Rk
    float HP[3][5]={{0}};
    for(int i=0;i<K;i++) for(int j=0;j<5;j++){
        float s=0; for(int k=0;k<5;k++) s += H[i][k]*kf->P[k][j];
        HP[i][j]=s;
    }
    float HPHt[3][3]={{0}};
    for(int i=0;i<K;i++) for(int j=0;j<K;j++){
        float s=0; for(int k=0;k<5;k++) s += HP[i][k]*H[j][k];
        HPHt[i][j]=s;
    }
    float S[3][3]={{0}}; for(int i=0;i<K;i++) for(int j=0;j<K;j++) S[i][j]=HPHt[i][j]+Rk[i][j];

    // S^{-1}
    float Si[3][3]; bool ok=false;
    if (K==1){ ok = inv1(S[0][0], &Si[0][0]); }
    else if (K==2){
        float S2[2][2]={{S[0][0],S[0][1]},{S[1][0],S[1][1]}};
        ok = inv2(S2,(float (*)[2])Si);
    } else { ok = inv3(S, Si); }
    if (!ok) return; // singular: omite actualización

    // K = P H' S^{-1}
    float Ht[5][3]={{0}}; for(int i=0;i<5;i++) for(int j=0;j<K;j++) Ht[i][j]=H[j][i];
    float PHt[5][3]={{0}}; for(int i=0;i<5;i++) for(int j=0;j<K;j++){
        float s=0; for(int k=0;k<5;k++) s += kf->P[i][k]*Ht[k][j];
        PHt[i][j]=s;
    }
    float Kk[5][3]={{0}}; for(int i=0;i<5;i++) for(int j=0;j<K;j++){
        float s=0; for(int k=0;k<K;k++) s += PHt[i][k]*Si[k][j];
        Kk[i][j]=s;
    }

    // x = x + K y
    for(int i=0;i<5;i++){
        float s=0; for(int j=0;j<K;j++) s += Kk[i][j]*y[j];
        kf->x[i] += s;
    }
    kf->x[2] = wrap_pi(kf->x[2]);

    // P = (I - K H) P
    float KH[5][5]={{0}};
    for(int i=0;i<5;i++) for(int j=0;j<5;j++){
        float s=0; for(int k=0;k<K;k++) s += Kk[i][k]*H[k][j];
        KH[i][j]=s;
    }
    float ImKH[5][5]; eye5(ImKH);
    for(int i=0;i<5;i++) for(int j=0;j<5;j++) ImKH[i][j] -= KH[i][j];

    float Pnew[5][5];
    mat5_mul(ImKH, kf->P, Pnew);
    memcpy(kf->P, Pnew, sizeof(float)*25);
}
