#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>

// Estructura de estado para cada rueda
typedef struct {
    float angle;     // Ángulo total girado (rad)
    float distance;  // Distancia total recorrida (m)
    float velocity;  // Velocidad lineal (m/s)
} WheelKinematics;

// Estructura de estado global del robot
typedef struct {
    WheelKinematics left;
    WheelKinematics right;

    float center_distance;
    float center_velocity;

    float theta;       // Orientación actual del robot (rad)
    float theta_prev;  // Orientación anterior

    float x;           // Posición actual en X
    float y;           // Posición actual en Y
    float x_prev;      // Posición anterior en X
    float y_prev;      // Posición anterior en Y

    float theta_L;     // Ángulo de la rueda izquierda (por encoder)
    float theta_R;     // Ángulo de la rueda derecha (por encoder)

    float wheel_radius; // Radio de las ruedas (m)
    float wheel_base;   // Distancia entre ruedas (m)

    float Wc;
	float Vc;
	float W_L_d;
	float W_R_d;

	float Vc_k_1, Vc_e, Vc_ek_1;
	float Wc_k_1, Wc_e, Wc_ek_1;

} RobotKinematics;

// Función de inicialización
void RobotKinematics_Init(RobotKinematics *robot, float wheel_radius, float wheel_base);

// Función para actualizar el estado del robot usando los ángulos de las ruedas
void RobotKinematics_Update(RobotKinematics *robot, float LAngle, float RAngle, float dt);

float W_Control_Law(RobotKinematics *robot, float Xd, float Yd);

void VL_Control_Law(RobotKinematics *robot, float Xd, float Yd);

void Angular_Vel(RobotKinematics *robot);

#endif // ROBOT_KINEMATICS_H
