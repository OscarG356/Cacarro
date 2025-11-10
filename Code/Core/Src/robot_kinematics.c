#include "robot_kinematics.h"

void RobotKinematics_Init(RobotKinematics *robot, float wheel_radius, float wheel_base)
{
    // Inicializa todos los campos a 0
    robot->left.angle = 0.0f;
    robot->left.distance = 0.0f;
    robot->left.velocity = 0.0f;

    robot->right.angle = 0.0f;
    robot->right.distance = 0.0f;
    robot->right.velocity = 0.0f;

    robot->center_distance = 0.0f;
    robot->center_velocity = 0.0f;

    robot->theta = 0.0f;
    robot->theta_prev = 0.0f;

    robot->x = 0.0f;
    robot->y = 0.0f;
    robot->x_prev = 0.0f;
    robot->y_prev = 0.0f;

    robot->theta_L = 0.0f;
    robot->theta_R = 0.0f;

    // Inicializa parámetros físicos
    robot->wheel_radius = wheel_radius;
    robot->wheel_base = wheel_base;


    //Velocidades
    robot->Wc = 0.0f;
    robot->Wc_e = 0.0f;
    robot->Wc_ek_1 = 0.0f;
	robot->Wc_k_1 = 0.0f;

    robot-> Vc = 0.0f;
    robot-> W_L_d = 0.0f;
    robot-> W_R_d = 0.0f;

    robot->Vc_ek_1 = 0.0f;
    robot->Vc_k_1 = 0.0f;

    //Correccion odometría(Poner los datos correctos)
    robot->b_corr = 0.324203f;
    robot->DL_corr = 0.094864f;
    robot->DR_corr = 0.105136f;
}

void RobotKinematics_Update(RobotKinematics *robot, float LAngle, float RAngle, float dt)
{
    // Guardar estados anteriores
    robot->x_prev = robot->x;
    robot->y_prev = robot->y;
    robot->theta_prev = robot->theta;

    // Δθ de ruedas
    float dThetaL = LAngle - robot->theta_L;
    float dThetaR = RAngle - robot->theta_R;

    // Actualizar ángulos
    robot->theta_L = LAngle;
    robot->theta_R = RAngle;

    // Distancias por rueda
    float dSL = dThetaL * robot->DL_corr;
    float dSR = dThetaR * robot->DR_corr;

    float dCenter = 0.5f * (dSL + dSR);
    float dTheta = (dSR - dSL) / robot->b_corr;

    // Actualización de orientación
//    robot->theta = robot->theta_prev + dTheta;
    robot->theta = atan2(sinf(robot->theta_prev + dTheta),cosf(robot->theta_prev + dTheta));

    // Usar theta previa para calcular nueva posición
    robot->x = robot->x_prev + dCenter * cosf(robot->theta_prev + 0.5*dTheta);
    robot->y = robot->y_prev + dCenter * sinf(robot->theta_prev + 0.5*dTheta);

    // Velocidades
    robot->left.velocity = dSL / dt;
    robot->right.velocity = dSR / dt;
    robot->center_velocity = dCenter / dt;

    // Acumulados
    robot->left.distance += dSL;
    robot->right.distance += dSR;
    robot->center_distance += dCenter;
}


float W_Control_Law(RobotKinematics *robot, float Xd, float Yd){

	float phi_d = atan2((Yd-robot->y),(Xd-robot->x));
	float e_aux = phi_d - robot->theta;
	float e_k = atan2(sin(e_aux), cos(e_aux));
//	float min_e = 0.125;
	float min_e = 0.25;
	float Kp = 0.55, Ki = 0.001, T = 0.001;
//	float Kp = 0.31, Ki = 0.005, T = 0.001;
//	float Kp = 0.31, Ki = 0.005, T = 0.001;

	if(sqrt(pow(e_k,2)) <= min_e || robot->Vc == 0 ){
		robot->Wc = 0;
//		robot->Wc_e = 0;
//		robot->Wc_ek_1 = 0;
//		robot->Wc_k_1 = 0;
		e_k = 0;
	}else{

//	robot->Wc = Kp*e_k;
	robot->Wc_e = e_k;
	robot->Wc = (2*robot->Wc_k_1 + 2*Kp*robot->Wc_e - 2*Kp*robot->Wc_ek_1 + T*Ki*robot->Wc_e + T*Ki*robot->Wc_ek_1)/2;

	robot->Wc_ek_1 = robot->Wc_e;
	robot->Wc_k_1 = robot->Wc;
	}

	return e_k;
}

void VL_Control_Law(RobotKinematics *robot, float Xd, float Yd){

	double d = sqrt(pow((Yd-robot->y), 2) + pow((Xd-robot->x),2));
	float min_e = 0.06;
	float alpha_p = 0.1;
	float Kp = 0.08, Ki = 0.002, T = 0.001;
//	float Kp = 0.075, Ki = 0.0028, T = 0.001;
//	float Kp = 0.08, Ki = 0.0028, T = 0.001;
	//robot->Vc = alpha_p*d;

	if(d <= min_e ){
		robot->Vc = 0;
		robot->Vc_e = 0;
		robot->Vc_ek_1 = 0;
		robot->Vc_k_1 = 0;
		d=0;
	}

	robot->Vc_e = d;
	robot->Vc = (2*robot->Vc_k_1 + 2*Kp*robot->Vc_e - 2*Kp*robot->Vc_ek_1 + T*Ki*robot->Vc_e + T*Ki*robot->Vc_ek_1)/2;
	//robot->Vc = robot->Vc_k_1 + 0.001*robot->Vc_e - 0.00099*robot->Vc_ek_1;

	robot->Vc_ek_1 = robot->Vc_e;
	robot->Vc_k_1 = robot->Vc;


}

void Angular_Vel(RobotKinematics *robot){ // Reconsiderar el hecho de pasarlas como argumento o que sea parámetros de la instancia

	//Considerar lo de retornar o atributo
	robot-> W_L_d = ( 2*robot->Vc - robot->wheel_base*robot->Wc)/(2*robot->wheel_radius);
	robot-> W_R_d = ( 2*robot->Vc + robot->wheel_base*robot->Wc)/(2*robot->wheel_radius);
}

void Modelo(RobotKinematics *robot){

}
