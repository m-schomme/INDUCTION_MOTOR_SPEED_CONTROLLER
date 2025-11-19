/*
 * PI_CONTROLLER.c
 *
 *  Created on: Nov 5, 2025
 *      Author: maryr
 */

#include "arm_math.h"
#include "math.h"
#include "PI_CONTROLLER.h"

arm_pid_instance_f32 PI_Id;
arm_pid_instance_f32 PI_Iq;
arm_pid_instance_f32 PI_Speed;
// initialize PI controllers for Id and Iq
void pi_controller_init(float Kp_Id, float Ki_Id,float Kp_Iq, float Ki_Iq, float Kp_Speed, float Ki_Speed)
{
    // Id Controller
    PI_Id.Kp = Kp_Id;
    PI_Id.Ki = Ki_Id;
    PI_Id.Kd = 0.0f;
    arm_pid_init_f32(&PI_Id, 1);

    // Iq Controller
    PI_Iq.Kp = Kp_Iq;
    PI_Iq.Ki = Ki_Iq;
    PI_Iq.Kd = 0.0f;
    arm_pid_init_f32(&PI_Iq, 1);

    // Speed Controller
	PI_Speed.Kp = Kp_Speed;
	PI_Speed.Ki = Ki_Speed;
	PI_Speed.Kd = 0.0f;
	arm_pid_init_f32(&PI_Speed, 1);

}
// PI controller for Id
float pi_controller_Id(float Id_ref, float Id_meas) {
    float error = Id_ref - Id_meas;
    return arm_pid_f32(&PI_Id, error);
}
// PI controller for Iq
float pi_controller_Iq(float Iq_ref, float Iq_meas) {
    float error = Iq_ref - Iq_meas;
    return arm_pid_f32(&PI_Iq, error);
}
// PI controller for Speed
float pi_controller_speed(float speed_ref, float speed_meas) {
    float error = speed_ref - speed_meas;
    return arm_pid_f32(&PI_Speed, error);
}

