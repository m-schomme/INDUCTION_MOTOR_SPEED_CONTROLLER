/*
 * PI_CONTROLLER.h
 *
 *  Created on: Nov 5, 2025
 *      Author: maryr
 */

#ifndef SRC_PI_CONTROLLER_H_
#define SRC_PI_CONTROLLER_H_
#include "stdint.h"

void pi_controller_init(float Kp_Id, float Ki_Id,float Kp_Iq, float Ki_Iq, float Kp_Speed, float Ki_Speed);
float pi_controller_Id(float Id_ref, float Id_meas);
float pi_controller_Iq(float Iq_ref, float Iq_meas);
float pi_controller_speed(float speed_ref, float speed_meas);

#endif /* SRC_PI_CONTROLLER_H_ */
