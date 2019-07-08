#include<iostream>
#include<cmath>
#include<pigpio.h>
#include"PigpioMS/PigpioMS.hpp"
#include"RasPiDS3/RasPiDS3.hpp"

RPMS::MotorSerial ms;
RPDS3::DualShock3 controller;

int main(){
	constexpr int undercarriage_mdd_num = 13;
	constexpr int mechanism_mdd_num = 14;
	constexpr int right_front_motor_num = 2;
	constexpr int right_back_motor_num  = 3;
	constexpr int left_front_motor_num  = 4;
	constexpr int left_back_motor_num   = 5;
	constexpr int pwm_max_value = 150;
	constexpr int stick_max_value = 250;

	double regulation = 0.5;

	bool control_mode_flag = true; 

	controller.update();
	try{
		ms.init();
	}catch(std::runtime_error exception){
		std::cout << "error" << std::endl;
		return -1;
	}

	UPDATELOOP(controller, !(controller.button(RPDS3::START) && controller.button(RPDS3::RIGHT))){

		double left_x = 0;
		double left_y = 0;
		double left_distance = 0;
		double left_theta = 0;

		double right_x = 0;
		double right_y = 0;
		double right_distance = 0;
		double right_theta = 0;

		int left_front = 0;
		int left_back  = 0;
		int revolve = 0;

		left_x = controller.stick(RPDS3::LEFT_X);
		left_y = controller.stick(RPDS3::LEFT_Y);
		left_distance = std::sqrt(std::pow(left_x,2) + std::pow(left_y,2)) * 2;

		if(control_mode_flag == true){
			if(left_distance > stick_max_value){
				left_distance = pwm_max_value;
			}

			left_theta = std::atan2(-left_y,left_x) + M_PI;

			if(left_theta >= 0 && left_theta <= (M_PI/2)){
				left_back  = (left_theta * 4 / M_PI) - 1;
				left_front = 1;
			}else if(left_theta > (M_PI/2) && left_theta <= (M_PI)){
				left_back  = 1;
				left_front = -(left_theta * 4 / M_PI) + 3;
			}else if(left_theta > (M_PI) && left_theta <= (3*M_PI/2)){
				left_back  = -(left_theta * 4 / M_PI) + 5;
				left_front = -1;
			}else if(left_theta > (3*M_PI/2) && left_theta <= (2*M_PI)){
				left_back  = -1;
				left_front = (left_theta * 4 / M_PI) - 7;
			}	

			revolve = controller.stick(RPDS3::RIGHT_T) - controller.stick(RPDS3::LEFT_T);
		}	

		if(controller.button(RPDS3::R1) == true){
			regulation = 0.5;
		}else{
			regulation = 1.0;
		}

		ms.send(undercarriage_mdd_num, left_front_motor_num, -left_distance * left_front * regulation + revolve);//左前
		ms.send(undercarriage_mdd_num, left_back_motor_num, -left_distance * left_back  * regulation + revolve);//左後
		ms.send(undercarriage_mdd_num, right_front_motor_num, left_distance  * left_back  * regulation - revolve);//右前
		ms.send(undercarriage_mdd_num, right_back_motor_num,  left_distance  * left_front * regulation - revolve);//右後
	}
}	
