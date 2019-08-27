#include<pigpio.h>
#include"PigpioMS/PigpioMS.hpp"
#include"RasPiDS3/RasPiDS3.hpp"
#include<iostream>
#include<cmath>

RPMS::MotorSerial ms;
RPDS3::DualShock3 controller;


int main(void){
	constexpr short UNDERCARRIAGE_MDD_NUM = 16;
	constexpr short MECHANISM_MDD_NUM = 10;
	constexpr short RIGHT_FRONT_MOTOR_NUM = 2;
	constexpr short RIGHT_BACK_MOTOR_NUM  = 3;
	constexpr short LEFT_FRONT_MOTOR_NUM  = 4;
	constexpr short LEFT_BACK_MOTOR_NUM   = 5;
	constexpr short BOX = 2;
	constexpr short Z_ARM = 5; 
	constexpr short Y_ARM = 4;
	constexpr short PWM_MAX_VALUE = 50;
	constexpr short RIGHT_T_ARM = ;
	constexpr short LEFT_T_ARM = ;
	//constexpr short STICK_MAX_VALUE = 250;
	constexpr short HANGER_LEFT_SOLENOID = 3;
	constexpr short HANGER_RIGHT_SOLENOID = 3;
	//constexpr short POWER_WINDOW_MOTOR_NUM = 4;

	double regulation = 0.3;

	int changer = -1;
	constexpr short limiter = 4;

	controller.update();
	try{
		ms.init();
	}catch(std::runtime_error exception){
		std::cout << "error" << std::endl;
		return -1;
	}

	gpioSetMode(13,PI_OUTPUT);
	gpioWrite(13,true);

	bool right_hanger_flag = true;
	bool left_hanger_flag = true;

	bool box_flag = true;

	bool y_arm_flag = true;
		
	double accelaration = 2;

	std::cout << "携行型通信操縦、命令移動システム、dualshock3,起動しました"
	UPDATELOOP(controller, !(controller.button(RPDS3::START) && controller.button(RPDS3::RIGHT))){

		double left_distance = 0;
		double left_theta = 0;

		//double right_distance = 0;
		//double right_theta = 0;

		//double left_front = 0;
		//double left_back  = 0;
		//int revolve = 0;

		double left_x = controller.stick(RPDS3::LEFT_X);
		double left_y = controller.stick(RPDS3::LEFT_Y);
		

		double right_x = controller.stick(RPDS3::RIGHT_X) * changer;
		double right_y = controller.stick(RPDS3::RIGHT_Y) ;
		left_distance = std::sqrt(std::pow(left_x,2) + std::pow(left_y,2)) * 2;
		double right_t_arms_pwm;
		double left_t_arms_pwm;

		double sum_turn = controller.stick(RPDS3::RIGHT_T) - controller.stick(RPDS3::LEFT_T);

		double right_front_motor_pwm;
		double right_back_motor_pwm;
		double left_front_motor_pwm;
		double left_back_motor_pwm;

		if(sum_turn == 0){
			if(controller.button(RPDS3::UP)){
				if((right_front_motor_pwm == 0 && right_back_motor_pwm == 0) && (left_front_motor_pwm == 0 && left_back_motor_pwm == 0)){
					right_front_motor_pwm = -20;
					right_back_motor_pwm = -20;
					left_front_motor_pwm = 20;
					left_back_motor_pwm = 20;
				}else if((right_front_motor_pwm > -PWM_MAX_VALUE && right_back_motor_pwm > -PWM_MAX_VALUE) && (left_front_motor_pwm < PWM_MAX_VALUE && left_back_motor_pwm < PWM_MAX_VALUE)){
					right_front_motor_pwm = right_front_motor_pwm - accelaration;
					right_back_motor_pwm = right_back_motor_pwm - accelaration;
					left_front_motor_pwm = left_front_motor_pwm + accelaration;
					left_back_motor_pwm = left_back_motor_pwm  + accelaration;
				}else{
					right_front_motor_pwm = -PWM_MAX_VALUE;	
					right_back_motor_pwm = -PWM_MAX_VALUE;
					left_front_motor_pwm = PWM_MAX_VALUE;
					left_back_motor_pwm = PWM_MAX_VALUE;
				}
				std::cout << "forward" << std::endl;
			}else if(controller.button(RPDS3::DOWN)){
				if((right_front_motor_pwm == 0 && right_back_motor_pwm == 0) && (left_front_motor_pwm == 0 && left_back_motor_pwm == 0)){
					right_front_motor_pwm = 20;
					right_back_motor_pwm = 20;
					left_front_motor_pwm = -20;
					left_back_motor_pwm = -20;
				}else if((right_front_motor_pwm < PWM_MAX_VALUE && right_back_motor_pwm < PWM_MAX_VALUE) && (left_front_motor_pwm > -PWM_MAX_VALUE && left_back_motor_pwm > -PWM_MAX_VALUE)){
					right_front_motor_pwm = right_front_motor_pwm + accelaration;
					right_back_motor_pwm = right_back_motor_pwm + accelaration;
					left_front_motor_pwm = left_front_motor_pwm - accelaration;
					left_back_motor_pwm = left_back_motor_pwm - accelaration;
				}else{
					right_front_motor_pwm = PWM_MAX_VALUE;	
					right_back_motor_pwm = PWM_MAX_VALUE;
					left_front_motor_pwm = -PWM_MAX_VALUE;
					left_back_motor_pwm = -PWM_MAX_VALUE;

				}		
				std::cout << "back" <<std::endl;	
			}else if(controller.button(RPDS3::RIGHT)){
				if((right_front_motor_pwm == 0 && right_back_motor_pwm == 0) && (left_front_motor_pwm == 0 && left_back_motor_pwm == 0)){
					right_front_motor_pwm = 20;
					right_back_motor_pwm = -20;
					left_front_motor_pwm = 20;
					left_back_motor_pwm = -20;
				}else if((right_front_motor_pwm < PWM_MAX_VALUE && right_back_motor_pwm > -PWM_MAX_VALUE) && (left_front_motor_pwm < PWM_MAX_VALUE && left_back_motor_pwm > -PWM_MAX_VALUE)){
					right_front_motor_pwm = right_front_motor_pwm + accelaration;
					right_back_motor_pwm = right_back_motor_pwm - accelaration;
					left_front_motor_pwm = left_front_motor_pwm + accelaration;
					left_back_motor_pwm = left_back_motor_pwm - accelaration;			
				}else{
					right_front_motor_pwm = PWM_MAX_VALUE;	
					right_back_motor_pwm = -PWM_MAX_VALUE;
					left_front_motor_pwm = PWM_MAX_VALUE;
					left_back_motor_pwm = -PWM_MAX_VALUE;				
				}
				std::cout << "right" << std::endl;
			}else if(controller.button(RPDS3::LEFT)){
				if((right_front_motor_pwm == 0 && right_back_motor_pwm == 0) && (left_front_motor_pwm == 0 && left_back_motor_pwm == 0)){
					right_front_motor_pwm = -20;
					right_back_motor_pwm = 20;
					left_front_motor_pwm = -20;
					left_back_motor_pwm = 20;
				}else if((right_front_motor_pwm > -PWM_MAX_VALUE && right_back_motor_pwm < PWM_MAX_VALUE) && (left_front_motor_pwm > -PWM_MAX_VALUE && left_back_motor_pwm < PWM_MAX_VALUE)){
					right_front_motor_pwm = right_front_motor_pwm - accelaration;
					right_back_motor_pwm = right_back_motor_pwm + accelaration;
					left_front_motor_pwm = left_front_motor_pwm - accelaration;
					left_back_motor_pwm = left_back_motor_pwm + accelaration;
				}else{
					right_front_motor_pwm = -PWM_MAX_VALUE;	
					right_back_motor_pwm = PWM_MAX_VALUE;
					left_front_motor_pwm = -PWM_MAX_VALUE;
					left_back_motor_pwm = PWM_MAX_VALUE;
				}
				std::cout << "left" << std::endl;
			}else{
				right_front_motor_pwm = 0;
				right_back_motor_pwm = 0;
				left_front_motor_pwm = 0;
				left_back_motor_pwm = 0;
				std::cout << "はようごかせやおらああああ！ " << std::endl;
			}
		}else{
			right_front_motor_pwm = sum_turn / limiter * regulation;
			right_back_motor_pwm = sum_turn / limiter* regulation;
			left_front_motor_pwm = sum_turn / limiter* regulation;
			left_back_motor_pwm = sum_turn / limiter* regulation;
		}
		ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,right_front_motor_pwm*regulation);
		ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,right_back_motor_pwm*regulation);
		ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,left_front_motor_pwm*regulation);
		ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,left_back_motor_pwm*regulation);

		if(controller.button(RPDS3::L1) ){
			accelaration = 0;
		}else{
			accelaration = 2;
		}

		if(controller.press(RPDS3::SQUARE)){
			if(right_hanger_flag == true){
				ms.send(MECHANISM_MDD_NUM,HANGER_LEFT_SOLENOID,261);
				right_hanger_flag = false;
			}else{
				ms.send(MECHANISM_MDD_NUM,HANGER_LEFT_SOLENOID,-261);
				right_hanger_flag = true;
			}
		}

		if(controller.press(RPDS3::CIRCLE)){
			if(left_hanger_flag == true){
				ms.send(MECHANISM_MDD_NUM,HANGER_RIGHT_SOLENOID,262);
				left_hanger_flag = false;
			}else{
				ms.send(MECHANISM_MDD_NUM,HANGER_RIGHT_SOLENOID,-262);
				left_hanger_flag = true;
			}
		}


		ms.send(MECHANISM_MDD_NUM,Y_ARM,right_x * 2 * regulation);
		ms.send(MECHANISM_MDD_NUM,Z_ARM,right_y * 2 * regulation);

		if(controller.press(RPDS3::CROSS)){
			if(y_arm_flag == true){
				changer = 1;
				y_arm_flag = false;
			}else if(y_arm_flag == false){
				changer = -1;
				y_arm_flag = true;
			}
		}
		//追加
		if(left_x != 0 || left_y != 0){
			left_theta = std::atan2(left_x,left_y)/M_PI;
			if( left_theta >= 0 && left_theta < 1/4){
				rigth_t_arms_pwm = left_distance * regulation;
				left_t_arms_pwm = 0;	
			}else if( left_theta >= 1/4 && left_theta <= 3/4){
				rigth_t_arms_pwm = left_distance * regulation;
				left_t_arms_pwm = left_distance * regulation * -1;
			}else if( left_theta > 3/4  && left_theta <= 1){
				rigth_t_arms_pwm = 0;
				left_t_arms_pwm = left_distance * regulation * -1;
			}else if( left_theta > 1    && left_theta < 5/4){
				rigth_t_arms_pwm = left_distance * regulation;
				left_t_arms_pwm = -left_distance * regulation * -1;
			}else if( left_theta >= 5/4 && left_theta <= 7/4){
				rigth_t_arms_pwm = -left_distance * regulation;
				left_t_arms_pwm = -left_distance * regulation * -1;
			}else if( left_theta > 7/4  && left_theta < 2){
				rigth_t_arms_pwm = -left_distance * regulation;
				left_t_arms_pwm = 0;
			}
		}else{
			rigth_t_arms_pwm = 0;
			left_t_arms_pwm = 0;
		}
		ms.send(MECHANISM_MDD_NUM,RIGHT_T_ARM,right_t_arms_pwm);
		ms.send(MECHANISM_MDD_NUM,LEFT_T_ARM,left_t_arms_pwm);

		//終わり
		//if(controller.button(RPDS3::TRIANGLE)){
		//	ms.send(MECHANISM_MDD_NUM,BOX,50);
		//}else if(controller.button(RPDS3::CROSS)){
		//	ms.send(MECHANISM_MDD_NUM,BOX,-50);
		//}else 
		if(controller.press(RPDS3::TRIANGLE)){
			if(box_flag == true){
				ms.send(MECHANISM_MDD_NUM,BOX,263);
				box_flag = false;
			}else{
				ms.send(MECHANISM_MDD_NUM,BOX,0);
				box_flag = true;	
			}


		}
		//else{
		//	ms.send(MECHANISM_MDD_NUM,BOX,0);
		//}

		if(controller.button(RPDS3::R1) == true){
			regulation = 0.5;
			std::cout << "reg" << std::endl;
		}else{
			regulation = 1.0;
		}

	}
        ms.send(MECHANISM_MDD_NUM,Y_ARM,0);
        ms.send(MECHANISM_MDD_NUM,Z_ARM,0);

        ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,0);
        ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,0);
        ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,0);
        ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,0);

	gpioWrite(13,false);
	return 0;
}
