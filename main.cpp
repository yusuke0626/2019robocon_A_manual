#include<pigpio.h>
#include"PigpioMS/PigpioMS.hpp"
#include"RasPiDS3/RasPiDS3.hpp"
#include<iostream>
#include<cmath>

RPMS::MotorSerial ms;
RPDS3::DualShock3 controller;


int main(void){
	constexpr short BATH_TOWEL_MDD_NUM = 17;
	constexpr short UNDERCARRIAGE_MDD_NUM = 16;
	constexpr short MECHANISM_MDD_NUM = 10;
	constexpr short RIGHT_FRONT_MOTOR_NUM = 3;//2
	constexpr short RIGHT_BACK_MOTOR_NUM  = 2;//3
	constexpr short LEFT_FRONT_MOTOR_NUM  = 5;//4
	constexpr short LEFT_BACK_MOTOR_NUM   = 4;//5
	constexpr short BOX = 2;
	constexpr short Z_ARM = 5; 
	constexpr short Y_ARM = 4;
	constexpr short PWM_MAX_VALUE = 150;
	constexpr short RIGHT_T_ARM = 2;
	constexpr short LEFT_T_ARM = 3;
	constexpr short TOWEL_SOLENOID = 8;
	//constexpr short STICK_MAX_VALUE = 250;
	constexpr short HANGER_SOLENOID = 3;
	//constexpr short POWER_WINDOW_MOTOR_NUM = 4;

	double regulation = 0.3;

	int changer = 1;

	controller.update();
	try{
		ms.init();
	}catch(std::runtime_error exception){
		std::cout << "error" << std::endl;
		return -1;
	}
	

	gpioSetMode(13,PI_OUTPUT);
	gpioWrite(13,true);

	gpioSetMode(12,PI_INPUT);
	gpioSetPullUpDown(12,PI_PUD_UP);

	gpioSetMode(16,PI_INPUT);
	gpioSetPullUpDown(16,PI_PUD_UP);

        gpioSetMode(11,PI_INPUT);
        gpioSetPullUpDown(11,PI_PUD_UP);

        gpioSetMode(22,PI_INPUT);
        gpioSetPullUpDown(22,PI_PUD_UP);


	bool hanger_flag = true;

	bool box_flag = true;

	bool coat_flag = true;

	int right_moving_mode = 1;
	int left_moving_mode = 1;

	UPDATELOOP(controller, !(controller.button(RPDS3::START) && controller.button(RPDS3::RIGHT))){

		double left_distance = 0;
		double left_theta = 0;
	
		//double right_distance = 0;
		double right_theta = 0;

		double left_front = 0;
		double left_back  = 0;
		int revolve = 0;

		double left_x = controller.stick(RPDS3::LEFT_X);
		double left_y = controller.stick(RPDS3::LEFT_Y);
		
		double arms_x;
		double arms_y;

		double right_x = controller.stick(RPDS3::RIGHT_X) * changer;
		double right_y = controller.stick(RPDS3::RIGHT_Y) ;
		left_distance = std::sqrt(std::pow(left_x,2) + std::pow(left_y,2)) * 2;

		int t_arm_limit_right_up;
		int t_arm_limit_right_down;
		int t_arm_limit_left_up;
		int t_arm_limit_left_down;

		double theta = 0;

		left_theta = std::atan2(-left_y,left_x) + M_PI;

		//平行移動
		if(left_distance > PWM_MAX_VALUE){
			left_distance = PWM_MAX_VALUE;
		}

		if(controller.button(RPDS3::L1)){
			if(left_theta >= (M_PI/4) && left_theta <= (M_PI/4) * 3){
				theta = (M_PI/2);
			}else if(left_theta > (M_PI/4)*3 && left_theta < (M_PI/4)*5){
				theta = 1;
			}else if(left_theta >= (M_PI/4)*5 && left_theta <= (M_PI/4)*7){
				theta = (M_PI/2) * 3;
			}else{
				theta = 0;
			}
		}else{
			theta = left_theta;		
		}

		if(theta >= 0 && theta <= (M_PI/2)){
			left_back  = (theta * 4 / M_PI) - 1;
			left_front = 1;
		}else if(theta > (M_PI/2) && theta <= (M_PI)){
			left_back  = 1;
			left_front = -(theta * 4 / M_PI) + 3;
		}else if(theta > (M_PI) && theta <= (3*M_PI/2)){
			left_back  = -(theta * 4 / M_PI) + 5;
			left_front = -1;
		}else if(theta > (3*M_PI/2) && theta <= (2*M_PI)){
			left_back  = -1;
			left_front = (theta * 4 / M_PI) - 7;
		}


		revolve = (controller.stick(RPDS3::RIGHT_T) - controller.stick(RPDS3::LEFT_T)) * 0.3;

		ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_FRONT_MOTOR_NUM,   left_distance * left_front * 0.4 * regulation + revolve);
		ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_BACK_MOTOR_NUM,    left_distance * left_back  * 0.4 * regulation + revolve);
		ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_FRONT_MOTOR_NUM, -left_distance * left_back  * 0.4 * regulation + revolve);
		ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_BACK_MOTOR_NUM,  -left_distance * left_front * 0.4 * regulation + revolve); 

		//ハンガー昇降機構
		if(controller.press(RPDS3::SQUARE)){
			if(hanger_flag == true){
				ms.send(MECHANISM_MDD_NUM,HANGER_SOLENOID,1);
             
				hanger_flag = false;
			}else{
                                ms.send(MECHANISM_MDD_NUM,HANGER_SOLENOID,2);
				hanger_flag = true;
			}
		}
		//コートチェンジ
		if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::TRIANGLE)){
			if(coat_flag == true){
				changer = -1;
				coat_flag = false;
			}else{
				changer = 1;
				coat_flag = true;
			}
		}


		//回収機構のアーム

                right_theta = std::atan2(right_y,right_x) + M_PI;

		if(right_theta >= (M_PI/4) && right_theta <= (M_PI/4) * 3){
                	arms_x = 0;
			arms_y = right_y;
                }else if(right_theta > (M_PI/4)*3 && right_theta < (M_PI/4)*5){
                        arms_x = right_x;
                        arms_y = 0;
                }else if(right_theta >= (M_PI/4)*5 && right_theta <= (M_PI/4)*7){
                        arms_x = 0;
                        arms_y = right_y;
                }else{
                        arms_x = right_x;
                        arms_y = 0;
                }

		ms.send(MECHANISM_MDD_NUM,Y_ARM,arms_x * 2 * regulation);
		ms.send(MECHANISM_MDD_NUM,Z_ARM,arms_y * 2 * regulation * -1);
		
		//回収機構の箱
		if(controller.press(RPDS3::CROSS)){
			if(box_flag == true){
				ms.send(MECHANISM_MDD_NUM,BOX,263);
				box_flag = false;
			}else{
				ms.send(MECHANISM_MDD_NUM,BOX,0);
				box_flag = true;	
			}	
		}

		if(controller.button(RPDS3::R1) == true){
			regulation = 0.5;
			std::cout << "reg" << std::endl;
		}else{
			regulation = 1.0;
		}

		//バスタオルのソレノイド
		if(controller.button(RPDS3::L1) && controller.button(RPDS3::CIRCLE)){
			ms.send(BATH_TOWEL_MDD_NUM,TOWEL_SOLENOID,1);
			ms.send(BATH_TOWEL_MDD_NUM,TOWEL_SOLENOID,2);

		}else{
			ms.send(BATH_TOWEL_MDD_NUM,TOWEL_SOLENOID,-1);
			ms.send(BATH_TOWEL_MDD_NUM,TOWEL_SOLENOID,-2);
		}

		t_arm_limit_right_up = gpioRead(12);
		t_arm_limit_right_down = gpioRead(16);
                t_arm_limit_left_up = gpioRead(11);
                t_arm_limit_left_down = gpioRead(22);

		//バスタオルのアーム

		/*
		mode 1 -> stop
		mode 2 -> up
		mode 3 -> down
		*/

		if(right_moving_mode == 1 && left_moving_mode == 1){
			if((controller.press(RPDS3::CIRCLE)) && !(controller.button(RPDS3::L1))){
				ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,50);
				ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,50);//モータのハンダ付けが違う向きのため同じ符号
				right_moving_mode = 2;
				left_moving_mode = 2;
			}else if(controller.press(RPDS3::CROSS)){
                                ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,-50);
                                ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,-50);
				right_moving_mode = 3;
				left_moving_mode = 3;
			}
		}else{
			//アーム停止
			if((controller.press(RPDS3::CIRCLE) || controller.press(RPDS3::CROSS)) && !(controller.button(RPDS3::L1))){
				right_moving_mode = 1;
				left_moving_mode = 1;
				ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,0);
				ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,0);
				std::cout << "stop\n";
			} 

			//右リミットスイッチの反応
			if(right_moving_mode == 2 && t_arm_limit_right_down == 1){
                                right_moving_mode = 1;
                                ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,0);
				std::cout << "r_limit\n";
			}else if(right_moving_mode == 3 && t_arm_limit_right_up == 1){
                                right_moving_mode = 1;
                                ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,0);
				std::cout << "r_limit\n";
			}
			
			//左リミットスイッチの反応
			if(left_moving_mode == 2 && t_arm_limit_left_down == 1){
                                left_moving_mode = 1;
                                ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,0);
				std::cout << "l_limit\n";
                        }else if(left_moving_mode == 3 && t_arm_limit_left_up == 1){
                                left_moving_mode = 1;
                                ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,0);
				std::cout << "l_limit\n";
                        }

			if(right_moving_mode == 2 && left_moving_mode == 2){
	                        ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,50);
				ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,50);
			}else if(right_moving_mode == 3 && left_moving_mode == 3){
                                ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,-50);
 			        ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,-50);	
			}

		}
	}
	ms.send(MECHANISM_MDD_NUM,Y_ARM,0);
	ms.send(MECHANISM_MDD_NUM,Z_ARM,0);

	ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,0);
	ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,0);
	ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,0);
	ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,0);

	ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,0);
	ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,0);

	gpioWrite(13,false);
	return 0;
}
