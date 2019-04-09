#include<pigpio.h>
#include"PigpioMS/PigpioMS.hpp"
#include"RasPiDS3/RasPiDS3.hpp"
#include"Sensor-master/GY521/GY521.hpp"
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
	constexpr short BOX = 6;
	constexpr short Z_ARM = 5;
	constexpr short Y_ARM = 4;
	//constexpr short PWM_MAX_VALUE = 180;
	constexpr short RIGHT_T_ARM = 2;
	constexpr short LEFT_T_ARM = 3;
	constexpr short TOWEL_SOLENOID = 8;
	//constexpr short STICK_MAX_VALUE = 250;
	constexpr short HANGER_SOLENOID = 7;

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

	std::cout << "Please calibrate (push SELECT and START button) " << std::endl;
	
	UPDATELOOP(controller,!(controller.button(RPDS3::SELECT) && controller.button(RPDS3::START))){
	}

	RPGY521::GY521 gyro;
	std::cout << "Calibration finished" << std::endl;
	std::cout << "Start Main program" << std::endl;

	gyro.start();
	//gyro.resetYaw(0);
	controller.yReverseSet(true);


	UPDATELOOP(controller, !(controller.button(RPDS3::START) && controller.button(RPDS3::RIGHT))){

		//double left_distance = 0;
		//double left_theta = 0;
		//double right_distance = 0;
		double right_theta = 0;

		//double left_front = 0;
		//double left_back  = 0;
		double rotation = 0;

		double left_x = controller.stick(RPDS3::LEFT_X);
		double left_y = controller.stick(RPDS3::LEFT_Y);

		double arms_x;
		double arms_y;

		double right_x = controller.stick(RPDS3::RIGHT_X) * changer;
		double right_y = controller.stick(RPDS3::RIGHT_Y) ;

		//left_distance = std::sqrt(std::pow(left_x,2) + std::pow(left_y,2)) * 2;

		int t_arm_limit_right_up;
		int t_arm_limit_right_down;
		int t_arm_limit_left_up;
		int t_arm_limit_left_down;

		double wheel_velocity[4];
		gyro.updata();

		//double theta = 0;
		//left_theta = std::atan2(-left_y,left_x) + M_PI;

		if(controller.button(RPDS3::LEFT) && controller.press(RPDS3::SELECT)) {
			gyro.resetYaw(0);
			std::cout << "yaw" << std::endl;
		}

		//平行移動
		/*if(left_distance > PWM_MAX_VALUE){
			left_distance = PWM_MAX_VALUE;
		}*/

		double gyro_rad = gyro.yaw * M_PI / 180;

		rotation = (controller.stick(RPDS3::RIGHT_T) - controller.stick(RPDS3::LEFT_T)) * 0.3;//rotation component

		wheel_velocity[0] = -std::sin(M_PI/4 + gyro_rad) * left_x + std::cos(M_PI/4 + gyro_rad) * left_y + rotation;
		wheel_velocity[1] = -std::cos(M_PI/4 + gyro_rad) * left_x + -std::sin(M_PI/4 + gyro_rad) * left_y + rotation;
		wheel_velocity[2] = std::sin(M_PI/4 + gyro_rad) * left_x + -std::cos(M_PI/4 + gyro_rad) * left_y + rotation;
		wheel_velocity[3] = std::cos(M_PI/4 + gyro_rad) * left_x + std::sin(M_PI/4 + gyro_rad) * left_y + rotation;


		ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_FRONT_MOTOR_NUM, wheel_velocity[1] * 0.4 * regulation + rotation);
		ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_BACK_MOTOR_NUM,  wheel_velocity[2] * 0.4 * regulation + rotation);
		ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_FRONT_MOTOR_NUM,wheel_velocity[0] * 0.4 * regulation + rotation);
		ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_BACK_MOTOR_NUM, wheel_velocity[3] * 0.4 * regulation + rotation);

		//ハンガー昇降機（△　）
		if(controller.press(RPDS3::TRIANGLE)){
			if(hanger_flag == true){
				ms.send(MECHANISM_MDD_NUM,HANGER_SOLENOID,1);
				ms.send(MECHANISM_MDD_NUM,HANGER_SOLENOID,2);
				hanger_flag = false;
				std::cout << "up" << std::endl;
			}else{
				ms.send(MECHANISM_MDD_NUM,HANGER_SOLENOID,0);
				hanger_flag = true;
				std::cout << "down" << std::endl;
			}
		}
		//コートチェンジ（SELECT　＋　△　）
		if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::TRIANGLE)){
			if(coat_flag == true){
				changer = -1;
				coat_flag = false;
			}else{
				changer = 1;
				coat_flag = true;
			}
		}


		//回収機構のアーム（右スティック）

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

		t_arm_limit_right_up = gpioRead(12);
		t_arm_limit_right_down = gpioRead(16);
		t_arm_limit_left_up = gpioRead(11);
		t_arm_limit_left_down = gpioRead(22);

		if(t_arm_limit_right_down == 1 && t_arm_limit_left_down == 1){
			if(controller.press(RPDS3::SQUARE)){
				if(box_flag == true){
					ms.send(MECHANISM_MDD_NUM,BOX,2);
					box_flag = false;
				}else{
					ms.send(MECHANISM_MDD_NUM,BOX,0);
					box_flag = true;
				}
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
			}

			if(controller.press(RPDS3::CROSS)){
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
	                        ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,-50);
				ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,-50);
			}else if(right_moving_mode == 3 && left_moving_mode == 3){
                                ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,50);
 			        ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,50);	
			}		

		}
	}

	ms.send(255,255,0);
	gpioWrite(13,false);
	return 0;
}

