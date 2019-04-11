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

	constexpr int RIGHT_UP_T_ARM_LIMIT   = 12;
	constexpr int RIGHT_DOWN_T_ARM_LIMIT = 16;
	constexpr int LEFT_UP_T_ARM_LIMIT    = 11;
	constexpr int LEFT_DOWN_T_ARM_LIMIT  = 22;

	constexpr int Y_FRONT_TAIL_LIMIT = 26;
	constexpr int Y_BACK_TAIL_LIMIT = 19;
	constexpr int Z_UP_TAIL_LIMIT = 9;
	constexpr int Z_DOWN_TAIL_LIMIT = 10;
	//constexpr short POWER_WINDOW_MOTOR_NUM = 4;

	bool y_tail_mode = false;
	bool z_tail_mode = false;
	bool sleep_flag = false;

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

	gpioSetMode(RIGHT_UP_T_ARM_LIMIT,PI_INPUT);
	gpioSetPullUpDown(RIGHT_UP_T_ARM_LIMIT,PI_PUD_UP);
	gpioSetMode(RIGHT_DOWN_T_ARM_LIMIT,PI_INPUT);
	gpioSetPullUpDown(RIGHT_DOWN_T_ARM_LIMIT,PI_PUD_UP);
	gpioSetMode(LEFT_UP_T_ARM_LIMIT,PI_INPUT);
	gpioSetPullUpDown(LEFT_UP_T_ARM_LIMIT,PI_PUD_UP);
	gpioSetMode(LEFT_DOWN_T_ARM_LIMIT,PI_INPUT);
	gpioSetPullUpDown(LEFT_DOWN_T_ARM_LIMIT,PI_PUD_UP);

	gpioSetMode(Y_FRONT_TAIL_LIMIT,PI_INPUT);
	gpioSetPullUpDown(Y_FRONT_TAIL_LIMIT,PI_PUD_UP);
	gpioSetMode(Y_BACK_TAIL_LIMIT,PI_INPUT);
	gpioSetPullUpDown(Y_BACK_TAIL_LIMIT,PI_PUD_UP);
	gpioSetMode(Z_UP_TAIL_LIMIT,PI_INPUT);
	gpioSetPullUpDown(Z_UP_TAIL_LIMIT,PI_PUD_UP);
	gpioSetMode(Z_DOWN_TAIL_LIMIT,PI_INPUT);
	gpioSetPullUpDown(Z_DOWN_TAIL_LIMIT,PI_PUD_UP);


	bool hanger_flag = true;
	bool box_flag = true;
	bool coat_flag = true;
	int right_moving_mode = 1;
	int left_moving_mode = 1;
	
	std::cout << "Your coat is :: RED -> SELECT and TRIANGLE BLUE -> SELECT and CROSS" << std::endl;
	if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::TRIANGLE)){
		coat_flag = true;
		std::cout << "Red coat selected" << std::endl;
	}else if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::CROSS)){
		coat_flag = false;
		std::cout << "Blue coat selected" << std::endl;
	}

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
		if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::SQUARE)){
			sleep_flag = false;
			std::cout << "wake up" << std::endl;
		}
		UPDATELOOP(controller, sleep_flag == false){
			if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::SQUARE)){
				sleep_flag = true;
				std::cout << "zzz" << std::endl;
				break;
			}

			//double left_distance = 0;
			//double left_theta = 0;
			//double right_distance = 0;
			double right_theta = 0;

			//double left_front = 0;
			//double left_back  = 0;
			double rotation = 0;

			double left_x = controller.stick(RPDS3::LEFT_X);
			double left_y = controller.stick(RPDS3::LEFT_Y);

			double sent_y;
			double sent_z;

			double right_x = controller.stick(RPDS3::RIGHT_X) * changer;
			double right_y = controller.stick(RPDS3::RIGHT_Y) ;

			//left_distance = std::sqrt(std::pow(left_x,2) + std::pow(left_y,2)) * 2;
			double right_distance = std::sqrt(std::pow(right_x,2) + std::pow(right_y,2)) * 2;

			bool t_arm_limit_right_up;
			bool t_arm_limit_right_down;
			bool t_arm_limit_left_up;
			bool t_arm_limit_left_down;

			bool pochama_limit_y_front;
			bool pochama_limit_y_back;
			bool pochama_limit_z_up;
			bool pochama_limit_z_down;

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


			ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_FRONT_MOTOR_NUM, wheel_velocity[1] * 0.55 * regulation + rotation);
			ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_BACK_MOTOR_NUM,  wheel_velocity[2] * 0.55 * regulation + rotation);
			ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_FRONT_MOTOR_NUM,wheel_velocity[0] * 0.55 * regulation + rotation);
			ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_BACK_MOTOR_NUM, wheel_velocity[3] * 0.55 * regulation + rotation);

			//ハンガー昇降機（△　）
			if(controller.press(RPDS3::SQUARE)){
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
			//コートチェンジ（L1長押し）
			if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::TRIANGLE)){
				coat_flag = !(coat_flag);
			}

			if(coat_flag == true){
				changer = 1;
			}else{
				changer = -1;
			}

			if(controller.button(RPDS3::L1)){
				if(coat_flag == true){
					changer = -1;
				}else{
					changer = 1;
				}
			}

			//std::cout << changer << std::endl;
			//回収機構のアーム（右ステ
			pochama_limit_y_front = gpioRead(Y_FRONT_TAIL_LIMIT);
			pochama_limit_y_back  = gpioRead(Y_BACK_TAIL_LIMIT);
			pochama_limit_z_up    = gpioRead(Z_UP_TAIL_LIMIT);
			pochama_limit_z_down  = gpioRead(Z_DOWN_TAIL_LIMIT);

			/* z_tail_mode 及び　y_tail_modeは、モーターの動きが
			 * */
			right_theta = std::atan2(right_y,right_x);


			if(right_distance >= 20){
				if(right_theta >= (M_PI/4) && right_theta <= (M_PI/4) * 3){
					sent_y = 0; 
					sent_z = -right_y;
				}else if(right_theta > (M_PI/4)*3 || right_theta < -(M_PI/4)*3){
					sent_y = right_x;
					sent_z = 0;
				}else if(right_theta >= -(M_PI/4)*3 && right_theta <= -(M_PI/4)){
					sent_y = 0;
					sent_z = -right_y;
				}else if(right_theta > -(M_PI/4)  && right_theta < (M_PI/4)){
					sent_y = right_x;
					sent_z = 0;
				}
			}
			if(right_distance < 20 && y_tail_mode == false){
				sent_y = 0;
			}

			if(right_distance < 20 && z_tail_mode == false){
				sent_z = 0;
			}

			if(y_tail_mode == true){
				if(sent_y > 0){
					sent_y = 200;
				}else if(sent_y < 0){
					sent_y = -200;
				}
			}

			if(z_tail_mode == true){
				if(sent_z > 0){
					sent_z = 200;
				}else if(sent_z < 0){
					sent_z = -200;
				}
			}

			if(y_tail_mode == true){
				if((right_x != 0 || right_y != 0) || ((controller.press(RPDS3::RIGHT)) || controller.press(RPDS3::LEFT))){
					sent_y = 0;
					y_tail_mode = false;
				}
			}else{
				if(right_x == 0){
					if(controller.press(RPDS3::RIGHT)){
						sent_y =  120 * changer;
						y_tail_mode = true;
					}else if(controller.press(RPDS3::LEFT)){
						sent_y = -150 * changer;
						y_tail_mode = true;
					}
				}
			}

			if(z_tail_mode == true){
				if((right_x != 0 || right_y != 0) || ((controller.press(RPDS3::UP)) || controller.press(RPDS3::DOWN))){
					sent_z = 0;
					z_tail_mode = false;
				}
			}else{
				if(right_y == 0){
					if(controller.press(RPDS3::UP)){
						sent_z =  -250;
						z_tail_mode = true;
					}else if(controller.press(RPDS3::DOWN)){
						sent_z = 200;
						z_tail_mode = true;
					}
				}

			}
			if(pochama_limit_y_back == true && sent_y >= 0){
				sent_y = 0;
				y_tail_mode = false;
			}else if(pochama_limit_y_front == true && sent_y <= 0){
				sent_y = 0;
				y_tail_mode = false;
			}

			if(pochama_limit_z_down == true && sent_z <= 0){
				sent_z = 0;
				std::cout << "limit" << std::endl;
			}else if(pochama_limit_z_up == true && sent_z >= 0){
				sent_z = 0;
				z_tail_mode = false;
			}

			ms.send(MECHANISM_MDD_NUM,Y_ARM, sent_y * regulation);
			ms.send(MECHANISM_MDD_NUM,Z_ARM, sent_z * regulation );

			//回収機構の箱

			t_arm_limit_right_up = gpioRead(RIGHT_UP_T_ARM_LIMIT);
			t_arm_limit_right_down = gpioRead(RIGHT_DOWN_T_ARM_LIMIT);
			t_arm_limit_left_up = gpioRead(LEFT_UP_T_ARM_LIMIT);
			t_arm_limit_left_down = gpioRead(LEFT_DOWN_T_ARM_LIMIT);

			if(t_arm_limit_right_down == true && t_arm_limit_left_down == true){
				if(controller.press(RPDS3::TRIANGLE)){
					if(box_flag == true){
						ms.send(MECHANISM_MDD_NUM,BOX,2);
						box_flag = false;
						std::cout << "on" << std::endl;
					}else{
						ms.send(MECHANISM_MDD_NUM,BOX,0);
						box_flag = true;
						std::cout << "off" << std::endl;
					}
				}
			}else{
				ms.send(MECHANISM_MDD_NUM,BOX,0);
				box_flag = true;
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
				if(right_moving_mode == 2 && t_arm_limit_right_down == true){
					right_moving_mode = 1;
					ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,0);
					std::cout << "r_limit\n";
				}else if(right_moving_mode == 3 && t_arm_limit_right_up == 1){
					right_moving_mode = 1;
					ms.send(BATH_TOWEL_MDD_NUM,RIGHT_T_ARM,0);
					std::cout << "r_limit\n";
				}

				//左リミットスイッチの反応
				if(left_moving_mode == 2 && t_arm_limit_left_down == true){
					left_moving_mode = 1;
					ms.send(BATH_TOWEL_MDD_NUM,LEFT_T_ARM,0);
					std::cout << "l_limit\n";
				}else if(left_moving_mode == 3 && t_arm_limit_left_up == true){
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
	}

	ms.send(255,255,0);
	gpioWrite(13,false);
	return 0;
}
