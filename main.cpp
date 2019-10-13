#include<pigpio.h>
#include"PigpioMS/PigpioMS.hpp"
#include"RasPiDS3/RasPiDS3.hpp"
#include"Sensor-master/GY521/GY521.hpp"
#include<iostream>
#include<cmath>
#include<unistd.h>

RPMS::MotorSerial ms;
RPDS3::DualShock3 controller;


int main(void){

	//------MDD------//
	constexpr short BOTTOM_MDD = 2;
	constexpr short DOWN_MDD   = 11;
	constexpr short UP_MDD     = 10;
	constexpr short TOP_MDD    = 16;
	constexpr short TAPE_LED   = 12;

	//-----PORT------//
	constexpr short UNC_PORT        = 2;
	constexpr short SOLENOID_PORT   = 4;
	constexpr short ARM_PORT        = 3;

	constexpr short TAPELED = 84;

	constexpr int RIGHT_UP_T_ARM_LIMIT   = 16;//12
	constexpr int RIGHT_DOWN_T_ARM_LIMIT = 12;//16
	constexpr int LEFT_UP_T_ARM_LIMIT    = 22;//11
	constexpr int LEFT_DOWN_T_ARM_LIMIT  = 11;//22

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
	gpioSetMode(27,PI_OUTPUT);

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
	bool coat_flag = true;
	int right_moving_mode = 1;
	int left_moving_mode = 1;
	bool box_permission_flag = false;
	bool triangle_press = false;
	int arm_count = 0;
	bool y_arm_special_mode = false;
	bool coat_chosen = false;
	int solenoid_count = 0;
	bool limit_emergency_flag = false;
	int led_lightning_mode = 0;
	int past_led = 0;
	constexpr int wait = 60;

	std::cout << "Your coat is :: RED -> SELECT and TRIANGLE BLUE -> SELECT and CROSS" << std::endl;

	while(coat_chosen == false){
		controller.update();
		if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::TRIANGLE)){
			coat_flag = true;
			std::cout << "Red coat selected" << std::endl;
			coat_chosen = true;
			ms.send(TAPELED,20,wait);
			ms.send(TAPELED,20,wait);
			ms.send(TAPELED,20,wait);
		}else if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::CROSS)){
			coat_flag = false;
			std::cout << "Blue coat selected" << std::endl;
			coat_chosen = true;
			ms.send(TAPELED,30,wait);
			ms.send(TAPELED,30,wait);
			ms.send(TAPELED,30,wait);
		}
	}

	std::cout << "Please calibrate (push SELECT and START button) " << std::endl;

	UPDATELOOP(controller,!(controller.button(RPDS3::SELECT) && controller.button(RPDS3::START))){
	}

	RPGY521::GY521 gyro;
	std::cout << "Calibration finished" << std::endl;
	std::cout << "Start Main program" << std::endl;

	ms.send(TAPELED,5,wait);

	gyro.start();
	//gyro.resetYaw(0);
	controller.yReverseSet(true);

	UPDATELOOP(controller, !(controller.button(RPDS3::START) && controller.button(RPDS3::RIGHT))){
		if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::SQUARE)){
			sleep_flag = false;
			led_lightning_mode = 1;
			std::cout << "wake up" << std::endl;
		}

		UPDATELOOP(controller, sleep_flag == false){
			past_led = led_lightning_mode;
			if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::SQUARE)){
				sleep_flag = true;
				std::cout << "zzz" << std::endl;
				ms.send(TAPELED,40,wait);
				ms.send(TAPELED,40,wait);
				ms.send(TAPELED,40,wait);
				break;
			}else{
				gpioWrite(27,true);
			}

			double left_x = controller.stick(RPDS3::LEFT_X);
			double left_y = controller.stick(RPDS3::LEFT_Y);
			double wheel_velocity[4];
			gyro.updata();

			if(controller.press(RPDS3::LEFT) && controller.button(RPDS3::SELECT)) {
				gyro.resetYaw(0);
				std::cout << "yaw" << std::endl;
				led_lightning_mode = 5;
			}

			double gyro_rad = gyro.yaw * M_PI / 180;
			double rotation = (controller.stick(RPDS3::RIGHT_T) - controller.stick(RPDS3::LEFT_T)) * 0.6;//rotation component

			wheel_velocity[0] = -std::sin(M_PI/4 + gyro_rad) * left_x + std::cos(M_PI/4 + gyro_rad) * left_y + rotation;
			wheel_velocity[1] = -std::cos(M_PI/4 + gyro_rad) * left_x + -std::sin(M_PI/4 + gyro_rad) * left_y + rotation;
			wheel_velocity[2] = std::sin(M_PI/4 + gyro_rad) * left_x + -std::cos(M_PI/4 + gyro_rad) * left_y + rotation;
			wheel_velocity[3] = std::cos(M_PI/4 + gyro_rad) * left_x + std::sin(M_PI/4 + gyro_rad) * left_y + rotation;


			ms.send(BOTTOM_MDD, UNC_PORT , -wheel_velocity[1] * 0.8 * regulation + rotation * 1.2);
			ms.send(DOWN_MDD,   UNC_PORT , -wheel_velocity[2] * 0.8 * regulation + rotation * 1.2);
			ms.send(UP_MDD,     UNC_PORT  , -wheel_velocity[0] * 0.8 * regulation + rotation * 1.2);
			ms.send(TOP_MDD,    UNC_PORT , -wheel_velocity[3] * 0.8 * regulation + rotation * 1.2);

			//std::cout << "[0]"  << -wheel_velocity[1] * 0.8 * regulation + rotation * 1.2 << std::endl;
			//std::cout << "[1]"  << -wheel_velocity[2] * 0.8 * regulation + rotation * 1.2 << std::endl;
			//std::cout << "[2]"  << -wheel_velocity[0] * 0.8 * regulation + rotation * 1.2 << std::endl;
			//std::cout << "[3]"  << -wheel_velocity[3] * 0.8 * regulation + rotation * 1.2 << std::endl;




			//---------------------ハンガー昇降機（▲　）----------------------------------------//
			if(controller.press(RPDS3::SQUARE) && !(controller.button(RPDS3::SELECT))){
				if(hanger_flag == true){
					ms.send(UP_MDD, SOLENOID_PORT, 251);
					ms.send(UP_MDD, SOLENOID_PORT, 252);
					hanger_flag = false;
					std::cout << "up" << std::endl;
					led_lightning_mode = 6;
				}else{
					ms.send(UP_MDD, SOLENOID_PORT, 0);
					hanger_flag = true;
					std::cout << "down" << std::endl;
					led_lightning_mode = 7;
				}
			}

			//---------------コートチェンジ-------------------------------//
			if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::TRIANGLE)){
				coat_flag = !(coat_flag);
				if(coat_flag == true){
					led_lightning_mode = 2;
				}else{
					led_lightning_mode = 3;
				}
			}

			if(coat_flag == true){
				changer = -1;
			}else{
				changer = 1;
			}

			if(controller.button(RPDS3::L1)){
				if(coat_flag == true){
					changer = 1;
				}else{
					changer = -1;
				}
			}

			//------------------------------------------------------------------------//

			//-------------------回収機構のアーム（右スティック）---------------------------------//

			bool pochama_limit_y_front;
			bool pochama_limit_y_back;
			bool pochama_limit_z_up;
			bool pochama_limit_z_down;

			if(controller.button(RPDS3::SELECT) && controller.press(RPDS3::CIRCLE)){
				limit_emergency_flag = !(limit_emergency_flag);
			}

			if(limit_emergency_flag == true){
				pochama_limit_y_front = false;//trueの時おされてない
				pochama_limit_y_back  = false;
				pochama_limit_z_up    = false;
				pochama_limit_z_down  = false;
				std::cout << "em" << std::endl;
			}else{
				pochama_limit_y_front = gpioRead(Y_FRONT_TAIL_LIMIT);
				pochama_limit_y_back  = gpioRead(Y_BACK_TAIL_LIMIT);
				pochama_limit_z_up    = gpioRead(Z_UP_TAIL_LIMIT);
				pochama_limit_z_down  = gpioRead(Z_DOWN_TAIL_LIMIT);
			}

			double right_x = controller.stick(RPDS3::RIGHT_X) * changer;
			double right_y = controller.stick(RPDS3::RIGHT_Y) ;
			double right_theta = std::atan2(right_y,right_x);
			double right_distance = std::sqrt(std::pow(right_x,2) + std::pow(right_y,2)) * 2;

			double sent_y;
			double sent_z;

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
			}else{

				if(y_tail_mode == false){
					sent_y = 0;
				}

				if(z_tail_mode == false){
					sent_z = 0;
				}
			}

			if(pochama_limit_y_back == true && sent_y > 0){
				sent_y = 0;
				y_tail_mode = false;
				//std::cout << "y_back" << std::endl;
			}else if(pochama_limit_y_front == true && sent_y < 0){
				sent_y = 0;
				y_tail_mode = false;
				//std::cout << " y_front" << std::endl;
			}

			if(y_tail_mode == true){
				if(sent_y > 0){
					sent_y = 200;
				}else if(sent_y < 0){
					sent_y = -200;
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
						led_lightning_mode = 12;
					}else if(controller.press(RPDS3::LEFT)){
						sent_y = -150 * changer;
						y_tail_mode = true;
						led_lightning_mode = 12;
					}
				}
			}

			if(pochama_limit_z_down == true && sent_z < 0){
				sent_z = 0;
				z_tail_mode = false;
			}else if(pochama_limit_z_up == true && sent_z > 0){
				sent_z = 0;
				z_tail_mode = false;
			}

			if(z_tail_mode == true){
				if(sent_z > 0){
					sent_z = 200;
				}else if(sent_z < 0){
					sent_z = -200;
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
						led_lightning_mode = 11;
					}else if(controller.press(RPDS3::DOWN)){
						sent_z = 200;
						z_tail_mode = true;
						led_lightning_mode = 11;
					}
				}

			}

			ms.send(TOP_MDD, ARM_PORT, sent_y * regulation);
			ms.send(UP_MDD,  ARM_PORT, sent_z * regulation );


			//回収機構の箱
			bool  t_arm_limit_right_up = gpioRead(RIGHT_UP_T_ARM_LIMIT);
			bool  t_arm_limit_right_down = gpioRead(RIGHT_DOWN_T_ARM_LIMIT);
			bool  t_arm_limit_left_up = gpioRead(LEFT_UP_T_ARM_LIMIT);
			bool  t_arm_limit_left_down = gpioRead(LEFT_DOWN_T_ARM_LIMIT);

			if(limit_emergency_flag == false){
				if(controller.press(RPDS3::TRIANGLE) && !(controller.button(RPDS3::SELECT))){
					if(t_arm_limit_right_down == false){
						right_moving_mode = 2;
					}

					if(t_arm_limit_left_down == false){
						left_moving_mode = 2;
					}

					if(pochama_limit_z_down == false){
						z_tail_mode = true;
						sent_z = -200;
					}

					//if(arm_count < 15){
					sent_y = 120;
					y_tail_mode = true;
					y_arm_special_mode = true;
					//}

					triangle_press = true;
				}
			}else{
				if(controller.button(RPDS3::TRIANGLE)){
					ms.send(TOP_MDD,SOLENOID_PORT,251);
				}else{
					ms.send(TOP_MDD,SOLENOID_PORT,0);
				}
			}


			if(arm_count >= 30){
				sent_y = 0;
				y_tail_mode = false;
				y_arm_special_mode = false;
			}



			//std::cout << arm_count << std::endl;


			if((t_arm_limit_right_down == true && t_arm_limit_left_down == true) && (pochama_limit_z_down == true && arm_count >= 15)){
				box_permission_flag = true;
				arm_count = 0;
				y_arm_special_mode = false;
			}

			if(y_arm_special_mode == true){
				++arm_count;
			}

			//std::cout << "triangle_press:" << triangle_press << std::endl;
			//std::cout << "solenoid_count" << solenoid_count << std::endl;

			if(triangle_press == true && box_permission_flag == true){
				if(solenoid_count <= 15){
					ms.send(TOP_MDD,SOLENOID_PORT,251);
					++solenoid_count;
					led_lightning_mode = 10;
				}else{
					ms.send(TOP_MDD,SOLENOID_PORT,0);
					solenoid_count = 0;
					triangle_press = false;
					box_permission_flag = false;
					arm_count = 0;
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
				ms.send(DOWN_MDD,SOLENOID_PORT,251);
				ms.send(DOWN_MDD,SOLENOID_PORT,252);

			}else{
				ms.send(DOWN_MDD,SOLENOID_PORT,0);
			}

			//バスタオルのアーム

			/*
			   mode 1 -> stop
			   mode 2 -> up
			   mode 3 -> down
			   */

			if(right_moving_mode == 1 && left_moving_mode == 1){
				if((controller.press(RPDS3::CIRCLE)) && !(controller.button(RPDS3::L1))){
					ms.send(DOWN_MDD,   ARM_PORT,50);//右
					ms.send(BOTTOM_MDD, ARM_PORT,50);//モータのハンダ付けが違う向きのため同じ符号
					led_lightning_mode = 8;
					right_moving_mode = 2;
					left_moving_mode = 2;

				}

				if(controller.press(RPDS3::CROSS)){
					ms.send(DOWN_MDD   ,ARM_PORT,-50);
					ms.send(BOTTOM_MDD ,ARM_PORT,-50);
					led_lightning_mode = 9;
					right_moving_mode = 3;
					left_moving_mode = 3;
				}
			}else{

				//アーム停止
				if((controller.press(RPDS3::CIRCLE) || controller.press(RPDS3::CROSS)) && !(controller.button(RPDS3::L1))){
					right_moving_mode = 1;
					left_moving_mode = 1;
					ms.send(DOWN_MDD   ,ARM_PORT,0);
					ms.send(BOTTOM_MDD ,ARM_PORT,0);
					std::cout << "stop\n";
				}

				if(limit_emergency_flag == true){
					if(controller.button(RPDS3::CIRCLE)){
						ms.send(DOWN_MDD,ARM_PORT,-150);
						ms.send(BOTTOM_MDD,ARM_PORT,-150);
					}else if(controller.button(RPDS3::CROSS)){
						ms.send(DOWN_MDD,ARM_PORT,150);
						ms.send(BOTTOM_MDD,ARM_PORT,150);
					}else{
						ms.send(DOWN_MDD,ARM_PORT,0);
						ms.send(BOTTOM_MDD,ARM_PORT,0);
					}
				}else{

					//右リミットスイッチの反応
					if(right_moving_mode == 2 && t_arm_limit_right_down == true){
						right_moving_mode = 1;
						ms.send(DOWN_MDD,ARM_PORT,0);
						std::cout << "r_limit\n";
					}else if(right_moving_mode == 3 && t_arm_limit_right_up == true){
						right_moving_mode = 1;
						ms.send(DOWN_MDD,ARM_PORT,0);
						std::cout << "rrr_limit\n";
					}

					//左リミットスイッチの反応
					if(left_moving_mode == 2 && t_arm_limit_left_down == true){
						left_moving_mode = 1;
						ms.send(BOTTOM_MDD,ARM_PORT,0);
						std::cout << "l_limit\n";
					}else if(left_moving_mode == 3 && t_arm_limit_left_up == true){
						left_moving_mode = 1;
						ms.send(BOTTOM_MDD,ARM_PORT,0);
						std::cout << "lll_limit\n";
					}



					if(right_moving_mode == 2 && left_moving_mode == 2){
						ms.send(DOWN_MDD,   ARM_PORT,-150);
						ms.send(BOTTOM_MDD, ARM_PORT,-150);
					}else if(right_moving_mode == 3 && left_moving_mode == 3){
						ms.send(DOWN_MDD,   ARM_PORT,150);
						ms.send(BOTTOM_MDD, ARM_PORT,150);
					}
				}

			}


			//std::cout << led_lightning_mode << std::endl;
			if(past_led != led_lightning_mode){
				//std::cout << led_lightning_mode << std::endl;
				switch(led_lightning_mode){
					case 1:
						ms.send(TAPELED,10,wait);
						ms.send(TAPELED,10,wait);
						ms.send(TAPELED,10,wait);
						break;
					case 2:
						ms.send(TAPELED,20,wait);
						ms.send(TAPELED,20,wait);
						ms.send(TAPELED,20,wait);
						break;
					case 3:
						ms.send(TAPELED,30,wait);
						ms.send(TAPELED,30,wait);
						ms.send(TAPELED,30,wait);
						break;
					case 4:
						ms.send(TAPELED,40,wait);
						ms.send(TAPELED,40,wait);
						ms.send(TAPELED,40,wait);
						break;
					case 5:
						ms.send(TAPELED,50,wait);
						ms.send(TAPELED,50,wait);
						ms.send(TAPELED,50,wait);
						break;
					case 6:
						ms.send(TAPELED,60,wait);
						ms.send(TAPELED,60,wait);
						ms.send(TAPELED,60,wait);
						break;
					case 7:
						ms.send(TAPELED,70,wait);
						ms.send(TAPELED,70,wait);
						ms.send(TAPELED,70,wait);
						break;
					case 8:
						ms.send(TAPELED,80,100);
						ms.send(TAPELED,80,100);
						ms.send(TAPELED,80,100);
						break;
					case 9:
						ms.send(TAPELED,90,100);
						ms.send(TAPELED,90,100);
						ms.send(TAPELED,90,100);
						break;
					case 10:
						ms.send(TAPELED,100,wait);
						ms.send(TAPELED,100,wait);
						ms.send(TAPELED,100,wait);
						break;
					case 11:
						ms.send(TAPELED,110,wait);
						ms.send(TAPELED,110,wait);
						ms.send(TAPELED,110,wait);
						break;
					case 12:
						ms.send(TAPELED,120,wait);
						ms.send(TAPELED,120,wait);
						ms.send(TAPELED,120,wait);
						break;
						/*default:
						  ms.send(TAPELED,120,wait);
						  break;*/
				}
			}
			//usleep(100);
		}
		//usleep(10000);
		ms.send(255,255,0);
	}
	gpioWrite(27,false);
	ms.send(255,255,0);
	gpioWrite(13,false);
	ms.send(TAPELED,200,40);
	return 0;
}

