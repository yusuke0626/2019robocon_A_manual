#include<iostream>
#include<cmath>
#include<pigpio.h>
#include"PigpioMS/PigpioMS.hpp"
#include"RasPiDS3/RasPiDS3.hpp"

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
	constexpr short PWM_MAX_VALUE = 150;
	//constexpr short STICK_MAX_VALUE = 250;
	constexpr short HANGER_LEFT_SOLENOID = 3;
	constexpr short HANGER_RIGHT_SOLENOID = 3;
	//constexpr short POWER_WINDOW_MOTOR_NUM = 4;

	double regulation = 0.3;

	bool control_mode_flag = true;

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
	int x=0;
	int y;
	int a;

	UPDATELOOP(controller, !(controller.button(RPDS3::START) && controller.button(RPDS3::RIGHT))){

		double left_distance = 0;
		double left_theta = 0;

		//double right_distance = 0;
		//double right_theta = 0;

		double left_front = 0;
		double left_back  = 0;
		int revolve = 0;

		double left_x = controller.stick(RPDS3::LEFT_X);
		double left_y = controller.stick(RPDS3::LEFT_Y);
		//double right_x = controller.stick(RPDS3::RIGHT_X);
		double right_y = controller.stick(RPDS3::RIGHT_Y);
		left_distance = std::sqrt(std::pow(left_x,2) + std::pow(left_y,2)) * 2;

		if(controller.press(RPDS3::R1) == true && controller.press(RPDS3::L1) == true){
			if(control_mode_flag == true){
				control_mode_flag = false;
			}else{
				control_mode_flag = true;
				std::cout << "mode" << std::endl;
			}
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
				left_hanger_flag = true;
			}else{
				ms.send(MECHANISM_MDD_NUM,HANGER_RIGHT_SOLENOID,-262);
				left_hanger_flag = true;
			}
		}

		if(controller.button(RPDS3::UP)){
            if(x<100){
                y=1/2*x;
            }else{
                y=50;
            }
			ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,y);
            ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,y);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,-y);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,-y);
			std::cout<<front<<stdm:endl;
		}else{
			ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,0);
		}else if(controller.button(RPDS3::DOWN)){
			if(x<100){
                y=1/2*x;
            }else{
                y=50;
            }
			ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,-y);
            ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,-y);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,y);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,y);
			std::cout<<back<<std::endl;
		}else{
			ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,0);
		}

		if(controller.button(RPDS3::RIGHT)){
			if(x<100){
                y=1/2*x;
            }else{
                y=50;
            }
			ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,-y);
            ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,y);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,-y);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,y);
			std::cout<<right<<std::endl;
		}else{
	    ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,0);
			
			
		}else if(controller.button(RPDS3::LEFT)){
			if(x<100){
                y=1/2*x;
            }else{
                y=50;
            }
			ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,y);
            ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,-y);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,y);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,-y);
			std::cout<<left<<std::endl;
		}else{
			ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_FRONT_MOTOR_NUM,0);
		    ms.send(UNDERCARRIAGE_MDD_NUM,RIGHT_BACK_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_FRONT_MOTOR_NUM,0);
            ms.send(UNDERCARRIAGE_MDD_NUM,LEFT_BACK_MOTOR_NUM,0);
        }

		if(controller.button(RPDS3::TRIANGLE)){
			ms.send(MECHANISM_MDD_NUM,Z_ARM,y);
		}else if(controller.button(RPDS3::CROSS)){
			ms.send(MECHANISM_MDD_NUM,Z_ARM,-y);
		}else{
			ms.send(MECHANISM_MDD_NUM,Z_ARM,0);
		}


		if(control_mode_flag == true){
			if(left_distance > PWM_MAX_VALUE){
				left_distance = PWM_MAX_VALUE;
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


			revolve = (controller.stick(RPDS3::RIGHT_T) - controller.stick(RPDS3::LEFT_T)) * 0.3;

			//std::cout << -left_distance <<"::"<< left_distance<< std::endl;
			std::cout << left_front<< std::endl;
			ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_FRONT_MOTOR_NUM, -left_distance * left_front * 0.4 * regulation + revolve);//左前
			ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_BACK_MOTOR_NUM,  -left_distance * left_back  * 0.4 * regulation + revolve);//左後
			ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_FRONT_MOTOR_NUM, left_distance * left_back  * 0.4 * regulation + revolve);//右前
			ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_BACK_MOTOR_NUM,  left_distance * left_front * 0.4 * regulation + revolve);//右後 
		}else{
			ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_FRONT_MOTOR_NUM,  -left_y * 0.5 * regulation);
		      	ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_BACK_MOTOR_NUM,   -left_y * 0.5 * regulation);
		      	ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_FRONT_MOTOR_NUM, right_y * 0.5 * regulation);
		      	ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_BACK_MOTOR_NUM,  right_y * 0.5 * regulation);
		}

		//revolve = (controller.stick(RPDS3::LEFT_T) - controller.stick(RPDS3::RIGHT_T)) * 0.3;

                        //std::cout << -right_distance <<"::"<< right_distance<< std::endl;
                        //std::cout << right_front<< std::endl;
                        //ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_FRONT_MOTOR_NUM, -right_distance * right_front * 0.4 * regulation + revolve);//左前
                        //ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_BACK_MOTOR_NUM,  -right_distance * right_back  * 0.4 * regulation + revolve);//左後
                        //ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_FRONT_MOTOR_NUM, right_distance * right_back  * 0.4 * regulation + revolve);//右前
                        //ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_BACK_MOTOR_NUM,  right_distance * right_front * 0.4 * regulation + revolve);//右後 
                //}else{
                        //ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_FRONT_MOTOR_NUM,  -right_y * 0.5 * regulation);
                        //ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_BACK_MOTOR_NUM,   -right_y * 0.5 * regulation);
                        //ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_FRONT_MOTOR_NUM, left_y * 0.5 * regulation);
                        //ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_BACK_MOTOR_NUM,  left_y * 0.5 * regulation);
                //}


	

		if(controller.button(RPDS3::R1) == true){
			regulation = 0.5;
			std::cout << "reg" << std::endl;
		}else{
			regulation = 1.0;
		}
		x++;

	}
	gpioWrite(13,false);
	return 0;
}
