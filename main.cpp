#include<iostream>
#include<cmath>
#include<pigpio.h>
#include"PigpioMS/PigpioMS.hpp"
#include"RasPiDS3/RasPiDS3.hpp"

RPMS::MotorSerial ms;
RPDS3::DualShock3 controller;


int main(void){
	constexpr short UNDERCARRIAGE_MDD_NUM = 16; /*足回りのモータードライバドライバの番号*/
	constexpr short MECHANISM_MDD_NUM = 10; /*機構のモータードライバドライバの番号*/
	constexpr short RIGHT_FRONT_MOTOR_NUM = 2; /*足回り右前のタイヤの番号*/
	constexpr short RIGHT_BACK_MOTOR_NUM  = 3; /*足回り右後ろのタイヤの番号*/
	constexpr short LEFT_FRONT_MOTOR_NUM  = 4; /*足回り左前のタイヤの番号*/
	constexpr short LEFT_BACK_MOTOR_NUM   = 5; /*足回り左後ろのタイヤの番号*/
	constexpr short BOX = 2; /*回収機構の番号*/
	constexpr short Z_ARM = 5; /*回収機構のZ軸方向のモータ番号*/
	constexpr short Y_ARM = 4; /*回収機構のY軸方向のモータ番号*/
	constexpr short PWM_MAX_VALUE = 150; /*出力(PWM)の最大値*/
	//constexpr short STICK_MAX_VALUE = 250; /*コントローラスティックの最大値*/
	constexpr short HANGER_LEFT_SOLENOID = 3; /*ハンガー機構左ソレノイドの番号*/
	constexpr short HANGER_RIGHT_SOLENOID = 3; /*ハンガー機構右ソレノイドの番号*/
	//constexpr short POWER_WINDOW_MOTOR_NUM = 4;

	double regulation = 0.3;/*出力を1/3にする*/

	//bool control_mode_flag = true; /*コントローラのモードはON*/

	controller.update(); /*コントローラをアップデートする(通信する)*/
	try{
		ms.init(); /*アップデートを試みる*/
	}catch(std::runtime_error exception){
		std::cout << "error" << std::endl; /*コントローラをアップデートできなければ"error"と表示*/
		return -1;  /*-1を返す*/
	}

	gpioSetMode(13,PI_OUTPUT);
	gpioWrite(13,true);

	bool right_hanger_flag = true; /*右のハンガーをONにする*/
	bool left_hanger_flag = true; /*左のハンガーをONにする*/

	UPDATELOOP(controller, !(controller.button(RPDS3::START) && controller.button(RPDS3::RIGHT))){ /*コントローラの"スタート"と"右"が両方押されていればコントローラのアップデートループ(通信)を開始する*/
		
		double left_distance = 0; /*コントローラの左スティックの動く距離*/
		double left_theta = 0; /*コントローラの左スティックの動いた角度*/

		double right_distance = 0; /*コントローラの右スティックの動く距離*/
		double right_theta = 0; /*コントローラの右スティックの動いた角度*/

		double left_front = 0; /**/
		double left_back  = 0; /**/
		int revolve = 0; /*ロボットの回転数*/

		double left_x = controller.stick(RPDS3::LEFT_X); /*コントローラの左スティックの動いたX軸値*/
		double left_y = controller.stick(RPDS3::LEFT_Y); /*コントローラの左スティックの動いたY軸値*/
		double right_x = controller.stick(RPDS3::RIGHT_X); /*コントローラの右スティックの動いたX軸値*/
		double right_y = controller.stick(RPDS3::RIGHT_Y); /*コントローラの右スティックの動いたY軸値*/
		left_distance = std::sqrt(std::pow(left_x,2) + std::pow(left_y,2)) * 2; /*コントローラの左スティックの動いた距離*/
		right_distance = std::sqrt(std::pow(right_x,2) + std::pow(right_y,2)) * 2; /*コントローラの右スティックの動いた距離*/

		//if(controller.press(RPDS3::R1) == true && controller.press(RPDS3::L1) == true){ /*コントローラの"R1"と"L1"が押されている時*/
			//if(control_mode_flag == true){ /*コントローラのモードがONの時*/
				//control_mode_flag = true; /*コントローラのモードをONにする*/
			//}else{
				//control_mode_flag = true; /*コントローラのモードをONにする*/
				//std::cout << "mode" << std::endl; /*"mode"と表示する*/
			//}
		//}

		if(controller.press(RPDS3::SQUARE)){ /*"□"が押されたら*/
			if(right_hanger_flag == true){ /*右ハンガーがONの時*/
				ms.send(MECHANISM_MDD_NUM,HANGER_LEFT_SOLENOID,261);
				right_hanger_flag = false; /*右ハンガーをOFFにする*/
			}else{ /*右ハンガーがOFFの時*/
				ms.send(MECHANISM_MDD_NUM,HANGER_LEFT_SOLENOID,-261);
				right_hanger_flag = true; /*右ハンガーをONにする*/
			}
		}

		if(controller.press(RPDS3::CIRCLE)){ /*"○"が押されたら*/
			if(left_hanger_flag == true){ /*左ハンガーがONの時*/
				ms.send(MECHANISM_MDD_NUM,HANGER_RIGHT_SOLENOID,262);
				left_hanger_flag = false; /*左ハンガーをOFFにする*/
			}else{ /*左ハンガーがOFFの時*/
				ms.send(MECHANISM_MDD_NUM,HANGER_RIGHT_SOLENOID,-262);
				left_hanger_flag = true; /*左ハンガーをONにする*/
			}
		}

		//if(controller.button(RPDS3::UP)){ /*"UP"が押されたら*/
			//ms.send(MECHANISM_MDD_NUM,BOX,50); /*ボックスを押し出す*/
		//}else if(controller.button(RPDS3::DOWN)){ /*"DOWN"が押されたら*/
			//ms.send(MECHANISM_MDD_NUM,BOX,-50); /*ボックスを元に戻す*/
		//}else{ /*何も押されていなければ*/
			//ms.send(MECHANISM_MDD_NUM,BOX,0); /*ボックスは動かさない*/
		//}

		//if(controller.button(RPDS3::RIGHT)){ /*"RIGHT"が押されたら*/
			//ms.send(MECHANISM_MDD_NUM,Y_ARM,50); /*ボックスを前に出す*/
		//}else if(controller.button(RPDS3::LEFT)){ /*"LEFT"が押されたら*/
			//ms.send(MECHANISM_MDD_NUM,Y_ARM,-50); /*ボックスを後ろに引く*/
		//}else{ /*何も押されてなかったら*/
			//ms.send(MECHANISM_MDD_NUM,Y_ARM,0); /*ボックスを動かさない*/
		//}

		//if(controller.button(RPDS3::TRIANGLE)){ /*"△"が押されたら*/
			//ms.send(MECHANISM_MDD_NUM,Z_ARM,50); /*ボックスを上げる*/
		//}else if(controller.button(RPDS3::CROSS)){ /*"×"が押されたら*/
			//ms.send(MECHANISM_MDD_NUM,Z_ARM,-50); /*ボックスを下げる*/
		//}else{ /*何も押されていなければ*/
			//ms.send(MECHANISM_MDD_NUM,Z_ARM,0); /*ボックスを動かさない*/
		//}

		if(right_distance > PWM_MAX_VALUE){ /*右スティックの動いた距離がPWMの最大値より大きかったら*/
			right_distance = PWM_MAX_VALUE; /*右スティックの動いた距離をPWMの最大値にする*/
		}

		right_theta = std::atan2(-right_y,right_x) + M_PI; /*左スティックのX軸Y軸より動いた角度を求める*/
		
		if(right_theta >= 0 && right_theta <= (M_PI/4)){
			ms.send(MECHANISM_MDD_NUM,Y_ARM,50);	
		}else if(right_theta > (M_PI/4) && right_theta <= (3*M_PI/4)){
			ms.send(MECHANISM_MDD_NUM,Z_ARM,50);
		}else if(right_theta > (3*M_PI/4) && right_theta <= (5*M_PI/4)){
			ms.send(MECHANISM_MDD_NUM,Y_ARM,-50);
		}else if(right_theta > (5*M_PI/4) && right_theta <= (7*M_PI/4)){
			ms.send(MEDHANISM_MDD_NUM,Z_ARM,-50);
		}else{
			ms.send(MECHANISM_MDD_NUM,Y_ARM,50);
		}

		//if(control_mode_flag == true){ /*コントローラのモードがONならば*/
			if(left_distance > PWM_MAX_VALUE){ /*左スティックの動いた距離がPWMの最大値より大きかったら*/
				left_distance = PWM_MAX_VALUE; /*左スティックの動いた距離をPWMの最大値にする*/
			}


			left_theta = std::atan2(-left_y,left_x) + M_PI; /*左スティックのX軸Y軸より動いた角度を求める*/
			
			if(controller.button(RPDS3::UP)){
				left_theta = (M_PI/2);
				regulation=0;
				reguration=regulation+0.01;
			}			

			if(left_theta >= 0 && left_theta <= (M_PI/2)){ /*左スティックの角度が第一象限の時*/
				left_back  = (left_theta * 4 / M_PI) - 1;
				left_front = 1;
			}else if(left_theta > (M_PI/2) && left_theta <= (M_PI)){ /*左スティックの角度が第二象限の時*/
				left_back  = 1;
				left_front = -(left_theta * 4 / M_PI) + 3;
			}else if(left_theta > (M_PI) && left_theta <= (3*M_PI/2)){ /*左スティックの角度が第三象限の時*/
				left_back  = -(left_theta * 4 / M_PI) + 5;
				left_front = -1;
			}else if(left_theta > (3*M_PI/2) && left_theta <= (2*M_PI)){ /*左スティックの角度が第四象限の時*/
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
		//}else{ /*コントロールのモードがOFFならば*/
			//ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_FRONT_MOTOR_NUM,  -left_y * 0.5 * regulation);
		      	//ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_BACK_MOTOR_NUM,   -left_y * 0.5 * regulation);
		      	//ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_FRONT_MOTOR_NUM, right_y * 0.5 * regulation);
		      	//ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_BACK_MOTOR_NUM,  right_y * 0.5 * regulation);
		//}
	
		//if(controller.button(RPDS3::UP)){
			//ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_FRONT_MOTOR_NUM,  -left_y * 0.5 * speed);
			//ms.send(UNDERCARRIAGE_MDD_NUM, LEFT_BACK_MOTOR_NUM,   -left_y * 0.5 * speed);
			//ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_FRONT_MOTOR_NUM, right_y * 0.5 * speed);
			//ms.send(UNDERCARRIAGE_MDD_NUM, RIGHT_BACK_MOTOR_NUM,  right_y * 0.5 * speed);
		//}

		//if(controller.button(RPDS3::R1) == true){ /*"R1"が押されていたら*/
			//regulation = 0.5; /*出力を1/2にする*/
			//std::cout << "reg" << std::endl; /*"reg"と表示する*/
		//}else{ /*何も押されていなければ*/
			regulation = 1.0; /*出力を1にする*/
		//}

	}
	gpioWrite(13,false);
	return 0;
}
