#include"../Sensor-master/GY521/GY521.hpp"
#include<iostream>
#include<pigpio.h>
#include<unistd.h>

using std::cout;
using std::cin;
using std::endl;
int main(){
	int decision;
	cout << "caribrate,plz" << endl;
	cin >> decision;
	
	gpioInitialise();
	RPGY521::GY521 gyro;
	cout << "ok,let`s go!" << endl;
	sleep(1);
	gyro.start();
	while(1){
		double gyro_rad = gyro.yaw;
		gyro.updata();
		cout << gyro_rad << endl;
	}
}
