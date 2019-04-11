#include<iostream>
#include"../RasPiDS3/RasPiDS3.hpp"
#include<unistd.h>

using namespace std;
RPDS3::DualShock3 controller;

int main(){
	double ax = controller.acceleration(RPDS3::X_AXIS);
	double sum = 0;
	controller.update();
	while(1){
		controller.update;
		sum = sum + ax;
		cout << sum << endl;
		sleep(1);
	}
}
