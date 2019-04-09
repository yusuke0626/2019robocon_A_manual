#include<iostream>
#include"pigpio.h"

int main(){
	int pin;
	std::cout << "チェックするピンの番号を入力してください" << std::endl;
	std::cin >> pin;
	gpioInitialise();
	gpioSetMode(pin,PI_INPUT);
	gpioSetPullUpDown(pin,PI_PUD_UP);
	while(1){
		int val;
		val = gpioRead(pin);
		std::cout << val << std::endl; 
	
	}
}
