#include<iostream>
#include<pigpio.h>

int main(){
	gpioSetMode(14,PI_INPUT);
	gpioSetPullUpDown(14,PI_PUD_UP);
	int x;
	while(1){
		x = gpioRead(14);
		std::cout << x << std::endl;
	}
}
