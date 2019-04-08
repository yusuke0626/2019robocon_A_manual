#include<iostream>
#include<pigpio.h>

int main(){
	gpioInitialise();
	gpioSetMode(12,PI_INPUT);
	gpioSetPullUpDown(12,PI_PUD_UP);
	int x;
	while(1){
		x = gpioRead(12);
		std::cout << x << std::endl;
	}
	return 0;
}
