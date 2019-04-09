#include<iostream>
#include<pigpio.h>

int main(){
	gpioInitialise();
	gpioSetMode(22,PI_INPUT);
	gpioSetPullUpDown(22,PI_PUD_UP);
	int x;
	while(1){
		x = gpioRead(22);
		std::cout << x << std::endl;
	}
	return 0;
}
