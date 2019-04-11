#include<iostream>
#include<pigpio.h>
#include<stdlib.h>

int main(){
	std::cout << "リミットスイッチチェッカー起動しました" << std::endl;
	std::cout << "Ctrl + C でプログラム終了します" << std::endl;
	std::cout << "通電中には０が、スイッチが押されているときは１が表示され続けます" << std::endl;
	std::cout << "スイッチから手を離してください" << std::endl;
	std::cout << "確認したいピンの番号を入力してください" << std::endl;
	std::cout << "M-2 右上　→　12 右下　→　16 左上　→  11 左下　→　22" << std::endl;
	std::cout << "M-3 Y軸前　→　26　Y軸後ろ　→　19　Z軸上　→　9　Z軸下　→　10　" << std::endl;
	int pin;
	std::cin >> pin;

	gpioInitialise();
	gpioSetMode(pin,PI_INPUT);
	gpioSetPullUpDown(pin,PI_PUD_UP);

	int x;
	x = gpioRead(pin);
	
	if( x == 1){
		std::cout << "通電していません。配線及びスイッチの確認をしてください。" << std::endl;
		exit(0);
	}else{
		while(1){
			x = gpioRead(pin);
			std::cout << x << std::endl;
		}
	}
	return 0;
}
