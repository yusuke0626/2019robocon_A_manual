#include<iostream>
#include<random>

int main(){
	for(int i = 1 ; i <= 2 ; ++i){
		std::uniform_int_distribution<int> val_dice(1,6);
		std::random_device val;
		std::mt19937 returner(val());
		int num = val_dice(val);
		std::cout << num << std::endl;
	}
}
