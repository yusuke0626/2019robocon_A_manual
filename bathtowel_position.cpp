#include<iostream>
#include<random>

using std::cout;
using std::endl;
using std::flush;

int red(){
	cout << "\x1b[41m" << flush;
	cout << ".." << flush;
}

int blue(){
	cout << "\x1b[44m" << flush;
	cout << ".." << flush;
}

int main(){
	int towel[11];
	for( int i = 0 ; i <= 11 ; ++i){
		std::uniform_int_distribution<int> val_dice1(1,12);
		std::random_device val1;
		std::mt19937 returner1(val1());
		towel[i] = val_dice1(val1);
		if(i != 0){
			for(int j = i - 1 ; j >= 0 ; --j){
				if(towel[i] == towel[n]){
					
				}
			}
		}
	}
	/*std::uniform_int_distribution<int> val_dice2(1,6);
	std::random_device val2;
	std::mt19937 returner(val2());
	int num2 = val_dice2(val2);*/

	cout << num1 << endl;
	//cout << num2 << endl;
	num1 =	1;
	switch(num1){
		case 1:
		red();
		blue();
		red();
		red();
		blue();
		blue();
		break;
		case 2:
		blue();
		red();
		blue();
		blue();
		red();
		red();
		break;
		case 3:
		nnw
		ppk
		nnw
		ppk
		ppk
		nnw
		break;
		case 4:

		break;
		case 5:
		break;
		case 6:
		break;
		case 7:
		break;
		case 8:
		break;
		case 9:
		break;
		case 10:
		break;
		case 11:
		break;
		case 12:
		break;
	}
	cout << "\x1b[49m" << endl;

}
