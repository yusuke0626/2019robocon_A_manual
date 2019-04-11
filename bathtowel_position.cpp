#include<iostream>
#include<random>

using std::cout;
using std::endl;
using std::flush;
using std::cin;
int red(){
	cout << "\x1b[41m" << flush;
	cout << ".." << flush;
	cout << "\x1b[49m" << flush;
	cout << "|" << flush;
}

int blue(){
	cout << "\x1b[44m" << flush;
	cout << ".." << flush;
	cout << "\x1b[49m" << flush;
	cout << "|" << flush;
}

int main(){
	int battle_mode = 0;
	cout << "予選なら1を、決勝なら2をおしてください。" << endl;
	while(battle_mode != 1 && battle_mode != 2){
		cin >> battle_mode;
		if(battle_mode == 1){
			cout << "　予選  " ;
		}else if(battle_mode == 2){
			cout << "　決勝　" ;
		}else{
			cout << "予選なら1を、決勝なら2をおしてください。" << endl;
		}
	}
	int position;
	std::uniform_int_distribution<int> val_dice1(1,12);
	std::random_device val1;
	std::mt19937 returner1(val1());
	position = val_dice1(val1);
	cout << "-- " << position << endl;
	if(battle_mode == 1){
		switch(position){
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
				red();
				blue();
				red();
				blue();
				blue();
				red();
				break;
			case 4:
				blue();
				red();
				blue();
				red();
				red();
				blue();
				break;
			case 5:
				red();
				red();
				blue();
				blue();
				red();
				blue();
				break;
			case 6:
				blue();
				blue();
				red();
				red();
				blue();
				red();
				break;
			case 7:
				blue();
				red();
				red();
				blue();
				blue();
				red();
				break;
			case 8:
				red();
				blue();
				blue();
				red();
				red();
				blue();
				break;
			case 9:
				red();
				blue();
				red();
				blue();
				red();
				blue();
				break;
			case 10:
				blue();
				red();
				blue();
				red();
				blue();
				red();
				break;
			case 11:
				red();
				blue();
				blue();
				red();
				blue();
				red();
				break;
			case 12:
				blue();
				red();
				red();
				blue();
				red();
				blue();
				break;
		}
	}else{
		switch(position){
			case 1:
				red();
				red();
				blue();
				blue();
				red();
				red();
				blue();
				blue();
				break;
			case 2:
				blue();
				blue();
				red();
				red();
				blue();
				blue();
				red();
				red();

				break;
			case 3:
				red();
				red();
				blue();
				blue();
				blue();
				blue();
				red();
				red();
				break;
			case 4:
				blue();
				blue();
				red();
				red();
				red();
				red();
				blue();
				blue();

				break;
			case 5:
				red();
				red();
				red();
				red();
				blue();
				blue();
				blue();
				blue();

				break;
			case 6:
				blue();
				blue();
				blue();
				blue();
				red();
				red();
				red();
				red();

				break;
			case 7:
				red();
				blue();
				blue();
				red();
				red();
				blue();
				blue();
				red();
				break;
			case 8:
				blue();
				red();
				red();
				blue();
				blue();
				red();
				red();
				blue();
				break;
			case 9:
				red();
				red();
				blue();
				red();
				blue();
				blue();
				red();
				blue();
				break;
			case 10:
				blue();
				blue();
				red();
				blue();
				red();
				red();
				blue();
				red();
				break;
			case 11:
				red();
				blue();
				red();
				red();
				blue();
				red();
				blue();
				blue();
				break;
			case 12:
				blue();
				red();
				blue();
				blue();
				red();
				blue();
				red();
				red();
				break;
		}}
	cout << "\x1b[49m" << endl;

}

