Main: PigpioMS.o RasPiDS3.o main.o
	g++ -Wall main.cpp -o Main  RasPiDS3.o PigpioMS.o -lpigpio -std=c++11 -pthread -lrt
PigpioMS.o: ./PigpioMS/PigpioMS.cpp
	g++ -Wall -c ./PigpioMS/PigpioMS.cpp -lpigpio -std=c++11 -pthread
RasPiDS3.o: ./RasPiDS3/RasPiDS3.cpp
	g++ -Wall -c ./RasPiDS3/RasPiDS3.cpp -std=c++11 -pthread
main.o: main.cpp
	g++ -Wall -c main.cpp -lpigpio -std=c++11 -pthread -lrt
<<<<<<< HEAD
clean: 
=======
clean:
>>>>>>> fd93541b74e9e0af78aee597ace67280d68e98ad
	rm -f *.o Do
