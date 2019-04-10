Main: PigpioMS.o RasPiDS3.o main.o GY521.o
	g++ -Wall main.cpp -o Main RasPiDS3.o PigpioMS.o GY521.o -lpigpio -std=c++11 -pthread -lrt
morizumi: PigpioMS.o RasPiDS3.o main_by_yootee.o
	g++ -Wall main_by_yootee.cpp -o morizumi RasPiDS3.o PigpioMS.o -lpigpio -std=c++11 -pthread -lrt
PigpioMS.o: ./PigpioMS/PigpioMS.cpp
	g++ -Wall -c ./PigpioMS/PigpioMS.cpp -lpigpio -std=c++11 -pthread
RasPiDS3.o: ./RasPiDS3/RasPiDS3.cpp
	g++ -Wall -c ./RasPiDS3/RasPiDS3.cpp -std=c++11 -pthread
main.o: main.cpp
	g++ -Wall -c main.cpp -lpigpio -std=c++11 -pthread -lrt
Limit: 
	g++ -Wall limit_test.cpp -o Limit -lpigpio -std=c++11 -pthread -lrt
GY521.o: ./Sensor-master/GY521/GY521.cpp
	g++ -Wall -c ./Sensor-master/GY521/GY521.cpp -std=c++11 -pthread
clean: 
	rm -f *.o Do
