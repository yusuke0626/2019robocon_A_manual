Do: main.o RasPiDS3.o PigpioMS.o RotaryInc.o GY521.o
	g++ -Wall -o Do main.o RasPiDS3.o PigpioMS.o RotaryInc.o GY521.o -lpigpio -std=c++11 -pthread -lrt
GY521.o:/home/pi/2018robocon/Sensor-master/GY521/GY521.cpp
	g++ -Wall -c /home/pi/2018robocon/Sensor-master/GY521/GY521.cpp -lpigpio -std=c++11 -pthread -lrt
RotaryInc.o:/home/pi/2018robocon/Sensor-master/RotaryInc/RotaryInc.cpp
	g++ -Wall -c /home/pi/2018robocon/Sensor-master/RotaryInc/RotaryInc.cpp -lpigpio -std=c++11 -pthread -lrt
PigpioMS.o: /home/pi/2018robocon/PigpioMS/PigpioMS.cpp
	g++ -Wall -c /home/pi/2018robocon/PigpioMS/PigpioMS.cpp -lpigpio -std=c++11 -pthread
RasPiDS3.o: /home/pi/2018robocon/RasPiDS3/RasPiDS3.cpp
	g++ -Wall -c /home/pi/2018robocon/RasPiDS3/RasPiDS3.cpp -std=c++11 -pthread
main.o: main.cpp
	g++ -Wall -c main.cpp -lpigpio -std=c++11 -pthread -lrt
clean: 
	rm -f *.o Do
