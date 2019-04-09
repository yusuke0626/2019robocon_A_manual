gyro_study: gyro_study.o GY521.o
	g++ -Wall gyro_study.cpp -o Main GY521.o -lpigpio -std=c++11 -pthread -lrt
GY521.o: ./../Sensor-master/GY521/GY521.cpp
	g++ -Wall -c ./../Sensor-master/GY521/GY521.cpp -std=c++11 -pthread
gyro_study.o: gyro_study.cpp
	g++ -Wall -c gyro_study.cpp -lpigpio -std=c++11 -pthread -lrt
clean: 
	rm -f *.o Do
