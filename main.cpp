#include <iostream>
#include <cstdint>
#include <cmath>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <string.h>
#include "MPU9250.hpp"
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

static const char* dev_name = "/dev/i2c-1";

double delta_t(struct timeval tim, struct timeval tim0){
	long dt, dt_usec;
	dt_usec = tim.tv_usec - tim0.tv_usec;
	dt = tim.tv_sec - tim0.tv_sec;
	return ( (double)dt + (double)(dt_usec)*1e-6 );
}

int main(){
	int mag[3];
	int acc[3], rot[3];
	double dmag[3], dacc[3], drot[3];
	double pi = acos(-1.0);
	MPU9250 sensor(dev_name);

	sensor.SetOffsetMag(245, -101, -253);
	
	if(sensor.CheckConnection_Mag() != 1)
		return -1;
	
	if(sensor.ReadData_Mag(mag) != 1){
		return -1;
	}

	while(1){
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if(sensor.ReadData_Mag(mag) == 1){
			sensor.ReadData(acc, rot);
			sensor.AccFix(acc, dacc);
			sensor.GyroFix(rot, drot);
			sensor.MagFix(mag, dmag);
			std::cout << "acc x " << acc[0] << " acc y " << acc[1] << " acc z " << acc[2] << std::endl;
			std::cout << "rot x " << rot[0] << " rot x " << rot[1] << " rot z " << rot[2] << std::endl;
			std::cout << "mag x " << mag[0] << " mag y " << mag[1] << " mag z " << mag[2] << std::endl;

			std::cout << "d acc x " << dacc[0] << " d acc y " << dacc[1] << " d acc z " << dacc[2] << std::endl;
			std::cout << "d rot x " << drot[0] << " d rot x " << drot[1] << " d rot z " << drot[2] << std::endl;
			std::cout << "d mag x " << dmag[0] << " d mag y " << dmag[1] << " d mag z " << dmag[2] << std::endl;

			auto deg = std::atan2((double)(mag[0]), (double)(mag[1]));
			deg = deg * 180.0 / pi;
			std::cout << deg << std::endl;
		}
	}
	
}