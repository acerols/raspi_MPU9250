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
	double fix[3];
	double pi = acos(-1.0);
	int maxMagx = -99999;
    int maxMagy = -99999;
    int minMagx = 99999;
    int minMagy = 99999;
	int offsetx, offsety;
	MPU9250 sensor(dev_name);

	if(sensor.CheckConnection_Mag() != 1)
		return -1;
	
	if(sensor.ReadData_Mag(mag) != 1){
		return -1;
	}
	while(1){
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if(sensor.ReadData_Mag(mag) == 1){
			std::cout << "mag x " << mag[0] << " mag y " << mag[1] << " mag z " << mag[2] << std::endl;
			if(maxMagx < mag[0]){
				maxMagx = mag[0];
			}
			if(minMagx > mag[0]){
				minMagx = mag[0];
			}
			if(maxMagy < mag[1]){
				maxMagy = mag[1];
			}
			if(minMagy > mag[1]){
				minMagy = mag[1];
			}
			
			offsetx = (maxMagx + minMagx) / 2;
			offsety = (maxMagy + minMagy) / 2;
			std::cout << "max " << maxMagx << " " << maxMagy << std::endl;
			std::cout << "min " << minMagx << " " << minMagy << std::endl;
			std::cout << "off " << offsetx << " " << offsety << std::endl;
			auto deg = std::atan2((double)(mag[0] - offsetx), (double)(mag[1] - offsety));
			deg = deg * 180.0 / pi;
			std::cout << deg << std::endl;
		}
	}
	
}