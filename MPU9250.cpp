/*
MIT License

Copyright (c) 2020 Yuto Kaihara

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

https://github.com/acerols/raspi_MPU9250
*/

#include <cmath>
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <algorithm>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "MPU9250.hpp"

MPU9250::MPU9250(const char *DEV_NAME)
{
    
    Reg_Conf = Reg_AccConf = Reg_AccConf2 = Reg_GyroConf = 0;

    dev_addr = 0x68;
    dev_addr_mag = 0x0c;
    magRange = 4912;
    magCoefficient16 = magRange / 32760.0;
    accelRange = gyroRange = 0;


    fd = open(DEV_NAME, O_RDWR);
    if(fd == -1){
        std::cout << "Can't Open I2C Device" << std::endl;
    }
    else{

        Init();

        if(CheckConnection() != 1){
            close(fd);
            fd = -1;
        }
        if(CheckConnection_Mag() != 1){
            close(fd);
            fd = -1;
        }
        if(fd != -1){
            std::cout << "Connection All Success" << std::endl;
        }
    }
    
    
}

int MPU9250::Init()
{
    uint8_t data = 0x00;
    // MPU9250 clear
    write(MPU9250_PWR_MGMT_1, &data, 1);
    data = 0x02;

    //Enable to Access AK8963
    write(MPU9250_INT_PIN_CFG, &data, 1);

    data = 0x16;
    write(MPU9250_MAG_CNTL, &data, 1);

    SetAccFullScale(Acc_FS_8G);    
    SetGyroFullScale(Gyro_FS_1000dps);

    calibration();

    return 0;

}

int MPU9250::CheckConnection()
{
    uint8_t data;
    if(read(0x75, &data, 1) == 0){
        if(data == 0x71){
            return 1;
        }
    }
    return -1;
}

int MPU9250::write(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    
    uint8_t *buf = new uint8_t[length + 1];
    if(buf == NULL){
        std::cerr << "Could not allocate memory" << std::endl;
        return -1;
    }
    buf[0] = reg_addr;
    memcpy(&buf[1], data, length);
    
    struct i2c_msg message = {
        dev_addr, 0, length + 1, buf,
    };

    struct i2c_rdwr_ioctl_data ioctl_data = {
        &message, 1
    };

    if(ioctl(fd, I2C_RDWR, &ioctl_data) != 1){
        std::cerr << "Could not Connection i2c device" << std::endl;
        delete buf;
        close(fd);
        return -1;
    }

    delete buf;
    return 0;
    
}
int MPU9250::read(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    struct i2c_msg messages[] = {
        { dev_addr, 0, 1, &reg_addr },         /* レジスタアドレスをセット. */
        { dev_addr, I2C_M_RD, length, data },  /* dataにlengthバイト読み込む. */
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { 
        messages, 2
    };

    if(ioctl(fd, I2C_RDWR, &ioctl_data) != 2){
        std::cerr << "Could not Connection i2c device" << std::endl;
        close(fd);
        return -1;
    }

    return 0;
}

void MPU9250::ReadData(int acc[], int rot[])
{
	uint8_t buf[14];

	read(MPU9250_ACCEL_XOUT_H, buf, 14);

    acc[0] = (int)((short)(buf[0] << 8) | (short)buf[1]);
    acc[1] = (int)((short)(buf[2] << 8) | (short)buf[3]);
    acc[2] = (int)((short)(buf[4] << 8) | (short)buf[5]);

    
    acc[0] = acc[0] + offsetAccX;
    acc[1] = acc[1] + offsetAccY;
    acc[2] = acc[2] + offsetAccZ;
    
    rot[0] = (int)((short)(buf[8] << 8) | buf[9]);
    rot[1] = (int)((short)(buf[10] << 8) | buf[11]);
    rot[2] = (int)((short)(buf[12] << 8) | buf[13]);

    rot[0] = rot[0] + offsetRotX;
    rot[1] = rot[1] + offsetRotY;
    rot[2] = rot[2] + offsetRotZ;

}

void MPU9250::calibration(int count)
{
    int sumAcc[3] = {0};
    int sumRot[3] = {0};
    int acc[3];
    int rot[3];
    offsetAccX = offsetAccY = offsetAccZ = 0;
    offsetRotX = offsetRotY = offsetRotZ = 0;
    for(auto i = 0; i < count; i++){
        ReadData(acc,rot);
        sumAcc[0] += acc[0];
        sumAcc[1] += acc[1];
        sumAcc[2] += acc[2];
        sumRot[0] += rot[0];
        sumRot[1] += rot[1];
        sumRot[2] += rot[2]; 
    }

    offsetAccX = -1.0 * sumAcc[0] / count;
    offsetAccY = -1.0 * sumAcc[1] / count;
    offsetAccZ = -1.0 * sumAcc[2] / count;

    offsetRotX = -1.0 * sumRot[0] / count;
    offsetRotY = -1.0 * sumRot[1] / count;
    offsetRotZ = -1.0 * sumRot[2] / count;

}

int MPU9250::SetAccFullScale(AccFS_t fs){
	uint8_t b;
	switch(fs){
		case Acc_FS_2G:
            accelRange = 2;
			b = 0x00; 	
			break;
		case Acc_FS_4G:
            accelRange = 4;
			b = 0x01 << 3;
			break;
		case Acc_FS_8G:
            accelRange = 8;
			b = 0x02 << 3;
			break;
		case Acc_FS_16G:
            accelRange = 16;
			b = 0x03 << 3;
			break;
		default:
			b = 0;
		break;
	}

    accelCoefficient = accelRange / (double)((int)0x8000);

	Reg_AccConf &= ~(0x03 << 3);
	Reg_AccConf |= b;
	
	write(MPU9250_ACCEL_CONFIG, &Reg_AccConf, 1);
	return 0;   
}

int MPU9250::SetGyroFullScale(GyroFS_t fs){
	uint8_t b;
	switch(fs){
		case Gyro_FS_250dps:
            gyroRange = 250;
			b = 0x00; 	
			break;
		case Gyro_FS_500dps:
            gyroRange = 500;
			b = 0x01 << 3;
			break;
		case Gyro_FS_1000dps:
            gyroRange = 1000;
			b = 0x02 << 3;
			break;
		case Gyro_FS_2000dps:
            gyroRange = 2000;
			b = 0x03 << 3;
			break;
		default:
			b = 0;
		break;
	}

    gyroCoefficient = gyroRange / (double)((int)0x8000);

	Reg_GyroConf &= ~(0x03 << 3);
	Reg_GyroConf |= b;
	
	write(MPU9250_GYRO_CONFIG, &Reg_GyroConf, 1);
	return 0;   
}

int MPU9250::SetAccRate( AccRate_t rate ){
	uint8_t b;
	b = 0;
	switch(rate){
		case Acc_BW1130Hz_SR4k:
			b |= (1<<3);
			break;
		case Acc_BW460Hz_SR1k:
			b = 0;
			break;
		case Acc_BW184Hz_SR1k:
			b |= 0x01;
			break;
		case Acc_BW92Hz_SR1k:
			b |= 0x02;
			break;
		case Acc_BW41Hz_SR1k:
			b |= 0x03;
			break;
		case Acc_BW20Hz_SR1k:
			b |= 0x04;
			break;
		case Acc_BW10Hz_SR1k:
			b |= 0x05;
			break;
		case Acc_BW5Hz_SR1k:
			b |= 0x06;
			break;
		default:
			break;
	}
	Reg_AccConf2 = b;

	write(MPU9250_ACCEL_CONFIG_2, &Reg_AccConf2, 1);
    return 0;   
}


int MPU9250::SetGyroRate( GyroRate_t rate ){
	uint8_t b, c;
	b = c = 0;
	switch(rate){
		case Gyro_BW8800Hz_SR32k:
			b =  0x01; //FCHOISE = 0bx0, FCHOISE_b = 0bx1 = 1 or 3
			break;
		case Gyro_BW3600Hz_SR32k:
			b =  0x02;	//FCHOISE = 0b01, FCHOISE_b = 0b10 = 2
			break;
		case Gyro_BW250Hz_SR8k:
			b =  0x00;	//FCHOISE = 0b11, FCHOISE_b = 0b00
			c =  0x00;
			break;
		case Gyro_BW184Hz_SR1k:
			c =  0x01;
			break;
		case Gyro_BW92Hz_SR1k:
			c =  0x02;
			break;
		case Gyro_BW41Hz_SR1k:
			c =  0x03;
			break;
		case Gyro_BW20Hz_SR1k:
			c =  0x04;
			break;
		case Gyro_BW10Hz_SR1k:
			c =  0x05;
			break;
		case Gyro_BW5Hz_SR1k:
			c =  0x06;
			break;
		case Gyro_BW3600Hz_SR8k:
			c =  0x07;
			break;
		default:
			b = 0;
			c = 0;
			break;
	}

	Reg_Conf &= 0x07;
	Reg_Conf |= c;
	Reg_GyroConf &= 0x03;
	Reg_GyroConf |= b;

	write(MPU9250_GYRO_CONFIG, &Reg_GyroConf, 1);
	write(MPU9250_CONFIG, &Reg_Conf, 1);
	return 0;   
}

int MPU9250::writeMag(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    
    uint8_t *buf = new uint8_t[length + 1];
    if(buf == NULL){
        std::cerr << "Could not allocate memory" << std::endl;
        return -1;
    }
    buf[0] = reg_addr;
    memcpy(&buf[1], data, length);
    
    struct i2c_msg message = {
        dev_addr_mag, 0, length + 1, buf,
    };

    struct i2c_rdwr_ioctl_data ioctl_data = {
        &message, 1
    };

    if(ioctl(fd, I2C_RDWR, &ioctl_data) != 1){
        std::cerr << "Could not Connection i2c device" << std::endl;
        delete buf;
        close(fd);
        return -1;
    }

    delete buf;
    return 0;
    
}
int MPU9250::readMag(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    struct i2c_msg messages[] = {
        { dev_addr_mag, 0, 1, &reg_addr },         /* レジスタアドレスをセット. */
        { dev_addr_mag, I2C_M_RD, length, data },  /* dataにlengthバイト読み込む. */
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { 
        messages, 2
    };

    if(ioctl(fd, I2C_RDWR, &ioctl_data) != 2){
        std::cerr << "Could not Connection i2c device" << std::endl;
        close(fd);
        return -1;
    }

    return 0;
}


int MPU9250::isDataReady_Mag()
{
    uint8_t data;
    int ret;

    ret = readMag(MPU9250_MAG_ST1, &data, 1);

    if(data & (0x01)){
        return 1;
    }
    return -1;
}

int MPU9250::CheckConnection_Mag()
{
    uint8_t data;
    int ret;

    ret = readMag(MPU9250_MAG_WIA, &data, 1);

    if(data == 0x48){
        return 1;
    }
    return -1;
}

int MPU9250::ReadData_Mag(int mag[])
{
    uint8_t buf[7] = {0};
    uint8_t st2;
    uint8_t data;
    uint8_t status;
    data = 0x11;

    writeMag(MPU9250_MAG_CNTL, &data, 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    readMag(MPU9250_MAG_ST1, &status, 1);
    while(isDataReady_Mag() == -1){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        readMag(MPU9250_MAG_ST1, &status, 1);
    }

    if(readMag(MPU9250_MAG_HXL, buf, 7) == 0){
        mag[0] = (int)(short)((buf[1] << 8) & 0xff00) | (buf[0] & 0xff);
        mag[1] = (int)(short)((buf[3] << 8) & 0xff00) | (buf[2] & 0xff);
        mag[2] = (int)(short)((buf[5] << 8) & 0xff00) | (buf[4] & 0xff);
    }

    mag[0] = mag[0] + offsetMagX;
    mag[1] = mag[1] + offsetMagY;
    mag[2] = mag[2] + offsetMagZ;

    st2 = buf[6];
    if(st2 & (1 << 3)){
        readMag(MPU9250_MAG_CNTL, &status, 1);
        std::cout << std::hex << status << std::endl;
        std::cerr << "Warning Overflow" << std::endl;
        return -1;
    }

    return 1;
    
}

void MPU9250::CalibMag(int count)
{
    int mag[3];

    int maxMagX, maxMagY, maxMagZ;
    int minMagX, minMagY, minMagZ;
    maxMagX = maxMagY = maxMagZ = -999999;
    minMagX = minMagY = minMagZ = 999999;
    offsetMagX = offsetMagY = offsetMagZ = 0;

    std::cout << "start" << std::endl;

    for(auto i = 0; i < count; i++){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::cout << "";
		if(ReadData_Mag(mag) == 1){
			maxMagX = std::max(maxMagX, mag[0]);
            minMagX = std::min(minMagX, mag[0]);
            maxMagY = std::max(maxMagY, mag[1]);
            minMagY = std::min(minMagY, mag[1]);
            maxMagZ = std::max(maxMagZ, mag[2]);
            minMagZ = std::min(minMagZ, mag[2]);
        }
    }
    offsetMagX = -1.0 * (maxMagX + minMagX) / 2;
	offsetMagY = -1.0 * (maxMagY + minMagY) / 2;
    offsetMagZ = -1.0 * (maxMagZ + minMagZ) / 2;

    std::cout << "offset X " << offsetMagX << " Y " << offsetMagY << " Z " << offsetMagZ << std::endl;

}

void MPU9250::SetOffsetMag(int x, int y, int z)
{
    offsetMagX = x;
    offsetMagY = y;
    offsetMagZ = z;
}

void MPU9250::MagFix(int mag[], double fix[])
{
    for(int i = 0; i < 3; i++){
        fix[i] = (double)mag[i] * magCoefficient16;
    }
}

void MPU9250::AccFix(int acc[], double fix[])
{
    for(int i = 0; i < 3; i++){
        fix[i] = (double)acc[i] * accelCoefficient;
    }
}

void MPU9250::GyroFix(int rot[], double fix[])
{
    for(int i = 0; i < 3; i++){
        fix[i] = (double)rot[i] * gyroCoefficient;
    }
}

MPU9250::~MPU9250(){
	close(fd);
}