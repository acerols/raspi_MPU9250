#include <cmath>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

MPU9250::MPU9250()
{
    
    Reg_Conf = Reg_AccConf = Reg_AccConf2 = Reg_GyroConf = 0;

    fd = open(DEV_PATH, O_RDWR);
    if(fd == -1){
        std::cout << "Can't Open I2C Device" << std::endl;
    }



}