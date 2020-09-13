#ifndef __MPU9250_H__
#define __MPU9250_H__
extern "C"{
#define MPU9250_SELF_TEST_X_GYRO 0x00
#define MPU9250_MAG_ASAZ 0x12
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_MAG_CNTL 0x0A
#define MPU9250_MAG_WIA 0x00
#define MPU9250_MAG_HXL 0x03
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG_2 0x1D
#define MPU9250_CONFIG 0x1A
#define MPU9250_MAG_ST1 0x02
}

//Select Full Scale
typedef enum{
	Acc_FS_2G = 0,
	Acc_FS_4G,
	Acc_FS_8G,
	Acc_FS_16G
}AccFS_t;

typedef enum{
	Gyro_FS_250dps = 0,
	Gyro_FS_500dps,
	Gyro_FS_1000dps,
	Gyro_FS_2000dps
}GyroFS_t;

//Select Bandwidth & Sample rate
typedef enum{
	Acc_BW1130Hz_SR4k = 0,
	Acc_BW460Hz_SR1k,
	Acc_BW184Hz_SR1k,
	Acc_BW92Hz_SR1k,
	Acc_BW41Hz_SR1k,
	Acc_BW20Hz_SR1k,
	Acc_BW10Hz_SR1k,
	Acc_BW5Hz_SR1k
}AccRate_t;

typedef enum{
	Gyro_BW8800Hz_SR32k = 0,
	Gyro_BW3600Hz_SR32k,
	Gyro_BW250Hz_SR8k,
	Gyro_BW184Hz_SR1k,
	Gyro_BW92Hz_SR1k,
	Gyro_BW41Hz_SR1k,
	Gyro_BW20Hz_SR1k,
	Gyro_BW10Hz_SR1k,
	Gyro_BW5Hz_SR1k,
	Gyro_BW3600Hz_SR8k
}GyroRate_t;


class MPU9250{
private:
	uint8_t dev_addr;
	uint8_t dev_addr_mag;
	int fd;
	int magRange;
	double magCoefficient16;
	uint8_t Reg_Conf, Reg_GyroConf, Reg_AccConf, Reg_AccConf2;
	int write(uint8_t reg_addr, uint8_t *data, uint16_t length);
	int read(uint8_t reg_addr, uint8_t *data, uint16_t length);
	int writeMag(uint8_t reg_addr, uint8_t *data, uint16_t length);
	int readMag(uint8_t reg_addr, uint8_t *data, uint16_t length);


public:
	MPU9250(const char *DEV_NAME);
	~MPU9250();
	int CheckConnection();
	int Init();
	
	void ReadData(int acc[], int rot[]);
	int SetAccFullScale(AccFS_t);
	int SetGyroFullScale(GyroFS_t);
	int SetAccRate(AccRate_t);
	int SetGyroRate(GyroRate_t);
	
	int isDataReady_Mag();
	int CheckConnection_Mag();
	int ReadData_Mag(int mag[]);
	void Magfix(int mag[], double fix[]);

};

#endif
