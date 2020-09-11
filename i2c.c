#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// ...
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

static const char* dev_name = "/dev/i2c-1";
#define I2C_ADDR 0x68
#define I2C_ADDR_MAG 0x0c

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

/*! I2Cスレーブデバイスからデータを読み込む.
 * @param[in] dev_addr デバイスアドレス.
 * @param[in] reg_addr レジスタアドレス.
 * @param[out] data 読み込むデータの格納場所を指すポインタ.
 * @param[in] length 読み込むデータの長さ.
 */
int8_t i2c_read(
    uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t length) {
  /* I2Cデバイスをオープンする. */
  int32_t fd = open(dev_name, O_RDWR);
  if (fd == -1) {
    printf("Can't Open device i2c");
    return -1;
  }

  /* I2C-Readメッセージを作成する. */
  struct i2c_msg messages[] = {
      { dev_addr, 0, 1, &reg_addr },         /* レジスタアドレスをセット. */
      { dev_addr, I2C_M_RD, length, data },  /* dataにlengthバイト読み込む. */
  };
  struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };

  /* I2C-Readを行う. */
  if (ioctl(fd, I2C_RDWR, &ioctl_data) != 2) {
    printf("Can't Open device i2c");
    close(fd);
    return -1;
  }

  close(fd);
  return 0;
}

/*! I2Cスレーブデバイスにデータを書き込む.
 * @param[in] dev_addr デバイスアドレス.
 * @param[in] reg_addr レジスタアドレス.
 * @param[in] data 書き込むデータの格納場所を指すポインタ.
 * @param[in] length 書き込むデータの長さ.
 */
int8_t i2c_write(
    uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, uint16_t length) {
    /* I2Cデバイスをオープンする. */
    int32_t fd = open(dev_name, O_RDWR);
    if (fd == -1) {
        printf("Can't Open device i2c");
        return -1;
    }

    /* I2C-Write用のバッファを準備する. */
    uint8_t* buffer = (uint8_t*)malloc(length + 1);
    if (buffer == NULL) {
      printf("Can't Open device i2c");
      close(fd);
      return -1;
    }
    buffer[0] = reg_addr;              /* 1バイト目にレジスタアドレスをセット. */
    memcpy(&buffer[1], data, length);  /* 2バイト目以降にデータをセット. */

    /* I2C-Writeメッセージを作成する. */
    struct i2c_msg message = { dev_addr, 0, length + 1, buffer };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };

    /* I2C-Writeを行う. */
    if (ioctl(fd, I2C_RDWR, &ioctl_data) != 1) {
      printf("Can't Open device i2c");
      free(buffer);
      close(fd);
      return -1;
    }

    free(buffer);
    close(fd);
    return 0;
}

int CheckConn()
{
    uint8_t data;
    if(i2c_read(I2C_ADDR, 0x75, &data, 1) == 0){
        if(data == 0x71){
            printf("Connection Success : %x\n", data);
            return 1;
        }
    }
    printf("Connection Failed");
    return -1;
}

int Init()
{
    uint8_t data = 0x00;
    i2c_write(I2C_ADDR, MPU9250_PWR_MGMT_1, &data, 1);
    data = 0x02;
    i2c_write(I2C_ADDR, MPU9250_INT_PIN_CFG, &data, 1);

    data = 0x16;
    i2c_write(I2C_ADDR, MPU9250_MAG_CNTL, &data, 1);

    return 0;

}

int isDataReadyMag()
{
    uint8_t data;
    int ret;

    ret = i2c_read(I2C_ADDR_MAG, MPU9250_MAG_ST1, &data, 1);

    printf("isDataReady Mag %x\n", data);

    if(data & (0x01)){
        return 1;
    }
    return -1;
    
}

int CheckConnMag()
{
    uint8_t data;
    int ret;

    ret = i2c_read(I2C_ADDR_MAG, MPU9250_MAG_WIA, &data, 1);

    printf("MAG WIA DATA %x\n", data);

    if(data == 0x48){
        return 1;
    }
    return -1;

}

int ReadData(short acc[], short rot[], short mag[], short *temp)
{
    int ret1, ret2;
    uint8_t buf[14];
    uint8_t bufMag[7];
    uint8_t st2;
    ret1 = i2c_read(I2C_ADDR, MPU9250_ACCEL_XOUT_H, buf, 14);
    acc[0] = ((buf[0] << 8) & 0xff00) | (buf[1] & 0xff);
    acc[1] = ((buf[2] << 8) & 0xff00) | (buf[3] & 0xff);
    acc[2] = ((buf[4] << 8) & 0xff00) | (buf[5] & 0xff);

    *temp = ((buf[6] << 8) & 0xff00) | (buf[7] & 0xff);

    rot[0] = ((buf[8] << 8) & 0xff00) | (buf[9] & 0xff);
    rot[0] = ((buf[10] << 8) & 0xff00) | (buf[11] & 0xff);
    rot[0] = ((buf[12] << 8) & 0xff00) | (buf[113] & 0xff);

    ret2 = i2c_read(I2C_ADDR_MAG, MPU9250_MAG_HXL, bufMag, 7);

    mag[0] = ((bufMag[0] << 8) & 0xff00) | (buf[1] & 0xff);
    mag[1] = ((bufMag[2] << 8) & 0xff00) | (buf[3] & 0xff);
    mag[2] = ((bufMag[4] << 8) & 0xff00) | (buf[5] & 0xff);

    st2 = buf[6];

    if(st2 & (1<<3)){
        printf("Warnign overflow!\n");
        return -1;
    }

    return ret1 + ret2;

}


int main()
{
    short acc[3], rot[3], temp;
    short mag[3];
    int magcheck;
    int magready;
    uint16_t len;
    if(CheckConn() == -1){
        return -1;
    }
    Init();
    magcheck = CheckConnMag();
    printf("Mag Check %x\n", magcheck);
    magready = isDataReadyMag();
    printf("Mag Ready %x\n", magready);
    
    if(ReadData(acc, rot, mag, &temp) == 0){
        printf("Mag1 : %d, Mag2 : %d, Mag3 : %d\n", mag[0], mag[1], mag[2]);
    }

    atan2f(0.0, 0.0);
    printf("yaw %f", atan2(mag[1], mag[2]));

    /*
    if(CheckConnMag() == 1){
        if(isDataReadyMag() == 1){
            if(ReadData(acc, rot, mag, &temp) == 0)
                printf("Mag1 : %d, Mag2 : %d, Mag3 : %d\n", mag[0], mag[1], mag[2]);
        }
    }
    */
    
    


    return 0;
}