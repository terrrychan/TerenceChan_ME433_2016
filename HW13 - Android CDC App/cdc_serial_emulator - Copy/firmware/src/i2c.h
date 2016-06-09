#ifndef I2C__H__
#define I2C__H__

#define CS LATBbits.LATB7   // chip select pin
#define PI 3.14159265 // used in HW4

// addresses used for IMU
#define WHO_AM_I 0x0F
#define IMU_ADDR 0b11010110 // full 8 bit address, has pull up resistors
#define IMU_7bit 0b1101011 // 7 bit address - shift over for 1/0 r/w
#define CTRL1_XL 0x10 // turn on accelerometer
#define CTRL2_G 0x11 // turn on gyroscope
#define CTRL3_C 0x12 // turn on IF_INC --- enable multiple read
#define OUT_TEMP_L 0x20 // call this for multiple read

void PIC32_init(void);

// i2c functions
void i2c_master_start(void);
void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);
void i2c_master_setup(void);

// IMU functions
void imu_setup(void);
unsigned char I2C_read_single(void);
void I2C_read_multiple(char, char, unsigned char *, char);

#endif