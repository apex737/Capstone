#ifndef MPU6050_H
#define MPU6050_H
#include "../mpu6050RM.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <unistd.h>

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float gx_ofs, gy_ofs, gz_ofs;
} MPUData;

void mpu_write_reg(int fd, uint8_t reg, uint8_t data);
uint8_t mpu_read_reg(int fd, uint8_t reg);
int mpu_init(int fd, MPUData& mpu); 
void error_handler(const char* msg);
int mpu_get_value(int fd, MPUData &mpu);
void calibrate_gyro(int fd, MPUData& mpu);
void mpu_print_all(MPUData& mpu);

#endif