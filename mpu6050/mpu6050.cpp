#include "mpu6050.h"

void error_handler(const char* msg)
{
    perror(msg);
    exit(1);
}
// 레지스터 하나에 데이터 쓰기
void mpu_write_reg(int fd, uint8_t reg, uint8_t data) 
{
    uint8_t buf[2] = {reg, data};
    write(fd, buf, 2);
}

// 레지스터에서 데이터 읽기
uint8_t mpu_read_reg(int fd, uint8_t reg) 
{
    uint8_t data;
    write(fd, &reg, 1);
    read(fd, &data, 1);
    return data;
}


void calibrate_gyro(int fd, MPUData& mpu) 
{
    float sumX = 0, sumY = 0, sumZ = 0;
    int samples = 200;

    for(int i = 0; i < samples; i++) {
        mpu_read_all(fd, mpu);
        sumX += mpu.gx; sumY += mpu.gy; sumZ += mpu.gz;
        usleep(5000); // 200Hz 샘플링
    }
    mpu.gx_ofs = sumX / samples;
    mpu.gy_ofs = sumY / samples;
    mpu.gz_ofs = sumZ / samples;
    printf("Offsets: X:%.2f, Y:%.2f, Z:%.2f\n", 
        mpu.gx_ofs, mpu.gy_ofs, mpu.gz_ofs);
}

int mpu_init(int fd, MPUData& mpu) 
{
    // 1. 연결 확인 (WHO_AM_I)
    if (mpu_read_reg(fd, WHO_AM_I) != WHO_AM_I_VAL)
        error_handler("Device connection failed");
    
    // 2. Wake & 클럭 설정
    mpu_write_reg(fd, PWR_MGMT_1, CLKSEL_PLL_XGYRO);

    uint8_t burst_data[5] = {
        SMPLRT_DIV,             // 레지스터 시작주소
        SAMPLE_RATE_1000HZ,     // SMPLRT_DIV (1000Hz)
        DLPF_BW_42,             // CONFIG (DLPF 42Hz)
        GYRO_FS_500,            // GYRO_CONFIG (500dps)
        ACCEL_FS_8              // ACCEL_CONFIG (8g)
    };

    if (write(fd, burst_data, 5) != 5) 
        error_handler("Burst Write Failed");

    usleep(100000); // 리셋 후 안정화 대기 (100ms)
    calibrate_gyro(fd, mpu);

    return 0;
}


int mpu_get_value(int fd, MPUData &mpu) {
    uint8_t reg = ACCEL_XOUT_H;
    uint8_t buf[14];

    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;

    // 첫 번째 메시지: 
    msgs[0] = {DEV_ADDR, 0, 1, &reg}; // 읽을 레지스터 주소 Write
    msgs[1] = {DEV_ADDR, 1, 14, buf}; // Burst Read 

    msgset = {msgs, 2};

    if (ioctl(fd, I2C_RDWR, &msgset) < 0) 
        error_handler("mpu_read_all");
    
    mpu.ax = (float)((int16_t)((buf[0] << 8) | buf[1])) / ACCEL_LSB_8G;
    mpu.ay = (float)((int16_t)((buf[2] << 8) | buf[3])) / ACCEL_LSB_8G;
    mpu.az = (float)((int16_t)((buf[4] << 8) | buf[5])) / ACCEL_LSB_8G;
    
    mpu.gx = (float)((int16_t)((buf[8] << 8) | buf[9])) / GYRO_LSB_500 - mpu.gx_ofs;
    mpu.gy = (float)((int16_t)((buf[10] << 8) | buf[11])) / GYRO_LSB_500 - mpu.gy_ofs;
    mpu.gz = (float)((int16_t)((buf[12] << 8) | buf[13])) / GYRO_LSB_500 - mpu.gz_ofs;

    return 0;
}

void mpu_print_all(MPUData& mpu)
{
    printf("Accel: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f\n", 
                   mpu.ax, mpu.ay, mpu.az, 
                   mpu.gx, mpu.gy, mpu.gz);
}