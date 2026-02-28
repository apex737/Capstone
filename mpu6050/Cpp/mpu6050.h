#ifndef MPU6050_LINUX_H
#define MPU6050_LINUX_H

#include <iostream>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <unistd.h>
#include <cmath>

#include "../mpu6050RM.h" 
using namespace std;
class MPU6050 {
public:
    struct Data {
        float ax, ay, az;
        float gx, gy, gz;
    };

    MPU6050(const char* device_path = "/dev/i2c-1", uint8_t dev_addr = 0x68)
        : device_path_(device_path), dev_addr_(dev_addr), fd_(-1) {
        data_ = {0};
        gx_ofs_ = gy_ofs_ = gz_ofs_ = 0.0f;
        ax_ofs_ = ay_ofs_ = az_ofs_ = 0.0f;
    }

    ~MPU6050() {
        if (fd_ >= 0) close(fd_); // 자원 해제 (RAII)
    }

    bool begin() 
    {
        if ((fd_ = open(device_path_, O_RDWR)) < 0) {
            cerr << "[begin] open failed\n";
            return false;
        }
        
        if (ioctl(fd_, I2C_SLAVE, dev_addr_) < 0) {
            cerr << "[begin] ioctl failed\n";
            return false;
        }

        // WHO_AM_I 확인
        if (readRegister(WHO_AM_I) != WHO_AM_I_VAL){
            cerr << "[begin] readRegister failed\n";
            return false;
        }

        // Wake & Clock 설정 (0x6B)
        writeRegister(PWR_MGMT_1, CLKSEL_PLL_XGYRO);

        // Burst Write: SMPLRT_DIV(0x19) ~ ACCEL_CONFIG(0x1C)
        uint8_t config_burst[5] = { SMPLRT_DIV, SAMPLE_RATE_1000HZ, DLPF_BW_42, GYRO_FS_500, ACCEL_FS_8 };
        if (write(fd_, config_burst, 5) != 5) {
            cerr << "[begin] write failed\n";
            return false;
        }

        usleep(100000); // 안정화 대기
        calibrate();    // 초기 캘리브레이션 수행
        return true;
    }

    bool update() 
    {
        uint8_t reg = ACCEL_XOUT_H;
        uint8_t buf[14];
        struct i2c_msg msgs[2];
        struct i2c_rdwr_ioctl_data msgset;

        msgs[0] = { dev_addr_, 0, 1, &reg }; // Write register address
        msgs[1] = { dev_addr_, I2C_M_RD, 14, buf }; // Read 14 bytes burst
        msgset = { msgs, 2 };

        if (ioctl(fd_, I2C_RDWR, &msgset) < 0) return false;

        // 단위 변환 및 오프셋 적용
        data_.ax = ((float)((int16_t)((buf[0] << 8) | buf[1])) / ACCEL_LSB_8G) - ax_ofs_; // 8G 기준 LSB
        data_.ay = ((float)((int16_t)((buf[2] << 8) | buf[3])) / ACCEL_LSB_8G) - ay_ofs_;
        data_.az = ((float)((int16_t)((buf[4] << 8) | buf[5])) / ACCEL_LSB_8G) - az_ofs_;
        
        data_.gx = ((float)((int16_t)((buf[8] << 8) | buf[9])) / GYRO_LSB_500) - gx_ofs_; // 500dps 기준 LSB
        data_.gy = ((float)((int16_t)((buf[10] << 8) | buf[11])) / GYRO_LSB_500) - gy_ofs_;
        data_.gz = ((float)((int16_t)((buf[12] << 8) | buf[13])) / GYRO_LSB_500) - gz_ofs_;

        return true;
    }

    // 데이터 접근용 Getter (캡슐화)
    const Data& getData() const { return data_; }

    void print() const {
        printf("Ax Ay Az: %.2f %.2f %.2f | Gx Gy Gz: %.2f %.2f %.2f\n", 
               data_.ax, data_.ay, data_.az, data_.gx, data_.gy, data_.gz);
    }

private:
    const char* device_path_;
    uint8_t dev_addr_;
    int fd_;
    Data data_;
    float ax_ofs_, ay_ofs_, az_ofs_;
    float gx_ofs_, gy_ofs_, gz_ofs_;

    void writeRegister(uint8_t reg, uint8_t val) 
    {
        uint8_t buf[2] = { reg, val };
        write(fd_, buf, 2);
    }

    uint8_t readRegister(uint8_t reg) 
    {
        uint8_t val;
        write(fd_, &reg, 1);
        read(fd_, &val, 1);
        return val;
    }

    void calibrate(int samples = 200) 
    {
        float sumGx = 0, sumGy = 0, sumGz = 0;
        float sumAx = 0, sumAy = 0, sumAz = 0;
        for(int i = 0; i < samples; i++) {
            update();
            sumAx += data_.ax; sumAy += data_.ay; sumAz += data_.az;
            sumGx += data_.gx; sumGy += data_.gy; sumGz += data_.gz;
            usleep(5000); // 200Hz sampling
        }
        ax_ofs_ = sumAx / samples;
        ay_ofs_ = sumAy / samples;
        az_ofs_ = (sumAz / samples) - 1.0f;
        gx_ofs_ = sumGx / samples; 
        gy_ofs_ = sumGy / samples;
        gz_ofs_ = sumGz / samples;

        printf("Offsets: Ax:%.2f, Ay:%.2f, Az:%.2f | Gx:%.2f, Gy:%.2f, Gz:%.2f\n", 
            ax_ofs_, ay_ofs_, az_ofs_, gx_ofs_, gy_ofs_, gz_ofs_);
    }

    bool error_handler(const char* msg)
    {
        perror(msg);
        exit(1);
    }
};

#endif