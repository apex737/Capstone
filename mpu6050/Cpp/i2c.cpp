#include "mpu6050.h"
int main() {
    MPU6050 imu("/dev/i2c-1", 0x68);

    if (!imu.begin()) {
        std::cerr << "IMU Initialization Failed!\n";
        return -1;
    }

    while (true) {
        if (imu.update()) imu.print();
        sleep(1);
    }
    return 0;
}