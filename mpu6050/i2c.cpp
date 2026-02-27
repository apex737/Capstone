
#include "mpu6050.h"

int main() {
    int file;
    const char *bus = "/dev/i2c-1";
    
    // 1. I2C 버스 열기
    if ((file = open(bus, O_RDWR)) < 0) 
        error_handler("open");

    // 2. 슬레이브 주소 설정 (0x68)
    if (ioctl(file, I2C_SLAVE, 0x68) < 0)
        error_handler("ioctl");

    MPUData mpu = {0};
    mpu_init(file, mpu);
    while(1)
    {
        if(mpu_get_value(file, mpu) < 0) continue;
        mpu_print_all(mpu);
        sleep(1);
    }
    
    close(file);
    return 0;
}