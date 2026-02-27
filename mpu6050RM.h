#ifndef MPU6050_REGISTERS_H
#define MPU6050_REGISTERS_H

/* MPU-6000/MPU-6050 Register Map */
#define POWER_ON             0x00
#define POWER_OFF            0x01
#define INTR_ON              0x01
#define INTR_OFF             0x00
#define DEV_ID               0x98

// Self Test Registers
#define SELF_TEST_X          0x0D
#define SELF_TEST_Y          0x0E
#define SELF_TEST_Z          0x0F
#define SELF_TEST_A          0x10

// Configuration Registers
#define SMPLRT_DIV           0x19
#define CONFIG               0x1A
#define GYRO_CONFIG          0x1B
#define ACCEL_CONFIG         0x1C

// FIFO & I2C Master Control
#define FIFO_EN              0x23
#define I2C_MST_CTRL         0x24
#define I2C_SLV0_ADDR        0x25
#define I2C_SLV0_REG         0x26
#define I2C_SLV0_CTRL        0x27
#define I2C_SLV1_ADDR        0x28
#define I2C_SLV1_REG         0x29
#define I2C_SLV1_CTRL        0x2A
#define I2C_SLV2_ADDR        0x2B
#define I2C_SLV2_REG         0x2C
#define I2C_SLV2_CTRL        0x2D
#define I2C_SLV3_ADDR        0x2E
#define I2C_SLV3_REG         0x2F
#define I2C_SLV3_CTRL        0x30
#define I2C_SLV4_ADDR        0x31
#define I2C_SLV4_REG         0x32
#define I2C_SLV4_DO          0x33
#define I2C_SLV4_CTRL        0x34
#define I2C_SLV4_DI          0x35
#define I2C_MST_STATUS       0x36

// Interrupt Setup
#define INT_PIN_CFG          0x37
#define INT_ENABLE           0x38
#define INT_STATUS           0x3A

// Accelerometer Measurements
#define ACCEL_XOUT_H         0x3B
#define ACCEL_XOUT_L         0x3C
#define ACCEL_YOUT_H         0x3D
#define ACCEL_YOUT_L         0x3E
#define ACCEL_ZOUT_H         0x3F
#define ACCEL_ZOUT_L         0x40

// Temperature Measurement
#define TEMP_OUT_H           0x41
#define TEMP_OUT_L           0x42

// Gyroscope Measurements
#define GYRO_XOUT_H          0x43
#define GYRO_XOUT_L          0x44
#define GYRO_YOUT_H          0x45
#define GYRO_YOUT_L          0x46
#define GYRO_ZOUT_H          0x47
#define GYRO_ZOUT_L          0x48

// External Sensor Data
#define EXT_SENS_DATA_00     0x49
#define EXT_SENS_DATA_01     0x4A
#define EXT_SENS_DATA_02     0x4B
#define EXT_SENS_DATA_03     0x4C
#define EXT_SENS_DATA_04     0x4D
#define EXT_SENS_DATA_05     0x4E
#define EXT_SENS_DATA_06     0x4F
#define EXT_SENS_DATA_07     0x50
#define EXT_SENS_DATA_08     0x51
#define EXT_SENS_DATA_09     0x52
#define EXT_SENS_DATA_10     0x53
#define EXT_SENS_DATA_11     0x54
#define EXT_SENS_DATA_12     0x55
#define EXT_SENS_DATA_13     0x56
#define EXT_SENS_DATA_14     0x57
#define EXT_SENS_DATA_15     0x58
#define EXT_SENS_DATA_16     0x59
#define EXT_SENS_DATA_17     0x5A
#define EXT_SENS_DATA_18     0x5B
#define EXT_SENS_DATA_19     0x5C
#define EXT_SENS_DATA_20     0x5D
#define EXT_SENS_DATA_21     0x5E
#define EXT_SENS_DATA_22     0x5F
#define EXT_SENS_DATA_23     0x60

// I2C Slave Data Out
#define I2C_SLV0_DO          0x63
#define I2C_SLV1_DO          0x64
#define I2C_SLV2_DO          0x65
#define I2C_SLV3_DO          0x66

// Master & System Control
#define I2C_MST_DELAY_CTRL   0x67
#define SIGNAL_PATH_RESET    0x68
#define USER_CTRL            0x6A
#define PWR_MGMT_1           0x6B
#define PWR_MGMT_2           0x6C

// FIFO Count & Read/Write
#define FIFO_COUNT_H         0x72
#define FIFO_COUNT_L         0x73
#define FIFO_R_W             0x74
#define WHO_AM_I             0x75


// --- [필수 및 권장 설정 비트 정의] ---
// 1. PWR_MGMT_1 (0x6B) - 클록 소스 설정
#define CLKSEL_INTERNAL      0x00 // 내부 8MHz 오실레이터 
#define CLKSEL_PLL_XGYRO     0x01 // X축 자이로 PLL (권장: 더 안정적)
#define PWR_DEVICE_RESET     0x80 // 전체 리셋 비트 
#define PWR_SLEEP            0x40 // 슬립 모드 비트 

// 2. CONFIG (0x1A) - DLPF(디지털 저역 통과 필터) 설정 
// 로봇 프로젝트에서는 42Hz~188Hz 사이를 주로 사용합니다.
#define DLPF_BW_256          0x00 // 필터 없음 (지연 최소)
#define DLPF_BW_188          0x01
#define DLPF_BW_98           0x02
#define DLPF_BW_42           0x03 // 권장: 진동 노이즈 억제 효율적
#define DLPF_BW_20           0x04
#define DLPF_BW_10           0x05

// 3. GYRO_CONFIG (0x1B) - 자이로 범위 설정 
#define GYRO_FS_250          0x00 // +/- 250 deg/s (민감도: 131 LSB/dps) 
#define GYRO_FS_500          0x08 // +/- 500 deg/s (민감도: 65.5 LSB/dps) 
#define GYRO_FS_1000         0x10 // +/- 1000 deg/s (민감도: 32.8 LSB/dps) 
#define GYRO_FS_2000         0x18 // +/- 2000 deg/s (민감도: 16.4 LSB/dps) 

// 4. ACCEL_CONFIG (0x1C) - 가속도 범위 설정 
#define ACCEL_FS_2           0x00 // +/- 2g (민감도: 16384 LSB/g) 
#define ACCEL_FS_4           0x08 // +/- 4g (민감도: 8192 LSB/g) 
#define ACCEL_FS_8           0x10 // +/- 8g (민감도: 4096 LSB/g) 
#define ACCEL_FS_16          0x18 // +/- 16g (민감도: 2048 LSB/g) 

// 5. INT_ENABLE (0x38) - 인터럽트 활성화 
#define INT_DATA_RDY_EN      0x01 // 데이터 준비 완료 인터럽트 활성화

// 6. USER_CTRL (0x6A) - 시스템 제어 
#define USER_FIFO_EN         0x40 // FIFO 활성화
#define USER_FIFO_RESET      0x04 // FIFO 리셋
#define USER_SIG_COND_RESET  0x01 // 센서 신호 경로 리셋 

// 7. Sensitivity (민감도 계수) - 변환용 API에서 사용 
#define ACCEL_LSB_2G         16384.0f
#define ACCEL_LSB_8G         4096.0f
#define GYRO_LSB_250         131.0f
#define GYRO_LSB_500         65.5f

// 8. 샘플 레이트 설정 (DLPF 활성화 시 1kHz 기준)
// 공식: Sample Rate = 1kHz / (1 + SMPLRT_DIV) 
#define SAMPLE_RATE_1000HZ   0x00 // 1000 / (1+0) = 1000Hz 
#define SAMPLE_RATE_500HZ    0x01 // 1000 / (1+1) = 500Hz 
#define SAMPLE_RATE_250HZ    0x03 // 1000 / (1+3) = 250Hz 
#define SAMPLE_RATE_200HZ    0x04 // 1000 / (1+4) = 200Hz
#endif // MPU6050_REGISTERS_H