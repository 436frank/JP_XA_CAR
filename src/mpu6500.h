//
// Created by XIANGXI on 2022/11/9.
//

#ifndef JP_XA_CAR_MPU6500_H
#define JP_XA_CAR_MPU6500_H

#endif //JP_XA_CAR_MPU6500_H
#ifndef MPU6500_H
#define MPU6500_H

#include <Arduino.h>
//#define GYRO_250DPS
//#define GYRO_500DPS
//#define GYRO_1000DPS
#define GYRO_2000DPS

//#define ACCEL_2G
//#define ACCEL_4G
#define ACCEL_8G
//#define ACCEL_16G

#define HZ 1000.0f

#ifdef GYRO_250DPS
#define LSB2DEG 131.0f
#endif
#ifdef GYRO_500DPS
#define LSB2DEG 65.5f
#endif
#ifdef GYRO_1000DPS
#define LSB2DEG 32.8f
#endif
#ifdef GYRO_2000DPS
#define LSB2DEG 16.4f
#endif

//Set Accel self-test off
#ifdef ACCEL_2G
#define LSB2G 16384.0f
#endif
#ifdef ACCEL_4G
#define LSB2G 8192.0f
#endif
#ifdef ACCEL_8G
#define LSB2G 4096.0f
#endif
#ifdef ACCEL_16G
#define LSB2G 2048.0f
#endif

#define MPU6500_CS_ENABLE  digitalWrite(PIN_SPI1_SS,LOW)
#define MPU6500_CS_DISABLE digitalWrite(PIN_SPI1_SS,HIGH)


typedef struct {
    float X;
} X_axis;
typedef struct {
    int Y;
} Y_axis;
typedef struct {
    float Z;
} Z_axis;
typedef struct {
    float X;
    float Y;
    float Z;
} XYZ_axis;
typedef struct {
    XYZ_axis accel;
    XYZ_axis gyro;
    XYZ_axis accelRaw;
    XYZ_axis gyroRaw;
    XYZ_axis accelOffset;
    XYZ_axis gyroOffset;
    Z_axis gyroZeroSet;
    Z_axis angle;
} Sensor_TypeDef;

extern Sensor_TypeDef sen;

void MPU6500_Init();

void read_MPU6500_Acc_Gyro();

void printf_3AXIS(XYZ_axis AXIS, uint8_t ln = 1);

void printf_3AXIS_HZ(XYZ_axis AXIS, uint8_t ln = 1);

void printf_1AXIS(float AXIS);

void mpu6500AutoOffset(int cnt, bool wait = true);

#define MPU6500_ST_X_GYRO             0x00
#define MPU6500_ST_Y_GYRO             0x01
#define MPU6500_ST_Z_GYRO             0x02
#define MPU6500_ST_X_ACCEL            0x0D
#define MPU6500_ST_Y_ACCEL            0x0E
#define MPU6500_ST_Z_ACCEL            0x0F
#define MPU6500_XG_OFFS_USRH          0x13 //[15:0] XG_OFFS_USR
#define MPU6500_XG_OFFS_USRL          0x14
#define MPU6500_YG_OFFS_USRH          0x15 //[15:0] YG_OFFS_USR
#define MPU6500_YG_OFFS_USRL          0x16
#define MPU6500_ZG_OFFS_USRH          0x17 //[15:0] ZG_OFFS_USR
#define MPU6500_ZG_OFFS_USRL          0x18
#define MPU6500_SMPLRT_DIV            0x19
#define MPU6500_CONFIG                0x1A
#define MPU6500_GYRO_CONFIG           0x1B
#define MPU6500_ACCEL_CONFIG          0x1C
#define MPU6500_ACCEL_CONFIG_2        0x1D
#define MPU6500_LP_ACCEL_ODR          0x1E
#define MPU6500_WOM_THR               0x1F
#define MPU6500_FIFO_EN               0x23
#define MPU6500_I2C_MST_CTRL          0x24
#define MPU6500_I2C_SLV0_ADDR         0x25
#define MPU6500_I2C_SLV0_REG          0x26
#define MPU6500_I2C_SLV0_CTRL         0x27
#define MPU6500_I2C_SLV1_ADDR         0x28
#define MPU6500_I2C_SLV1_REG          0x29
#define MPU6500_I2C_SLV1_CTRL         0x2A
#define MPU6500_I2C_SLV2_ADDR         0x2B
#define MPU6500_I2C_SLV2_REG          0x2C
#define MPU6500_I2C_SLV2_CTRL         0x2D
#define MPU6500_I2C_SLV3_ADDR         0x2E
#define MPU6500_I2C_SLV3_REG          0x2F
#define MPU6500_I2C_SLV3_CTRL         0x30
#define MPU6500_I2C_SLV4_ADDR         0x31
#define MPU6500_I2C_SLV4_REG          0x32
#define MPU6500_I2C_SLV4_DO           0x33
#define MPU6500_I2C_SLV4_CTRL         0x34
#define MPU6500_I2C_SLV4_DI           0x35
#define MPU6500_I2C_MST_STATUS        0x36
#define MPU6500_INT_PIN_CFG           0x37
#define MPU6500_INT_ENABLE            0x38
#define MPU6500_DMP_INT_STATUS        0x39
#define MPU6500_INT_STATUS            0x3A
#define MPU6500_ACCEL_XOUT_H          0x3B
#define MPU6500_ACCEL_XOUT_L          0x3C
#define MPU6500_ACCEL_YOUT_H          0x3D
#define MPU6500_ACCEL_YOUT_L          0x3E
#define MPU6500_ACCEL_ZOUT_H          0x3F
#define MPU6500_ACCEL_ZOUT_L          0x40
#define MPU6500_TEMP_OUT_H            0x41
#define MPU6500_TEMP_OUT_L            0x42
#define MPU6500_GYRO_XOUT_H           0x43
#define MPU6500_GYRO_XOUT_L           0x44
#define MPU6500_GYRO_YOUT_H           0x45
#define MPU6500_GYRO_YOUT_L           0x46
#define MPU6500_GYRO_ZOUT_H           0x47
#define MPU6500_GYRO_ZOUT_L           0x48
#define MPU6500_EXT_SENS_DATA_00      0x49
#define MPU6500_EXT_SENS_DATA_01      0x4A
#define MPU6500_EXT_SENS_DATA_02      0x4B
#define MPU6500_EXT_SENS_DATA_03      0x4C
#define MPU6500_EXT_SENS_DATA_04      0x4D
#define MPU6500_EXT_SENS_DATA_05      0x4E
#define MPU6500_EXT_SENS_DATA_06      0x4F
#define MPU6500_EXT_SENS_DATA_07      0x50
#define MPU6500_EXT_SENS_DATA_08      0x51
#define MPU6500_EXT_SENS_DATA_09      0x52
#define MPU6500_EXT_SENS_DATA_10      0x53
#define MPU6500_EXT_SENS_DATA_11      0x54
#define MPU6500_EXT_SENS_DATA_12      0x55
#define MPU6500_EXT_SENS_DATA_13      0x56
#define MPU6500_EXT_SENS_DATA_14      0x57
#define MPU6500_EXT_SENS_DATA_15      0x58
#define MPU6500_EXT_SENS_DATA_16      0x59
#define MPU6500_EXT_SENS_DATA_17      0x5A
#define MPU6500_EXT_SENS_DATA_18      0x5B
#define MPU6500_EXT_SENS_DATA_19      0x5C
#define MPU6500_EXT_SENS_DATA_20      0x5D
#define MPU6500_EXT_SENS_DATA_21      0x5E
#define MPU6500_EXT_SENS_DATA_22      0x5F
#define MPU6500_EXT_SENS_DATA_23      0x60
#define MPU6500_MOT_DETECT_STATUS     0x61
#define MPU6500_I2C_SLV0_DO           0x63
#define MPU6500_I2C_SLV1_DO           0x64
#define MPU6500_I2C_SLV2_DO           0x65
#define MPU6500_I2C_SLV3_DO           0x66
#define MPU6500_I2C_MST_DELAY_CTRL    0x67
#define MPU6500_SIGNAL_PATH_RESET     0x68
#define MPU6500_MOT_DETECT_CTRL       0x69
#define MPU6500_USER_CTRL             0x6A
#define MPU6500_PWR_MGMT_1            0x6B
#define MPU6500_PWR_MGMT_2            0x6C
#define MPU6500_BANK_SEL              0x6D
#define MPU6500_MEM_START_ADDR        0x6E
#define MPU6500_MEM_R_W               0x6F
#define MPU6500_DMP_CFG_1             0x70
#define MPU6500_DMP_CFG_2             0x71
#define MPU6500_FIFO_COUNTH           0x72
#define MPU6500_FIFO_COUNTL           0x73
#define MPU6500_FIFO_R_W              0x74
#define MPU6500_WHO_AM_I              0x75
#define MPU6500_XA_OFFSET_H           0x77
#define MPU6500_XA_OFFSET_L           0x78
#define MPU6500_YA_OFFSET_H           0x7A
#define MPU6500_YA_OFFSET_L           0x7B
#define MPU6500_ZA_OFFSET_H           0x7D
#define MPU6500_ZA_OFFSET_L           0x7E

#define MPU6500_ID                    0x70
#endif //MPU6500_H
