//
// Created by XIANGXI on 2022/11/9.
//
#include "mpu6500.h"
#include <SPI.h>

extern SPIClassSAMD SPI1;
SPISettings mySPISettings;
Sensor_TypeDef sen = {};

float IIR_Filter(float input, float last, float IIR_a) {
    return last * IIR_a + (1 - IIR_a) * (input);
}

const float LSB2g = LSB2G / 9.80665f * HZ;  // (g/s)   -> (m/s2) -> (mm/ms)
const float LSB2Deg = LSB2DEG * HZ;         // (deg/s) -> (deg/ms)

int offsetCnt = 0;

void writeRegister8(uint8_t reg, uint8_t val) {
    SPI1.beginTransaction(mySPISettings);
    MPU6500_CS_ENABLE;
    SPI1.transfer(reg);
    SPI1.transfer(val);
    MPU6500_CS_DISABLE;
    SPI1.endTransaction();
}

uint8_t readRegister8(uint8_t reg) {
    uint8_t regValue = 0;

    reg |= 0x80;
    SPI1.beginTransaction(mySPISettings);
    MPU6500_CS_ENABLE;
    SPI1.transfer(reg);
    regValue = SPI1.transfer(0x00);
    MPU6500_CS_DISABLE;
    SPI1.endTransaction();

    return regValue;
}

int16_t readRegister16(uint8_t reg) {
    uint8_t MSByte = 0, LSByte = 0;

    reg |= 0x80;
    SPI1.beginTransaction(mySPISettings);
    MPU6500_CS_ENABLE;
    SPI1.transfer(reg);
    MSByte = SPI1.transfer(0x00);
    LSByte = SPI1.transfer(0x00);
    MPU6500_CS_DISABLE;
    SPI1.endTransaction();

    return (int16_t) ((MSByte << 8) + LSByte);
}

void MPU6500_Init(void) {
    // Set 1 MBit/s
    pinMode(PIN_SPI1_SS, OUTPUT);
    MPU6500_CS_DISABLE;
    SPI1.begin();
    mySPISettings = SPISettings(1000000, MSBFIRST, SPI_MODE0);

    // Reset device
    writeRegister8(MPU6500_PWR_MGMT_1, 0x80);
    delay(100);
    writeRegister8(MPU6500_SIGNAL_PATH_RESET, 0x07);
    delay(100);

    while (readRegister8(MPU6500_WHO_AM_I) != MPU6500_ID) {
        //MPU6500 Error!
        delay(100);
        SerialUSB.println("MPU6500 Error!");
    };

    writeRegister8(MPU6500_PWR_MGMT_2, 0x00);   //EN ACC,Gyro X,Y,Z-axis
    //writeRegister8(MPU6500_PWR_MGMT_2, 0b00101110);	 //EN ACC Y-axis,Gyro Z-axis	-> O O XA YA ZA XG YG ZG

    // Reset DMP , FIFO , I2C_MST , SIG_COND
    writeRegister8(MPU6500_USER_CTRL, 0x1F);
    // FIFO OFF
    writeRegister8(MPU6500_FIFO_EN, 0x00);
    // Set Config
//    writeRegister8(MPU6500_CONFIG, 0x00);             //DLPF_CFG = 0, GYRO_Bandwidth =  250Hz, Fs = 8kHz,delay=0.97ms
    writeRegister8(MPU6500_CONFIG, 0x01);        //DLPF_CFG = 1, GYRO_Bandwidth =  184Hz, Fs = 1kHz,delay= 2.9 ms
//    writeRegister8(MPU6500_CONFIG, 0x02);             //DLPF_CFG = 2, GYRO_Bandwidth =   92Hz, Fs = 1kHz,delay= 3.9 ms
//    writeRegister8(MPU6500_CONFIG, 0x03);			    //DLPF_CFG = 3, GYRO_Bandwidth =   41Hz, Fs = 1kHz,delay= 5.9 ms
//    writeRegister8(MPU6500_CONFIG, 0x04);			    //DLPF_CFG = 4, GYRO_Bandwidth =   20Hz, Fs = 1kHz,delay= 9.9 ms
//    writeRegister8(MPU6500_CONFIG, 0x05);			    //DLPF_CFG = 5, GYRO_Bandwidth =   10Hz, Fs = 1kHz,delay=17.85ms
//    writeRegister8(MPU6500_CONFIG, 0x06);			    //DLPF_CFG = 6, GYRO_Bandwidth =    5Hz, Fs = 1kHz,delay=33.48ms
//    writeRegister8(MPU6500_CONFIG, 0x07);		        //DLPF_CFG = 7, GYRO_Bandwidth = 3600Hz, Fs = 8kHz,delay= 0.17ms

    // Set FCHOICE_B=0, Gyro self-test off
#ifdef GYRO_250DPS
    writeRegister8(MPU6500_GYRO_CONFIG, 0x00); //GyroFS = +-250dps
#endif
#ifdef GYRO_500DPS
    writeRegister8(MPU6500_GYRO_CONFIG, 0x08); //GyroFS = +-500dps
#endif
#ifdef GYRO_1000DPS
    writeRegister8(MPU6500_GYRO_CONFIG, 0x10); //GyroFS = +-1000dps
#endif
#ifdef GYRO_2000DPS
    writeRegister8(MPU6500_GYRO_CONFIG, 0x18); //GyroFS = +-2000dps
#endif

    // Set Accel self-test off
#ifdef ACCEL_2G
    writeRegister8(MPU6500_ACCEL_CONFIG, 0x00); //Accel FS = +- 2g
#endif
#ifdef ACCEL_4G
    writeRegister8(MPU6500_ACCEL_CONFIG, 0x08); //Accel FS = +- 4g
#endif
#ifdef ACCEL_8G
    writeRegister8(MPU6500_ACCEL_CONFIG, 0x10); //Accel FS = +- 8g
#endif
#ifdef ACCEL_16G
    writeRegister8(MPU6500_ACCEL_CONFIG, 0x18); //Accel FS = +-16g
#endif


    writeRegister8(MPU6500_ACCEL_CONFIG_2, 0x00);  // Accel 460Hz bandwidth, delay 1.94ms, 1khz Fs
//    writeRegister8(MPU6500_ACCEL_CONFIG_2, 0x01);        // Accel 184Hz bandwidth, delay 5.80ms, 1khz Fs

    // Set 12 MBit/s
    mySPISettings = SPISettings(2000000, MSBFIRST, SPI_MODE0);
    delay(10);
}

void printf_3AXIS(XYZ_axis AXIS, uint8_t ln) {
    SerialUSB.print(AXIS.X, 6);
    SerialUSB.print('\t');
    SerialUSB.print(AXIS.Y, 6);
    SerialUSB.print('\t');
    SerialUSB.print(AXIS.Z, 6);
    SerialUSB.print(ln == 1 ? '\n' : '\t');
}

void printf_3AXIS_HZ(XYZ_axis AXIS, uint8_t ln) {
    SerialUSB.print(AXIS.X * HZ, 6);
    SerialUSB.print('\t');
    SerialUSB.print(AXIS.Y * HZ, 6);
    SerialUSB.print('\t');
    SerialUSB.print(AXIS.Z * HZ, 6);
    SerialUSB.print(ln == 1 ? '\n' : '\t');
}

void printf_1AXIS(float AXIS) {
    SerialUSB.println(AXIS, 6);
}

void read_MPU6500_Acc_Gyro() {
    // 讀取加速度、陀螺儀
//    sen.accelRaw.X = readRegister16(MPU6500_ACCEL_XOUT_H);
    sen.accelRaw.Y = readRegister16(MPU6500_ACCEL_YOUT_H);
//    sen.accelRaw.Z = readRegister16(MPU6500_ACCEL_ZOUT_H);
//    sen.gyroRaw.X = readRegister16(MPU6500_GYRO_XOUT_H);
//    sen.gyroRaw.Y = readRegister16(MPU6500_GYRO_YOUT_H);
    sen.gyroRaw.Z = readRegister16(MPU6500_GYRO_ZOUT_H);

    // 取加速度、陀螺儀偏移值
    if (offsetCnt > 0) {
        offsetCnt--;
//        sen.accelOffset.X = IIR_Filter(sen.accelRaw.X, sen.accelOffset.X, 0.994f);
        sen.accelOffset.Y = IIR_Filter(sen.accelRaw.Y, sen.accelOffset.Y, 0.994f);
//        sen.accelOffset.Z = IIR_Filter(sen.accelRaw.Z, sen.accelOffset.Z, 0.994f);
//        sen.gyroOffset.X = IIR_Filter(sen.gyroRaw.X, sen.gyroOffset.X, 0.994f);
//        sen.gyroOffset.Y = IIR_Filter(sen.gyroRaw.Y, sen.gyroOffset.Y, 0.994f);
        sen.gyroOffset.Z = IIR_Filter(sen.gyroRaw.Z, sen.gyroOffset.Z, 0.994f);
        if (offsetCnt == 0){
            sen.angle.Z = 0;
//            sen.accelOffset.Z -= LSB2G;
        }
    } else {
        // 計算數值
//        sen.accel.X = (sen.accelRaw.X - sen.accelOffset.X) / LSB2g; // (mm/ms)
        sen.accel.Y = (sen.accelRaw.Y - sen.accelOffset.Y) / LSB2g; // (mm/ms)
//        sen.accel.Z = (sen.accelRaw.Z - sen.accelOffset.Z) / LSB2g; // (mm/ms)
//        sen.gyro.X = (sen.gyroRaw.X - sen.gyroOffset.X) / LSB2Deg; // (deg/ms)
//        sen.gyro.Y = (sen.gyroRaw.Y - sen.gyroOffset.Y) / LSB2Deg; // (deg/ms)
        sen.gyro.Z = (sen.gyroRaw.Z - sen.gyroOffset.Z) / LSB2Deg; // (deg/ms)
        // 計算Z軸角度
        sen.angle.Z += sen.gyro.Z;  // (deg)
    }
}

void mpu6500AutoOffset(int cnt, bool wait) {
    sen.accelOffset = sen.accelRaw; // 設定加速規初值
    sen.gyroOffset = sen.gyroRaw;   // 設定陀螺儀初值
    offsetCnt = cnt;                // 開始校正
    if (wait)
        delay(cnt + 100);           // 等待校正完成
}

