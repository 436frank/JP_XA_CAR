#include <Arduino.h>
#include <Init_car_index.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU6500_WE.h>
#include "Moton.h"
#include "menu.h"
/** SPI MPU6500 **/
const int csPin = PIN_SPI1_SS;  // Chip Select Pin
bool useSPI = true;    // SPI use flag
MPU6500_WE myMPU6500 = MPU6500_WE(&SPI1, csPin, useSPI);
/**             **/
void setup()
{
    AdcBooster();
    Init_Peripherals();
    setupTimers();
    setupPWM();
    SerialUSB.begin(115200);
    /**SPI MPU6500**/
    Wire.begin();
    delay(2000);//方便觀察
    if(!myMPU6500.init()){
        SerialUSB.println("MPU6500 does not respond");
    }
    else{
        SerialUSB.println("MPU6500 is connected");
    }
    SerialUSB.println("Position you MPU6500 flat and don't move it - calibrating...");
    delay(1000);
    myMPU6500.autoOffsets();
    SerialUSB.println("Done!");
    myMPU6500.enableGyrDLPF();
    //myMPU6500.disableGyrDLPF(MPU6500_BW_WO_DLPF_8800); // bandwdith without DLPF
    myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
    myMPU6500.setSampleRateDivider(5);
    myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
    myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
    myMPU6500.enableAccDLPF(true);
    myMPU6500.setAccDLPF(MPU6500_DLPF_6);
    /**           **/
}

void loop()
{
    /**SPI MPU6500 test**/
//    xyzFloat gValue = myMPU6500.getGValues();
//    xyzFloat gyr = myMPU6500.getGyrValues();
//    float temp = myMPU6500.getTemperature();
//    float resultantG = myMPU6500.getResultantG(gValue);
//    SerialUSB.print(gValue.x);
//    SerialUSB.print("\t");
//    SerialUSB.print(gValue.y);
//    SerialUSB.print("\t");
//    SerialUSB.print(gValue.z);
//    SerialUSB.print("\t");
//    SerialUSB.print(resultantG);
//    SerialUSB.print("\t");
//    SerialUSB.print(gyr.x);
//    SerialUSB.print("\t");
//    SerialUSB.print(gyr.y);
//    SerialUSB.print("\t");
//    SerialUSB.println(gyr.z);
//    delay(10);
    /**                **/
    for (int i = 0; i < 7; ++i) {
    SerialUSB.print(IRsensors[i]);
    SerialUSB.print("\t");
    }
    SerialUSB.println();
    delay(10);
//    REG_TCC0_CC2 = 4000;                               // TCC0 CC3 - on D2
//    while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
//    REG_TCC0_CC3 = 4000;                               // TCC0 CC3 - on D2
//    while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization
//    SerialUSB.println(Lp);

}
// TC3 Interrupt Service Routine
void TC3_Handler()
{
    checkButton();
    StateMachine(sButton);
    READ_QEI();
    QEI_filter();
    // clear the interrupt flag
    REG_TC3_INTFLAG |= TC_INTFLAG_MC0;
}