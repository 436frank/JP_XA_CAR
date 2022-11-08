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
xyzFloat gValue ;
xyzFloat gyr ;
int a2=0;
int a1=0;

/**             **/
void Observer();
void setup()
{
    AdcBooster();
    Init_Peripherals();
    setupTimers();
    setupPWM();
    SerialUSB.begin(115200);
    /**SPI MPU6500**/
    Wire.begin();

    delay(2000);
    if(!myMPU6500.init()){
        SerialUSB.println("MPU6500 does not respond");
    }
    else{
        SerialUSB.println("MPU6500 is connected");
    }
    SerialUSB.println("Position you MPU6500 flat and don't move it - calibrating...");
    myMPU6500.setSPIClockSpeed(20000000);
    delay(1000);
    myMPU6500.autoOffsets();
    SerialUSB.println("Done!");
    myMPU6500.enableGyrDLPF();
    //myMPU6500.disableGyrDLPF(MPU6500_BW_WO_DLPF_8800); // bandwdith without DLPF
    myMPU6500.setGyrDLPF(MPU6500_DLPF_7);
    myMPU6500.setSampleRateDivider(5);
    myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_2000);
    myMPU6500.setAccRange(MPU6500_ACC_RANGE_8G);
    myMPU6500.enableAccDLPF(true);
    myMPU6500.setAccDLPF(MPU6500_DLPF_7);


    /**           **/
    /** Timer tc3 en **/
//    NVIC_EnableIRQ(TC3_IRQn);
//    NVIC_DisableIRQ(TC3_IRQn);
    /**              **/
}

void loop()
{
    /** MENU **/
    StateMachine_to_loop(sButton);

    /**SPI MPU6500 test**/

//    gyr = myMPU6500.getGyrValues();
//    Observer();
//    SerialUSB.print(a1);
//    SerialUSB.print("\t");
    SerialUSB.print(pos_now);
    SerialUSB.print("\t");
    SerialUSB.print(velocity);
    SerialUSB.print("\t");
    SerialUSB.print(posFeedBack);
    SerialUSB.print("\t");
    SerialUSB.print(velFeedBack);
    SerialUSB.print("\n");

//    SerialUSB.print(gyr.y);
//    SerialUSB.print("\t");
//    SerialUSB.print(gyr.z);
//    SerialUSB.print("\t");
//    SerialUSB.print(gValue.z);
//    SerialUSB.print("\n");


    delay(10);
    /**        IR        **/
//    readAllIR_flag=1;
//        for (int i = 0; i < 7; ++i) {
//        SerialUSB.print(IRsensors[i]);
//        SerialUSB.print(i==6?'\n':'\t');
//    }
    //
//        readAllIR_values();
//        IR_calibrations();
//        Lp = LINE_estimation(IRsensors);
//        SerialUSB.print(Lp);
//        SerialUSB.print("\t");
//    for (int i = 1; i < 6; ++i) {
//        SerialUSB.print(IRsensors[i]);
//        SerialUSB.print(i==5?'\n':'\t');
//    }
//    for (int i = 1; i < 6; ++i) {
//        SerialUSB.print(IR_vMin[i]);
//        SerialUSB.print(i==5?'\n':'\t');
//    }
//        SerialUSB.print(IRsensors[0]);
//        SerialUSB.print("\t");
//        SerialUSB.print(IRsensors[6]);
//        SerialUSB.print("\n");
//    delay(5);

//    delay(1);
    /** MOTOR **/
//    REG_TCC0_CC2 = 1000;                               // TCC0 CC3 - on D2
//    while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
//    REG_TCC0_CC3 = 4000;                               // TCC0 CC3 - on D2
//    while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization
//    SerialUSB.println(pos_now);
//    Motor_control(1000,1000);
    /** check_point **/
//    check_point();
}
// TC3 Interrupt Service Routine
void TC3_Handler()
{
    if ( start_flag==1)
    {
        if(start_cont<5000) start_cont++;
    }
    if ( readAllIR_flag==1)
    {
        readAllIR_values();
    }
    if ( LINE_following_VC_flag==1)
    {
        IR_calibrations();
        check_point();
        Lp = LINE_estimation(IR_caliValues);// calculate weighted average 計算權重平均
        LINE_following_VC();
        Protect();
    }
    checkButton();
    READ_QEI();
    Observer();
//    QEI_filter();
    if (tcont>3){tcont=0;} tcont++;
    // clear the interrupt flag
    REG_TC3_INTFLAG |= TC_INTFLAG_MC0;
}
void Observer() {
    // wn = 0.08, zeta = 1, Kp = wn^2 = 0.0064, Kv = 2*zeta*wn = 0.16
//    a2=micros();
//            a1=micros()-a2;
//    gValue = myMPU6500.getGValues();
//    gyr = myMPU6500.getGyrValues();
    Pcount_C = (Pcount_R + Pcount_L) / 2.0f;
//    omegaFeedBack = -gyr.z;
//    angleFeedBack = -gValue.z;
    posFeedBack += u_p;
    velFeedBack += u_v;
    u_p = (0.14f * (Pcount_C - posFeedBack)) + velFeedBack;
    u_v = (0.0064f * (Pcount_C - posFeedBack)) + ((gValue.y*9.80665/1000)* mm2p);

}