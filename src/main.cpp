#include <Arduino.h>
#include <Init_car_index.h>
#include "Moton.h"
#include <Wire.h>
#include "mpu6500.h"
#include "menu.h"

/** SPI MPU6500 **/
const int csPin = PIN_SPI1_SS;  // Chip Select Pin
//bool useSPI = true;    // SPI use flag
int a2=0;
int a1=0;
bool cnt_=0;

/**             **/
void Observer();
void setup()
{
    /**  Init  **/
    AdcBooster();
    Init_Peripherals();
    setupPWM();
    SerialUSB.begin(115200);
    /**  Init SPI MPU6500  **/
    MPU6500_Init();
//    mpu6500AutoOffset(1,1);
    /**  Init timer  **/
    setupTimers();
    /**  boot sound  **/

    tone(Buzzer_PIN, 500, 200);
    delay(100);
    tone(Buzzer_PIN, 1318, 200);
}

void loop()
{

    /** MENU **/
    StateMachine_to_loop(sButton);
    /**SPI MPU6500 test**/
//    SerialUSB.print(a2);
//    SerialUSB.print("\n");
//    SerialUSB.print(pos_now);
//    SerialUSB.print("\t");
//    SerialUSB.print(velocity);
//    SerialUSB.print("\t");
//    SerialUSB.print(omegaFeedBack);
//    SerialUSB.print("\t");
//    SerialUSB.print(omegaFeedBack);
//    SerialUSB.print("\n");

//    SerialUSB.print(sen.gyro.Z);
//    SerialUSB.print("\t");
//    SerialUSB.print(sen.angle.Z);
//    SerialUSB.print("\t");
//    SerialUSB.print(sen.accel.Y);
//    SerialUSB.print("\n");
//    delay(10);
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
/** TC3 Interrupt Service Routine **/
void TC3_Handler()  //
{

    checkButton();
    READ_QEI();
//    QEI_filter();
    read_MPU6500_Acc_Gyro();
    Observer();
    if(test_1m_flag==1)
    {
        test_1_v[test_1m_cont]=vc_command;
        test_1_cmd[test_1m_cont]=velFeedBack;
        test_1_pwm_r[test_1m_cont]=Speed_cmd_integral;
        test_1_pwm_l[test_1m_cont]=posFeedBack;
        test_1m_cont++;
    }
    if ( start_flag==1)
    {
        if(start_cont<5000) start_cont++;
    }
    if ( readAllIR_flag==1)
    {
        readAllIR_values();
        check_point();

    }
    if ( LINE_following_VC_flag==1)
    {
        IR_calibrations();
        check_point();
        Lp = LINE_estimation(IR_caliValues);// calculate weighted average 計算權重平均
        LINE_following_VC();
        Protect();
    }
    if ( IR_MAX_MIN_value_flag==1){
        readAllIR_values();
        IR_Max_Min();
    }
    if(run_flag==1){
        switch (run_mod_flag) {
            case 1:
                vc_Command(1);
                break;
            case 2:
                vc_Command(2);
                break;
            case 3:
                vc_Command(3);
                break;
        }
        LINE_following_PC();
    }
//    a2=micros()-a1;
//    if (tcont>3){tcont=0;} tcont++;
    REG_TC3_INTFLAG |= TC_INTFLAG_MC0;// clear the interrupt flag
}
void Observer() {
    Pcount_C = (Pcount_R + Pcount_L) / 2.0f;
    omegaFeedBack = -sen.gyro.Z;    // L- R+
    angleFeedBack = -sen.angle.Z;   // L- R+
    posFeedBack += u_p;
    velFeedBack += u_v;
    u_p = (0.14f * (Pcount_C - posFeedBack)) + velFeedBack;
    u_v = (0.0064f * (Pcount_C - posFeedBack)) + (sen.accel.Y* mm2p);

}