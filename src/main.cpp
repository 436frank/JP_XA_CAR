#include <Arduino.h>
#include <Init_car_index.h>
#include "Moton.h"
#include <Wire.h>
#include "mpu6500.h"
#include "menu.h"


void setup()
{
    /**  Init IO **/
    Init_Peripherals();  //IO  setup
    SerialUSB.begin(115200);//USB Baud Rate
    /**  Init SPI MPU6500  **/
    MPU6500_Init();      //MPU6500 setup
    /**  Init ADC PWM timer  **/
    AdcBooster();        //ADC setup
    setupPWM();          //PWM setup
    setupTimers();       //timer setup
    /**  boot sound  **/
    tone(Buzzer_PIN, 500, 200);
    delay(100);
    tone(Buzzer_PIN, 1318, 200);
}
void loop()
{
//    readAllIR_flag =1;
//        for (int i = 0; i < 7; ++i) {
//        SerialUSB.print(IRsensors[i]);
//        SerialUSB.print(i==6?'\n':'\t');
//    }
//    delay(10);
//    SerialUSB.print(sButton);
//    delay(100);
    /** MENU **/
    StateMachine_2023(sButton);
}
/** TC3 Interrupt Service Routine **/
void TC3_Handler()
{
//    check_point();
    checkButton();
    READ_QEI();
    read_MPU6500_Acc_Gyro();
    Observer();
    if (test_1m_flag == 1) {
        test_1_v[test_1m_cont] = vc_command;
        test_1_cmd[test_1m_cont] = velFeedBack;
        test_1_pwm_r[test_1m_cont] = Speed_cmd_integral;
        test_1_pwm_l[test_1m_cont] = posFeedBack;
        test_1m_cont++;
    }
    if (start_flag == 1) {
        if (start_cont < 5000) start_cont++;
    }
    if (readAllIR_flag == 1) {
        readAllIR_values();
    }
    if (LINE_following_PC_flag == 1) {
        readAllIR_values();
        IR_calibrations();
        check_point();
        Lp = LINE_estimation(IR_caliValues);// calculate weighted average 計算權重平均
        if(run_mod_flag==old_run_mod_flag)
        {}
        else if((run_mod_flag>old_run_mod_flag)||(run_mod_flag<old_run_mod_flag))
        {sp_save_flag=1;}
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
        old_run_mod_flag=run_mod_flag;
        LINE_following_PC();
        Protect();
    }
    if (LINE_flag == 1) {
        IR_calibrations();
        Lp = LINE_estimation(IR_caliValues);// calculate weighted average 計算權重平均
        LINE_following();
        Protect();
    }
    if (IR_MAX_MIN_value_flag == 1) {
        readAllIR_values();
        IR_Max_Min();
    }
    if (run_flag == 1) {
        readAllIR_values();
        IR_calibrations();
        check_point();
        Protect();
        Lp = LINE_estimation(IR_caliValues);// calculate weighted average 計算權重平均
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
    REG_TC3_INTFLAG |= TC_INTFLAG_MC0;// clear the interrupt flag
}
