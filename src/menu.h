//
// Created by XIANGXI on 2022/10/18.
//

#ifndef JP_XA_CAR_MENU_H
#define JP_XA_CAR_MENU_H
#endif //JP_XA_CAR_MENU_H
#include <Arduino.h>
//#define old_menu
#define new_menu
int a=0;
/**  old_menu  **/
void StateMachine(unsigned char value);
#ifdef old_check
void StateMachine(unsigned char value) {
    unsigned char index;
    switch (value)
    {
        case 0:
            digitalWrite(LED_L_PIN, OFF);
            digitalWrite(LED_R_PIN, OFF);
            for (index = 0; index < 6; index++)
            {
                IR_vMax[index] = 0;
                IR_vMin[index] = 900;
            }
            break;
        case 1:
            digitalWrite(LED_L_PIN, ON);
            digitalWrite(LED_R_PIN, ON);
            if (cCount == 600) Motor_control(800, 800);   // not balanced in left and right wheels
            else if (cCount == 800) Motor_control(0, 0);
            if (cCount == 1400) MotorRest();
            readAllIR_values();
            IR_Max_Min();
            // calculate weighted average 計算權重平均
                  Lp = LINE_estimation(IRsensors);
                  pos_now=0;
            break;
        case 2:
//            NVIC_DisableIRQ(TC3_IRQn);
            digitalWrite(LED_L_PIN, OFF);
            digitalWrite(LED_R_PIN, OFF);
            if(start_cont<1000) start_cont++;
            if(start_cont==1000)
            {

                //test_1m_flag=1;
                readAllIR_values();
                IR_calibrations();
                check_point();
                // calculate weighted average 計算權重平均
                Lp = LINE_estimation(IR_caliValues);
//                /**  **/
//                a=vc_following();
//                Motor_control(a,a);
//                if(vc_f==1)
//                {
//                    vc_Command(1);
//                    digitalWrite(LED_R_PIN, OFF);
//                }
//                if(vc_command>=1*mm2p)
//                {
//                    vc_f=0;
//                }
//                if(vc_f==0 && pos_now>=216.36)
//                {
//                    digitalWrite(LED_R_PIN, ON);
//                    vc_Command(3);
//                    if(velocity<=0.1f)
//                    {
//                        test_1m_flag=0;
//                        sButton = 0;
//                        MotorRest();
//                    }
//                }
//                /**  **/
//
                LINE_following_VC();
////                  LINE_following() ;
//                if(test_1m_flag==1)
//                {
////                    test_1m[test_1m_cont]=velocity;
////                    test_1_pwm[test_1m_cont]=deltaPWM_;
////                    test_1_cmd[test_1m_cont]=vc_command;
////                    speed_integral+=velocity;
////                    test_1m_cont++;
//                }

            }
            break;
        case 3:
            digitalWrite(LED_L_PIN, OFF);
            digitalWrite(LED_R_PIN, ON);
            if(pos_now<pos_stop)
            {
                readAllIR_values();
                IR_calibrations();
                check_point();
                // calculate weighted average 計算權重平均
                Lp = LINE_estimation(IR_caliValues);
                LINE_following_park_well();
                vc_flag_=1;
            }
            else
            {
                if(park_well_cont<500)
                {
                    readAllIR_values();
                    IR_calibrations();
                    check_point();
                    // calculate weighted average 計算權重平均
                    Lp = LINE_estimation(IR_caliValues);
                    LINE_following_park_well();
                    park_well_cont++;
                }
                else
                {
                    digitalWrite(LED_R_PIN, ON);
                    digitalWrite(LED_L_PIN, ON);
                    MotorRest();
                }
            }
            break;

        case 4:
            digitalWrite(LED_R_PIN, ON);
            digitalWrite(LED_L_PIN, ON);
            MotorRest();

            break;
        case 5:  //假衝刺
            if(start_cont<1000) {
                start_cont++;

                if(vc_flag_==1)vc_command=1.35*mm2p;
                if(vc_flag_==2)vc_command=1.5*mm2p;

            }

            if(start_cont==1000)
            {
                readAllIR_values();
                IR_calibrations();
                check_point();
                // calculate weighted average 計算權重平均
                Lp = LINE_estimation(IR_caliValues);
                if(all_road_radius[prompt_cont]>300)
                {
                    vc_command=1*mm2p;
                }
                else
                {
                    vc_command=0.5*mm2p;
                }
                LINE_following_VC();
            }
            break;
    }
}
#endif
/**  new_menu  **/
void Selector();
void count(unsigned char value);
void StateMachine_to_loop(unsigned char value);
#ifdef new_menu
void Selector()
{

    if(eMotionR.pNEW<10)
    {
        if(old_select!=0)tone(Buzzer_PIN,1318,200);old_select=0;
    }
    else if(eMotionR.pNEW>=10 && eMotionR.pNEW<20)
    {
        if(old_select!=1)tone(Buzzer_PIN,1318,200);old_select=1;
        digitalWrite(LED_1_PIN, ON);
        digitalWrite(LED_2_PIN, OFF);
        digitalWrite(LED_3_PIN, OFF);
    }
    else if(eMotionR.pNEW>=20 && eMotionR.pNEW<30)
    {
        if(old_select!=2)tone(Buzzer_PIN,1318,200);old_select=2;
        digitalWrite(LED_1_PIN, OFF);
        digitalWrite(LED_2_PIN, ON);
        digitalWrite(LED_3_PIN, OFF);
    }
    else if(eMotionR.pNEW>=40)
    {
        if(old_select!=3)tone(Buzzer_PIN,1318,200);old_select=3;
        digitalWrite(LED_1_PIN, OFF);
        digitalWrite(LED_2_PIN, OFF);
        digitalWrite(LED_3_PIN, ON);
    }
}
void count_flag(unsigned char value)
{
}
void StateMachine_to_loop(unsigned char value)
{
    unsigned char index;
    switch (value)
    {
        case 0:
            readAllIR_flag=0;
            Selector();
            SerialUSB.print(eMotionR.pNEW);
            SerialUSB.print("\t");
            SerialUSB.println(eMotionL.pNEW);

            break;
        case 1:

            if(start_cont==2000)
            {
                Motor_control(800, 800);
            }
            if(start_cont==2500)
            {
                start_flag=0;
                start_cont=0;
                MotorRest();
                sButton = 0;
            }
            readAllIR_values();
            IR_Max_Min();
            break;
        case 2:
            if(start_cont==1000)
            {
                //test_1m_flag=1;
                readAllIR_values();
                IR_calibrations();
                check_point();
                // calculate weighted average 計算權重平均
                Lp = LINE_estimation(IR_caliValues);
            }

            break;
        case 3:
            break;
        case 4:
            break;
        case 5:
            break;
    }
}
#endif