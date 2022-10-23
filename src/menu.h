//
// Created by XIANGXI on 2022/10/18.
//

#ifndef JP_XA_CAR_MENU_H
#define JP_XA_CAR_MENU_H

#endif //JP_XA_CAR_MENU_H

#include <Arduino.h>
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
            if (cCount == 500) Motor_control(800, 800);   // not balanced in left and right wheels
            else if (cCount == 750) Motor_control(0, 0);
            if (cCount == 1300) MotorRest();
            readAllIR_values();
            IR_Max_Min();
            // calculate weighted average 計算權重平均
                  Lp = LINE_estimation(IRsensors);
            break;
        case 2:
            digitalWrite(LED_L_PIN, ON);
            digitalWrite(LED_R_PIN, OFF);
            if(start_cont<1000) start_cont++;
            if(start_cont==1000)
            {
                //test_1m_flag=1;
                readAllIR_values();
                IR_calibrations();
                //check_point();
                check_point2();
                // calculate weighted average 計算權重平均
                Lp = LINE_estimation(IR_caliValues);
//                if(vc_command<0.5*mm2p &&vc_f==1)
//                {
//                    vc_Command(1);
//                    digitalWrite(LED_R_PIN, OFF);
//                }
//                else if(vc_command>=1*mm2p)
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
//                        sButton = 3;
//                    }
//                }

                LINE_following_VC();
//                  LINE_following() ;
                if(test_1m_flag==1)
                {
                    test_1m[test_1m_cont]=velocity;
                    test_1_pwm[test_1m_cont]=deltaPWM_;
                    test_1_cmd[test_1m_cont]=vc_command;
                    speed_integral+=velocity;
                    test_1m_cont++;
                }

            }
            break;
        case 3:
            digitalWrite(LED_L_PIN, OFF);
            digitalWrite(LED_R_PIN, ON);
            if(pos_now<pos_stop)
            {
                readAllIR_values();
                IR_calibrations();
                //check_point();
                check_point2();
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
                    //check_point();
                    check_point2();
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
                //check_point();
                check_point2();
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
//