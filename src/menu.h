//
// Created by XIANGXI on 2022/10/18.
//

#ifndef JP_XA_CAR_MENU_H
#define JP_XA_CAR_MENU_H
#endif //JP_XA_CAR_MENU_H

#include <Arduino.h>
//#define old_menu
#define new_menu
/**  variable  **/
extern float Speed_cmd_integral;
int sprint_cnt=0;
int distance_flag=0;
float distance=0;
int Notice_subtract_distance_flag=0;

int old_sprint_cnt=0;



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
void clearAll();
void clearAll_();
void Selector_QEI();
void Selector_Observer();
void StateMachine_to_loop(unsigned char value);
#ifdef new_menu

void clearAll() {
    end_flag=0;
    distance_flag=0;
    distance=0;
    Notice_subtract_distance_flag=0;
    run_flag = 0;
    vc_command=0;
    sprint_cnt=0;
    old_pos=0;
    old_sprint_cnt=0;
    record_data_flag=0;
    run_mod_flag=0;

    start_flag = 0;
    IR_MAX_MIN_value_flag = 0;
    readAllIR_flag = 0;
    LINE_following_PC_flag = 0;
    LINE_flag=0;
    MotorRest();
    start_cont = 0;
    error_old=0;
    error_new=0;
    pos_error_old=0;
    angle_error_old=0;
    pos_error=0;
    deltaPWM=0;
    angle_velocity=0;
    vc_integral = 0;
    angle_error=0;
    posFeedBack=0;
    angleFeedBack=0;
    Speed_integral=0;
    Speed_cmd_integral=0;
    Pcount_R = 0;
    Pcount_L = 0;
    count_L = 0;
    count_R = 0;
    sButton=0;
}
void clearAll_() {
    run_flag = 0;
    vc_command=0;
    sprint_cnt=0;
    old_pos=0;
    old_sprint_cnt=0;
    end_flag=0;
    distance_flag=0;
    distance=0;
    Notice_subtract_distance_flag=0;
    run_mod_flag=0;

    start_flag = 0;
    IR_MAX_MIN_value_flag = 0;
    readAllIR_flag = 0;
    LINE_following_PC_flag = 0;
    LINE_flag=0;
    MotorRest();
    start_cont = 0;
    error_old=0;
    error_new=0;
    pos_error_old=0;
    angle_error_old=0;
    pos_error=0;
    deltaPWM=0;
    angle_velocity=0;
    vc_integral = 0;
    angle_error=0;
    posFeedBack=0;
    angleFeedBack=0;
    Speed_integral=0;
    Speed_cmd_integral=0;
    Pcount_R = 0;
    Pcount_L = 0;
    count_L = 0;
    count_R = 0;
}
void Selector_QEI() {
    if (eMotionL.pNEW < 20) {
        if (old_enter != 0) {
            tone(Buzzer_PIN, 800, 200);
            digitalWrite(LED_L_PIN, OFF);
            digitalWrite(LED_R_PIN, OFF);
            old_enter = 0;
        }
        if (eMotionR.pNEW < 10) {
            if (old_select != 0) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, OFF);
                digitalWrite(LED_2_PIN, OFF);
                digitalWrite(LED_3_PIN, OFF);
                old_select = 0;
            }
        } else if (eMotionR.pNEW >= 10 && eMotionR.pNEW < 20)//選擇1
        {
            if (old_select != 1) {
                digitalWrite(LED_1_PIN, ON);
                digitalWrite(LED_2_PIN, OFF);
                digitalWrite(LED_3_PIN, OFF);
                tone(Buzzer_PIN, 1318, 200);
                old_select = 1;
            }


        } else if (eMotionR.pNEW >= 20 && eMotionR.pNEW < 30)//選擇2
        {
            if (old_select != 2) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, OFF);
                digitalWrite(LED_2_PIN, ON);
                digitalWrite(LED_3_PIN, OFF);
                old_select = 2;
            }
        } else if (eMotionR.pNEW >= 40 && eMotionR.pNEW < 50)//選擇3
        {
            if (old_select != 3) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, OFF);
                digitalWrite(LED_2_PIN, OFF);
                digitalWrite(LED_3_PIN, ON);
                old_select = 3;
            }
        } else if (eMotionR.pNEW >= 50 && eMotionR.pNEW < 60)//選擇4
        {
            if (old_select != 4) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, OFF);
                digitalWrite(LED_2_PIN, ON);
                digitalWrite(LED_3_PIN, ON);
                old_select = 4;
            }
        } else if (eMotionR.pNEW >= 60 && eMotionR.pNEW < 70)//選擇5
        {
            if (old_select != 5) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, ON);
                digitalWrite(LED_2_PIN, OFF);
                digitalWrite(LED_3_PIN, ON);
                old_select = 5;
            }
        } else if (eMotionR.pNEW >= 70)                     //選擇6
        {
            if (old_select != 6) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, ON);
                digitalWrite(LED_2_PIN, ON);
                digitalWrite(LED_3_PIN, OFF);
                old_select = 6;
            }
        }
    } else if (eMotionL.pNEW >= 20) {
        if (old_enter != 1) {
            tone(Buzzer_PIN, 800, 200);
            digitalWrite(LED_L_PIN, ON);
            digitalWrite(LED_R_PIN, OFF);
            old_enter = 1;
        }
    }
}
void Selector_Observer() {
    if (Pcount_L < 20)//未確認
    {
        if (old_enter != 0) {
            tone(Buzzer_PIN, 800, 200);
            digitalWrite(LED_L_PIN, OFF);
            digitalWrite(LED_R_PIN, OFF);
            old_enter = 0;
        }                //選擇0
        if (Pcount_R < 10)
        {
            if (old_select != 0)
            {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, OFF);
                digitalWrite(LED_2_PIN, OFF);
                digitalWrite(LED_3_PIN, OFF);
                old_select = 0;
            }
        }
        else if (Pcount_R >= 10 && Pcount_R < 20)//選擇1
        {
            if (old_select != 1) {
                digitalWrite(LED_1_PIN, ON);
                digitalWrite(LED_2_PIN, OFF);
                digitalWrite(LED_3_PIN, OFF);
                tone(Buzzer_PIN, 1318, 200);
                old_select = 1;
            }
        }
        else if (Pcount_R >= 20 && Pcount_R < 30)//選擇2
        {
            if (old_select != 2) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, OFF);
                digitalWrite(LED_2_PIN, ON);
                digitalWrite(LED_3_PIN, OFF);
                old_select = 2;
            }
        }
        else if (Pcount_R >= 40 && Pcount_R < 50)//選擇3
        {
            if (old_select != 3) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, OFF);
                digitalWrite(LED_2_PIN, OFF);
                digitalWrite(LED_3_PIN, ON);
                old_select = 3;
            }
        }
        else if (Pcount_R >= 50 && Pcount_R < 60)//選擇4
        {
            if (old_select != 4) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, OFF);
                digitalWrite(LED_2_PIN, ON);
                digitalWrite(LED_3_PIN, ON);
                old_select = 4;
            }
        }
        else if (Pcount_R >= 60 && Pcount_R < 70)//選擇5
        {
            if (old_select != 5) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, ON);
                digitalWrite(LED_2_PIN, OFF);
                digitalWrite(LED_3_PIN, ON);
                old_select = 5;
            }
        }
        else if (Pcount_R >= 70)                 //選擇6
        {
            if (old_select != 6) {
                tone(Buzzer_PIN, 1318, 200);
                digitalWrite(LED_1_PIN, ON);
                digitalWrite(LED_2_PIN, ON);
                digitalWrite(LED_3_PIN, OFF);
                old_select = 6;
            }
        }
    }
    else if (Pcount_L >= 20)
    {
        if (old_enter != 1) {
            tone(Buzzer_PIN, 800, 200);
            digitalWrite(LED_L_PIN, ON);
            digitalWrite(LED_R_PIN, OFF);
            clearAll();
            old_enter = 1;
        }
    } //確認
}
void StateMachine_to_loop(unsigned char value)
{
    switch (value)
    {
        case 0:  //選擇模式
            MotorRest();
            Selector_Observer();
            break;
        case 1: //取IR最大最小
            delay(1000);
            Motor_control(500, 500);
            IR_MAX_MIN_value_flag = 1;
            delay(300);
            IR_MAX_MIN_value_flag = 0;
            clearAll();
            break;
        case 2: //搜尋
            record_data_flag=1;
            if(mpu6500_set_flag==1){
                clearAll_();
                delay(200);
                mpu6500AutoOffset(1000, 100);
                mpu6500_set_flag=0;
            }
            if(vc_command<1.1*mm2p) {
                run_mod_flag = 1;

            } else{
                run_mod_flag = 2;
                vc_command = 1.1*mm2p;
            }
            LINE_following_PC_flag = 1;
            if(end_flag==1){
            delay(200);
            clearAll();
            }
            break;
        case 3:
            delay(1000);
            mpu6500AutoOffset(1000, 100);
            Motor_control(1100, -1100);
//            Motor_control(700, -700);
//            Motor_control(950, -950);
            test_1m_flag = 1;//抓數據 旗標
            delay(800);
            test_1m_flag = 0;//關閉 抓數據旗標
            clearAll();
            break;
        case 4: //抓資料
            NVIC_DisableIRQ(TC3_IRQn);
            for (int i = 0; i < test_1m_cont; ++i)
            {
                SerialUSB.print(test_1_v[i] * p2mm, 4);
                SerialUSB.print("\t");
                SerialUSB.print(test_1_cmd[i] * p2mm, 4);
                SerialUSB.print("\t");
                SerialUSB.print(test_1_pwm_r[i] * p2mm, 1);
                SerialUSB.print("\t");
                SerialUSB.print(test_1_pwm_l[i] * p2mm, 1);
                SerialUSB.print("\n");
            }
            NVIC_EnableIRQ(TC3_IRQn);
            clearAll();
            break;
        case 5:
            Calculate_road();
            for (int i = 0; i < prompt_cont; ++i) {
                SerialUSB.print(all_road_distance[i]);
                SerialUSB.print("\t");
                SerialUSB.print(all_road_radius[i]);
                SerialUSB.print("\t");
                SerialUSB.print(all_road_speed_max[i]);
                SerialUSB.print("\t");
                SerialUSB.print(all_PROMPT[i]);
                SerialUSB.print("\n");
            }
            clearAll();
            break;
        case 6:
            if(mpu6500_set_flag==1){
                clearAll_();
                delay(200);
                mpu6500AutoOffset(1000, 100);
                mpu6500_set_flag=0;
                run_flag = 1;
            }
            if(run_flag==1)
            {
                if(old_sprint_cnt==0){
                    if (vc_command < all_road_speed_max[sprint_cnt]) {
                        run_mod_flag = 1;
                    } else{run_mod_flag = 2;}
                }
                if(distance_flag==1)
                {
                distance+=all_road_distance[sprint_cnt];
                distance_flag=0;
                }
                if(old_sprint_cnt!=sprint_cnt){
                    if (vc_command == all_road_speed_max[sprint_cnt]) {
                        run_mod_flag = 2;
                        old_sprint_cnt = sprint_cnt;
                        Notice_subtract_distance_flag=1;
                    }
                    else if(vc_command>all_road_speed_max[sprint_cnt]){
                        run_mod_flag = 3;
                    }
                    else if(vc_command<all_road_speed_max[sprint_cnt]){
                        run_mod_flag = 1;
                    }
                }
                if(Notice_subtract_distance_flag==1)
                {
                    if(posFeedBack>=(distance-Calculate_Acc_dec_distance()))
                    {
                        run_mod_flag = 3;
                        Notice_subtract_distance_flag=0;
                    }

                }
            }
            if(end_flag==1){
                delay(200);
                clearAll();
            }
            break;

    }
}
#endif