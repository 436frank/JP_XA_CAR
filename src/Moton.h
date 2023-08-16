//
// Created by XIANGXI on 2022/10/18.
//

#ifndef JP_XA_CAR_MOTON_H
#define JP_XA_CAR_MOTON_H
#endif //JP_XA_CAR_MOTON_H

#include <Arduino.h>
#include "ck_point.h"

#define sample_time             0.0005f              // 控制時間(s)
#define wheel_diameter          24.85f                // 輪直徑(mm)
#define wheel_pulses            8                    // (pulses)
#define encoder_resolution      (wheel_pulses * 4)   // (pulses/r)
#define acceleration            0.01f              // 加速度 10m/s
#define CAR_WIDE                100.0f               // 車寬(mm)
const float mm2p = (encoder_resolution / (wheel_diameter * PI)); // 1 mm    ~= 0.449893 pulse
const float p2mm = ((wheel_diameter * PI) / encoder_resolution); // 1 pulse ~=  2.227437mm   直徑 x 圓周率=圓周長   圓周長/encoder解析度 = 1個dpi 移動多少
const float p2r = ((wheel_diameter * PI) / (encoder_resolution * 85));//解析度放大85倍
const float acceleration_p = (acceleration * mm2p);  // 加速度(pulse/1ms)
#define PWMLimit  2400
#define center  300
typedef struct vc_wc{
    float vc;
    float wc;
}vw;
float spd_L, spd_R;
float pos_error=0,angle_error=0;

/** Pc-PD **/
float vc_command =0;

float Pc_kp=74.3,Pc_kd=4580.6;
float wc_kp=2.2  ,wc_kd=21.4;
float Kp=0.0292, Kd=4.865, basePWM =0 ;

bool run_flag=0;

char run_mod_flag=0;


float deltaPWM=0;
float angle_velocity=0;
int error_new, error_old;
float Speed_integral=0;
float Speed_cmd_integral=0;
extern volatile float angleFeedBack;
extern volatile float posFeedBack;
extern volatile float pos_error_old;
extern volatile float angle_error_old;



void Motor_control(int speed_L, int speed_R);
void MotorRest();
void vc_Command(char mod);
float LINE_following();
void LINE_following_VC();
void LINE_following_PC();
void Calculate_road();
vw PC_WC_PD();

/**()**/
void Motor_control(int speed_L, int speed_R) {
    // Left motor
    if (speed_L > 0) {
        if (speed_L > PWMLimit) speed_L = PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_L, HIGH);
        REG_TCC0_CC3 = speed_L;                               // TCC0 CC0 - on D3
        while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization
    } else {
        if (speed_L < -PWMLimit) speed_L = -PWMLimit;
        digitalWrite(MOTOR_DIR_L, LOW);
        REG_TCC0_CC3 = -speed_L;                               // TCC0 CC0 - on D3
        while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization
    }

    // Right motor
    if (speed_R > 0) {
        if (speed_R > PWMLimit) speed_R = PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_R, HIGH);
        REG_TCC0_CC2 = speed_R;                               // TCC0 CC3 - on D2
        while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
    } else {
        if (speed_R < -PWMLimit) speed_R = -PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_R, LOW);
        REG_TCC0_CC2 = -speed_R;                               // TCC0 CC3 - on D2
        while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
    }
}
void MotorRest() {
    REG_TCC0_CC3 = 0;                               // TCC0 CC0 - on D3
    while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization
    REG_TCC0_CC2 = 0;                               // TCC0 CC3 - on D2
    while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
}
float LINE_following() {

    error_new = center - Lp;
    deltaPWM = Kp * error_new + Kd * (error_new - error_old);
    error_old = error_new;
    return deltaPWM;
}
vw PC_WC_PD(){
    pos_error=0;angle_error=0;
    angle_velocity+=LINE_following();
    vw output;
    pos_error=Speed_cmd_integral-posFeedBack;//Speed_cmd_integral
    angle_error=angle_velocity-angleFeedBack;
    output.vc= Pc_kp*pos_error + Pc_kd *(pos_error-pos_error_old);
    output.wc= wc_kp*angle_error + wc_kd *(angle_error-angle_error_old);
    pos_error_old=pos_error;
    angle_error_old=angle_error;
    return output;
}
void LINE_following_PC() {
    spd_L=0; spd_R=0;
    vw input;
    input =PC_WC_PD();
    spd_L = input.vc-input.wc;
    spd_R = input.vc+input.wc;
    Motor_control(spd_L, spd_R);
}
void vc_Command(char mod) {
    switch (mod) {
        case 1:
            vc_command +=acceleration_p;//(sample_time * acceleration_p);
            break;
        case 2:

            break;
        case 3:
            vc_command -= acceleration_p;//acceleration_p;//(sample_time * acceleration_p);
            if (vc_command <= 0)vc_command = 0;
            break;
    }
    Speed_integral+=velFeedBack;
    Speed_cmd_integral+=vc_command;
}
void LINE_following_park_well() {
    int spd_L, spd_R, b_pwm = 1000, deltaPWM;
    error_new = center - Lp;
    deltaPWM = Kp * error_new + Kd * (error_new - error_old);
    error_old = error_new;
    if (park_well_cont > 1) b_pwm = 0;
    spd_L = b_pwm - deltaPWM;
    spd_R = b_pwm + deltaPWM;
    Motor_control(spd_L, spd_R);
}
void Calculate_road() {
    for(int i = 0; i < 200; i++) {
        all_road_speed_max[i] = 1.25*mm2p;
        all_road_speed_max2[i] = 1.25*mm2p;
        all_road_speed_max3[i] = 1.25*mm2p;
    }
    for (int i = prompt_cont; i > 0; --i) {
        all_road_distance[i] = all_PROMPT[i] - all_PROMPT[i - 1];//線段距離 單位pos
        all_road_distance_mm[i] = all_road_distance[i] * p2mm;//線段距離 單位mm
        all_road_radius[i] = abs(
                (all_road_distance_mm[i] / 10) / ((all_PROMPT_w[i] - all_PROMPT_w[i - 1]) * p2r));//估測角度
        if (all_road_radius[i] > 999)all_road_radius[i] = 999;//估測出的線段半徑所上限999
        /** SP_MOD 1 選單3**/
        if ((all_road_radius[i] >= 270) && (all_road_distance_mm[i] > 100))//半徑大於300 且現段長度超過10cm 配速1m/s
        {all_road_speed_max[i] = 1.5* mm2p;}
        else{all_road_speed_max[i] = 1.5* mm2p;}
        /** SP_MOD 2 **/
        if ((all_road_radius[i] >= 270)&& (all_road_distance_mm[i] > 500) )//半徑大於300 且現段長度超過10cm 配速1m/s   //&& (all_road_distance_mm[i] > 300)
        {all_road_speed_max2[i] = 2.8 * mm2p;}
        else if ((all_road_radius[i] >= 270)&& (all_road_distance_mm[i] < 500) )//半徑大於300 且現段長度超過10cm 配速1m/s   //&& (all_road_distance_mm[i] > 300)
        {all_road_speed_max2[i] = 2.5 * mm2p;}
        else if((all_road_radius[i]<280)&&(all_road_radius[i]>180))
        {all_road_speed_max2[i] = 2.1 * mm2p;}
        else if((all_road_radius[i]<180)&&(all_road_radius[i]>50))
        {all_road_speed_max2[i] = 1.6* mm2p;}
        else if(all_road_radius[i]<50)
        {all_road_speed_max2[i] = 1.3 * mm2p;}
        /** SP_MOD 3**/
        if ((all_road_radius[i] >= 300) && (all_road_distance_mm[i] > 500))//半徑大於300 且現段長度超過10cm 配速1m/s
        {all_road_speed_max3[i] = 2.8 * mm2p;}  //2.6
        else if((all_road_radius[i]>300)&&(all_road_distance_mm[i] < 500) )
        {all_road_speed_max3[i] = 2.5 * mm2p;}//1.35
        else if((all_road_radius[i]<=300)&&(all_road_radius[i]>180)&&(all_road_radius[i]>200 ))
        {all_road_speed_max3[i] = 1.4 * mm2p;}
        else if((all_road_radius[i]<=200)&&(all_road_radius[i]>=100 ))
        {all_road_speed_max3[i] = 1.3 * mm2p;}
    }
    all_road_speed_max[0]=1.25*mm2p;
    all_road_speed_max2[0]=1.25*mm2p;
    all_road_speed_max3[0]=1.25*mm2p;
    all_road_speed_max[prompt_cont+1]=1.25*mm2p;
    all_road_speed_max2[prompt_cont+1]=1.25*mm2p;
    all_road_speed_max3[prompt_cont+1]=1.25*mm2p;
}
float Calculate_Acc_dec_distance()
{
    float result=0;
    result = (float)((vc_command*vc_command)-(all_road_speed_max[sprint_cnt+1]*all_road_speed_max[sprint_cnt+1]))/(2*acceleration_p );
    return result;
}