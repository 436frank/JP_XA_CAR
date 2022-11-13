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
//#define acceleration            0.01f              // 加速度 10m/s
#define acceleration            0.01f              // 加速度 5m/s
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
char vc_f=1;
//float vc_kp = 550, vc_ki = 11, vc_kd = 0;
//float vc_kp = 550, vc_ki = 35, vc_kd = 0;
float vc_kp = 200, vc_ki = 11, vc_kd = 0;
int Kp=40, Kd=300, basePWM =600 ;
//int Kp=40, Kd=300, basePWM =600 ;
float deltaPWM_ = 0;
int error_new, error_old;
float vc_error_new, vc_error_old, vc_integral = 0;
float Speed_integral=0;
float Speed_cmd_integral=0;
void Motor_control(int speed_L, int speed_R);
void MotorRest();
void vc_Command(char mod);
float vc_following();
void LINE_following();
void LINE_following_VC();
void LINE_following_PC();
void Calculate_road();
int Calculate_Acc_dec_distance(float V1);
vw PC_PD();
/** Pc-pd **/
float vc_command =0;//; //2*mm2p  // MAX  OR  等速1.8m/s
//float vc_kp_=240,vc_kv=9339,wc_kp=2.9,wc_kv=97.2; wn 0.0045
float Pc_kp=107.1,Pc_kd=5770.2,wc_kp=2.9,wc_kv=97.2;
extern volatile float angleFeedBack;
extern volatile float posFeedBack;
extern volatile float old_posFeedBack;
extern volatile float old_angleFeedBack;
extern volatile float pos_error_old;
extern volatile float angle_error_old;
bool run_flag=0;
char run_mod_flag=0;
/**()**/
void Calculate_road() {
    for (int i = prompt_cont; i > 0; --i) {
        all_road_distance[i] = all_PROMPT[i] - all_PROMPT[i - 1];
        all_road_distance_mm[i] = all_road_distance[i] * p2mm;
        all_road_radius[i] = abs((all_road_distance_mm[i] / 10) / ((all_PROMPT_w[i] - all_PROMPT_w[i - 1]) * p2r));
        if (all_road_radius[i] > 999)all_road_radius[i] = 999;
    }
}
vw PC_PD(){
    pos_error=0;angle_error=0;
    vw output;
    pos_error=Speed_cmd_integral-posFeedBack;//Speed_cmd_integral
    angle_error=0-angleFeedBack;
    output.vc= Pc_kp*pos_error + Pc_kd *(pos_error-pos_error_old);
    output.wc= wc_kp*angle_error + wc_kv *(angle_error-angle_error_old);
    pos_error_old=pos_error;
    angle_error_old=angle_error;
    return output;
}
void LINE_following_PC() {
    spd_L=0; spd_R=0;
    vw input;
    input =PC_PD();
    spd_L = input.vc;//-input.wc;
    spd_R = input.vc;//+input.wc;
    Motor_control(spd_L, spd_R);

}
void Motor_control(int speed_L, int speed_R) {
    // Left motor
    if (speed_L > 0) {
        if (speed_L > PWMLimit) speed_L = PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_L, HIGH);
        REG_TCC0_CC3 = speed_L;                               // TCC0 CC0 - on D3
        while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization
//        analogWrite(MOTOR_PWM_L, speed_L);
    } else {
        if (speed_L < -PWMLimit) speed_L = -PWMLimit;
        digitalWrite(MOTOR_DIR_L, LOW);
        REG_TCC0_CC3 = -speed_L;                               // TCC0 CC0 - on D3
        while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization
//        analogWrite(MOTOR_PWM_L, -speed_L);
    }

    // Right motor
    if (speed_R > 0) {
        if (speed_R > PWMLimit) speed_R = PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_R, HIGH);
        REG_TCC0_CC2 = speed_R;                               // TCC0 CC3 - on D2
        while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
//        analogWrite(MOTOR_PWM_R, speed_R);
    } else {
        if (speed_R < -PWMLimit) speed_R = -PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_R, LOW);
        REG_TCC0_CC2 = -speed_R;                               // TCC0 CC3 - on D2
        while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
//        analogWrite(MOTOR_PWM_R, -speed_R);
    }
}
void MotorRest() {
    // Left motor
//    analogWrite(MOTOR_PWM_L, 0);
    REG_TCC0_CC3 = 0;                               // TCC0 CC0 - on D3
    while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization
    // Right motor
//    analogWrite(MOTOR_PWM_R, 0);
    REG_TCC0_CC2 = 0;                               // TCC0 CC3 - on D2
    while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
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

float vc_following() {
    float deltaPWM, vc;
    vc_error_new = vc_command - velocity;  //現在速度誤差
    vc_integral += vc_error_new;
    float integral_temp = 2680 / vc_ki; //設一個變化量上下限 防止誤差積分飽和的問題
    //3600
    if (vc_integral > integral_temp) vc_integral = integral_temp;
        //限制 累積誤差 超過正的變化量上限 就等於正的變化量上限
    else if (vc_integral < -integral_temp) vc_integral = -integral_temp;
    //限制 累積誤差 小於負的變化量下限 就等於負的變化量下限
    deltaPWM = vc_kp * vc_error_new + vc_kd * (vc_error_new - vc_error_old) + vc_ki * vc_integral;

    vc_error_old = vc_error_new;
    if (deltaPWM>3800) deltaPWM = 3800;  //鎖 PWM最大值4000
    vc = deltaPWM;
    deltaPWM_ = deltaPWM;
    return vc;
}
void LINE_following() {
    float deltaPWM, spd_L, spd_R;
    error_new = center - Lp;
    deltaPWM = Kp * error_new + Kd * (error_new - error_old);
    error_old = error_new;
    spd_L = basePWM - deltaPWM;
    spd_R = basePWM + deltaPWM;
    Motor_control(spd_L, spd_R);
}
void LINE_following_VC() {
    float spd_L, spd_R, vc, deltaPWM;
    vc = vc_following();
    error_new = center - Lp;
    deltaPWM = Kp * error_new + Kd * (error_new - error_old);
    error_old = error_new;
    spd_L = vc - deltaPWM;
    spd_R = vc + deltaPWM;
    Motor_control(spd_L, spd_R);

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
int Calculate_Acc_dec_distance(float V1)
{
    int result=0;
    result = (float)((vc_command*vc_command)-(velocity*velocity))/(2*acceleration );
    return result;
}