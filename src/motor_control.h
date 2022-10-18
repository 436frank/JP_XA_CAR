
// Created by XIANGXI on 2022/8/3.
//

#ifndef ARDU_XAING_MOTOR_CONTROL_H
#define ARDU_XAING_MOTOR_CONTROL_H

#endif //ARDU_XAING_MOTOR_CONTROL_H

#define sample_time             0.0005f              // 控制時間(s)
#define wheel_diameter          24.9f                // 輪直徑(mm)
#define wheel_pulses            8                    // (pulses)
#define encoder_resolution      (wheel_pulses * 4)   // (pulses/r)
#define acceleration            0.002f               // 加速度(mm/0.5ms)
const float mm2p = (encoder_resolution / (wheel_diameter * PI)); // 1 mm    ~= 0.449893 pulse

const float p2mm = ((wheel_diameter * PI) / encoder_resolution); // 1 pulse ~=  2.227437mm   直徑 x 圓周率=圓周長   圓周長/encoder解析度 = 1個dpi 移動多少
const float p2r = ((wheel_diameter * PI) / (encoder_resolution * 85));//解析度放大85倍
const float acceleration_p = (acceleration * mm2p);  // 加速度(pulse/0.5ms)


//float vc_kp = 550, vc_ki = 11, vc_kd = 0;
float vc_kp = 3000, vc_ki = 35, vc_kd = 0;
float vc_command =0; //2*mm2p  // MAX  OR  等速1.8m/s
//float vc_command=600;// MAX  OR  等速2.m/s
//float vc_command=339;//    1.3m/s
//float vc_command=208;//    0.8m/s
float velocity = 0;

char vc_f=1;

float vc_error_new, vc_error_old, vc_integral = 0;

//uint32_t vc_command=373;

void Calculate_road_curvature(); //計算路段曲率
float deltaPWM_ = 0;


char vc_flag;

float vc_following();

void LINE_following();

void LINE_following_VC();

void LINE_following_park_well();

void vc_t_Command();

void Motor_control(int speed_L, int speed_R);

void MotorRest();

void pwm_Command(char mod);

void vc_Command(char vc);

void ladder_command();
//#define



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

void Calculate_road() {
    for (int i = prompt_cont; i > 0; --i) {
        all_road_distance[i] = all_PROMPT[i] - all_PROMPT[i - 1];
        all_road_distance_mm[i] = all_road_distance[i] * p2mm;
        all_road_radius[i] = abs((all_road_distance_mm[i] / 10) / ((all_PROMPT_w[i] - all_PROMPT_w[i - 1]) * p2r));
        if (all_road_radius[i] > 999)all_road_radius[i] = 999;
    }
}

void pwm_Command(char mod) {
    switch (mod) {
        case 1:
            Kp = 20;
            Kd = 250;
            basePWM = 2000;
            break;
        case 2:
            Kp = 35;
            Kd = 400;
            basePWM = 2800;
            break;
        case 3:

            break;
    }
}

void vc_t_Command_Calculate() {

}

void vc_Command(char vc) {
    switch (vc) {
        case 1:
            vc_command +=acceleration_p;//(sample_time * acceleration_p);
            break;
        case 2:
            vc_command =acceleration_p;//acceleration_p;//(sample_time * acceleration_p);
            break;
        case 3:
            vc_command -= acceleration_p;//acceleration_p;//(sample_time * acceleration_p);

            if (vc_command <= 0)vc_command = 0;
            break;


    }
}

void Motor_control(int speed_L, int speed_R) {
    // Left motor
    if (speed_L > 0) {
        if (speed_L > PWMLimit) speed_L = PWMLimit;
        // Left motor
        digitalWrite(BIN_1, HIGH);
        digitalWrite(BIN_2, LOW);
        REG_TCC0_CC0 = speed_L;                         // TCC0 CC0 - on D4
        while (TCC0->SYNCBUSY.bit.CC0);
        //analogWrite(PWML, speed_L);
    } else {
        if (speed_L < -PWMLimit) speed_L = -PWMLimit;
        digitalWrite(BIN_1, LOW);
        digitalWrite(BIN_2, HIGH);
        REG_TCC0_CC0 = -speed_L;                        // TCC0 CC0 - on D4
        while (TCC0->SYNCBUSY.bit.CC0);
        //analogWrite(PWML, -speed_L);
    }

    // Right motor
    if (speed_R > 0) {
        if (speed_R > PWMLimit) speed_R = PWMLimit;
        // Left motor
        digitalWrite(AIN_1, LOW);
        digitalWrite(AIN_2, HIGH);
        REG_TCC0_CC3 = speed_R;                         // TCC0 CC3 - on D3
        while (TCC0->SYNCBUSY.bit.CC3);
        //analogWrite(PWMR, speed_R);
    } else {
        if (speed_R < -PWMLimit) speed_R = -PWMLimit;
        // Left motor
        digitalWrite(AIN_1, HIGH);
        digitalWrite(AIN_2, LOW);
        REG_TCC0_CC3 = -speed_R;                         // TCC0 CC3 - on D3
        while (TCC0->SYNCBUSY.bit.CC3);
        //analogWrite(PWMR, -speed_R);
    }
}

// Motor at rest
void MotorRest() {
    // Left motor
    digitalWrite(BIN_1, LOW);
    digitalWrite(BIN_2, LOW);
    REG_TCC0_CC0 = 0;                        // TCC0 CC0 - on D4
    while (TCC0->SYNCBUSY.bit.CC0);
    //analogWrite(PWML, PWMLimit);
    // Right motor
    digitalWrite(AIN_1, LOW);
    digitalWrite(AIN_2, LOW);
    REG_TCC0_CC3 = 0;                         // TCC0 CC3 - on D3
    while (TCC0->SYNCBUSY.bit.CC3);
    //analogWrite(PWMR, PWMLimit);
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



