//
// Created by XIANGXI on 2022/10/18.
//

#ifndef JP_XA_CAR_MOTON_H
#define JP_XA_CAR_MOTON_H

#endif //JP_XA_CAR_MOTON_H

#include <Arduino.h>
#define PWMLimit  4000
#define center  300

int Kp=20, Kd=250, basePWM =1500 ;
int error_new, error_old;

void Motor_control(int speed_L, int speed_R);
void MotorRest();
void Motor_control(int speed_L, int speed_R) {
    // Left motor
    if (speed_L > 0) {
        if (speed_L > PWMLimit) speed_L = PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_L, LOW);
        analogWrite(MOTOR_PWM_L, speed_L);
    } else {
        if (speed_L < -PWMLimit) speed_L = -PWMLimit;
        digitalWrite(MOTOR_DIR_L, HIGH);
        analogWrite(MOTOR_PWM_L, -speed_L);
    }

    // Right motor
    if (speed_R > 0) {
        if (speed_R > PWMLimit) speed_R = PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_R, LOW);
        analogWrite(MOTOR_PWM_R, speed_R);
    } else {
        if (speed_R < -PWMLimit) speed_R = -PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_R, HIGH);
        analogWrite(MOTOR_PWM_R, -speed_R);
    }
}
void MotorRest() {
    // Left motor
    analogWrite(MOTOR_PWM_L, 0);
    // Right motor
    analogWrite(MOTOR_PWM_R, 0);
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