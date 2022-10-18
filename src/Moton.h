//
// Created by XIANGXI on 2022/10/18.
//

#ifndef JP_XA_CAR_MOTON_H
#define JP_XA_CAR_MOTON_H

#endif //JP_XA_CAR_MOTON_H
#include <Arduino.h>

#define PWMLimit  4000
void Motor_control(int speed_L, int speed_R) {
    // Left motor
    if (speed_L > 0) {
        if (speed_L > PWMLimit) speed_L = PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_L, LOW);
//        REG_TCC0_CC0 = speed_L;                         // TCC0 CC0 - on D4
//        while (TCC0->SYNCBUSY.bit.CC0);
        analogWrite(MOTOR_PWM_L, speed_L);
    } else {
        if (speed_L < -PWMLimit) speed_L = -PWMLimit;
        digitalWrite(MOTOR_DIR_L, HIGH);
//        REG_TCC0_CC0 = -speed_L;                        // TCC0 CC0 - on D4
//        while (TCC0->SYNCBUSY.bit.CC0);
        analogWrite(MOTOR_PWM_L, -speed_L);
    }

    // Right motor
    if (speed_R > 0) {
        if (speed_R > PWMLimit) speed_R = PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_R, LOW);
//        REG_TCC0_CC3 = speed_R;                         // TCC0 CC3 - on D3
//        while (TCC0->SYNCBUSY.bit.CC3);
        analogWrite(MOTOR_PWM_R, speed_R);
    } else {
        if (speed_R < -PWMLimit) speed_R = -PWMLimit;
        // Left motor
        digitalWrite(MOTOR_DIR_R, HIGH);
//        REG_TCC0_CC3 = -speed_R;                         // TCC0 CC3 - on D3
//        while (TCC0->SYNCBUSY.bit.CC3);
        analogWrite(MOTOR_PWM_R, -speed_R);
    }
}