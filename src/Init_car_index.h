//
// Created by XIANGXI on 2022/10/17.
//

#ifndef JP_XA_CAR_INIT_CAR_INDEX_H
#define JP_XA_CAR_INIT_CAR_INDEX_H

#endif //JP_XA_CAR_INIT_CAR_INDEX_H
/** Encoder **/
#define Encoder_L_A 6
#define Encoder_L_B 7
#define Encoder_R_A 0
#define Encoder_R_B 1
/** Motor **/
#define MOTOR_DIR_L 11
#define MOTOR_DIR_R 12
#define MOTOR_PWM_L 3
#define MOTOR_PWM_R 2
/** Buzzer button **/
#define Buzzer_PIN 4
#define Button_PIN 5
/** LED **/
#define ON  LOW
#define OFF HIGH
#define LED_R_PIN 14
#define LED_L_PIN 13
#define LED_1_PIN 30
#define LED_2_PIN 31
#define LED_3_PIN 32
/** IR control **/
#define IR0_4_OUT 8
#define IR1_5_OUT 9
#define IR2_6_OUT 24
#define IR3_OUT 10
#define IR1 A1
#define IR2 A0
#define IR3 A3
#define IR4 A4
#define IR5 A5
#define IR_hint_L A2
#define IR_StartStop_R A6

unsigned int cCount=0;    // calibration count
unsigned char IRindex[7] = {A1, A2, A0, A3, A4, A5, A6};
int IRsensors[7]={0}, IR_caliValues[7];
int IR_vMax[7]={0, 0, 0, 0, 0, 0,0}, IR_vMin[7]={900, 900, 900, 900, 900, 900,900};
int16_t start_cont=0;
// Weighted average
int Lp;
/**  **/
void Init_Peripherals();

void Init_Peripherals()
{
    analogReadResolution(12);   // ADC: 0~4095
    analogWriteResolution(12);  // PWM: 0~4095
    /****/
    pinMode(Button_PIN, INPUT);
    pinMode(Buzzer_PIN, OUTPUT);
    pinMode(LED_1_PIN , OUTPUT);
    pinMode(LED_2_PIN , OUTPUT);
    pinMode(LED_3_PIN , OUTPUT);
    pinMode(LED_L_PIN , OUTPUT); pinMode(LED_R_PIN, OUTPUT);
    digitalWrite(LED_1_PIN,OFF);
    digitalWrite(LED_2_PIN,OFF);
    digitalWrite(LED_3_PIN,OFF);
    digitalWrite(LED_L_PIN,OFF);
    digitalWrite(LED_R_PIN,OFF);

    /** Init MOTOR **/
    pinMode(MOTOR_DIR_L, OUTPUT);pinMode(MOTOR_PWM_L, OUTPUT);
    pinMode(MOTOR_DIR_R, OUTPUT);pinMode(MOTOR_PWM_R, OUTPUT);
    /** Init IR **/
    pinMode(IR0_4_OUT , OUTPUT);
    pinMode(IR1_5_OUT , OUTPUT);
    pinMode(IR2_6_OUT , OUTPUT);
    pinMode(IR3_OUT   , OUTPUT);
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);
    pinMode(IR_hint_L, INPUT);pinMode(IR_StartStop_R, INPUT);
    /** Init Encoder **/
    // Left wheel encoder pins and interrupts/
//    pinMode(Encoder_L_A, INPUT);  pinMode(Encoder_L_B, INPUT);
//    attachInterrupt(digitalPinToInterrupt(Encoder_L_A), Encoder_LA, CHANGE);
//    attachInterrupt(digitalPinToInterrupt(Encoder_L_B), Encoder_LB, CHANGE);
//    // Right wheel encoder pins and interrupts
//    pinMode(Encoder_R_A, INPUT);  pinMode(Encoder_R_B, INPUT);
//    attachInterrupt(digitalPinToInterrupt(Encoder_R_A), Encoder_RA, CHANGE);
//    attachInterrupt(digitalPinToInterrupt(Encoder_R_B), Encoder_RB, CHANGE);
}
