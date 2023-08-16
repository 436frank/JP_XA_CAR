//
// Created by XIANGXI on 2022/10/17.
//

#ifndef JP_XA_CAR_INIT_CAR_INDEX_H
#define JP_XA_CAR_INIT_CAR_INDEX_H
#endif //JP_XA_CAR_INIT_CAR_INDEX_H


#include <SPI.h>
#include "mpu6500.h"

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
#define countButton 5
/** LED **/
#define ON  LOW
#define OFF HIGH
#define LED_R_PIN 13
#define LED_L_PIN 14
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
#define IRmin         5
#define IRmax         4000
#define on_the_LINE_IR_val 1159
/** IR variable **/
unsigned int cCount=0;    // calibration count
unsigned char IRindex[7] = {A1, A2, A0, A3, A4, A5, A6};
int IRsensors[7]={0}, IR_caliValues[7];
int IR_vMax[7]={0, 0, 0, 0, 0, 0,0}, IR_vMin[7]={900, 900, 900, 900, 900, 900,900};
int Lp;// Weighted average
int OLD_LPos=0;// old Weighted average
bool readAllIR_flag=0;
bool IR_MAX_MIN_value_flag=0;
bool LINE_flag=0;
/** menu  variable **/
int16_t start_cont=0;
bool start_flag =0;//
/** Encoder variable **/
struct Motion_status {
    float  pOLD;
    float  pNEW;
    float  pERROR;
    float  vOLD;
    float  vNEW;
};
Motion_status eMotionR, eMotionL;
float EstR_acceleration = 0, EstL_acceleration;
volatile float count_L=0, count_R=0, Pcount_L=0, Pcount_R=0;  // initialize variables
volatile float pos_error_old=0;
volatile float angle_error_old=0;
volatile float Pcount_C=0;
volatile float omegaFeedBack=0;
volatile float old_posFeedBack;
volatile float angleFeedBack=0;
volatile float old_angleFeedBack;
volatile float posFeedBack=0;
volatile float velFeedBack=0;
volatile float u_p=0;
volatile float u_v=0;
volatile float  Vcount_L=0, Vcount_R=0;
float pos_now=0; //現在的pos位置
float pos_old=0; //現在的pos位置
float velocity = 0;
/** Button variable **/
boolean button, buttonPressed;
unsigned char sButton = 0;  // status of Button
unsigned int bPressCount = 0, bReleaseCount=0;
bool LINE_following_PC_flag = 0;
/**  selector variable **/
uint8_t old_select=0;
uint8_t old_enter=0;
/**  moton variable **/
extern const float mm2p;
/**    mpu6500 variable   **/
bool mpu6500_set_flag=0;
/**                    **/
void READ_QEI();
void Observer();
void QEI_filter();
void Encoder_LA();
void Encoder_LB();
void Encoder_RA();
void Encoder_RB();
void readAllIR_values();
void IR_calibrations();
void IR_Max_Min();
int LINE_estimation(int IRvalues[]);
void checkButton();
void Init_Peripherals();
void AdcBooster();
void setupTimers();
void setupPWM();


/** Encoder() **/
void READ_QEI() {
    // calulate the speed information of the wheel,  unit:pulses/T_interval
    Vcount_R = count_R - Pcount_R;
    Vcount_L = count_L - Pcount_L;
    // keep previous pulse counts
    Pcount_R = count_R;
    Pcount_L = count_L;

}
void Observer() {
    Pcount_C = (Pcount_R + Pcount_L) / 2.0f;
    omegaFeedBack = -sen.gyro.Z;    // L- R+
    angleFeedBack = -sen.angle.Z;   // L- R+
    posFeedBack += u_p;
    velFeedBack += u_v;
    u_p = (0.14f * (Pcount_C - posFeedBack)) + velFeedBack;
    u_v = (0.0064f * (Pcount_C - posFeedBack)) + (sen.accel.Y * mm2p);

}  //感測器融合
void QEI_filter() {
    pos_now=((eMotionL.pNEW + eMotionR.pNEW)/2);  //pos 左右相加/2
    velocity=((eMotionL.vNEW + eMotionR.vNEW)/2);  //pos 左右相加/2
    // wn = 0.1, zeta = 1, Kp = wn^2 = 0.01, Kv = 2*zeta*wn = 0.2
    // update for velocity estimation
    eMotionR.vNEW = eMotionR.vOLD + EstR_acceleration;
    eMotionL.vNEW = eMotionL.vOLD + EstL_acceleration;
    // update for position estimation
    eMotionR.pNEW = eMotionR.pOLD + eMotionR.vOLD;
    eMotionL.pNEW = eMotionL.pOLD + eMotionL.vOLD;
    // update for estimation error
    eMotionR.pERROR = Pcount_R - eMotionR.pNEW;
    eMotionL.pERROR = Pcount_L - eMotionL.pNEW;
    // update for acceleration estimation
    EstR_acceleration = eMotionR.pERROR / 100.0 - eMotionR.vNEW / 5.0; //pError*Kp - vNew*Kv
    EstL_acceleration = eMotionL.pERROR / 100.0 - eMotionL.vNEW / 5.0; //pError*Kp - vNew*Kv
    // update OLD estimation
    eMotionR.vOLD = eMotionR.vNEW;
    eMotionR.pOLD = eMotionR.pNEW;
    eMotionL.vOLD = eMotionL.vNEW;
    eMotionL.pOLD = eMotionL.pNEW;
}
void Encoder_LA() {
    if (digitalRead(Encoder_L_A)==digitalRead(Encoder_L_B)) {count_L--;}
    else {count_L++;}
}
void Encoder_LB() {
    if (digitalRead(Encoder_L_A)==digitalRead(Encoder_L_B)) {count_L++;}
    else {count_L--;}
}
void Encoder_RA() {
    if (digitalRead(Encoder_R_A)==digitalRead(Encoder_R_B)) {count_R--;}
    else {count_R++;}
}
void Encoder_RB() {
    if (digitalRead(Encoder_R_A)==digitalRead(Encoder_R_B)) {count_R++;}
    else {count_R--;}
}
/** IR() **/
void readAllIR_values() {
    digitalWrite(IR3_OUT,HIGH);
    IRsensors[3]=analogRead(IR3);
    digitalWrite(IR3_OUT,LOW);
    digitalWrite(IR0_4_OUT,HIGH);
    IRsensors[0]=analogRead(IR_hint_L);
    IRsensors[4]=analogRead(IR4);
    digitalWrite(IR0_4_OUT,LOW);
    digitalWrite(IR2_6_OUT,HIGH);
    IRsensors[6]=analogRead(IR_StartStop_R);
    IRsensors[2]=analogRead(IR2);
    digitalWrite(IR2_6_OUT,LOW);
    digitalWrite(IR1_5_OUT,HIGH);
    IRsensors[1]=analogRead(IR1);
    IRsensors[5]=analogRead(IR5);
    digitalWrite(IR1_5_OUT,LOW);
}
void IR_calibrations() {
    unsigned char index;
    long temp;
    for (index = 1; index < 6; index++) {
        temp = long(IRmax - IRmin) * long(IRsensors[index] - IR_vMin[index]);
        IR_caliValues[index] = IRmin + temp / (IR_vMax[index] - IR_vMin[index]);
        if (IR_caliValues[index] < 0)
            IR_caliValues[index] = 0;
    }
}
void IR_Max_Min() {
    unsigned char index;
    for (index = 1; index < 6; index++) {
        if (IR_vMax[index] < IRsensors[index])
            IR_vMax[index] = IRsensors[index];
        if (IR_vMin[index] > IRsensors[index])
            IR_vMin[index] = IRsensors[index];
    }
}
int LINE_estimation(int IRvalues[]) {
    float temp_n = 0, temp_d = 0;
    int LPos;
    if(IRvalues[3]>on_the_LINE_IR_val || IRvalues[2]>on_the_LINE_IR_val || IRvalues[4]>on_the_LINE_IR_val || IRvalues[5]>on_the_LINE_IR_val || IRvalues[1]>on_the_LINE_IR_val)
    {
        temp_n=(float) 1*IRvalues[1]+2*IRvalues[2]+3*IRvalues[3]+4*IRvalues[4]+5*IRvalues[5];
        temp_d=(float) IRvalues[1]+IRvalues[2]+IRvalues[3]+IRvalues[4]+IRvalues[5];
        LPos = (int) (temp_n / temp_d * 100.0);
    }
    else
    {
        LPos=OLD_LPos;
    }
    if (LPos > 460) LPos = 460;
    else if (LPos < 40) LPos = 40;
    OLD_LPos=LPos;
    return (LPos);
}
/** Button() **/
void checkButton() {
    if (cCount < 1500) cCount++;
    button = digitalRead(Button_PIN);
    if (button == LOW) // button pressed
    {
        if (buttonPressed == LOW) // button pressed not yet confirmed
        {
            bPressCount++;
            if (bPressCount > countButton) {
                buttonPressed = HIGH;           // button pressed confirmed
                bPressCount = 0;                // reset pressed count value
            }
        } else {                              // button pressed confirmed and button not yet released
            bReleaseCount = 0;
            bPressCount = 0;
        }
    } else // button==HIGH, released
    {
        if (buttonPressed == LOW) // button pressed not yet confirmed
        {
            bReleaseCount = 0;
            bPressCount = 0;
        } else                    // button pressed confirmed
        {
            bReleaseCount++;
            if (bReleaseCount > countButton) {
                buttonPressed = LOW;            // button released confirmed
                bReleaseCount = 0;              // reset released count value
//                sButton++;                      // change Button status value
                digitalWrite(LED_L_PIN,OFF);
                digitalWrite(LED_R_PIN,OFF);
                sButton=old_select;
                mpu6500_set_flag=1;
                start_flag=1;
                //start_cont=0;
                tone(Buzzer_PIN,1318,200);
//                if (sButton == 1) cCount = 0;     // reset
//                if (sButton > 5) sButton = 0;       // the largest value of Button status is 3
            }
        }
    }
}
/** Init() **/
void Init_Peripherals()
{
    /** Init ADC PWM ->12Bit **/
    analogReadResolution(12);   // ADC: 0~4095
    analogWriteResolution(12);  // PWM: 0~4095
    /** Init Button Buzzer Led */
    pinMode(Button_PIN, INPUT); //設為輸入
    pinMode(Buzzer_PIN, OUTPUT);//設為輸出
    pinMode(LED_1_PIN , OUTPUT);//設為輸出
    pinMode(LED_2_PIN , OUTPUT);//設為輸出
    pinMode(LED_3_PIN , OUTPUT);//設為輸出
    pinMode(LED_L_PIN , OUTPUT); pinMode(LED_R_PIN, OUTPUT);//設為輸出
    digitalWrite(LED_1_PIN,OFF);//LED關閉
    digitalWrite(LED_2_PIN,OFF);//LED關閉
    digitalWrite(LED_3_PIN,OFF);//LED關閉
    digitalWrite(LED_L_PIN,OFF);//LED關閉
    digitalWrite(LED_R_PIN,OFF);//LED關閉
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
    //Left wheel encoder pins and interrupts/
    pinMode(Encoder_L_A, INPUT);  pinMode(Encoder_L_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(Encoder_L_A), Encoder_LA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_L_B), Encoder_LB, CHANGE);
    // Right wheel encoder pins and interrupts
    pinMode(Encoder_R_A, INPUT);  pinMode(Encoder_R_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(Encoder_R_A), Encoder_RA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_R_B), Encoder_RB, CHANGE);
}
void AdcBooster() {
    ADC->CTRLA.bit.ENABLE = 0;                      // Disable ADC
    while (ADC->STATUS.bit.SYNCBUSY == 1);             // Wait for synchronization
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |    // Divide Clock by 128, ~31us per ADC sample
                     ADC_CTRLB_RESSEL_12BIT;        // Result on 12 bits
//  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |  // 1 sample
//                     ADC_AVGCTRL_ADJRES(0x00ul);// Adjusting result by 0
    ADC->SAMPCTRL.reg = 0x03;                       // Sampling Time Length = 3
    ADC->CTRLA.bit.ENABLE = 1;                      // Enable ADC
    while (ADC->STATUS.bit.SYNCBUSY == 1);             // Wait for synchronization
}
void setupTimers()
{
    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 when odd division factor used
                       GCLK_GENCTRL_GENEN |         // Enable GCLK generator
                       GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                       GCLK_GENCTRL_ID(5);          // Select GCLK5
//  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor N=3: 48MHz/3=16MHz
                      GCLK_GENDIV_ID(5);            // Select Generic Clock (GCLK) 5
//  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Feed GCLK5 to TCC2 and TC3
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK
                       GCLK_CLKCTRL_GEN_GCLK5 |     // Select GCLK5
                       GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK5 to TCC2 and TC3
//                     GCLK_CLKCTRL_ID_TC4_TC5;     // Feed GCLK5 to TC4 and TC5
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // set TC3 in 16 bits counting mode, Divide the GCLOCK signal by 16
    REG_TC3_CTRLA |= TC_CTRLA_MODE_COUNT16 |        // Set the counter to 16-bit mode
                     TC_CTRLA_PRESCALER_DIV16 |     // Set prescaler to 16, 16MHz/16 = 1MHz
                     TC_CTRLA_WAVEGEN_MFRQ |        // Set Match Frequency mode, cc0 would be period value
                     TC_CTRLA_ENABLE;               // Enable TC3
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization

    // timer TC3 counts up to CC0 value to generate 1ms interrupt,
    // this determines the frequency of the interrupt operation: Freq = 48Mhz/(N*CC0*Prescaler)
    REG_TC3_COUNT16_CC0 =1000;                    // Set the CC0 (period) register to 1000 for 1MHz clock  1kHZ  1ms
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);     // Wait for synchronization

    NVIC_SetPriority(TC3_IRQn, 1);                // Set the Nested Vector Interrupt Controller (NVIC) priority for TC3 to 3 (0 highest)
    NVIC_EnableIRQ(TC3_IRQn);                     // Connect TC3 to Nested Vector Interrupt Controller (NVIC)
    REG_TC3_INTFLAG |= TC_INTFLAG_MC0;            // Clears the interrupt flag
    REG_TC3_INTENSET = TC_INTENSET_MC0;
}
void setupPWM() {
    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                       GCLK_GENCTRL_GENEN |         // Enable GCLK generator
                       GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                       GCLK_GENCTRL_ID(4);          // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                      GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Feed GCLK4 to TCC0 and TCC1
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK
                       GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                       GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Enable the port multiplexer for the digital pin D2,D3
    PORT->Group[g_APinDescription[MOTOR_PWM_R].ulPort].PINCFG[g_APinDescription[MOTOR_PWM_R].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[MOTOR_PWM_L].ulPort].PINCFG[g_APinDescription[MOTOR_PWM_L].ulPin].bit.PMUXEN = 1;
    // Connect the TCC0 timer to digital output D2,D3 - port pins are paired odd PMUO and even PMUXE
    // F specify the timers: TCC0
    PORT->Group[g_APinDescription[MOTOR_PWM_R].ulPort].PMUX[g_APinDescription[MOTOR_PWM_R].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;//PA10 D2
    PORT->Group[g_APinDescription[MOTOR_PWM_L].ulPort].PMUX[g_APinDescription[MOTOR_PWM_L].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;//PA11 D3


    REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;         // Setup Single slope PWM on TCC0
    while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

    // Each timer counts up to a maximum or TOP value set by the PER register,
    // this determines the frequency of the PWM operation:
    REG_TCC0_PER = 2400;                            // Set the frequency of the PWM on TCC0 to 20kHz
    while (TCC0->SYNCBUSY.bit.PER);                 // Wait for synchronization

    // Set the PWM signal to output 0% duty cycle
    REG_TCC0_CC2 = 0;                               // TCC0 CC3 - on D2
    while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
    REG_TCC0_CC3 = 0;                               // TCC0 CC0 - on D3
    while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization
    // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
    REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                      TCC_CTRLA_ENABLE;             // Enable the TCC0 output
    while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}
