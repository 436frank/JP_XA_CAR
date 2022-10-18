#include <Arduino.h>
#include <Init_car_index.h>
#include "Moton.h"
#include "menu.h"
void setup()
{
    AdcBooster();
    Init_Peripherals();
    setupTimers();
    setupPWM();
    SerialUSB.begin(115200);
}

void loop()
{
//    for (int i = 0; i < 7; ++i) {
//    SerialUSB.print(IRsensors[i]);
//    SerialUSB.print("\t");
//    }
//    SerialUSB.println();
    readAllIR_values();
    IR_calibrations();
    //check_point();
    check_point2();
    // calculate weighted average 計算權重平均
    Lp = LINE_estimation(IR_caliValues);
    SerialUSB.println(Lp);

}
// TC3 Interrupt Service Routine
void TC3_Handler()
{
    checkButton();
//    StateMachine(sButton);
    READ_QEI();
    QEI_filter();
    // clear the interrupt flag
    REG_TC3_INTFLAG |= TC_INTFLAG_MC0;
}