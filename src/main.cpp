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
    REG_TCC0_CC2 = 1000;                               // TCC0 CC3 - on D2
    while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
    REG_TCC0_CC3 = 4000;                               // TCC0 CC3 - on D2
    while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization
    SerialUSB.println(Lp);

}
// TC3 Interrupt Service Routine
void TC3_Handler()
{
    checkButton();
    StateMachine(sButton);
    READ_QEI();
    QEI_filter();
    // clear the interrupt flag
    REG_TC3_INTFLAG |= TC_INTFLAG_MC0;
}