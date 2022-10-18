#include <Arduino.h>
#include <Init_car_index.h>
#include <Moton.h>
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
    Motor_control(1000,1000);
//    for (int i = 0; i < 7; ++i) {
//    SerialUSB.print(IRsensors[i]);
//    SerialUSB.print("\t");
//    }
//    SerialUSB.println();

}
// TC3 Interrupt Service Routine
void TC3_Handler()
{
    checkButton();
    READ_QEI();
    QEI_filter();
    // clear the interrupt flag
    REG_TC3_INTFLAG |= TC_INTFLAG_MC0;
}