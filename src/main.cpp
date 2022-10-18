#include <Arduino.h>
#include <Init_car_index.h>
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

    SerialUSB.println(Vcount_R);
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