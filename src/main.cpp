#include <Arduino.h>
#include <Init_car_index.h>
void setup()
{
    Init_Peripherals();
    SerialUSB.begin(115200);
}

void loop()
{
    digitalWrite(Buzzer_PIN,ON);
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
    digitalWrite(Buzzer_PIN,OFF);
    for (int i = 0; i < 7; ++i) {
    SerialUSB.print(IRsensors[i]);
    SerialUSB.print("\t");
    }
    SerialUSB.println();
}