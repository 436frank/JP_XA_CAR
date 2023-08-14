//
// Created by XIANGXI on 2022/10/18.
//

#ifndef JP_XA_CAR_CK_POINT_H
#define JP_XA_CAR_CK_POINT_H


//#include <Arduino.h>
#include <FlashAsEEPROM_SAMD.h>
//#include <Init_car_index.h>


#define all_white_val_max 1900
#define all_black_val_min 1500
#define not_judge_of_pos_range 28 //      20pos=5cm
#define pos_1cm_range 4.1 //              1cm
#define R_IR_ADC_trigger_level 3800//2800
#define L_IR_ADC_trigger_level 3800//2600


#define time_out_max 190 // 1ms *300 =300ms
int time_out=0;
float old_pos=0;
bool Protect_flag=0;
extern void clearAll();
extern void clearAll_();
/**record line DATA Parameters**/
unsigned long NOW_Time;//ms
unsigned long old_time;//ms
//char check_point =0;
bool record_data_flag=0;
bool record_speed_data_flag=0;
bool before_starting_pos_flag=0;
float before_starting_pos=0;
int prompt_cont=0;
float all_PROMPT[200];          //當前左右輪差pos    左-右
float all_PROMPT_w[200];        //當前車中心位置pos  (左+右)/2
float all_PROMPT_w_speed[200];        //當前車中心位置pos  (左+右)/2
float all_road_distance[200];   //道路距離 pos
float all_road_distance_mm[200];//道路距離 mm
float all_road_radius[200];     //半徑
float all_road_speed_max[200];     //線段最高的目標速度 /mod 3
float all_road_speed_max2[200];     //線段最高的目標速度 /mod 6
float all_road_speed_max3[200];     //線段最高的目標速度 /mod
float all_road_add_subtract_distance[200];   //線段加減距離
float all_road_Isokinetic_distance[200];     //線段等速距離
extern int sprint_cnt;
extern bool run_flag;
extern int distance_flag;

char test_1m_flag=0;
float test_1_pwm_r[10];
float test_1_pwm_l[10];
float test_1_v[1000];
float test_1_cmd[1000];

int test_1m_cont=0;
uint8_t vc_flag_=0;
uint8_t end_flag=0;

int tcont=0;
/**  to flash  **/
//void read_eeprom();
void SAVE_Flash_data();
/**  to check_point  **/
void Protect();
void check_point();
/**  to check_point Parameters  **/
int park_well_cont=0;
uint8_t point_cross_flag=0 ;        //preset = 0
uint8_t point_STOP_or_Hint_flag=1 ; //preset = 1     控制 是否 開啟 讀 左右符號
uint8_t stop_point_cont=0;      //起終點計數
uint8_t stop_point_state=0;     //起終點現在狀態
uint8_t old_stop_point_state=0; //起終點過去狀態
uint8_t hint_point_state=0;     //提示符號現在狀態
uint8_t old_hint_point_state=0; //提示符號過去狀態
uint8_t hint_point_buzz_state=0;

uint8_t hint_point_LED_state=0;
uint8_t hint_point_LED_state_cnt=0;
float pos_judge_cross=0;//十字鎖住的範圍
float pos_stop=0;//停車跑一段的範圍
/**  to check_point Parameters_END  **/
float test_cnttt=123123.12;
float test_cnttout=0;
void SAVE_Flash_data() {
    int address = 0;
    EEPROM.put(address, test_cnttt);
    if (!EEPROM.getCommitASAP()) {EEPROM.commit();}  // CommitASAP not set. Need commit()
    EEPROM.get(address , test_cnttout);
    SerialUSB.print(test_cnttout);
}
/**  check_point  **/
void Protect()
{
    if(IRsensors[1]>all_white_val_max && IRsensors[2]>all_white_val_max && IRsensors[3]>all_white_val_max && IRsensors[4]>all_white_val_max && IRsensors[5]>all_white_val_max
    || IRsensors[1]<all_black_val_min && IRsensors[2]<all_black_val_min && IRsensors[3]<all_black_val_min && IRsensors[4]<all_black_val_min && IRsensors[5]<all_black_val_min ) //IR1~5 = HIGH
    {
        if(Protect_flag!=1)
        {
            Protect_flag=1;
         tone(Buzzer_PIN,1000,100);
//            digitalWrite(LED_L_PIN,ON);
        }
        time_out++;
        if(time_out>time_out_max)
        {
            clearAll();
        }
    }
    else
    {
        if(Protect_flag!=0)
        {
            Protect_flag=0;
            time_out=0;
//            digitalWrite(LED_L_PIN,OFF);
        }

    }
}
void check_point()
{
    if(IRsensors[1]>1500 && IRsensors[2]>1500 && IRsensors[3]>1500 && IRsensors[4]>1500 && IRsensors[5]>1500) //IR1~5 = HIGH  上鎖
    {
        point_cross_flag=1;
        pos_judge_cross= posFeedBack+not_judge_of_pos_range;  //所住的位置+上一段距離(POS)
        //tone(2,523,100);
    }
    if(point_cross_flag==1) //確認十字是否鎖上
    {

        if(posFeedBack < pos_judge_cross)//還沒離開範圍就鎖住左右IR
        {
            point_STOP_or_Hint_flag=0;
        }
        else
        {
            point_STOP_or_Hint_flag=1;
            point_cross_flag=0;
        }
    }
    if(point_STOP_or_Hint_flag==1) //左右IR沒上鎖就
    {
        if(IRsensors[6]>R_IR_ADC_trigger_level){stop_point_state=1;}else{stop_point_state=0;}//右邊當下狀態變化 0
        if(IRsensors[0]>L_IR_ADC_trigger_level){hint_point_state=1;}else{hint_point_state=0;}//左邊當下狀態變化 6
        if(old_stop_point_state > stop_point_state)//下緣觸發 右邊 起終點 1變0
        {
            stop_point_cont++;
            if (stop_point_cont > 1)
            {
                if (sprint_cnt >= (prompt_cont - 3))
                {
                    stop_point_cont=2;
                }
                else
                {
                    stop_point_cont=1;
                }
            }
            if(stop_point_cont==2) {
//                pos_stop=pos_now+stop_pos_range;
                stop_point_cont = 0;
                if (record_data_flag==1) {
                    all_PROMPT_w[prompt_cont] = Pcount_R - Pcount_L;
                    all_PROMPT[prompt_cont] = posFeedBack;
                }
                if (record_speed_data_flag==1) {
                    all_PROMPT_w_speed[sprint_cnt] = posFeedBack;
                }
                record_data_flag=0;
                record_speed_data_flag=0;
                end_flag=1;

            }
            if(stop_point_cont==1)
            {
                if(before_starting_pos_flag==0)
                {
                    before_starting_pos=posFeedBack;
                    before_starting_pos_flag=1;
                }
                if (record_data_flag==1) {
                    all_PROMPT_w[prompt_cont] = Pcount_R - Pcount_L;
                    all_PROMPT[prompt_cont] = posFeedBack;
                    prompt_cont++;
                }
                if (record_speed_data_flag==1) {
                    all_PROMPT_w_speed[sprint_cnt] = posFeedBack;
                }

            }
            tone(Buzzer_PIN,1318,200);
            digitalWrite(LED_R_PIN,ON);
            hint_point_LED_state=1;
            sprint_cnt++;
            distance_flag=1;
        }
        if(old_hint_point_state > hint_point_state)//下緣觸發 左邊 路段提示 1變0
        {
            if ((posFeedBack - old_pos) > pos_1cm_range) {
                if (hint_point_buzz_state == 0) {
                    hint_point_buzz_state = 1;
                    tone(Buzzer_PIN, 1318, 200);
                    if (record_data_flag == 1) {
                        all_PROMPT_w[prompt_cont] = Pcount_R - Pcount_L;
                        all_PROMPT[prompt_cont] = posFeedBack;
                        prompt_cont++;
                    }
                    if (record_speed_data_flag==1) {
                        all_PROMPT_w_speed[sprint_cnt] = posFeedBack;
                    }
                    sprint_cnt++;
                } else {
                    hint_point_buzz_state = 0;
                    tone(Buzzer_PIN, 587, 100);
                    if (record_data_flag == 1) {
                        all_PROMPT_w[prompt_cont] = Pcount_R - Pcount_L;
                        all_PROMPT[prompt_cont] = posFeedBack;
                        prompt_cont++;
                    }
                    if (record_speed_data_flag==1) {
                        all_PROMPT_w_speed[sprint_cnt] = posFeedBack;
                    }
                    sprint_cnt++;
                }
                digitalWrite(LED_L_PIN, ON);
                hint_point_LED_state = 1;
                distance_flag=1;
                old_pos=posFeedBack;
            }
        }
        old_hint_point_state=hint_point_state;
        old_stop_point_state=stop_point_state;
    }
    if(hint_point_LED_state==1)
    {
        hint_point_LED_state_cnt++;
        if(hint_point_LED_state_cnt==3)
        {
            digitalWrite(LED_L_PIN,OFF);
            digitalWrite(LED_R_PIN,OFF);
            hint_point_LED_state=0;
            hint_point_LED_state_cnt=0;
        }
    }
}
/* check_point_END*/
#endif //JP_XA_CAR_CK_POINT_H