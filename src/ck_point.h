//
// Created by XIANGXI on 2022/8/2.
//
#ifndef ARDU_XAING_CK_POINT_H
#define ARDU_XAING_CK_POINT_H
#endif //ARDU_XAING_CK_POINT_H


//#define old_check
#define new_check

#define not_judge_of_pos_range 22.4375 //              50mm *  32/(22.7*PI())  =2872 pos
//#define not_judge_of_pos_range 2872 //              50mm *  32/(22.7*PI())*128  =2872 pos
#define stop_pos_range 44.875 //              100mm *  32/(22.7*PI())  =5744 pos
//#define stop_pos_range 5744 //              100mm *  32/(22.7*PI())*128  =5744 pos

#define not_judge_of_pos_range_hint 6.734 //              15mm *  32/(22.7*PI())*128  =862pos


/*record line DATA Parameters*/
unsigned long NOW_Time;//ms

unsigned long old_time;//ms

int prompt_cont=0;
float all_PROMPT[200];          //當前左右輪差pos    左-右
float all_PROMPT_w[200];        //當前車中心位置pos  (左+右)/2
float all_road_distance[200];   //道路距離 pos
float all_road_distance_mm[200];//道路距離 mm
float all_road_radius[200];     //半徑
float all_road_speed_max[200];     //半徑

char test_1m_flag=0;
float test_1m[2000];
float test_1_pwm[2000];
float test_1_cmd[2000];
float speed_integral =0;

int test_1m_cont=0;
uint8_t vc_flag_=0;
int icont=0;

/*record line DATA Parameters  END*/
#ifdef old_check
/* to check_point Parameters*/
uint8_t point_cross_flag=0 ;        //preset = 0
uint8_t point_STOP_or_Hint_flag=1 ; //preset = 1     控制 是否 開啟 讀 左右符號
uint8_t point_STOP_cont_flag=0 ;    //preset = 0     控制 經過一個起終點符號只動一次
int pos_now=0; //現在的pos位置
int pos_judge_cross=0;//十字鎖住的範圍
int pos_judge_hint=0;//提示符號鎖住的範圍
int  scont=0;// 起終點符號觸發次數
uint8_t hint_state=0;
/* to check_point Parameters END*/
/* check_point*/
void check_point(){
    pos_now=((eMotionL.pNEW + eMotionR.pNEW)>>1);  //pos 左右相加/2

    if(IRsensors[1]>1500 && IRsensors[2]>1500 && IRsensors[3]>1500 && IRsensors[4]>1500 && IRsensors[5]>1500)
    {
        point_cross_flag=1;
        pos_judge_cross= pos_now+not_judge_of_pos_range;
        //tone(2,523,100);
    }
    if(point_cross_flag==1)
    {
        if(pos_now < pos_judge_cross)
        {
            point_STOP_or_Hint_flag=0;
        }
        else
        {
            point_STOP_or_Hint_flag=1;
            point_cross_flag=0;
        }
    }

    if(point_STOP_or_Hint_flag==1)
    {
        if (IRsensors[0] > 1500 && IRsensors[6] < 1500 ) //STOP
        {
            //tone(2, 1000, 100);
            pos_judge_hint=pos_now+not_judge_of_pos_range_hint;
            point_STOP_cont_flag=1;
        }
        if (point_STOP_cont_flag==1)
        {
            if(!(pos_now < pos_judge_hint))
            {
                scont++;
                point_STOP_cont_flag=0;
            }
        }
        if(scont>=2)
        {
            tone(2, 1000, 100);
            scont=0;
            sButton=3;
        }

        if (IRsensors[6] > 1500 && IRsensors[0] < 1500)//Hint point
        {
            //ALL_LINE_Point_time[icont]=millis();
            icont++;
            tone(2,1318,250);
        }
    }
}
/* check_point_END*/
#endif

#ifdef new_check
void check_point2();
/* to check_point2 Parameters*/
int park_well_cont=0;
uint8_t point_cross_flag=0 ;        //preset = 0
uint8_t point_STOP_or_Hint_flag=1 ; //preset = 1     控制 是否 開啟 讀 左右符號

uint8_t stop_point_cont=0; //起終點計數
uint8_t stop_point_state=0;     //起終點現在狀態
uint8_t old_stop_point_state=0; //起終點過去狀態



uint8_t hint_point_state=0;     //提示符號現在狀態
uint8_t old_hint_point_state=0; //提示符號過去狀態
uint8_t hint_point_buzz_state=0;
float pos_now=0; //現在的pos位置
float pos_old=0; //現在的pos位置

float pos_judge_cross=0;//十字鎖住的範圍
float pos_stop=0;//停車跑一段的範圍

/* to check_point2 Parameters_END*/



/* check_point*/
void check_point2()
{

    if(IRsensors[1]>1500 && IRsensors[2]>1500 && IRsensors[3]>1500 && IRsensors[4]>1500 && IRsensors[5]>1500) //IR1~5 = HIGH  上鎖
    {
        point_cross_flag=1;
        pos_judge_cross= pos_now+not_judge_of_pos_range;  //所住的位置+上一段距離(POS)
        //tone(2,523,100);
    }
    if(point_cross_flag==1) //確認十字是否鎖上
    {
        if(pos_now < pos_judge_cross)//還沒離開範圍就鎖住左右IR
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
        if(IRsensors[6]>2800){stop_point_state=1;}else{stop_point_state=0;}//右邊當下狀態變化 0
        if(IRsensors[0]>2800){hint_point_state=1;}else{hint_point_state=0;}//左邊當下狀態變化 6

        if(old_stop_point_state > stop_point_state)//下緣觸發 右邊 起終點 1變0
        {
            stop_point_cont++;
            if(stop_point_cont==2) {
                pos_stop=pos_now+stop_pos_range;
                sButton = 3;
                stop_point_cont = 0;
                all_PROMPT_w[prompt_cont]=Pcount_R-Pcount_L;
                all_PROMPT[prompt_cont]=(Pcount_R+Pcount_L)/2;
                prompt_cont++;
                vc_flag_++;


            }
            if(stop_point_cont==1)
            {
                all_PROMPT_w[prompt_cont]=Pcount_R-Pcount_L;
                all_PROMPT[prompt_cont]=(Pcount_R+Pcount_L)/2;
                prompt_cont++;
                pcontRL_en=1;
//                test_1m_flag=1;

            }
            tone(Buzzer_PIN,1318,200);
        }
        if(old_hint_point_state > hint_point_state)//下緣觸發 左邊 路段提示 1變0
        {
                if(hint_point_buzz_state==0)
                {
                    hint_point_buzz_state=1;
                    tone(Buzzer_PIN,1318,200);
                    all_PROMPT_w[prompt_cont]=Pcount_R-Pcount_L;
                    all_PROMPT[prompt_cont]=(Pcount_R+Pcount_L)/2;
                    prompt_cont++;

                }
                else
                {
                    hint_point_buzz_state=0;
                    tone(Buzzer_PIN,587,100);
                    all_PROMPT_w[prompt_cont]=Pcount_R-Pcount_L;
                    all_PROMPT[prompt_cont]=(Pcount_R+Pcount_L)/2;
                    prompt_cont++;
                }
        }
        old_hint_point_state=hint_point_state;
        old_stop_point_state=stop_point_state;
    }

}
/* check_point_END*/
#endif