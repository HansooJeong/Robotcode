

#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pigpiod_if2.h>
//-------------------------------Initialization----------------------------//
//-----Ultrasonic Sensor-----//
#define TRIG_PINNO1 23
#define ECHO_PINNO1 24
#define TRIG_PINNO2 9
#define ECHO_PINNO2 10
#define TRIG_PINNO3 12
#define ECHO_PINNO3 26


//-----Motor-----//
#define INPUT1 21
#define INPUT2 20
#define INPUT3 17
#define INPUT4 27
#define Motor_L 19
#define Motor_R 13
//-----IMU Sensor-----//
#define MPU6050_ADDRESS (0x68)
#define MPU6050_REG_PWR_MGMT_1 (0x6b)
#define MPU6050_REG_DATA_START (0x3b)

//-----Define formula-----//
#define DUTYCYCLE(x,range) x/(float)range*100
#define PWM_Value(distance) (int)(distance*10)-46

int pi;
//-----IMU Variables-----//
uint32_t start_tick_, dist_tick_, start_tick_2,dist_tick_2, start_tick_3,dist_tick_3;
float Front_dist;
float Left_dist;
float Right_dist;
float pre_Left_dist;
float pre_Right_dist;
float ki,kp,kd;
int speed=0,pre_speed=0,cnt=0;
float err=0;
float i=0;
float d=0,p=0;
float lsterr=0;
float pivot;
int a,b,k=0;

//-----Setting-----//
void Setting_func(void);

//-----Motor Function------//
void Motor_Back_Start(void);
void Motor_Start(void);
void Motor_Go(void);
void Motor_Back(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);

//Ultrasonic Function-----//
void trigger(void);
void cb_func_echo1(int pi, unsigned gpio, unsigned level, uint32_t tick);
void cb_func_echo2(int pi, unsigned gpio, unsigned level, uint32_t tick);
void cb_func_echo3(int pi, unsigned gpio, unsigned level, uint32_t tick);

//-----Thread Function-----//
void *thread_function1(void *arg);
void *thread_function2(void *arg);
void *thread_function3(void *arg);
pthread_t threads[3];

pthread_mutex_t  mutex = PTHREAD_MUTEX_INITIALIZER;
//------------------------------- MAIN --------------------------------//        
int main(void)
{
    int default_range = 255;
    int range_L;
    int range_R;
    printf("ki , kp , kd \n");
    scanf("%f  %f  %f ",&ki ,&kp, &kd); 
    printf("%f , %f , %f \n",ki,kp,kd);

    if((pi=pigpio_start(NULL,NULL))<0){
     printf("End\n");
        return 1;
    }
    Setting_func();
    callback(pi, ECHO_PINNO1, EITHER_EDGE, cb_func_echo1);
    gpio_write(pi, TRIG_PINNO1, PI_OFF);
    time_sleep(1);     // delay 1 second
    callback(pi, ECHO_PINNO2, EITHER_EDGE, cb_func_echo2);
    gpio_write(pi, TRIG_PINNO2, PI_OFF);
    time_sleep(1);     // delay 1 second
    callback(pi, ECHO_PINNO3, EITHER_EDGE, cb_func_echo3);
    gpio_write(pi, TRIG_PINNO3, PI_OFF);
    time_sleep(1);     // delay 1 second
    pthread_mutex_init(&mutex,NULL);
    //Thread create
    pthread_create(&threads[0], NULL, &thread_function1, NULL);
    pthread_create(&threads[1], NULL, &thread_function2, NULL);
    pthread_create(&threads[2], NULL, &thread_function3, NULL);
    printf("%d, %d\n", 0, threads[0]);
    printf("%d, %d\n", 1, threads[1]);
    printf("%d, %d\n", 2, threads[2]);
    //    printf("%d, %d\n", 2, threads[2]);
    time_sleep(1);

    //PWM Setting
    set_PWM_range(pi, Motor_L, default_range);
    set_PWM_range(pi, Motor_R, default_range);
    range_L = get_PWM_range(pi, Motor_L);
    range_R = get_PWM_range(pi, Motor_R);
    set_PWM_dutycycle(pi, Motor_R,0);    //make dutycycle to 0
    set_PWM_dutycycle(pi, Motor_L, 0);    //make dutyc 13.6 ycle to 0

    //PWM
    //set_PWM_dutycycle(pi, PINNO, default_range);    //Select dutycycle
    int duty_L = get_PWM_dutycycle(pi, Motor_L);    //take the PWM_dutycycle
    int duty_R = get_PWM_dutycycle(pi, Motor_R);    //take the PWM_dutycycle
    time_sleep(0.02);
    printf("duty cycle:%.1f%% %d/%d\n", DUTYCYCLE(duty_L,range_L), 1, range_L);   //show the Dutyrate & range of PWM
    printf("duty cycle:%.1f%% %d/%d\n", DUTYCYCLE(duty_R,range_R), 1, range_R);   //show the Dutyrate & range of PWM

    float pre_Left_dist=0;
    pivot=6.7;
    printf("pivot: %3.1f \n", pivot);
    while(1){

        a=130-speed;
        b=124;//-speed;//*0.5;
        // Left_standard = Left_dist;
        //100
        if(Front_dist > 9 && Left_dist <=18 && Right_dist <= 18){

        if(Front_dist<30){
            a=a-12;
            b=b-12;
        }
            err= pivot-Left_dist;
            if(err<0)
            {
      //          printf("Left_err: %3.1f \n",err);
                err*=1.2;
            }
            else{
      //          printf("Right_err: %3.1f \n",err);
                err*=1.2;
            }
            p=kp*err;
            i=i+ki*err;
            d=(err-lsterr)*kd;

            speed=(i+p+d);
            //printf("i= %f //  d= %f // speed= %d // err = %f , %f,  %f    %f\n",i,d,speed,err,Front_dist, Left_dist,Right_dist);
            //printf("%d   %d \n",a,b);
            printf("%f,%f,%f,%f\n",p,i,d,err);
            if(k==0){
                Motor_Start();
                set_PWM_dutycycle(pi, Motor_R,a);    //make dutycycle to 0
                set_PWM_dutycycle(pi, Motor_L, b);    //make dutyc 13.6 ycle to 0
                k++; 
            } 
            set_PWM_dutycycle(pi, Motor_R,a);    //make dutycycle to 0
            set_PWM_dutycycle(pi, Motor_L, b);    //make dutyc 13.6 ycle to 0
            Motor_Go();
            lsterr=err;
        }
        //001
        else if(Front_dist <= 10 && Left_dist <=18 && Right_dist > 30){
            
            Motor_Stop();
            Motor_Start();
            Motor_Right();
            Motor_Stop();
            k=0;
            printf("turn Right\n");
            set_PWM_dutycycle(pi, Motor_R,a);    //make dutycycle to 0
            set_PWM_dutycycle(pi, Motor_L, b);    //make dutyc 13.6 ycle to 0
            Motor_Go();
            printf("i= %f //  d= %f // speed= %d // err = %f , %f,  %f    %f\n",i,d,speed,err,Front_dist, Left_dist,Right_dist);
            
        }
        //010
        else if(Front_dist <= 10 && Left_dist > 30 && Right_dist <= 18){
            Motor_Stop();
            Motor_Back_Start();
            Motor_Left();
            Motor_Stop();
            Motor_Start();
            set_PWM_dutycycle(pi, Motor_R,130);    //make dutycycle to 0
            set_PWM_dutycycle(pi, Motor_L, 124);    //make dutyc 13.6 ycle to 0
            Motor_Go();
            time_sleep(0.5);
            k=0;
            printf("turn Left\n");
            printf("i= %f //  d= %f // speed= %d // err = %f , %f,  %f    %f\n",i,d,speed,err,Front_dist, Left_dist,Right_dist);
        }
        //011
        else if(Front_dist <= 10 && Left_dist > 18 && Right_dist > 18){
            Motor_Start();
            Motor_Left();
            Motor_Stop();
            Motor_Start();
            set_PWM_dutycycle(pi, Motor_R,130);    //make dutycycle to 0
            set_PWM_dutycycle(pi, Motor_L, 124);    //make dutyc 13.6 ycle to 0
            Motor_Go();
            time_sleep(0.5);
            k=0;
            printf("turn Left\n");
        }
        //101
        else if(Front_dist > 9 && Left_dist <=18 && Right_dist > 18){
        if(Front_dist<30){
            a=a-12;
            b=b-12;
        }
            if(k==0){
                Motor_Start();
                k++; 
            }
            set_PWM_dutycycle(pi, Motor_R,a);    //make dutycycle to 0
            set_PWM_dutycycle(pi, Motor_L, b);    //make dutyc 13.6 ycle to 0
            Motor_Go();
        }
        //110
        else if(Front_dist > 9 && Left_dist > 18 && Right_dist <= 18){
        if(Front_dist<30){
            a=a-12;
            b=b-12;
        }
            if(k==0){
                Motor_Start();
                k++; 
            }
            set_PWM_dutycycle(pi, Motor_R,a);    //make dutycycle to 0
            set_PWM_dutycycle(pi, Motor_L, b);    //make dutyc 13.6 ycle to 0
            Motor_Go();
        }
        //111
        else if(Front_dist > 60 && Left_dist > 18 && Right_dist > 18){
            Motor_Stop();
        }
        //000
        else{
            Motor_Start();
            set_PWM_dutycycle(pi, Motor_R,a);    //make dutycycle to 0
            set_PWM_dutycycle(pi, Motor_L, b);    //make dutyc 13.6 ycle to 0
            Motor_Go();
            time_sleep(0.3);
            Motor_Stop();
            printf("else\n");
        }
        
            if(Left_dist<5){    
            printf("Left error\n");
                    if(pre_Left_dist-Left_dist < 0.3 || pre_Left_dist-Left_dist > -0.3)
                    {
                        cnt++;
                    }
                    else{
                        cnt=0;
                    }

                    if(cnt==30)
                    {
                        Motor_Back_Start();
                        set_PWM_dutycycle(pi, Motor_R,120);    //make dutycycle to 0
                        set_PWM_dutycycle(pi, Motor_L, 110);    //make dutyc 13.6 ycle to 0 
                        Motor_Back();
                        Motor_Start();
                        set_PWM_dutycycle(pi, Motor_R,0);    //make dutycycle to 0
                        set_PWM_dutycycle(pi, Motor_L, 120);    //make dutyc 13.6 ycle to 0
                        Motor_Go();
                        time_sleep(0.6);
                        Motor_Stop();
                        k=0;
                        cnt=0;
                    }
                    pre_Left_dist=Left_dist;
            }
            if(Right_dist<5){    
            printf("Right error\n");
                    if(pre_Right_dist-Right_dist < 0.3 || pre_Right_dist-Right_dist > -0.3)
                    {
                        cnt++;
                    }
                    else{
                        cnt=0;
                    }

                    if(cnt==30)
                    {
                        Motor_Back_Start();
                        set_PWM_dutycycle(pi, Motor_R,120);    //make dutycycle to 0
                        set_PWM_dutycycle(pi, Motor_L, 110);    //make dutyc 13.6 ycle to 0 
                        Motor_Back();
                        Motor_Start();
                        set_PWM_dutycycle(pi, Motor_R,120);    //make dutycycle to 0
                        set_PWM_dutycycle(pi, Motor_L, 0);    //make dutyc 13.6 ycle to 0
                        Motor_Go();
                        time_sleep(0.6);
                        Motor_Stop();
                        k=0;
                        cnt=0;
                    }
                    pre_Right_dist=Right_dist;
            }
            
        time_sleep(0.1);
    }
    pigpio_stop(pi);
    return 0;
}

//---------------------------------------------Functions-----------------------------------------//
void Setting_func()
{
    //Motor Setting
    set_mode(pi, INPUT1, PI_OUTPUT);
    set_mode(pi, INPUT2, PI_OUTPUT);
    set_mode(pi, INPUT3, PI_OUTPUT);
    set_mode(pi, INPUT4, PI_OUTPUT);

    //Ultrasonic Sensor Setting
    set_mode(pi, TRIG_PINNO1, PI_OUTPUT);
    set_mode(pi, ECHO_PINNO1, PI_INPUT);
    set_mode(pi, TRIG_PINNO2, PI_OUTPUT);
    set_mode(pi, ECHO_PINNO2, PI_INPUT);
    set_mode(pi, TRIG_PINNO3, PI_OUTPUT);
    set_mode(pi, ECHO_PINNO3, PI_INPUT);

}
//-----------------Call Back Function----------------------------//
void cb_func_echo1(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    if(level == PI_HIGH)
        start_tick_ = tick;
    else if(level == PI_LOW)
        dist_tick_ = tick - start_tick_;
    //printf("callback1:%u, %u\n",dist_tick_,start_tick_);
}
void cb_func_echo2(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    if(level == PI_HIGH)
        start_tick_2 = tick;
    else if(level == PI_LOW)
        dist_tick_2 = tick - start_tick_2;
    //  printf("callback2:%u, %u\n",dist_tick_2,start_tick_2);
}
void cb_func_echo3(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    if(level == PI_HIGH)
        start_tick_3 = tick;
    else if(level == PI_LOW)
        dist_tick_3 = tick - start_tick_3;
    //  printf("callback2:%u, %u\n",dist_tick_2,start_tick_2);
}
//-----------------Thread Function1-----------------------------//
void *thread_function1(void *arg)
{


    printf("Raspberry Pi HC-SR04 UltraSonic sensor\n" );

    while(1){
        start_tick_ = dist_tick_ = 0;
        pthread_mutex_lock(&mutex);
        gpio_trigger(pi, TRIG_PINNO1, 10, PI_HIGH);
        time_sleep(0.05);
        pthread_mutex_unlock(&mutex);

        // printf("%d, %d\n",dist_tick_,start_tick_);
        //   if(0<dist_tick_){
        if(dist_tick_ && start_tick_){
            //distance = (float)(dist_tick_) / 58.8235;
            Front_dist = dist_tick_ / 1000000. * 340 / 2 * 100;
            if(Front_dist < 2 || Front_dist > 400)
                printf("range error1\n");
            else {
    //            printf("interval : %6dus,======================== Front_dist : %6.2f cm\n", dist_tick_, Front_dist);
            }
        }
        else{
            printf("sense error1\n");
        }
        // }

        time_sleep(0.001);
    }
}
void *thread_function2(void *arg)
{

    printf("Raspberry Pi HC-SR04 UltraSonic sensor\n" );
    while(1){
        start_tick_2 = dist_tick_2 = 0;
        pthread_mutex_lock(&mutex);
        gpio_trigger(pi, TRIG_PINNO2, 10, PI_HIGH);
        time_sleep(0.05);
        pthread_mutex_unlock(&mutex);

        //       if(0<dist_tick_2){
        if(dist_tick_2 && start_tick_2){
            //distance = (float)(dist_tick_) / 58.8235;
            Left_dist = dist_tick_2 / 1000000. * 340 / 2 * 100;
            if(Left_dist < 2 || Left_dist > 400)
                printf("range error\n");
            else {
      //          printf("interval : %6dus,++++++++++++ Left_dist : %6.2f cm\n", dist_tick_2, Left_dist);
            }
        }
        else{
            printf("sense error\n");
        }
        //       }
        time_sleep(0.001);
    }
}
void *thread_function3(void *arg)
{
    printf("Raspberry Pi HC-SR04 UltraSonic sensor\n" );

    while(1){
        start_tick_3 = dist_tick_3 = 0;
        pthread_mutex_lock(&mutex);
        gpio_trigger(pi, TRIG_PINNO3, 10, PI_HIGH);
        time_sleep(0.05);
        pthread_mutex_unlock(&mutex);

        // printf("%d, %d\n",dist_tick_,start_tick_);
        //   if(0<dist_tick_){
        if(dist_tick_3 && start_tick_3){
            //distance = (float)(dist_tick_) / 58.8235;
            Right_dist = dist_tick_3 / 1000000. * 340 / 2 * 100;
            if(Right_dist < 2 || Right_dist > 400)
                printf("range error1\n");
            else {
    //            printf("interval : %6dus,======================== Right_dist : %6.2f cm\n", dist_tick_3, Right_dist);
            }
        }
        else{
            printf("sense error1\n");
        }
        // }

        time_sleep(0.001);
    }
}
//-----------------Motor Function-----------------------------//
void Motor_Start(void)
{
    set_PWM_dutycycle(pi, Motor_L,255 );
    set_PWM_dutycycle(pi,Motor_R,255);
    gpio_write(pi, INPUT1, PI_HIGH);    // A 방향
    gpio_write(pi, INPUT2, PI_LOW);
    gpio_write(pi, INPUT3, PI_HIGH);    // A 방향
    gpio_write(pi, INPUT4, PI_LOW);
    time_sleep(0.01);
}
void Motor_Back_Start(void)
{
    set_PWM_dutycycle(pi, Motor_L,255 );
    set_PWM_dutycycle(pi,Motor_R,255);
    gpio_write(pi, INPUT1, PI_LOW);    // A 방향
    gpio_write(pi, INPUT2, PI_HIGH);
    gpio_write(pi, INPUT3, PI_LOW);    // A 방향
    gpio_write(pi, INPUT4, PI_HIGH);
    time_sleep(0.01);
}
void Motor_Go(void)
{
    gpio_write(pi, INPUT1, PI_HIGH);    // A 방향
    gpio_write(pi, INPUT2, PI_LOW);
    gpio_write(pi, INPUT3, PI_HIGH);    // A 방향
    gpio_write(pi, INPUT4, PI_LOW);

    /*
       gpio_write(pi, INPUT1, PI_HIGH);    // A 방향
       gpio_write(pi, INPUT2, PI_LOW);
       gpio_write(pi, INPUT3, PI_HIGH);    // A 방향
       gpio_write(pi, INPUT4, PI_LOW);
     */
}
void Motor_Back(void)
{
    gpio_write(pi, INPUT1, PI_LOW);     // B 방향
    gpio_write(pi, INPUT2, PI_HIGH);
    gpio_write(pi, INPUT3, PI_LOW);     // B 방향
    gpio_write(pi, INPUT4, PI_HIGH);
    time_sleep(0.8);
}
void Motor_Right(void)
{
    
    set_PWM_dutycycle(pi, Motor_L, 150);
    set_PWM_dutycycle(pi,Motor_R,140);
    gpio_write(pi, INPUT1, PI_LOW);     // B 방향
    gpio_write(pi, INPUT2, PI_HIGH);
    gpio_write(pi, INPUT3, PI_HIGH);     // B 방향
    gpio_write(pi, INPUT4, PI_LOW);
    time_sleep(0.5);
}
void Motor_Left(void)
{
    set_PWM_dutycycle(pi, Motor_L, 148);
    set_PWM_dutycycle(pi,Motor_R,140);
    gpio_write(pi, INPUT1, PI_HIGH);    // A 방향
    gpio_write(pi, INPUT2, PI_LOW);
    gpio_write(pi, INPUT3, PI_LOW);    // A 방향
    gpio_write(pi, INPUT4, PI_HIGH);
    time_sleep(0.4);
}
void Motor_Stop(void)
{
    gpio_write(pi, INPUT1, PI_LOW);    // A 방향
    gpio_write(pi, INPUT2, PI_LOW);
    gpio_write(pi, INPUT3, PI_LOW);    // A 방향
    gpio_write(pi, INPUT4, PI_LOW);
    sleep(1);
}

