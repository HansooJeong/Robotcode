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


//-----Motor-----//
#define INPUT1 5
#define INPUT2 6
#define INPUT3 17
#define INPUT4 27
#define Motor_L 16
#define Motor_R 20
//-----IMU Sensor-----//
#define MPU6050_ADDRESS (0x68)
#define MPU6050_REG_PWR_MGMT_1 (0x6b)
#define MPU6050_REG_DATA_START (0x3b)

//-----Define formula-----//
#define DUTYCYCLE(x,range) x/(float)range*100

int pi;
//-----IMU Variables-----//
uint32_t start_tick_, dist_tick_;
float Front_dist;
float Left_dist;

//-----Setting-----//
void Setting_func(void);

//-----Motor Function------//
void Motor_Go(void);
void Motor_Back(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);

//Ultrasonic Function-----//
void trigger(void);
void cb_func_echo(int pi, unsigned gpio, unsigned level, uint32_t tick);

//-----Thread Function-----//
void *thread_function1(void *arg);
void *thread_function2(void *arg);
pthread_t threads[2];

//------------------------------- MAIN --------------------------------//
int main(void)
{
    int default_range = 255;
    int range_L;
    int range_R;
    if((pi=pigpio_start(NULL,NULL))<0) return 1;
    Setting_func();

    //Thread create
    pthread_create(&threads[0], NULL, &thread_function1, NULL);
    pthread_create(&threads[1], NULL, &thread_function2, NULL);
    printf("%d, %d\n", 0, threads[0]);
    printf("%d, %d\n", 1, threads[1]);
    time_sleep(1);

    //PWM Setting
    set_PWM_range(pi, Motor_L, default_range);
    set_PWM_range(pi, Motor_R, default_range);
    range_L = get_PWM_range(pi, Motor_L);
    range_R = get_PWM_range(pi, Motor_R);

    //PWM
    //set_PWM_dutycycle(pi, PINNO, default_range);    //Select dutycycle
    int duty_L = get_PWM_dutycycle(pi, Motor_L);    //take the PWM_dutycycle
    int duty_R = get_PWM_dutycycle(pi, Motor_R);    //take the PWM_dutycycle
    time_sleep(0.02);
    printf("duty cycle:%.1f%% %d/%d\n", DUTYCYCLE(duty_L,range_L), 1, range_L);   //show the Dutyrate & range of PWM
    printf("duty cycle:%.1f%% %d/%d\n", DUTYCYCLE(duty_R,range_R), 1, range_R);   //show the Dutyrate & range of PWM

    set_PWM_dutycycle(pi, Motor_L, 0);    //make dutycycle to 0
    set_PWM_dutycycle(pi, Motor_R, 0);    //make dutycycle to 0

    while(1){
        printf("Front_dist = %3.1f\n",Front_dist);
    /*   
        if(Front_dist<10 && Left_dist<10){
            Motor_Stop();
            time_sleep(1);
            Motor_Right();
            time_sleep(1);
        }
        else if(Front_dist>10 && Left_dist<10){
            Motor_Go();
        }
        else if(Front_dist<10 && Left_dist>10){
            Motor_Left();
            time_sleep(1);
        }
        else if(Front_dist>10 && Left_dist>10 ){
            Motor_Go();
        }
        // printf("distance: %6.2f\n",distance);
        // time_sleep(0.01);
        */
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

}
//-----------------Call Back Function----------------------------//
void cb_func_echo(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    if(level == PI_HIGH)
        start_tick_ = tick;
    else if(level == PI_LOW)
        dist_tick_ = tick - start_tick_;
}
//-----------------Thread Function1-----------------------------//
void *thread_function1(void *arg)
{
    callback(pi, ECHO_PINNO1, EITHER_EDGE, cb_func_echo);
    gpio_write(pi, TRIG_PINNO1, PI_OFF);
    time_sleep(1);     // delay 1 second

    printf("Raspberry Pi HC-SR04 UltraSonic sensor\n" );

    while(1){
        start_tick_ = dist_tick_ = 0;
        gpio_trigger(pi, TRIG_PINNO1, 10, PI_HIGH);
        time_sleep(0.05);

        if(dist_tick_ && start_tick_){
            //distance = (float)(dist_tick_) / 58.8235;
            Front_dist = dist_tick_ / 1000000. * 340 / 2 * 100;
            if(Front_dist < 2 || Front_dist > 400)
                printf("range error\n");
            else {
                //   printf("interval : %6dus, Distance : %6.2f cm\n", dist_tick_, distance);
            }
        }
        else{
            printf("sense error\n");
        }

        time_sleep(0.001);
    }
}
void *thread_function2(void *arg)
{
    int fd;

    if((fd=i2c_open(pi, 1, MPU6050_ADDRESS,0))<0){
        perror("i2c_open\n");
        return 1;
    }
    setbuf(stdout ,NULL);
    i2c_write_byte_data(pi,fd,MPU6050_REG_PWR_MGMT_1,0);
    while(1){
        //        printf("Thread_function3 Start\n");
        uint8_t msb=i2c_read_byte_data(pi,fd,MPU6050_REG_DATA_START+12);
        uint8_t lsb=i2c_read_byte_data(pi,fd,MPU6050_REG_DATA_START+13);
        short gyroZ=msb<<8|lsb;
        //        printf("gyroZ=%7d\n",gyroZ);
        time_sleep(0.001);
    }
}
//-----------------Motor Function-----------------------------//
void Motor_Go(void)
{
    gpio_write(pi, INPUT1, PI_HIGH);    // A 방향
    gpio_write(pi, INPUT2, PI_LOW);
    gpio_write(pi, INPUT3, PI_HIGH);    // A 방향
    gpio_write(pi, INPUT4, PI_LOW);
}
void Motor_Back(void)
{
    gpio_write(pi, INPUT1, PI_LOW);     // B 방향
    gpio_write(pi, INPUT2, PI_HIGH);
    gpio_write(pi, INPUT3, PI_LOW);     // B 방향
    gpio_write(pi, INPUT4, PI_HIGH);
}
void Motor_Left(void)
{
    gpio_write(pi, INPUT1, PI_LOW);     // B 방향
    gpio_write(pi, INPUT2, PI_HIGH);
    gpio_write(pi, INPUT3, PI_HIGH);     // B 방향
    gpio_write(pi, INPUT4, PI_LOW);
}
void Motor_Right(void)
{
    gpio_write(pi, INPUT1, PI_HIGH);    // A 방향
    gpio_write(pi, INPUT2, PI_LOW);
    gpio_write(pi, INPUT3, PI_LOW);    // A 방향
    gpio_write(pi, INPUT4, PI_HIGH);
}
void Motor_Stop(void)
{
    gpio_write(pi, INPUT1, PI_LOW);    // A 방향
    gpio_write(pi, INPUT2, PI_LOW);
    gpio_write(pi, INPUT3, PI_LOW);    // A 방향
    gpio_write(pi, INPUT4, PI_LOW);
}
