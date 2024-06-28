#ifndef __BSP_MOTOR_CONTROL_H
#define	__BSP_MOTOR_CONTROL_H

#include "stm32f1xx.h"
#include "bsp_tim.h"
#include "main.h"
#include "bsp_motor_control.h"
#include "bsp_pid.h"

/* 电机方向控制枚举 */
typedef enum
{
    MOTOR_FWD = 0,
    MOTOR_REV,
}motor_dir_t;

typedef struct
{
    char mode;               // 模式
    char positions[20];      // 位点
    int current_repeat_count;        // 循环次数
    char speed_str[5];        // M5 电机速度
} Command;

// 定义 CurrentPosition 结构体(水平模式、垂直模式、交叉模式、自定义模式)
typedef struct {
    char positions1[3];
    char positions2[3];
    char positions3[3];
    char positions4[3];
    char positions5[3];
} CurrentPosition;

typedef struct
{
    float vertical;         //仰角pid4
    float horizontal;       //水平摆角pid3
    float M1speed;
    float M2speed;
} Position;

/* 设置速度（占空比） */
#define MOTOR1_SET_FWD_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(TIM_CHANNEL_1,ChannelPulse)    // 设置比较寄存器的值
#define MOTOR1_SET_REV_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(TIM_CHANNEL_1,ChannelPulse)    // 设置比较寄存器的值

#define MOTOR2_SET_FWD_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(TIM_CHANNEL_2,ChannelPulse)    // 设置比较寄存器的值
#define MOTOR2_SET_REV_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(TIM_CHANNEL_2,ChannelPulse)    // 设置比较寄存器的值

#define MOTOR3_SET_FWD_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(TIM_CHANNEL_3,ChannelPulse)    // 设置比较寄存器的值
#define MOTOR3_SET_REV_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(TIM_CHANNEL_3,ChannelPulse)    // 设置比较寄存器的值

#define MOTOR4_SET_FWD_COMPAER(ChannelPulse)     TIM4_SetPWM_pulse(TIM_CHANNEL_1,ChannelPulse)    // 设置比较寄存器的值
#define MOTOR4_SET_REV_COMPAER(ChannelPulse)     TIM4_SetPWM_pulse(TIM_CHANNEL_2,ChannelPulse)    // 设置比较寄存器的值

#define MOTOR5_SET_FWD_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(TIM_CHANNEL_4,ChannelPulse)    // 设置比较寄存器的值
#define MOTOR5_SET_REV_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(TIM_CHANNEL_4,ChannelPulse)    // 设置比较寄存器的值

/* 使能输出 */
#define MOTOR1_FWD_ENABLE()      HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_1)
#define MOTOR1_REV_ENABLE()      HAL_TIMEx_PWMN_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_1)

#define MOTOR2_FWD_ENABLE()      HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_2)
#define MOTOR2_REV_ENABLE()      HAL_TIMEx_PWMN_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_2)

#define MOTOR3_FWD_ENABLE()      HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_3)
#define MOTOR3_REV_ENABLE()      HAL_TIMEx_PWMN_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_3)

#define MOTOR4_FWD_ENABLE()      HAL_TIM_PWM_Start(&TIM4_TimeBaseStructure,TIM_CHANNEL_1)   // 使能 PWM 通道 1
#define MOTOR4_REV_ENABLE()      HAL_TIM_PWM_Start(&TIM4_TimeBaseStructure,TIM_CHANNEL_2)   // 使能 PWM 通道 2

#define MOTOR5_FWD_ENABLE()      HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_4)
#define MOTOR5_REV_ENABLE()      HAL_TIMEx_PWMN_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_4)

/* 禁用输出 */
#define MOTOR1_FWD_DISABLE()     HAL_TIM_PWM_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_1)
#define MOTOR1_REV_DISABLE()     HAL_TIMEx_PWMN_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_1)

#define MOTOR2_FWD_DISABLE()     HAL_TIM_PWM_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_2)
#define MOTOR2_REV_DISABLE()     HAL_TIMEx_PWMN_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_2)

#define MOTOR3_FWD_DISABLE()     HAL_TIM_PWM_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_3)
#define MOTOR3_REV_DISABLE()     HAL_TIMEx_PWMN_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_3)

#define MOTOR4_FWD_DISABLE()     HAL_TIM_PWM_Stop(&TIM4_TimeBaseStructure,TIM_CHANNEL_1)
#define MOTOR4_REV_DISABLE()     HAL_TIM_PWM_Stop(&TIM4_TimeBaseStructure,TIM_CHANNEL_2)

#define MOTOR5_FWD_DISABLE()     HAL_TIM_PWM_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_4)
#define MOTOR5_REV_DISABLE()     HAL_TIMEx_PWMN_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_4)

void motor_init(void);
void set_motor1_speed(uint16_t v);
void set_motor1_direction(motor_dir_t dir);
void set_motor1_enable(void);
void set_motor1_disable(void);

void set_motor2_speed(uint16_t v);
void set_motor2_direction(motor_dir_t dir);
void set_motor2_enable(void);
void set_motor2_disable(void);

void set_motor3_speed(uint16_t v);
void set_motor3_direction(motor_dir_t dir);
void set_motor3_enable(void);
void set_motor3_disable(void);

void set_motor4_speed(uint16_t v);
void set_motor4_direction(motor_dir_t dir);
void set_motor4_enable(void);
void set_motor4_disable(void);

void set_motor5_speed(uint16_t v);
void set_motor5_direction(motor_dir_t dir);
void set_motor5_enable(void);
void set_motor5_disable(void);

void set_motor_position(int position);
void motor4_pid_control(void);
void motor3_pid_control(void);

void nSLEEP_gpio_config(void);

extern uint8_t     is_motor1_en;
extern uint8_t     is_motor2_en;
extern uint8_t     is_motor3_en;
extern uint8_t     is_motor4_en;
extern uint8_t     is_motor5_en;

extern uint16_t    motor1_dutyfactor;
extern uint16_t    motor2_dutyfactor;
extern uint16_t    motor3_dutyfactor;
extern uint16_t    motor4_dutyfactor;
extern uint16_t    motor5_dutyfactor;

extern motor_dir_t MOTOR1_direction;
extern motor_dir_t MOTOR2_direction;
extern motor_dir_t MOTOR3_direction;
extern motor_dir_t MOTOR4_direction;
extern motor_dir_t MOTOR5_direction;

void wakeup_motor(void);
void sleep_motor(void);
void motor_reset(void);

void motor5_control(void);

void BLE_control(void);
void parse_command(const char* data);
void execute_command(const Command* cmd);
void split_positions(const char* source, CurrentPosition* pos);

void motor1_motor2_motor3_motor4_control(Position selected_position);
void Fixed_control(void);
void random_control(void);
void horizontal_control(void);
void vertical_control(void);
void cross_control(void);
void custom_control(void);

int Fixed_chose(char *positions);
int freq_chose(const char *speed_str);
void random_chose(char *positions);

void repeat_function(void);

#endif

