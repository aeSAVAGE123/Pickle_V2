#include "bsp_motor_control.h"
#include "bsp_debug_usart.h"
#include "bsp_adc.h"
#include "bsp_led.h"
//#include "bsp_pid.h"
#include "bsp_basic_tim.h"
#include "protocol.h"
#include "bsp_tim.h"
#include <math.h>
#include "bsp_hc05.h"
#include "bsp_usart_blt.h"
#include <string.h>     //提供了一系列操作字符串的函数和宏定义，例如字符串的复制、连接、比较等。常用函数包括 strcpy()、strcat()、strcmp() 等
#include <stdlib.h>     //包含了一些常用的函数原型，用于内存分配、类型转换、随机数生成等。常用函数包括 malloc()、free()、atoi() 等。

motor_dir_t MOTOR1_direction  = MOTOR_REV;       // 记录方向
motor_dir_t MOTOR2_direction  = MOTOR_FWD;       // 记录方向
motor_dir_t MOTOR3_direction  = MOTOR_REV;       // 记录方向
motor_dir_t MOTOR4_direction  = MOTOR_FWD;       // 记录方向
motor_dir_t MOTOR5_direction  = MOTOR_FWD;       // 记录方向

uint16_t    motor1_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机1占空比
uint16_t    motor2_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机2占空比
uint16_t    motor3_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机3占空比
uint16_t    motor4_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机4占空比
uint16_t    motor5_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机5占空比

uint8_t     is_motor1_en = 0;            			// 电机1使能
uint8_t     is_motor2_en = 0;            			// 电机2使能
uint8_t     is_motor3_en = 0;            			// 电机3使能
uint8_t     is_motor4_en = 0;            			// 电机4使能
uint8_t     is_motor5_en = 0;            			// 电机5使能

unsigned int Task_Delay[NumOfTask];
char linebuff[1024];

uint16_t sensor_triggered = 0;
uint16_t M5_done = 0;

int  currentSelectPosition = -1;
int  currentSelectrepeat_count = -1;

int currentSelectmotor5_speed = 3000;

volatile int new_data_flag = 0; // 标志位，表示有新数据
uint8_t fixed_control_state = 0;
uint8_t random_state = 0;
int reset_flag = 0;
int mode_select = 0;//辨别执行哪个模式

typedef enum
{
    REPEAT_IDLE,
    REPEAT_RUNNING,
    REPEAT_WAITING_SENSOR
} RepeatState;

RepeatState repeat_state = REPEAT_IDLE;             //初始化状态机
int loop_count = 0;
int repeat_count_comparison_value = 0;
int motor5_current_count = 0;

Position all_positions[25] =
        {
                {1100,2500,2000,2000},{1100,2250,2000,2000}, {1100,2100,2000,2000},{1100,2100,2000,2000},{1100,1920,2000,2000},//A1-E1

                {1100,1900,2000,2000},{1100, 1900,2000,2000},{1100,1750,2000,2000},{1100,1780,2000,2000},{1100,1720,2000,2000},//A2-E2

                {1100,1738,2000,2000},{1100, 1591,2000,2000},{1100, 1580,2000,2000},{1100, 1530,2000,2000}, {1100, 1560,2000,2000}, //A3-E3

                {1100, 1290,2000,2000}, {1100,745,2000,2000}, {1100, 1200,2000,2000}, {1100, 1300,2000,2000}, {1100, 1230,2000,2000},//A4-E4

                {1100,800,2000,2000},{1100,789,2000,2000},{1100,1000,2000,2000},{1100,1200,2000,2000},{1100, 800,2000,2000}//A5-E5

        };

Position left_positions[15] =
        {
                {1100,2500,2000,2000},{1100,2250,2000,2000}, {1100,2100,2000,2000},{1100,2100,2000,2000},{1100,1920,2000,2000},//A1-E1

                {1100,1900,2000,2000},{1100, 1900,2000,2000},{1100,1750,2000,2000},{1100,1780,2000,2000},{1100,1720,2000,2000},//A2-E2

                {1100,1738,2000,2000},{1100, 1591,2000,2000},{1100, 1580,2000,2000},{1100, 1530,2000,2000}, {1100, 1560,2000,2000}, //A3-E3
        };

Position right_positions[15] =
        {
                {1100,1738,2000,2000},{1100, 1591,2000,2000},{1100, 1580,2000,2000},{1100, 1530,2000,2000}, {1100, 1560,2000,2000}, //A3-E3

                {1100, 1290,2000,2000}, {1100,745,2000,2000}, {1100, 1200,2000,2000}, {1100, 1300,2000,2000}, {1100, 1230,2000,2000},//A4-E4

                {1100,800,2000,2000},{1100,789,2000,2000},{1100,1000,2000,2000},{1100,1200,2000,2000},{1100, 800,2000,2000}//A5-E5
        };

// 函数来生成指定范围内的随机数
int generate_random_all_position(void) {
    return rand() % 25; // 返回0到24之间的随机数
}

int generate_random_left_position(void) {
    return rand() % 15; // 返回0到24之间的随机数
}

int generate_random_right_position(void) {
    return rand() % 15; // 返回0到24之间的随机数
}

void wakeup_motor(void)
{
    MOTOR_ENABLE_nSLEEP();
}

void sleep_motor(void)
{
    MOTOR_DISABLE_nSLEEP();
}

/**
  * @brief  设置电机1速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor1_speed(uint16_t v)
{
    motor1_dutyfactor = v;

    if (MOTOR1_direction == MOTOR_FWD)
    {
        MOTOR1_SET_FWD_COMPAER(motor1_dutyfactor);     // 设置速度
        MOTOR1_REV_DISABLE();
        MOTOR1_FWD_ENABLE();
    }
    else
    {
        MOTOR1_SET_REV_COMPAER(motor1_dutyfactor);     // 设置速度
        MOTOR1_FWD_DISABLE();
        MOTOR1_REV_ENABLE();
    }
}

/**
  * @brief  设置电机1方向
  * @param  无
  * @retval 无
  */
void set_motor1_direction(motor_dir_t dir)
{
    MOTOR1_direction = dir;

    if (MOTOR1_direction == MOTOR_FWD)
    {
        MOTOR1_SET_FWD_COMPAER(motor1_dutyfactor);      // 设置正向速度
        MOTOR1_REV_DISABLE();                           // 设置反向速度
        MOTOR1_FWD_ENABLE();
    }
    else
    {
        MOTOR1_FWD_DISABLE();                           // 设置正向速度
        MOTOR1_SET_REV_COMPAER(motor1_dutyfactor);      // 设置反向速度
        MOTOR1_REV_ENABLE();
    }
}

/**
  * @brief  使能电机1
  * @param  方向
  * @retval 无
  */
void set_motor1_enable(void)
{
    is_motor1_en = 1;
    MOTOR1_FWD_ENABLE();
    MOTOR1_REV_ENABLE();
}

/**
  * @brief  禁用电机1
  * @param  无
  * @retval 无
  */
void set_motor1_disable(void)
{
    is_motor1_en = 0;
    MOTOR1_FWD_DISABLE();
    MOTOR1_REV_DISABLE();
}

/**
  * @brief  设置电机2速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor2_speed(uint16_t v)
{
    motor2_dutyfactor = v;

    if (MOTOR2_direction == MOTOR_FWD)
    {
        MOTOR2_SET_FWD_COMPAER(motor2_dutyfactor);     // 设置速度
        MOTOR2_REV_DISABLE();
        MOTOR2_FWD_ENABLE();
    }
    else
    {
        MOTOR2_SET_REV_COMPAER(motor2_dutyfactor);     // 设置速度
        MOTOR2_FWD_DISABLE();
        MOTOR2_REV_ENABLE();
    }
}

/**
  * @brief  设置电机2方向
  * @param  无
  * @retval 无
  */
void set_motor2_direction(motor_dir_t dir)
{
    MOTOR2_direction = dir;

    if (MOTOR2_direction == MOTOR_FWD)
    {
        MOTOR2_SET_FWD_COMPAER(motor2_dutyfactor);      // 设置正向速度
        MOTOR2_REV_DISABLE();
        MOTOR2_FWD_ENABLE();
    }
    else
    {
        MOTOR2_FWD_DISABLE();                           // 设置正向速度
        MOTOR2_SET_REV_COMPAER(motor2_dutyfactor);      // 设置反向速度
        MOTOR2_REV_ENABLE();
    }
}

/**
  * @brief  使能电机2
  * @param  无
  * @retval 无
  */
void set_motor2_enable(void)
{
    is_motor2_en = 1;
    MOTOR2_FWD_ENABLE();
    MOTOR2_REV_ENABLE();
}

/**
  * @brief  禁用电机2
  * @param  无
  * @retval 无
  */
void set_motor2_disable(void)
{
    is_motor2_en = 0;
    MOTOR2_FWD_DISABLE();
    MOTOR2_REV_DISABLE();
}

/**
  * @brief  设置电机3速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor3_speed(uint16_t v)
{
    motor3_dutyfactor = v;

    if (MOTOR3_direction == MOTOR_FWD)
    {
        MOTOR3_SET_FWD_COMPAER(motor3_dutyfactor);     // 设置速度
        MOTOR3_REV_DISABLE();
        MOTOR3_FWD_ENABLE();
    }
    else
    {
        MOTOR3_SET_REV_COMPAER(motor3_dutyfactor);     // 设置速度
        MOTOR3_FWD_DISABLE();
        MOTOR3_REV_ENABLE();
    }
}

/**
  * @brief  设置电机3方向
  * @param  无
  * @retval 无
  */
void set_motor3_direction(motor_dir_t dir)
{
    MOTOR3_direction = dir;

    if (MOTOR3_direction == MOTOR_FWD)
    {
        MOTOR3_SET_FWD_COMPAER(motor3_dutyfactor);      // 设置正向速度
        MOTOR3_REV_DISABLE();
        MOTOR3_FWD_ENABLE();
    }
    else
    {
        MOTOR3_FWD_DISABLE();                           // 设置正向速度
        MOTOR3_SET_REV_COMPAER(motor3_dutyfactor);      // 设置反向速度
        MOTOR3_REV_ENABLE();
    }
}

/**
  * @brief  使能电机3
  * @param  无
  * @retval 无
  */
void set_motor3_enable(void)
{
    is_motor3_en = 1;
    MOTOR3_FWD_ENABLE();
    MOTOR3_REV_ENABLE();
}

/**
  * @brief  禁用电机3
  * @param  无
  * @retval 无
  */
void set_motor3_disable(void)
{
    is_motor3_en = 0;
    MOTOR3_FWD_DISABLE();
    MOTOR3_REV_DISABLE();
}

/**
  * @brief  设置电机4速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor4_speed(uint16_t v)
{
    motor4_dutyfactor = v;

    if (MOTOR4_direction == MOTOR_FWD)
    {
        MOTOR4_SET_FWD_COMPAER(motor4_dutyfactor);     // 设置速度
        MOTOR4_REV_DISABLE();
        MOTOR4_FWD_ENABLE();
    }
    else
    {
        MOTOR4_SET_REV_COMPAER(motor4_dutyfactor);     // 设置速度
        MOTOR4_FWD_DISABLE();
        MOTOR4_REV_ENABLE();
    }
}

/**
  * @brief  设置电机4方向
  * @param  无
  * @retval 无
  */
void set_motor4_direction(motor_dir_t dir)
{
    MOTOR4_direction = dir;

    if (MOTOR4_direction == MOTOR_FWD)
    {
        MOTOR4_SET_FWD_COMPAER(motor4_dutyfactor);      // 设置正向速度
        MOTOR4_REV_DISABLE();
    }
    else
    {
        MOTOR4_FWD_DISABLE();                           // 设置正向速度
        MOTOR4_SET_REV_COMPAER(motor4_dutyfactor);      // 设置反向速度
    }
}

/**
  * @brief  使能电机4
  * @param  无
  * @retval 无
  */
void set_motor4_enable(void)
{
    is_motor4_en = 1;
    MOTOR4_FWD_ENABLE();
    MOTOR4_REV_ENABLE();
}

/**
  * @brief  禁用电机4
  * @param  无
  * @retval 无
  */
void set_motor4_disable(void)
{
    is_motor4_en = 0;
    MOTOR4_FWD_DISABLE();
    MOTOR4_REV_DISABLE();
}

/**
  * @brief  设置电机5速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor5_speed(uint16_t v)
{
    motor5_dutyfactor = v;

    if (MOTOR5_direction == MOTOR_FWD)
    {
        MOTOR5_SET_FWD_COMPAER(motor5_dutyfactor);     // 设置速度
        MOTOR5_REV_DISABLE();
        MOTOR5_FWD_ENABLE();
    }
    else
    {
        MOTOR5_SET_REV_COMPAER(motor5_dutyfactor);     // 设置速度
        MOTOR5_FWD_DISABLE();
        MOTOR5_REV_ENABLE();
    }
}

/**
  * @brief  设置电机5方向
  * @param  无
  * @retval 无
  */
void set_motor5_direction(motor_dir_t dir)
{
    MOTOR5_direction = dir;

    if (MOTOR5_direction == MOTOR_FWD)
    {
        MOTOR5_SET_FWD_COMPAER(motor5_dutyfactor);      // 设置正向速度
        MOTOR5_REV_DISABLE();
        MOTOR5_FWD_ENABLE();
    }
    else
    {
        MOTOR5_FWD_DISABLE();                           // 设置正向速度
        MOTOR5_SET_REV_COMPAER(motor5_dutyfactor);      // 设置反向速度
        MOTOR5_REV_ENABLE();
    }
}

/**
  * @brief  使能电机5
  * @param  无
  * @retval 无
  */
void set_motor5_enable(void)
{
    is_motor5_en = 1;
    MOTOR5_FWD_ENABLE();
    MOTOR5_REV_ENABLE();
}

/**
  * @brief  禁用电机5
  * @param  无
  * @retval 无
  */
void set_motor5_disable(void)
{
    is_motor5_en = 0;
    MOTOR5_FWD_DISABLE();
    MOTOR5_REV_DISABLE();
}

/**
  * @brief  下电机3增量式 PID 控制实现(定时调用)
  * @param  无
  * @retval 无
  */
void motor3_pid_control(void)
{
    if (is_motor3_en == 1)    			 																										 										// 电机在使能状态下才进行控制处理
    {
        float cont_val = 0;    //存储PID计算的控制值
        int temp_val = 0;          //存储处理后的控制值   																										 										// 当前控制值

        cont_val = PID_realize(&pid3, positiondown_adc_mean, Pflag3, Iflag3, Iflagz3);    																 	 						// 将Pid3的值传递给函数进行 PID 计算

        if (cont_val > 0)   	 																														 										// 判断电机方向
        {
            set_motor3_direction(MOTOR_FWD);
        }
        else
        {
            set_motor3_direction(MOTOR_REV);
        }
        temp_val = (fabs(cont_val) > PWM_MAX_PERIOD_COUNT*0.9) ? PWM_MAX_PERIOD_COUNT*0.9 : fabs(cont_val);    // 速度上限处理    判断绝对值是否大于5500的80%，大于则是5500的80%，小于则是绝对值
        set_motor3_speed(temp_val);                                                                     			 // 设置 PWM 占空比
    }
}

/**
  * @brief  上电机4增量式 PID 控制实现(定时调用)
  * @param  无
  * @retval 无
  */
void motor4_pid_control(void)
{
    if (is_motor4_en == 1)    			 																										 										// 电机在使能状态下才进行控制处理
    {
        float cont_val = 0;
        int temp_val = 0;             																										 										// 当前控制值
        if(positionup_adc_mean < 659)
        {
            positionup_adc_mean = 659;
        }
        if(positionup_adc_mean > 2124)
        {
            positionup_adc_mean = 2124;
        }
        cont_val = PID_realize(&pid4, positionup_adc_mean, Pflag4, Iflag4, Iflagz4);    																 	 							// 进行 PID 计算

        int32_t err = pid4.err;
        int32_t err_last = pid4.err_last;
        int32_t err_next = pid4.err_next;

        if (cont_val > 0)   	 																														 										// 判断电机方向
        {
            set_motor4_direction(MOTOR_REV);
        }
        else
        {
            set_motor4_direction(MOTOR_FWD);
        }
        temp_val = (fabs(cont_val) > PWM_MAX_PERIOD_COUNT*0.9) ? PWM_MAX_PERIOD_COUNT*0.9 : fabs(cont_val);    // 速度上限处理
        set_motor4_speed(temp_val);                                                                     	    // 设置 PWM 占空比
    }
}

void BLE_control(void)
{
    char* redata;       //定义读数据的指针
    uint16_t len;       //定义数据大小

    if (IS_BLE_CONNECTED())                 //判断INT引脚电平是否发生变化
    {
        HAL_Delay(5);
        BLE_WAKEUP_LOW;                     //蓝牙wakeup引脚置0，启动蓝牙
        uint16_t linelen;                   //定义数据的长度
        /*获取数据*/
        redata = get_rebuff(&len);        //把蓝牙数据读取到redata
        linelen = get_line(linebuff, redata, len);  //计算接收到的数据的长度
        /*检查数据是否有更新*/
        if (linelen < 50 && linelen != 0)
        {
            parse_command(redata);            // 解析命令

            clean_rebuff();                     // 处理数据后，清空接收蓝牙模块数据的缓冲区
        }
    }
}

void parse_command(const char* data)  //把接收到的蓝牙数据进行解析
{
    char temp_data[50];                                  // 复制数据以避免破坏原始数据
    strncpy(temp_data, data, sizeof(temp_data) - 1);
    temp_data[sizeof(temp_data) - 1] = '\0';            // 确保字符串以 '\0' 结尾

    if(strcmp(temp_data, "8") == 0)
    {
        motor5_control();
    }
    else if(strcmp(temp_data, "9") == 0)
    {
        reset_flag = 1;
    }
    else if(strcmp(temp_data, "9") != 0 && strcmp(temp_data, "8") != 0)
    {
        Command cmd = {0};                                    // 初始化结构体为零
        // 分割字符串
        char* token = strtok(temp_data, "-");  //将一个字符串分割成一系列的标记（tokens），每个标记之间由指定的分隔符隔开。返回指向被分割的第一个标记的指针。如果没有更多的标记，则返回 NULL。
        int token_count = 0;
        while (token != NULL)
        {
            switch (token_count)
            {
                case 0:
                    // 提取模式，直接使用字符
                    cmd.mode = token[0];
                    break;
                case 1:
                    // 提取位点
                    strncpy(cmd.positions, token, sizeof(cmd.positions) - 1);
                    cmd.positions[sizeof(cmd.positions) - 1] = '\0';
                    break;
                case 2:
                    // 提取循环次数
                    cmd.current_repeat_count = strtol(token, NULL, 10);     //使用strtol转换为十进制整数
                case 3:
                    // 提取频率
                    strncpy(cmd.speed_str, token, sizeof(cmd.speed_str) - 1);
                    break;
                default:
                    break;
            }
            token = strtok(NULL, "-");      //提取 ‘-’ 前面的字符串
            token_count++;
        }
        execute_command(&cmd);              // 解析数据后，执行新命令
    }
}

// 定义全局变量
char current_random_positions[20]; // 假设最大长度为20，根据需要调整大小

void execute_command(const Command* cmd)
{
    int speed_value = freq_chose(cmd->speed_str);    //设置频率
    if (speed_value > 0)
    {
        currentSelectmotor5_speed = speed_value;
    }
    switch (cmd->mode)                  //区分模式
    {
        case '1':
            // 定点模式
            currentSelectPosition = Fixed_chose(cmd->positions); // 假设有个 Fixed_chose 函数处理位点选择
            mode_select = 1;
            break;
        case '2':
            //随机模式
            mode_select = 2;
            strcpy(current_random_positions, cmd->positions); // 更新 current_positions
            break;
        case '3':
            // 水平模式
            // 处理水平模式的位点选择
            break;
            // 其他模式逻辑...
    }
    // 设置循环次数
    if (cmd->current_repeat_count > 0)
    {
        currentSelectrepeat_count = cmd->current_repeat_count;
        motor5_current_count = currentSelectrepeat_count;
        repeat_flag = 1;
    }
    new_data_flag = 1;                  // 设置新数据标志位，表示更新数据
}

int Fixed_chose(char *positions)       //根据positions的判断，返回对应的值，作为数组的信号，确定对应的点位
{
    if (strcmp(positions, "A1") == 0)
    {
        return 0;
    }
    else if (strcmp(positions, "A2") == 0)
    {
        return 5;
    }
    else if (strcmp(positions, "A3") == 0)
    {
        return 10;
    }
    else if (strcmp(positions, "A4") == 0)
    {
        return 15;
    }
    else if (strcmp(positions, "A5") == 0)
    {
        return 20;
    }
    else if (strcmp(positions, "B1") == 0)
    {
        return 1;
    }
    else if (strcmp(positions, "B2") == 0)
    {
        return 6;
    }
    else if (strcmp(positions, "B3") == 0)
    {
        return 11;
    }
    else if (strcmp(positions, "B4") == 0)
    {
        return 16;
    }
    else if (strcmp(positions, "B5") == 0)
    {
        return 21;
    }
    else if (strcmp(positions, "C1") == 0)
    {
        return 2;
    }
    else if (strcmp(positions, "C2") == 0)
    {
        return 7;
    }
    else if (strcmp(positions, "C3") == 0)
    {
        return 12;
    }
    else if (strcmp(positions, "C4") == 0)
    {
        return 17;
    }
    else if (strcmp(positions, "C5") == 0)
    {
        return 22;
    }
    else if (strcmp(positions, "D1") == 0)
    {
        return 3;
    }
    else if (strcmp(positions, "D2") == 0)
    {
        return 8;
    }
    else if (strcmp(positions, "D3") == 0)
    {
        return 13;
    }
    else if (strcmp(positions, "D4") == 0)
    {
        return 18;
    }
    else if (strcmp(positions, "D5") == 0)
    {
        return 23;
    }
    else if (strcmp(positions, "E1") == 0)
    {
        return 4;
    }
    else if (strcmp(positions, "E2") == 0)
    {
        return 9;
    }
    else if (strcmp(positions, "E3") == 0)
    {
        return 14;
    }
    else if (strcmp(positions, "E4") == 0)
    {
        return 19;
    }
    else if (strcmp(positions, "E5") == 0)
    {
        return 24;
    }
    else
    {
        return  -1;
    }
}

Position selected_random_position;

void random_chose(char *positions)          //根据positions的判断，返回对应的值，作为数组的信号，确定对应的随机点位
{
    int index;
    if (strcmp(positions, "AA") == 0)
    {
        srand(HAL_GetTick()); // 初始化随机数发生器
        index = generate_random_all_position(); // 获取随机位置索引
        selected_random_position = all_positions[index]; // 获取选定的位置数据
    }
    else if (strcmp(positions, "BB") == 0)
    {
        srand(HAL_GetTick()); // 初始化随机数发生器
        index = generate_random_left_position(); // 获取随机位置索引
        selected_random_position = left_positions[index]; // 获取选定的位置数据
    }
    else if (strcmp(positions, "CC") == 0)
    {
        srand(HAL_GetTick()); // 初始化随机数发生器
        index = generate_random_right_position(); // 获取随机位置索引
        selected_random_position = right_positions[index]; // 获取选定的位置数据
    }
}

int freq_chose(const char *speed_str)
{
    if (strcmp(speed_str, "01")== 0)
    {
        return 3500;
    }
    if (strcmp(speed_str, "02")== 0)
    {
        return 4000;
    }
    if (strcmp(speed_str, "03")== 0)
    {
        return 4500;
    }
    if (strcmp(speed_str, "04")== 0)
    {
        return 5000;
    }
    if (strcmp(speed_str, "05")== 0)
    {
        return 5500;
    }
    return -1;  // 默认值，如果未匹配任何已知速度
}

void motor1_motor2_motor3_motor4_control(Position selected_position)
{
    set_pid_target3(&pid3, selected_position.vertical);
//                set_pid_target4(&pid4, selected_position.horizontal);
//                set_motor1_enable();
//                set_motor1_direction(MOTOR_FWD);
//                set_motor1_speed(selected_position.M1speed);
//
//                set_motor2_enable();
//                set_motor2_direction(MOTOR_REV);
//                set_motor2_speed(selected_position.M2speed);
}

void Fixed_control(void)
{
    static uint32_t last_tick = 0;
    static Position selected_position;
    switch (fixed_control_state)
    {
        case 0:
            // 初始化并启动 M1、M2、M3、M4
            selected_position = all_positions[currentSelectPosition];
            motor1_motor2_motor3_motor4_control(selected_position);
            last_tick = HAL_GetTick();  // 获取当前时间戳
            fixed_control_state = 1;
            break;

        case 1:
            LED3_TOGGLE
            // 等待一段时间以确保发球机移动到位
            if (HAL_GetTick() - last_tick >= 1000)  // 例如等待1000ms
            {
                fixed_control_state = 2;
            }
            break;

        case 2:
            // 检测Dropping_adc_mean以判断球是否落下
            if (Dropping_adc_mean < 400)
            {
                set_motor5_disable();                // 关闭电机5
                sensor_triggered = 1;
                // 处理重复次数
                if (motor5_current_count  > 0)
                {
                    motor5_current_count--; // 减少当前循环次数
                }
                if (motor5_current_count == 0) // 检查是否达到预定的循环次数
                {
                    fixed_control_state = 0;  // 重置状态机
                }
            }
            break;
    }
}

void random_control(void)
{
    static uint32_t last_tick = 0;
    switch (random_state)
    {
        case 0:
            random_chose(current_random_positions);
            motor1_motor2_motor3_motor4_control(selected_random_position);
            last_tick = HAL_GetTick();  // 获取当前时间戳
            random_state = 1;
            break;

        case 1:
            if (HAL_GetTick() - last_tick >= 3000)                  // 等待电机转动
            {
                random_state = 2;
            }
            break;

        case 2:
            if (Dropping_adc_mean < 400)
            {
                // 关闭电机5
                set_motor5_disable();
                sensor_triggered = 1;

                // 处理重复次数
//                if (motor5_current_count  > 0)
//                {
//                    motor5_current_count--; // 减少当前循环次数
                    random_state = 0;  // 重置状态机
//                }
            }
            break;
    }
}

void repeat_function(void)
{
    //每次进入函数，首先检查 new_data_flag 标志位
    if (new_data_flag == 1)                          //如果有数据更新，重置状态机
    {
        set_motor5_disable();
        repeat_state = REPEAT_IDLE;                 //重置状态机
        fixed_control_state = 0;                    //重置Fixed_control状态机
        random_state = 0;                           //重置random_control状态机
        loop_count = 1;                             //初始化重复计数器
        new_data_flag = 0;                          //清除新数据标志位
    }
    switch (repeat_state)
    {
        case REPEAT_IDLE:
            repeat_state = REPEAT_RUNNING;          //准备进入下一状态
            break;

        case REPEAT_RUNNING:
            repeat_state = REPEAT_WAITING_SENSOR;      //准备等待传感器触发
            break;

        case REPEAT_WAITING_SENSOR:
            repeat_count_comparison_value = currentSelectrepeat_count;      //传递需要循环的次数
            switch (mode_select)
            {
                case 1:
                    Fixed_control();            //定点模式
                    break;
                case 2:
                    random_control();           //随机模式
                    break;
            }
            if (M5_done == 1)
            {
                M5_done = 0;
                HAL_Delay(3000);
                set_motor5_enable();
            }
            if (sensor_triggered == 1)                                      //表示传感器已经触发
            {
                sensor_triggered = 0;                                       //表示尚未触发传感器
                if (loop_count >= repeat_count_comparison_value)            //检查是否达到需要重复的次数
                {
                    repeat_state = REPEAT_IDLE;                             //达到了表示任务完成
                }
                else
                {
                    loop_count++;                                            //将计数器自增1
                    M5_done = 1;
                    repeat_state = REPEAT_RUNNING;                          //未达到返回上一状态再循环
                }
            }
            break;
    }
}

void motor_reset(void)
{
    if(1 == reset_flag)
    {
        LED5_TOGGLE
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
    reset_flag = 0;
}

void motor5_control(void)
{
    LED5_TOGGLE
//    motor5_current_count = currentSelectrepeat_count;
    if (!is_motor5_en) // 当 is_motor5_en 为 0（假）时，这里的代码将会执行
    {
        set_motor5_direction(MOTOR_REV);
        set_motor5_speed(currentSelectmotor5_speed);
        set_motor5_enable(); // 启用电机
    }
    else
    {
        set_motor5_disable(); // 停止电机
    }
}
