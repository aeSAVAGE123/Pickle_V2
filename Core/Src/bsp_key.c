#include "bsp_adc.h"
#include "bsp_key.h"
#include "bsp_pid.h"
#include "bsp_led.h"
#include "bsp_usart_blt.h"
#include "bsp_hc05.h"

int downcurrent = 0;
int upcurrent = 0;

_Bool cnt1 = 1;
_Bool cnt2 = 1;
_Bool cnt3 = 1;
_Bool cnt4 = 1;
_Bool cnt5 = 1;

_Bool upcnt1 = 1;
_Bool upcnt2 = 1;
_Bool upcnt3 = 1;
_Bool upcnt4 = 1;
_Bool upcnt5 = 1;
/**
  * @brief  配置按键用到的I/O口
  * @param  无
  * @retval 无
  */
void Key_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*开启按键GPIO口的时钟*/
    KEY1_GPIO_CLK_ENABLE();
    KEY2_GPIO_CLK_ENABLE();
    KEY3_GPIO_CLK_ENABLE();
    KEY4_GPIO_CLK_ENABLE();
    KEY5_GPIO_CLK_ENABLE();

    /*选择按键的引脚*/
    GPIO_InitStructure.Pin = KEY1_PIN;

    /*设置引脚为输入模式*/
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;

    /*设置引脚不上拉也不下拉*/
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    /*使用上面的结构体初始化按键*/
    HAL_GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
    /*选择按键的引脚*/
    GPIO_InitStructure.Pin = KEY2_PIN;
    /*使用上面的结构体初始化按键*/
    HAL_GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);

    /*选择按键的引脚*/
    GPIO_InitStructure.Pin = KEY3_PIN;
    /*使用上面的结构体初始化按键*/
    HAL_GPIO_Init(KEY3_GPIO_PORT, &GPIO_InitStructure);

    /*选择按键的引脚*/
    GPIO_InitStructure.Pin = KEY4_PIN;
    /*使用上面的结构体初始化按键*/
    HAL_GPIO_Init(KEY4_GPIO_PORT, &GPIO_InitStructure);

    /*选择按键的引脚*/
    GPIO_InitStructure.Pin = KEY5_PIN;
    /*使用上面的结构体初始化按键*/
    HAL_GPIO_Init(KEY5_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief   检测是否有按键按下
  * @param   具体的端口和端口位
  *		@arg GPIOx: x可以是（A...G）
  *		@arg GPIO_PIN 可以是GPIO_PIN_x（x可以是1...16）
  * @retval  按键的状态
  *		@arg KEY_ON:按键按下
  *		@arg KEY_OFF:按键没按下
  */
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
    /*检测是否有按键按下 */
    if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON )
    {
        /*等待按键释放 */
        HAL_Delay(20);
        while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON);
        HAL_Delay(20);
        return 	KEY_ON;
    }
    else
        return KEY_OFF;
}

void Key_control(void)
{
    /* 扫描KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
        LED1_TOGGLE
        if(speedflag == 1)
        {
            speedflag = 0;
            set_motor1_disable();
            set_motor2_disable();
        }
        else
        {
            speedflag = 1;
            set_motor1_enable();                            //A2
            set_motor1_direction(MOTOR_FWD);
            set_motor1_speed(2000);
            set_motor2_enable();                            //A2
            set_motor2_direction(MOTOR_REV);
            set_motor2_speed(2000);
        }
    }

    /* 扫描KEY2 */
    if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
        LED2_TOGGLE
        if(Pid3flag == 1)
        {
            Pid3flag = 0;
        }
        else
        {
            Pid3flag = 1;
            set_pid_target3(&pid3, 900);

        }
    }

    /* 扫描KEY3 */
    if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON)
    {
        if(Pid4flag == 1)
        {
            Pid4flag = 0;
        }
        else
        {
            Pid4flag = 1;
            set_pid_target4(&pid4, 1400);
        }
        LED3_TOGGLE
    }

    /* 扫描KEY4 */
    if( Key_Scan(KEY4_GPIO_PORT, KEY4_PIN) == KEY_ON)
    {
        LED4_TOGGLE
        set_pid_target3(&pid3, 1800);
        set_pid_target4(&pid4, 700);
    }

    /* 扫描KEY5 */
    if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
    {
        if(!is_motor5_en)
        {
            set_motor5_enable();
            set_motor5_direction(MOTOR_FWD);
            set_motor5_speed(4000);
        }
        else
        {
            set_motor5_disable();
        }
        LED5_TOGGLE
    }
}

void Knob_control(void)
{
    if(speedflag == 1)
    {
        int32_t tempspeed = Rotation1_adc_mean;
        if(tempspeed > 3000)
        {
            tempspeed = 3000;
        }
        set_motor1_speed(tempspeed);
        set_motor2_speed(tempspeed);
    }
    if(Pid3flag == 1)
    {
        downcurrent = Rotation2_adc_mean;
        if(downcurrent > 0 && downcurrent < 819)
        {
            if(cnt1 == 1)
            {
                set_pid_target3(&pid3,780);
                cnt1 = 0;
            }
        } else
        {
            cnt1 = 1;
        }
        if(downcurrent >= 819 && downcurrent < 1638)
        {
            if(cnt2 == 1)
            {
                set_pid_target3(&pid3,1250);
                cnt2 = 0;
            }
        } else
        {
            cnt2 = 1;
        }
        if(downcurrent >= 1638 && downcurrent < 2457)
        {
            if(cnt3 == 1)
            {
                set_pid_target3(&pid3,1600);
                cnt3 = 0;
            }
        } else
        {
            cnt3 = 1;
        }
        if(downcurrent >= 2457 && downcurrent < 3276)
        {
            if(cnt4 == 1)
            {
                set_pid_target3(&pid3,1800);
                cnt4 = 0;
            }
        } else
        {
            cnt4 = 1;
        }
        if(downcurrent >= 3276 && downcurrent < 4095)
        {
            if(cnt5 == 1)
            {
                set_pid_target3(&pid3,2400);
                cnt5 = 0;
            }
        } else
        {
            cnt5 = 1;
        }
    }
    if(Pid4flag == 1)
    {
        upcurrent = Rotation3_adc_mean;
        if(upcurrent > 0 && upcurrent < 819)
        {
            if(upcnt1 == 1)
            {
                set_pid_target4(&pid4,700);
                upcnt1 = 0;
            }
        } else
        {
            upcnt1 = 1;
        }
        if(upcurrent >= 819 && upcurrent < 1638)
        {
            if(upcnt2 == 1)
            {
                set_pid_target4(&pid4,750);
                upcnt2 = 0;
            }
        } else
        {
            upcnt2 = 1;
        }
        if(upcurrent >= 1638 && upcurrent < 2457)
        {
            if(upcnt3 == 1)
            {
                set_pid_target4(&pid4,800);
                upcnt3 = 0;
            }
        } else
        {
            upcnt3 = 1;
        }
        if(upcurrent >= 2457 && upcurrent < 3276)
        {
            if(upcnt4 == 1)
            {
                set_pid_target4(&pid4,900);
                upcnt4 = 0;
            }
        } else
        {
            upcnt4 = 1;
        }
        if(upcurrent >= 3276 && upcurrent < 4095)
        {
            if(upcnt5 == 1)
            {
                set_pid_target4(&pid4,1000);
                upcnt5 = 0;
            }
        } else
        {
            upcnt5 = 1;
        }
    }
}
/*********************************************END OF FILE**********************/

