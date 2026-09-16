#include "motor.h"
#include "config.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;

static void set_dir(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s){ HAL_GPIO_WritePin(port,pin,s); }

void motor_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g={0};
    g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin=MOTOR_AIN1_PIN|MOTOR_AIN2_PIN|MOTOR_BIN1_PIN|MOTOR_BIN2_PIN;
    HAL_GPIO_Init(GPIOB,&g);
    g.Pin=MOTOR_STBY_PIN; HAL_GPIO_Init(GPIOC,&g);

    HAL_GPIO_WritePin(GPIOB,MOTOR_AIN1_PIN|MOTOR_AIN2_PIN|MOTOR_BIN1_PIN|MOTOR_BIN2_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC,MOTOR_STBY_PIN,GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

static void pwm_left(uint16_t v){ __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,v); }
static void pwm_right(uint16_t v){ __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,v); }

void motor_set_left(int16_t speed)
{
    if(speed>MOTOR_PWM_MAX)speed=MOTOR_PWM_MAX; if(speed<-MOTOR_PWM_MAX)speed=-MOTOR_PWM_MAX;
    if(speed>0){set_dir(GPIOB,MOTOR_AIN1_PIN,GPIO_PIN_SET);set_dir(GPIOB,MOTOR_AIN2_PIN,GPIO_PIN_RESET);pwm_left((uint16_t)speed);}
    else if(speed<0){set_dir(GPIOB,MOTOR_AIN1_PIN,GPIO_PIN_RESET);set_dir(GPIOB,MOTOR_AIN2_PIN,GPIO_PIN_SET);pwm_left((uint16_t)-speed);}
    else{set_dir(GPIOB,MOTOR_AIN1_PIN,GPIO_PIN_RESET);set_dir(GPIOB,MOTOR_AIN2_PIN,GPIO_PIN_RESET);pwm_left(0);}
}
void motor_set_right(int16_t speed)
{
    if(speed>MOTOR_PWM_MAX)speed=MOTOR_PWM_MAX; if(speed<-MOTOR_PWM_MAX)speed=-MOTOR_PWM_MAX;
    if(speed>0){set_dir(GPIOB,MOTOR_BIN1_PIN,GPIO_PIN_SET);set_dir(GPIOB,MOTOR_BIN2_PIN,GPIO_PIN_RESET);pwm_right((uint16_t)speed);}
    else if(speed<0){set_dir(GPIOB,MOTOR_BIN1_PIN,GPIO_PIN_RESET);set_dir(GPIOB,MOTOR_BIN2_PIN,GPIO_PIN_SET);pwm_right((uint16_t)-speed);}
    else{set_dir(GPIOB,MOTOR_BIN1_PIN,GPIO_PIN_RESET);set_dir(GPIOB,MOTOR_BIN2_PIN,GPIO_PIN_RESET);pwm_right(0);}
}
void motor_enable(void){HAL_GPIO_WritePin(MOTOR_STBY_PORT,MOTOR_STBY_PIN,GPIO_PIN_SET);}
void motor_disable(void){HAL_GPIO_WritePin(MOTOR_STBY_PORT,MOTOR_STBY_PIN,GPIO_PIN_RESET);}
void motor_brake(void){motor_set_left(0);motor_set_right(0);}
