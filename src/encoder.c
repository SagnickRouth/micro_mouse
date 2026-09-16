#include "encoder.h"
#include "config.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

static int32_t count_left, count_right;
static uint16_t last_left, last_right;
static float speed_left, speed_right;

void encoder_init(void)
{
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
    encoder_reset();
}

void encoder_update(void)
{
    uint16_t l=(uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t r=(uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t dl=(int16_t)(l-last_left);
    int16_t dr=(int16_t)(r-last_right);
    count_left+=dl; count_right+=dr;
    speed_left=encoder_ticks_to_mm(dl)/CONTROL_DT;
    speed_right=encoder_ticks_to_mm(dr)/CONTROL_DT;
    last_left=l; last_right=r;
}
void encoder_reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2,0); __HAL_TIM_SET_COUNTER(&htim4,0);
    last_left=0; last_right=0; count_left=0; count_right=0; speed_left=0; speed_right=0;
}
int32_t encoder_get_left_count(void){return count_left;}
int32_t encoder_get_right_count(void){return count_right;}
float encoder_get_left_speed(void){return speed_left;}
float encoder_get_right_speed(void){return speed_right;}
float encoder_ticks_to_mm(int32_t ticks){return (float)ticks*MM_PER_TICK;}
