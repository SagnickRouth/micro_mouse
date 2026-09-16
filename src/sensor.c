#include "sensor.h"
#include "stm32f4xx_hal.h"
#include "vl53l0x_api.h"
#include <string.h>

/*
 * This file is the STM32 integration layer for ST's VL53L0X API.
 * Add the official VL53L0X API sources (core + platform) to the CubeIDE
 * project. The platform layer must implement I2C/delay using hi2c1.
 */
extern I2C_HandleTypeDef hi2c1;

SensorData sensor_data;

static const struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t address;
} sensor_hw[SENSOR_COUNT] = {
    {VL53_LEFT_XSHUT_PORT,  VL53_LEFT_XSHUT_PIN,  VL53_ADDR_LEFT},
    {VL53_FL_XSHUT_PORT,    VL53_FL_XSHUT_PIN,    VL53_ADDR_FL},
    {VL53_FR_XSHUT_PORT,    VL53_FR_XSHUT_PIN,    VL53_ADDR_FR},
    {VL53_RIGHT_XSHUT_PORT, VL53_RIGHT_XSHUT_PIN, VL53_ADDR_RIGHT}
};

static VL53L0X_Dev_t dev[SENSOR_COUNT];
static bool ready[SENSOR_COUNT];

static void xshut_all(bool on)
{
    for (int i=0;i<SENSOR_COUNT;i++)
        HAL_GPIO_WritePin(sensor_hw[i].port,sensor_hw[i].pin,on?GPIO_PIN_SET:GPIO_PIN_RESET);
}

static void xshut_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g={0};
    g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_LOW;
    g.Pin=VL53_LEFT_XSHUT_PIN|VL53_FL_XSHUT_PIN|VL53_FR_XSHUT_PIN|VL53_RIGHT_XSHUT_PIN;
    HAL_GPIO_Init(GPIOA,&g);
    xshut_all(false);
}

bool sensor_assign_addresses(void)
{
    memset(ready,0,sizeof(ready));
    xshut_all(false);
    HAL_Delay(5);

    for (int i=0;i<SENSOR_COUNT;i++) {
        HAL_GPIO_WritePin(sensor_hw[i].port,sensor_hw[i].pin,GPIO_PIN_SET);
        HAL_Delay(3);

        memset(&dev[i],0,sizeof(dev[i]));
        dev[i].I2cDevAddr = VL53_DEFAULT_ADDR;

        if (VL53L0X_DataInit(&dev[i]) != VL53L0X_ERROR_NONE) return false;
        if (VL53L0X_SetDeviceAddress(&dev[i], sensor_hw[i].address) != VL53L0X_ERROR_NONE) return false;
        dev[i].I2cDevAddr = sensor_hw[i].address;
        ready[i]=true;
    }
    return true;
}

bool sensor_init(void)
{
    xshut_init();
    if (!sensor_assign_addresses()) return false;

    for (int i=0;i<SENSOR_COUNT;i++) {
        if (!ready[i]) return false;
        if (VL53L0X_StaticInit(&dev[i]) != VL53L0X_ERROR_NONE) return false;
        if (VL53L0X_PerformRefCalibration(&dev[i], 0, 0) != VL53L0X_ERROR_NONE) return false;
        if (VL53L0X_SetDeviceMode(&dev[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING) != VL53L0X_ERROR_NONE) return false;
        if (VL53L0X_StartMeasurement(&dev[i]) != VL53L0X_ERROR_NONE) return false;
    }
    memset(&sensor_data,0,sizeof(sensor_data));
    return true;
}

bool sensor_read_all(void)
{
    bool ok=true;
    for (int i=0;i<SENSOR_COUNT;i++) {
        if (!ready[i]) { ok=false; continue; }
        VL53L0X_RangingMeasurementData_t m;
        if (VL53L0X_GetRangingMeasurementData(&dev[i],&m) != VL53L0X_ERROR_NONE) { ok=false; continue; }
        sensor_data.distance_mm[i]=m.RangeMilliMeter;
        (void)VL53L0X_ClearInterruptMask(&dev[i],0x01);
    }
    sensor_data.wall_left = sensor_data.distance_mm[SENSOR_LEFT] <= VL53_WALL_THRESHOLD_MM;
    sensor_data.wall_front_left = sensor_data.distance_mm[SENSOR_FRONT_LEFT] <= VL53_FRONT_THRESHOLD_MM;
    sensor_data.wall_front_right = sensor_data.distance_mm[SENSOR_FRONT_RIGHT] <= VL53_FRONT_THRESHOLD_MM;
    sensor_data.wall_right = sensor_data.distance_mm[SENSOR_RIGHT] <= VL53_WALL_THRESHOLD_MM;
    sensor_data.wall_front = sensor_data.wall_front_left || sensor_data.wall_front_right;
    return ok;
}

const SensorData *sensor_get_data(void){return &sensor_data;}
bool sensor_front_wall(void){return sensor_data.wall_front;}
bool sensor_left_wall(void){return sensor_data.wall_left;}
bool sensor_right_wall(void){return sensor_data.wall_right;}

uint8_t sensor_to_absolute_walls(Direction facing)
{
    uint8_t w=0;
    if (sensor_data.wall_front) {
        if(facing==DIR_NORTH)w|=WALL_NORTH; else if(facing==DIR_EAST)w|=WALL_EAST;
        else if(facing==DIR_SOUTH)w|=WALL_SOUTH; else w|=WALL_WEST;
    }
    if (sensor_data.wall_left) {
        if(facing==DIR_NORTH)w|=WALL_WEST; else if(facing==DIR_EAST)w|=WALL_NORTH;
        else if(facing==DIR_SOUTH)w|=WALL_EAST; else w|=WALL_SOUTH;
    }
    if (sensor_data.wall_right) {
        if(facing==DIR_NORTH)w|=WALL_EAST; else if(facing==DIR_EAST)w|=WALL_SOUTH;
        else if(facing==DIR_SOUTH)w|=WALL_WEST; else w|=WALL_NORTH;
    }
    return w;
}
