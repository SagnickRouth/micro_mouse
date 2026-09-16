#include "sensor.h"
#include "stm32f4xx_hal.h"
#include "vl53l0x_api.h"
#include <string.h>

/* Add ST's VL53L0X API core + platform sources to the CubeIDE project. */
extern I2C_HandleTypeDef hi2c1;
SensorData sensor_data;

static const struct { GPIO_TypeDef *port; uint16_t pin; uint8_t addr7; } hw[SENSOR_COUNT] = {
    {GPIOA, VL53_LEFT_XSHUT_PIN,  VL53_ADDR_LEFT},
    {GPIOA, VL53_FL_XSHUT_PIN,    VL53_ADDR_FL},
    {GPIOA, VL53_FR_XSHUT_PIN,    VL53_ADDR_FR},
    {GPIOA, VL53_RIGHT_XSHUT_PIN, VL53_ADDR_RIGHT}
};
static VL53L0X_Dev_t dev[SENSOR_COUNT];
static bool ready[SENSOR_COUNT];

static void all_xshut(GPIO_PinState s){
    for(int i=0;i<SENSOR_COUNT;i++) HAL_GPIO_WritePin(hw[i].port,hw[i].pin,s);
}
static void xshut_init(void){
    __HAL_RCC_GPIOA_CLK_ENABLE(); GPIO_InitTypeDef g={0};
    g.Pin=VL53_LEFT_XSHUT_PIN|VL53_FL_XSHUT_PIN|VL53_FR_XSHUT_PIN|VL53_RIGHT_XSHUT_PIN;
    g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA,&g); all_xshut(GPIO_PIN_RESET);
}

bool sensor_assign_addresses(void){
    memset(ready,0,sizeof(ready)); all_xshut(GPIO_PIN_RESET); HAL_Delay(5);
    for(int i=0;i<SENSOR_COUNT;i++){
        HAL_GPIO_WritePin(hw[i].port,hw[i].pin,GPIO_PIN_SET); HAL_Delay(3);
        memset(&dev[i],0,sizeof(dev[i]));
        dev[i].I2cDevAddr = VL53_DEFAULT_ADDR << 1; /* ST API uses 8-bit address in Dev */
        /* Program the new 7-bit address through the ST API as 8-bit. */
        if(VL53L0X_SetDeviceAddress(&dev[i],(uint8_t)(hw[i].addr7<<1)) != VL53L0X_ERROR_NONE) return false;
        dev[i].I2cDevAddr = (uint8_t)(hw[i].addr7<<1);
        if(VL53L0X_DataInit(&dev[i]) != VL53L0X_ERROR_NONE) return false;
        ready[i]=true;
    }
    return true;
}

bool sensor_init(void){
    xshut_init();
    if(!sensor_assign_addresses()) return false;
    for(int i=0;i<SENSOR_COUNT;i++){
        uint8_t vhv=0, phase=0; uint32_t spads=0; uint8_t aperture=0;
        if(VL53L0X_StaticInit(&dev[i]) != VL53L0X_ERROR_NONE) return false;
        if(VL53L0X_PerformRefSpadManagement(&dev[i],&spads,&aperture) != VL53L0X_ERROR_NONE) return false;
        if(VL53L0X_PerformRefCalibration(&dev[i],&vhv,&phase) != VL53L0X_ERROR_NONE) return false;
        if(VL53L0X_SetDeviceMode(&dev[i],VL53L0X_DEVICEMODE_CONTINUOUS_RANGING) != VL53L0X_ERROR_NONE) return false;
        if(VL53L0X_StartMeasurement(&dev[i]) != VL53L0X_ERROR_NONE) return false;
        ready[i]=true;
    }
    memset(&sensor_data,0,sizeof(sensor_data));
    return true;
}

bool sensor_read_all(void){
    bool ok=true;
    for(int i=0;i<SENSOR_COUNT;i++){
        if(!ready[i]){ok=false;continue;}
        VL53L0X_RangingMeasurementData_t m;
        if(VL53L0X_GetRangingMeasurementData(&dev[i],&m)!=VL53L0X_ERROR_NONE){ok=false;continue;}
        sensor_data.distance_mm[i]=m.RangeMilliMeter;
        (void)VL53L0X_ClearInterruptMask(&dev[i],0x01);
    }
    sensor_data.wall_left=sensor_data.distance_mm[SENSOR_LEFT]<=VL53_WALL_THRESHOLD_MM;
    sensor_data.wall_front_left=sensor_data.distance_mm[SENSOR_FRONT_LEFT]<=VL53_FRONT_THRESHOLD_MM;
    sensor_data.wall_front_right=sensor_data.distance_mm[SENSOR_FRONT_RIGHT]<=VL53_FRONT_THRESHOLD_MM;
    sensor_data.wall_right=sensor_data.distance_mm[SENSOR_RIGHT]<=VL53_WALL_THRESHOLD_MM;
    sensor_data.wall_front=sensor_data.wall_front_left||sensor_data.wall_front_right;
    return ok;
}
const SensorData* sensor_get_data(void){return &sensor_data;}
bool sensor_front_wall(void){return sensor_data.wall_front;}
bool sensor_left_wall(void){return sensor_data.wall_left;}
bool sensor_right_wall(void){return sensor_data.wall_right;}

uint8_t sensor_to_absolute_walls(Direction f){
    uint8_t w=0;
    if(sensor_data.wall_front) w|=(f==DIR_NORTH?WALL_NORTH:f==DIR_EAST?WALL_EAST:f==DIR_SOUTH?WALL_SOUTH:WALL_WEST);
    if(sensor_data.wall_left)  w|=(f==DIR_NORTH?WALL_WEST:f==DIR_EAST?WALL_NORTH:f==DIR_SOUTH?WALL_EAST:WALL_SOUTH);
    if(sensor_data.wall_right) w|=(f==DIR_NORTH?WALL_EAST:f==DIR_EAST?WALL_SOUTH:f==DIR_SOUTH?WALL_WEST:WALL_NORTH);
    return w;
}
