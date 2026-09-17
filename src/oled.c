#include "oled.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define OLED_ADDR (0x3C << 1)
#define OLED_W 128
#define OLED_H 64

I2C_HandleTypeDef hi2c1;
static uint8_t fb[OLED_W * OLED_H / 8];

static const uint8_t font5x7[26][5] = {
{0x7E,0x09,0x09,0x09,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
{0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
{0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
{0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
{0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},{0x63,0x14,0x08,0x14,0x63},
{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
};

static void cmd(uint8_t c) { uint8_t b[2]={0x00,c}; HAL_I2C_Master_Transmit(&hi2c1,OLED_ADDR,b,2,100); }
static void data(const uint8_t *p,uint16_t n){ uint8_t b[17]; b[0]=0x40; while(n){uint16_t k=n>16?16:n; memcpy(&b[1],p,k); HAL_I2C_Master_Transmit(&hi2c1,OLED_ADDR,b,k+1,100); p+=k; n-=k;} }
static void pixel(int x,int y){if(x<0||x>=OLED_W||y<0||y>=OLED_H)return; fb[x+(y/8)*OLED_W]|=(uint8_t)(1u<<(y&7));}
static void text(const char *s,int x,int y,int scale){while(*s){char c=*s++;if(c==' '){x+=6*scale;continue;}if((c>='A'&&c<='Z')||(c>='a'&&c<='z')||c=='*'){char cc=c;if(cc>='a'&&cc<='z')cc=(char)(cc-'a'+'A');for(int col=0;col<5;col++)for(int row=0;row<7;row++){bool on;if(cc=='*')on=(col==2||(row==2&&(col==0||col==4))||(row==0&&col==2));else on=(font5x7[cc-'A'][col]&(1u<<row));if(on)for(int dx=0;dx<scale;dx++)for(int dy=0;dy<scale;dy++)pixel(x+col*scale+dx,y+row*scale+dy);}x+=6*scale;}}}
static void refresh(void){for(uint8_t page=0;page<8;page++){cmd((uint8_t)(0xB0|page));cmd(0x00);cmd(0x10);data(&fb[page*128],128);}}

void oled_init(void){
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_I2C1_CLK_ENABLE();
    GPIO_InitTypeDef g={0}; g.Pin=GPIO_PIN_8|GPIO_PIN_9; g.Mode=GPIO_MODE_AF_OD; g.Pull=GPIO_PULLUP; g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF4_I2C1; HAL_GPIO_Init(GPIOB,&g);
    hi2c1.Instance=I2C1; hi2c1.Init.ClockSpeed=400000; hi2c1.Init.DutyCycle=I2C_DUTYCYCLE_2; hi2c1.Init.OwnAddress1=0; hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT; hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE; hi2c1.Init.OwnAddress2=0; hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE; hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE; HAL_I2C_Init(&hi2c1);
    HAL_Delay(20); uint8_t init[]={0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0x8D,0x14,0x20,0x00,0xA1,0xC8,0xDA,0x12,0x81,0x7F,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF}; for(unsigned i=0;i<sizeof(init);i++)cmd(init[i]); oled_clear();
}
void oled_clear(void){memset(fb,0,sizeof(fb));refresh();}
void oled_show_algorithm(const char *name,bool running){memset(fb,0,sizeof(fb));text("ALGORITHM",20,4,2);text(name,20,25,2);text(running?"RUNNING":"READY",28,48,1);refresh();}
void oled_show_message(const char *line1,const char *line2){memset(fb,0,sizeof(fb));text(line1,10,18,2);text(line2,10,40,2);refresh();}
