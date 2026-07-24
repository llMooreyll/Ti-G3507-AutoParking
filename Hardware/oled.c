#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 

#include "board.h"
#include "stdio.h"

/*
 * OLED 调试记录：
 *
 * 这个 OLED 驱动刚接入时并不稳定。只在 SysConfig 中新增 OLED 引脚，
 * 然后直接调用 OLED_Init()，程序会直接卡死，表现为 LED 不闪、电机不转、
 * OLED 也不显示。
 *
 * 根因不在 OLED 的显示、绘图、刷新 API，而在 OLED 这一组 GPIO 的启动状态。
 * SysConfig 会生成分组 GPIO 初始化代码，可能一次性把多个 OLED 引脚配置为
 * 输出并驱动到指定电平。在当前小车硬件上，某些启动组合会导致系统异常。
 *
 * 实测现象如下：按照示例工程的 syscfg 配置，如果 OLED 两个及以上引脚
 * 全部配置为输出且置低，程序会直接崩溃；如果三个及以上引脚配置为输出且
 * 置高，程序也会崩溃。最终可运行的启动配置是：三个 OLED 引脚配置为输出
 * 且置高，一个 OLED 引脚配置为输入。程序启动并执行 SYSCFG_DL_init() 后，
 * 再调用 OLED_ConfigurePinsInitialState()，由软件重新把四个 OLED 引脚按
 * 受控顺序配置为输出并置为 OLED 空闲状态。这里每一步之间的 5ms 分步延时
 * 是关键，去掉这些延时后系统会再次卡死。
 *
 * OLED_Init() 中还必须在 RST 拉高后保留一段延时。参考示例在 OLED_RST_Set()
 * 后立刻发送第一条命令，这在单独 OLED 示例里可以运行，但在本项目里不稳定。
 * 保留 RST 拉低延时和 RST 释放后的延时后，初始化、显示和快速刷新均可正常运行。
 *
 * 如果后续 OLED 启动再次异常，优先检查生成的 ti_msp_dl_config.c 中 OLED 引脚
 * 是否出现了不期望的分组输出状态，然后再检查 RST 复位时序，不要一开始就怀疑
 * 显示或绘图函数。
 */

/*
 * OLED bring-up notes:
 *
 * This driver was not stable when the OLED pins were only added through
 * SysConfig and OLED_Init() was called directly. The car firmware could stop
 * before the LED/motor code ran.
 *
 * The root cause was the startup state of the OLED GPIO group, not the OLED
 * drawing APIs. SysConfig emits grouped GPIO initialization code that can drive
 * multiple output pins at once. On this board, some combinations of the OLED
 * pins being driven during startup made the rest of the system fail. To avoid
 * that transient state, main calls OLED_ConfigurePinsInitialState() after
 * SYSCFG_DL_init(). This function releases SCL/SDA first, then re-selects the
 * pins as GPIO outputs, then drives RST/DC/SCL/SDA high in a controlled order.
 * The 5 ms delay between these steps is essential; removing those staged
 * delays made the system hang again.
 *
 * OLED_Init() also needs a delay after RST is released high. The reference
 * driver sent the first command immediately after OLED_RST_Set(), which worked
 * in the standalone OLED example but was not reliable in this project. Keeping
 * a reset-low delay and a reset-release delay makes initialization stable.
 *
 * If OLED startup fails again, first check the generated ti_msp_dl_config.c for
 * unintended grouped output states on OLED pins, then check the reset timing
 * before changing the display drawing code.
 */

static void OLED_DrivePinHigh(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_setPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

static void OLED_ReleasePin(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_disableOutput(port, pin);
}

void OLED_ConfigurePinsInitialState(void)
{
    OLED_ReleasePin(OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN);
    OLED_ReleasePin(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN);
    delay_ms(5);

    DL_GPIO_initDigitalOutput(OLED_RST_PIN_RST_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_DC_PIN_DC_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SCL_PIN_SCL_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SDA_PIN_SDA_IOMUX);
    delay_ms(5);

    OLED_DrivePinHigh(OLED_RST_PORT, OLED_RST_PIN_RST_PIN);
    delay_ms(5);
    OLED_DrivePinHigh(OLED_DC_PORT, OLED_DC_PIN_DC_PIN);
    delay_ms(5);
    OLED_DrivePinHigh(OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN);
    delay_ms(5);
    OLED_DrivePinHigh(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN);
    delay_ms(5);
}

uint8_t OLED_GRAM[128][8];	 
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：刷新OLED屏幕
入口参数：无
返回  值：无
**************************************************************************/
void OLED_Refresh_Gram(void)
{
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //Set page address (0~7) //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //Set the display location - column low address //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //Set the display location - column height address //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : Dat: data/command to write, CMD: data/command flag 0, represents the command;1, represents data
Output  : none
函数功能：向OLED写入一个字节
入口参数：dat:要写入的数据/命令，cmd:数据/命令标志 0,表示命令;1,表示数据
返回  值：无
**************************************************************************/  
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{	
	uint8_t i;			  
	if(cmd)
	  OLED_RS_Set();
	else 
	  OLED_RS_Clr();		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_RS_Set();   	  
} 
/**************************************************************************
Function: Turn on the OLED display
Input   : none
Output  : none
函数功能：开启OLED显示 
入口参数：无
返回  值：无
**************************************************************************/  
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC command //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
/**************************************************************************
Function: Turn off the OLED display
Input   : none
Output  : none
函数功能：关闭OLED显示 
入口参数：无			  
返回  值：无
**************************************************************************/  
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC command //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}	
/**************************************************************************
Function: Screen clear function, clear the screen, the entire screen is black, and did not light up the same
Input   : none
Output  : none
函数功能：清屏函数,清完屏,整个屏幕是黑色的，和没点亮一样
入口参数：无		  
返回  值：无
**************************************************************************/  
void OLED_Clear(void)  
{  
	uint8_t i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram(); //Update the display //更新显示
}
/**************************************************************************
Function: Draw point
Input   : x,y: starting coordinate;T :1, fill,0, empty
Output  : none
函数功能：画点 
入口参数：x,y :起点坐标; t:1,填充,0,清空			  
返回  值：无
**************************************************************************/ 
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	uint8_t pos,bx,temp=0;
	if(x>127||y>63)return;//超出范围了.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}
/**************************************************************************
Function: Displays a character, including partial characters, at the specified position
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode :0, anti-white display,1, normal display
Output  : none
函数功能：在指定位置显示一个字符,包括部分字符
入口参数：x,y :起点坐标; len :数字的位数; size:字体大小; mode:0,反白显示,1,正常显示	   
返回  值：无
**************************************************************************/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode)
{      			    
	uint8_t temp,t,t1;
	uint8_t y0=y;
	chr=chr-' '; //Get the offset value //得到偏移后的值				   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=oled_asc2_1206[chr][t];  //Invoke 1206 font   //调用1206字体
		else temp=oled_asc2_1608[chr][t];		      //Invoke the 1608 font //调用1608字体 	                          
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
/**************************************************************************
Function: Find m to the NTH power
Input   : m: base number, n: power number
Output  : none
函数功能：求m的n次方的函数
入口参数：m：底数，n：次方数
返回  值：无
**************************************************************************/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}

/**************************************************************************
Function: Displays 2 numbers
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode: mode, 0, fill mode, 1, overlay mode;Num: value (0 ~ 4294967295);
Output  : none
函数功能：显示2个数字
入口参数：x,y :起点坐标; len :数字的位数; size:字体大小; mode:模式, 0,填充模式, 1,叠加模式; num:数值(0~4294967295);	 
返回  值：无
**************************************************************************/
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
/**************************************************************************
Function: Display string
Input   : x,y: starting coordinate;*p: starting address of the string
Output  : none
函数功能：显示字符串
入口参数：x,y :起点坐标; *p:字符串起始地址 
返回  值：无
**************************************************************************/
void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}	 
/**************************************************************************
Function: Initialize the OLED
Input   : none
Output  : none
函数功能：初始化OLED	
入口参数: 无 
返回  值：无
**************************************************************************/	    
void OLED_Init(void)
{ 	


	OLED_RST_Clr();
	delay_ms(120);

	OLED_RST_Set(); 
	delay_ms(120);
				  
	OLED_WR_Byte(0xAE,OLED_CMD); //Close display //关闭显示
	OLED_WR_Byte(0xD5,OLED_CMD); //The frequency frequency factor, the frequency of the shock //设置时钟分频因子,震荡频率
	OLED_WR_Byte(80,OLED_CMD);   //[3:0], the frequency dividing factor;[7:4], oscillation frequency //[3:0],分频因子;[7:4],震荡频率
	OLED_WR_Byte(0xA8,OLED_CMD); //Set the number of driver paths //设置驱动路数
	OLED_WR_Byte(0X3F,OLED_CMD); //Default 0x3f(1/64) //默认0X3F(1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //Setting display deviation //设置显示偏移
	OLED_WR_Byte(0X00,OLED_CMD); //Default is 0//默认为0

	OLED_WR_Byte(0x40,OLED_CMD); //Sets the number of rows to display starting line [5:0] //设置显示开始行 [5:0],行数
													
	OLED_WR_Byte(0x8D,OLED_CMD); //Charge pump setup //电荷泵设置
	OLED_WR_Byte(0x14,OLED_CMD); //Bit2, on/off //bit2，开启/关闭
	OLED_WR_Byte(0x20,OLED_CMD); //Set up the memory address mode //设置内存地址模式
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00, column address mode;01, line address mode;10. Page address mode;The default 10; //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
	OLED_WR_Byte(0xA1,OLED_CMD); //Segment redefine setting,bit0:0,0- >;0;1, 0 - & gt;127; //段重定义设置,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //Set the COM scan direction;Bit3:0, normal mode;1, Re-define schema COM[n-1]- >;COM0;N: Number of driving paths//设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
	OLED_WR_Byte(0xDA,OLED_CMD); //Set the COM hardware pin configuration //设置COM硬件引脚配置
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]configuration //[5:4]配置
	 
	OLED_WR_Byte(0x81,OLED_CMD); //Contrast Settings //对比度设置
	OLED_WR_Byte(0xEF,OLED_CMD); //1~ 255; Default 0x7f (brightness Settings, the bigger the brighter) //1~255;默认0X7F (亮度设置,越大越亮)
	OLED_WR_Byte(0xD9,OLED_CMD); //Set the pre-charging cycle //设置预充电周期
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //Setting vcomh voltage multiplier//设置VCOMH 电压倍率
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //Global display; Bit0:1, open; 0, close; (white screen/black screen)//全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
	OLED_WR_Byte(0xA6,OLED_CMD); //Settings display mode; Bit0:1, anti-phase display; 0, normal display//设置显示方式;bit0:1,反相显示;0,正常显示	    						   
	OLED_WR_Byte(0xAF,OLED_CMD); //Open display //开启显示	 
	OLED_Clear();
}  

/**************************************************************************
Function: Display character
Input   : x: indicates the horizontal coordinates displayed; Y: the vertical coordinates that show the display;
          no: the line number in the array of the Chinese character (module) in the hzk-and "array", which is determined by the line number to determine the characters shown in the array,
          The value of the width of the font here must be consistent with the size of the dot matrix value of the use of the word mold.
          font_height: the font is high for the use of the word mold, because my screen pixels are 32hours, 128----0~ 7, and four bits per page
Output  : none
Note: this method is used to show that the Chinese character must satisfy the size of the word that the word model generates the software to generate the same size as the dot matrix
函数功能：显示汉字	
入口参数: x：表示显示的水平坐标; y: 表示显示的垂直坐标;
          no: 表示要显示的汉字（模组）在hzk[][]数组中的行号,通过行号来确定在数组中要显示的汉字,
              这里字体的宽font_width的值必须与用字模制作软件生成字模时的点阵值大小一致;
          font_height:为用字模制作软件生成字模时字体的高,由于我的屏像素为32*128-----0~7共8页，每页4个位
返回  值：无
注意：用这种方法来显示汉字一定要满足用字模生成软件生成的字宽与点阵大小相同才行，否者容易乱码
**************************************************************************/	    
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t font_width,uint8_t font_height)
{     			    
	 uint8_t t, i;
   for(i=0;i<(font_height/8);i++)	//The maximum height of font_height is 32. this screen is only 8 pages (line), four digits per page
	                                //font_height最大值为32，这张屏只有8个页（行），每页4个位
	 {
			OLED_Set_Pos(x,y+i);	
			for(t=0;t<font_width;t++)		//The maximum value of font_width is 128. the screen is only that large 
		                              //font_width最大值为128，屏幕只有这么大
			{	
					OLED_WR_Byte(Hzk16[(font_height/8)*no+i][t],OLED_DATA);
			}		
	 }
}	 
/**************************************************************************
Function: Set the coordinates (position) displayed on the screen.
Input   : x, y: starting point coordinates
Output  : none
函数功能：设置汉字在屏幕上显示的坐标（位置）
入口参数: x,y :起点坐标
返回  值：无
**************************************************************************/	  
void OLED_Set_Pos(unsigned char x, unsigned char y)
{ 	
	 OLED_WR_Byte(0xb0+y,OLED_CMD);
	 OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	 OLED_WR_Byte((x&0x0f),OLED_CMD); 
} 


void OLED_RST_Clr(void)
{
DL_GPIO_clearPins(OLED_RST_PORT,OLED_RST_PIN_RST_PIN);
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");

} 	
		  //RST
void OLED_RST_Set(void)
{
 DL_GPIO_setPins(OLED_RST_PORT,OLED_RST_PIN_RST_PIN);
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}  

void OLED_RS_Clr(void)
{

DL_GPIO_clearPins(OLED_DC_PORT,OLED_DC_PIN_DC_PIN); ////DC
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_RS_Set(void) 
{
 DL_GPIO_setPins(OLED_DC_PORT,OLED_DC_PIN_DC_PIN) ; ////DC
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
} 	

void OLED_SCLK_Clr(void)
{

DL_GPIO_clearPins(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN); ////SCL
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_SCLK_Set(void)
{
DL_GPIO_setPins(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN); ////SCL
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_SDIN_Clr(void)
{
DL_GPIO_clearPins(OLED_SDA_PORT,OLED_SDA_PIN_SDA_PIN); // //SDA
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_SDIN_Set(void) 
{
DL_GPIO_setPins(OLED_SDA_PORT,OLED_SDA_PIN_SDA_PIN); //   //SDA
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
} 	
