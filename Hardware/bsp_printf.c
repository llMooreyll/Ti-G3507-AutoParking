#include "bsp_printf.h"

//任意串口打印
char Ux_TxBuff[256];
void any_printf(UART_Regs *uart,char *format,...)
{	
	uint8_t i=0;      
	va_list listdata;                                
	va_start(listdata,format);                 
	vsprintf((char *)Ux_TxBuff,format,listdata); 
	va_end(listdata);          

    while( Ux_TxBuff[i]!='\0' )      
    {
		while( DL_UART_isBusy(uart) == true ){}
        DL_UART_transmitDataBlocking(uart,Ux_TxBuff[i++]);
    }  

}

/*
 * fputc(), fputs(), and puts() are provided by board.c in this project.
 * Do not redefine them here, or the linker will report duplicate symbols.
 */
