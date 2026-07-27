#include "bsp_printf.h"

//任意串口打印
static char Ux_TxBuff[256];
void any_printf(UART_Regs *uart, const char *format, ...)
{
    int length;
    int i = 0;
    va_list listdata;

    va_start(listdata, format);
    length = vsnprintf(Ux_TxBuff, sizeof(Ux_TxBuff), format, listdata);
    va_end(listdata);

    if (length < 0)
    {
        return;
    }

    if (length >= (int)sizeof(Ux_TxBuff))
    {
        length = (int)sizeof(Ux_TxBuff) - 1;
    }

    while (i < length)
    {
        while (DL_UART_isBusy(uart) == true)
        {
        }
        DL_UART_transmitDataBlocking(uart, Ux_TxBuff[i++]);
    }
}

/*
 * fputc(), fputs(), and puts() are provided by board.c in this project.
 * Do not redefine them here, or the linker will report duplicate symbols.
 */
