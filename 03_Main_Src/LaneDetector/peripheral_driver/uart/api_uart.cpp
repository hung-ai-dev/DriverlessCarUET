
#include "api_uart.h"

// return the index of comport
int
api_uart_open()
{
    int cport_nr = 24;     /* /dev/ttyACM0 */
//    int cport_nr = 16;     /* /dev/ttyUSB0 */
    int bdrate=115200;       /* 115200 baud */

    char mode[]={'8','N','1',0};

    if(RS232_OpenComport(cport_nr, bdrate, mode))
    {
        printf("Can not open comport\n");

        return(-1);
    }
    return cport_nr;
}

int
api_uart_write( int cport_nr,  char* buf_send)
{
    RS232_cputs(cport_nr, buf_send);
    return 0;
}

int
api_uart_read(int cport_nr, char *buf_recv)
{
    return RS232_PollComport(cport_nr, buf_recv, BUFF_SIZE);
}
