
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "rs232.h"

#define BUFF_SIZE  20

int main()
{
    int cport_nr = 24;     /* /dev/ttyACM0 */
    int bdrate=9600;       /* 9600 baud */
    int n = 0;

    char mode[]={'8','N','1',0};

    char buf_send[BUFF_SIZE];

    char buf_recv[BUFF_SIZE];

    strcpy(buf_send, "djshfkjsdhfkdsjf\n");

    if(RS232_OpenComport(cport_nr, bdrate, mode))
    {
        printf("Can not open comport\n");

        return(0);
    }

    while(1)
    {
        RS232_cputs(cport_nr, buf_send);

        printf("sent: %s\n", buf_send);

        usleep(1000000);  /* sleep for 1 Second */

        n = RS232_PollComport(cport_nr, buf_recv, BUFF_SIZE-1);

        if(n > 0)
        {
            buf_recv[n] = 0;   /* always put a "null" at the end of a string! */

            printf("received %i bytes: %s\n", n, buf_recv);
        }
    }

    return(0);
}


