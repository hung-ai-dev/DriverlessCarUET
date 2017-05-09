#include "api_i2c_pwm.h"
#include "api_uart.h"
#include <iostream>

using namespace std;

FILE *logUart;
int cport_nr;
char buf_send[BUFF_SIZE], buf_recv[BUFF_SIZE];
int logId = 0;

void loadKey(char ch) {
    int toSend = (ch - '0') * 5;
    sprintf(buf_send, "f%d\n", toSend);
	api_uart_write(cport_nr, buf_send);
	fprintf(stderr, "sent %d\n", toSend);
	fprintf(logUart, "Event write: %d\n", toSend);
}

int main(int argc, char **argv) {
    logUart = fopen("speedlog.txt", "w");
    PCA9685 *pca9685 = new PCA9685() ;
    api_pwm_pca9685_init( pca9685 );

    int dir = 0, throttle_val = 32;
    double theta = 0;
    int current_state = 0;
    char key = 0;
    dir = DIR_REVERSE;
    theta = 0;
    if (pca9685->error >= 0)
        api_pwm_set_control( pca9685, dir, throttle_val, theta, current_state );
    cport_nr = api_uart_open();

    if( cport_nr == -1 ) {
        cerr<< "Error: Canot Open ComPort";
        return -1;
    }
    fprintf(stderr, "port %d\n", cport_nr);
    while (true) {
        key = getkey();
        if (key != 's') continue;
        fprintf(stderr, "start\n");
        usleep(200);
        for(char ch = '4'; ch <= '9'; ++ch) {
            loadKey(ch);
            usleep(500 * 1000);
        }
        theta = -150;
        throttle_val = 45;
        api_pwm_set_control( pca9685, dir, throttle_val, theta, current_state );
        usleep(500 * 1000);
        for(char ch = '8'; ch >= '4'; --ch) {
            loadKey(ch);
            usleep(500 * 1000);
        }
        loadKey('0');
        theta = 0; throttle_val = 0;
        api_pwm_set_control( pca9685, dir, throttle_val, theta, current_state );
        break;
    }
    return 0;
}
