#include "peripheral_driver/i2c/api_i2c_pwm.h"
#include <cstdio>
#include <cstdlib>
#include <sys/time.h>

struct ThetaController {
    const double delayingTime = 0.2;
    double lastSetTime;
    PCA9685 *pca9685;
    ThetaController() {}

    // Get time of day
    static double wallTime() {
        timeval tv;
        gettimeofday(&tv, NULL);
        return tv.tv_sec + tv.tv_usec * 1e-6;
    }

    void init() {
        pca9685 = new PCA9685();
        lastSetTime = wallTime();
    }

    // Move servo to theta with restrict of delayingTime
    int setTheta(double theta) {
        double curTime = wallTime();
        if (curTime - lastSetTime < delayingTime) return -1;
        lastSetTime = curTime;
        int pwm2 = api_pwm_set_theta(pca9685, theta);
        return pwm2;
    }

    void release() {
        api_pwm_pca9685_release(pca9685);
    }
};

int main(int argc, char **argv) {
    ThetaController controlTheta;
    double theta = 0;
    while (true) {
        if (controlTheta.pca9685->error < 0) break;
        char keypress = getkey();
        if (keypress == 'f') break;
        double theta = rand() % 41 - 20;
        controlTheta.setTheta(theta);
    }
    controlTheta.release();
    return 0;
}