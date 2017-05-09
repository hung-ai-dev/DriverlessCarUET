#include "peripheral_driver/i2c/api_i2c_pwm.h"
#include <cstdio>
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
    while(controlTheta.pca9685->error >= 0) {
        int n = scanf("%lf", &theta);
        if (n == 1) {
            if (theta <= -30 | theta >= 30) break;
            fprintf(stderr, "theta = %lf\n", theta);
            int pwm2 = controlTheta.setTheta(theta);
            fprintf(stderr, "pwm2 = %d\n", pwm2);
        }
    }
    controlTheta.release();
    return 0;
}