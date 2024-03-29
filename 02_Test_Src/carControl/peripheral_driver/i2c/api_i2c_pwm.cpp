
#include "api_i2c_pwm.h"


int
getkey()
{
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}


int
map (double x, int in_min, int in_max, int out_min, int out_max)
{
    /*double toReturn =  1.0 * (x - in_min) * (out_max - out_min) /
            (in_max - in_min) + out_min ;
    return (int)(round(toReturn));*/
    return STEERING_NEUTRAL;
}


void
api_pwm_pca9685_init( PCA9685 *pca9685)
{
    // Initialize the PWM board
    int err = pca9685->openPCA9685();
    if (err < 0)
    {
        cout<< endl<< "Error: %d"<< pca9685->error<< flush;
    }

    cout<< endl<< "PCA9685 Device Address: 0x"<< hex
        << pca9685->kI2CAddress<< dec<< endl;

    pca9685->setAllPWM(0,0) ;
    pca9685->reset() ;
    pca9685->setPWMFrequency( PWM_FREQ ) ;
    // Set the PWM to "neutral" (1.5ms)
    sleep(1) ;
    int pwm2 = map( 0, MIN_ANGLE, MAX_ANGLE, STEERING_MAX_RIGHT, STEERING_MAX_LEFT );
    pca9685->setPWM(STEERING_CHANNEL2, 0, pwm2);

}

void
api_pwm_pca9685_release( PCA9685 *pca9685 )
{

    double theta = 0;

    if (pca9685->error >= 0)
        api_pwm_set_theta( pca9685, theta);

    pca9685->closePCA9685();
}


int
api_pwm_set_theta( PCA9685 *pca9685,
            double &theta)
{
    if( theta < MIN_ANGLE)
        theta = MIN_ANGLE;

    if( theta > MAX_ANGLE )
        theta = MAX_ANGLE;
        
    int pwm2 = map( theta, MIN_ANGLE, MAX_ANGLE, STEERING_MAX_RIGHT, STEERING_MAX_LEFT ); 
    pca9685->setPWM(STEERING_CHANNEL2,0, pwm2);
 
    return pwm2;
}

