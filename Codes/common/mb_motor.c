/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motor drivers
*
*******************************************************************************/
#include <stdio.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"

// preposessor macros
#define unlikely(x) __builtin_expect (!!(x), 0)
#define HIGH 0
#define LOW 1

// global initialized flag
static int init_flag = 1;

/*******************************************************************************
* int mb_motor_init()
* 
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init(){
    
    return mb_motor_init_freq(DEFAULT_PWM_FREQ);
}

/*******************************************************************************
* int mb_motor_init_freq()
* 
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz){
    
    rc_gpio_init(MDIR1_CHIP,MDIR1_PIN,GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(MDIR2_CHIP,MDIR2_PIN,GPIOHANDLE_REQUEST_OUTPUT);
    rc_pwm_init(1,pwm_freq_hz);

    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying motor_init before motors have been initialized\n");
        return -1;
    }

    return 0;
}

/*******************************************************************************
* mb_motor_cleanup()
* 
*******************************************************************************/
int mb_motor_cleanup(){
    
    mb_motor_disable();
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying cleanup before motors have been initialized\n");
        return -1;
    }

    return 0;
}

/*******************************************************************************
* mb_motor_brake()
* 
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }

   return 0;
}

/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_motor_disable(){
    
    rc_pwm_set_duty(1, 'A', 0.0);
    rc_pwm_set_duty(1, 'B', 0.0);
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }
    
    return 0;
}


/*******************************************************************************
* int mb_motor_set(int motor, double duty)
* 
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* uses the defines in mb_defs.h
* returns 0 on success
*******************************************************************************/
int mb_motor_set(int motor, double duty){
    
    switch(motor){
        case 1:
            if(duty<0){
                rc_gpio_set_value(MDIR1_CHIP,MDIR1_PIN, 0);
                rc_pwm_set_duty(1, 'A', -1*duty);
            }
            else {
                rc_gpio_set_value(MDIR1_CHIP,MDIR1_PIN, 1);
                rc_pwm_set_duty(1, 'A', duty);
            }
            break;
        case 2:
            if(duty<0){
                rc_gpio_set_value(MDIR2_CHIP,MDIR2_PIN, 1);
                rc_pwm_set_duty(1, 'B', -1*duty);
            }
            else {
                rc_gpio_set_value(MDIR2_CHIP,MDIR2_PIN, 0);
                rc_pwm_set_duty(1, 'B', duty);
            }
            break;
    }

    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    return 0;
}

/*******************************************************************************
* int mb_motor_set_all(double duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty){

    if (duty<0){
        rc_gpio_set_value(MDIR1_CHIP,MDIR1_PIN, 0);
        rc_pwm_set_duty(1, 'A', -1*duty);
        rc_gpio_set_value(MDIR2_CHIP,MDIR2_PIN, 1);
        rc_pwm_set_duty(1, 'B', -1*duty);
    }
    else{
        rc_gpio_set_value(MDIR1_CHIP,MDIR1_PIN, 1);
        rc_pwm_set_duty(1, 'A', duty);
        rc_gpio_set_value(MDIR2_CHIP,MDIR2_PIN, 0);
        rc_pwm_set_duty(1, 'B', duty);
    }

    if(unlikely(!init_flag)){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    
    return 0;
}


/*******************************************************************************
* int mb_motor_read_current(int motor)
* 
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor){
    //DRV8801 driver board CS pin puts out 500mV/A
    return rc_adc_read_volt(motor-1)*2;
    return 0.0;
}