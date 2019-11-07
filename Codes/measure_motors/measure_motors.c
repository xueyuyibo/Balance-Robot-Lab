/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
* TODO: Capture encoder readings, current readings, timestamps etc. to a file
*       to analyze and determine motor parameters
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

FILE* f1;

#define V 12.2
#define R_L 6.3
#define R_R 5.8

static int running = 1;
static time_begin,time_stop;
static flag = 0;
int64_t utime_now(void){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	// if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
 //        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
 //        return -1;
 //    }

    if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

    printf("\nRaw encoder positions\n");
    printf("      E_LEFT   |");
    printf("      E_RIGHT   |");
    printf("   current_L(i) |");
    printf("   current_R(i) |");
    printf("   omega_shaff_l |");
    printf("   omega_shaff_r |");
    printf("       Motor Constant_L(K)| ");
    printf("       Motor Constant_R(K)|  ");
    printf("       torque_l(t)|  ");
    printf("       torque_r(t)  ");
    printf(" \n");
    rc_set_state(RUNNING);
    mb_motor_set_all(1.0);
    // while(rc_get_state()!=EXITING){
    // }

    int count = 0;
    int initial_enc_l,initial_enc_r,current_enc_l,current_enc_r,interval_enc_l,interval_enc_r;
    float initial_i_l,initial_i_r,current_i_l,current_i_r,omega_shaff_l,omega_shaff_r,mean_i_l,mean_i_r,K_L,K_R,torque_l,torque_r;
    rc_nanosleep(4E9);
    while(count<300){
        initial_enc_l = rc_encoder_eqep_read(LEFT_MOTOR);
        initial_enc_r = rc_encoder_eqep_read(RIGHT_MOTOR);
        initial_i_l = mb_motor_read_current(LEFT_MOTOR);
        initial_i_r = mb_motor_read_current(LEFT_MOTOR);
        rc_nanosleep(1E8);
        current_enc_l = rc_encoder_eqep_read(LEFT_MOTOR);
        current_enc_r = rc_encoder_eqep_read(RIGHT_MOTOR);
        current_i_l = mb_motor_read_current(LEFT_MOTOR);
        current_i_r = mb_motor_read_current(LEFT_MOTOR);

        interval_enc_l = current_enc_l - initial_enc_l;
        interval_enc_r = current_enc_r - initial_enc_r;

        omega_shaff_l = (interval_enc_l*2*PI)/(0.1*ENCODER_RES);
        omega_shaff_r = (interval_enc_r*2*PI)/(0.1*ENCODER_RES);
        mean_i_l = (current_i_l+initial_i_l)/2;
        mean_i_r = (current_i_l+initial_i_r)/2;

        K_L = (V - R_L*mean_i_l)/omega_shaff_l;
        K_R = (V - R_R*mean_i_r)/omega_shaff_r;
        torque_l = K_L*mean_i_l;
        torque_r = K_R*mean_i_r;

        printf("%6d |", interval_enc_l);
        printf("%6d |", interval_enc_r);
        printf("%6.8f |", mean_i_l);
        printf("%6.8f |", mean_i_r);
        printf("%6.8f |", omega_shaff_l);
        printf("%6.8f |", omega_shaff_r);
        printf("%6.4f |", K_L);
        printf("%6.4f |", K_R);
        printf("%6.4f |", torque_l);
        printf("%6.4f ", torque_r);
        // printf("%6.2f |", rc_adc_dc_jack());
        printf("\n");
        count += 1;
        fflush(stdout);
    }
    mb_motor_disable();
    mb_motor_cleanup();


    // rc_set_state(RUNNING);
    // while(rc_get_state()!=EXITING){
    // 	rc_nanosleep(1E9);
    // }

	// exit cleanly
    rc_adc_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}