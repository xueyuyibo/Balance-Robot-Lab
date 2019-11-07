/*******************************************************************************
* measure_moments.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the moments of inertia of your Balancebot
* 
* TODO: capture the gyro data and timestamps to a file to determine the period.
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
#include <rc/mpu.h>

FILE* f1;

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

static int running = 1;
static rc_mpu_data_t data;

static void __print_data(void);
static void __print_header(void);

float time_begin, time_stop;

static int64_t utime_now(void){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

static void __print_data(void)
{
    printf("\r");
    printf(" ");
    printf(" %5.2f %5.2f %5.2f |",  data.accel[0],\
                                                data.accel[1],\
                                                data.accel[2]);
    printf(" %5.1f %5.1f %5.1f |",  data.gyro[0],\
                                                data.gyro[1],\
                                                data.gyro[2]);
    printf("%6.1f %6.1f %6.1f |",   data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
                                            data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
                                            data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
    // time_stop = utime_now();
    // printf("%15.3f", (time_stop-time_begin)/1E3);
    fflush(stdout);
    return;
}

static void __print_header(void)
{
    printf(" ");
    printf(" Accel XYZ (m/s^2) |");
    printf("  Gyro XYZ (deg/s) |");
    printf(" DMP TaitBryan (deg) |");
    // printf("       Time Now(ms)  ");
    printf("\n");
}

static void __signal_handler(__attribute__ ((unused)) int dummy)
{
    running=0;
    return;
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

    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = I2C_BUS;
    conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
    conf.dmp_fetch_accel_gyro = 1;
    conf.orient = ORIENTATION_Z_DOWN;

    if(rc_mpu_initialize_dmp(&data, conf)){
        printf("rc_mpu_initialize_failed\n");
        return -1;
    }

    __print_header();
    time_begin = utime_now();
    rc_mpu_set_dmp_callback(&__print_data);
    while(running)  rc_usleep(100000);

    rc_mpu_power_off();
    printf("\n");
    fflush(stdout);

    // exit cleanly
    rc_encoder_eqep_cleanup();
    rc_remove_pid_file();   // remove pid file LAST
    return 0;

 //    rc_set_state(RUNNING);
 //    while(rc_get_state()!=EXITING){
 //    	rc_nanosleep(1E9);
 //    }

	// // exit cleanly
	// rc_encoder_eqep_cleanup();
	// rc_remove_pid_file();   // remove pid file LAST
	// return 0;
}