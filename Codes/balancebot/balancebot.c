/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/
#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/math/filter.h>
#include "../common/mb_structs.h"

#include "balancebot.h"

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

int controller = 6; 
int odometry_mode = 1; //1=motion model, 2=velocity model
//inner loop pid
double D1_KP = -3.0; // -3.0
double D1_KI = -14.0; // -14.0
double D1_KD = -0.05; // -0.05
double wc1 = 1.8/0.05;
double D1_u= 0.0;
//outer loop pid
double D2_KP = 0.01; //0.005
double D2_KI = 0.0001; //0.0003
double D2_KD = 0.005; //0.003
double wc2 = 1.8/0.35; //1.8/0.35
double D2_u= 0.0;
//0310@1pm 0.0005, 0.00, 0.00015 
//0310@10pm 0.005, 0.0005, 0.0015
//0315 0.005/0.0003/0.003

//steering
double D3_KP = 0.01;
double D3_KI = 0.0003;
double D3_KD = 0.003;
double wc3 = 0.025;//0.025;
double D3_u= 0.0;

double D4_KP = 0.01;
double D4_KI = 0.0;
double D4_KD = 0.005;
double wc4 = 0.025;//1.8/0.35;
double D4_u= 0.0;

//lqr
double K1 = 5.5238;
double K2 = 0.7421;
double K3 = 0.3667;
double K4 = 0.1756;

rc_filter_t D1 = RC_FILTER_INITIALIZER;
rc_filter_t D2 = RC_FILTER_INITIALIZER;
rc_filter_t D3 = RC_FILTER_INITIALIZER;
rc_filter_t D4 = RC_FILTER_INITIALIZER;
rc_filter_t lowPassFilter1 = RC_FILTER_INITIALIZER;
rc_filter_t lowPassFilter2 = RC_FILTER_INITIALIZER;

float curr_time;
float prev_time;
float left_phi;
float right_phi;

float speed;

float setpoint_phi;
float setpoint_gamma;
float setpoint_dist;
float setpoint_psi;

float position_dist = 0.0;
float position_prev = 0.0;
float position_now = 0.0;
float position_angle = 0.0;

double phi_error;
double theta_error;
double gamma_error;
double dist_error=0.25;
double phi_dot_error;
double theta_dot_error;

int flag1 = 0;
int flag2 = 0;
int flag3 = 1;
int flag4 = 1;
float flag5 = 0.0; 

int plan = 0;
int plan_change = 0;

float stop = 10.0;
float stop2 = 10.0;

int num_loop = 500;

float dist_goal = 1.0;
int count;

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
    mb_state.theta_ref = 0.0;
	mb_state.phi_ref = 0.0;
	mb_state.gamma_ref = 0.0;
	mb_state.phi_prev = 0.0;
	mb_state.gamma_prev = 0.0;
	mb_state.dist_prev = 0.0;
	// mb_state.dist_ref = 0.305; 
	mb_state.phi_dot = 0.0;
	mb_state.dist = 0.0;
	count = (int)(dist_goal/0.5);
	speed = 0.15;

	if (controller == 5) {
		D1_KP = -3.0; // -3.0
		D1_KI = -14.0; // -14.0
		D1_KD = -0.05; // -0.05
		wc1 = 1.8/0.05;

		D2_KP = 0.005; //0.0005
		D2_KI = 0.0003; //0.00 //0.00005
		D2_KD = 0.003; //0.00015
		wc2 = 1.8/0.35;

		D3_KP = 0.01;
		D3_KI = 0.0003;
		D3_KD = 0.003;
		wc3 = 0.025;//1.8/0.35;
	}

	curr_time = rc_nanos_since_epoch();

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

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	printf("initializing xbee... \n");
	// initialize XBee Radio
	// int baudrate = BAUDRATE;
	if(XBEE_init(BAUDRATE)==-1){
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	if(rc_encoder_init()==-1){
		fprintf(stderr,"failed to initialize encoders\n");
		return -1;
	}
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;

	mpu_config.i2c_bus = I2C_BUS;
	mpu_config.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    mpu_config.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
	mpu_config.dmp_fetch_accel_gyro = 1;

	// int baudRate = BAUDRATE;
	// XBEE_init(BAUDRATE);

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init();

	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	rc_filter_pid(&D1,D1_KP,D1_KI,D1_KD,1/wc1,DT);
	rc_filter_enable_saturation(&D1,-1.0,1.0);
	
	rc_filter_pid(&D2,D2_KP,D2_KI,D2_KD,1/wc2,DT);
	rc_filter_enable_saturation(&D2,-10.0,10.0);

	rc_filter_pid(&D3,D3_KP,D3_KI,D3_KD,1/wc3,DT);
	rc_filter_enable_saturation(&D3,-1.0,1.0);

	rc_filter_pid(&D4,D4_KP,D4_KI,D4_KD,1/wc4,DT);
	rc_filter_enable_saturation(&D4,-0.15,0.15);

	

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_dsm_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller(){

	// printf("%d\n",utime_now());
	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	//start time
	curr_time = rc_nanos_since_epoch();
	// Read IMU
	mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];
	mb_state.gamma = mpu_data.dmp_TaitBryan[TB_YAW_Z];
	// Read encoders
	mb_state.left_encoder = rc_encoder_eqep_read(1);
	mb_state.right_encoder = rc_encoder_eqep_read(2);
	// Read theta dot
	mb_state.theta_dot = mpu_data.gyro[0]*DEG_TO_RAD;
	// Update phi 
	left_phi = mb_state.left_encoder*(1/ENCODER_RES)*ENC_1_POL*2*PI;
 	right_phi = mb_state.right_encoder*(1/ENCODER_RES)*ENC_2_POL*2*PI;
 	mb_state.phi = (left_phi + right_phi)/2.0;
	mb_state.phi_dot = (mb_state.phi - mb_state.phi_prev)*1E9/(curr_time-prev_time);
    // Update odometry 
	mb_odometry_update(&mb_odometry, &mb_state, odometry_mode);

	//lqr
	////////////////////////////////////////////////
	mb_state.theta_LQR = mpu_data.dmp_TaitBryan[TB_PITCH_X];
	mb_state.theta_dot_LQR = mpu_data.gyro[0]*DEG_TO_RAD;
	float l = mb_state.left_encoder*ENC_1_POL*2*PI/(ENCODER_RES*20.4);
	float r = mb_state.right_encoder*ENC_2_POL*2*PI/(ENCODER_RES*20.4);
	mb_state.phi_LQR = (l+r)/2 + mb_state.theta_LQR;
	// mb_state.phi_LQR = (l+r)/2;
	// mb_state.phi_dot_LQR = (mb_state.phi - mb_state.phi_prev)/DT;
	mb_state.phi_dot_LQR = (mb_state.phi_LQR - mb_state.phi_prev_LQR)/DT;
	////////////////////////////////////////////////

    // Calculate controller outputs
    if (controller == 1) {
		phi_error = mb_state.phi_ref + (mb_state.phi_ref!=0)*mb_state.phi_prev + setpoint_phi - mb_state.phi;
		D2_u = rc_filter_march(&D2, phi_error);//DSM_FB + prev_phi
		theta_error = -D2_u + mb_state.theta-0.03;//+fwd_volocity //offset = -0.018
		D1_u = rc_filter_march(&D1, theta_error);
		gamma_error = mb_state.gamma_ref + (mb_state.gamma_ref!=0)*mb_state.gamma_prev + setpoint_gamma - mb_state.gamma;//DSM_RL*45 + prev_gamma
		D3_u = rc_filter_march(&D3, gamma_error);
		mb_state.left_cmd = D1_u - D3_u;
		mb_state.right_cmd = D1_u + D3_u;
	} else if (controller == 2) {
		if (dist_goal>0 && count==(int)(dist_goal/0.5)) {
			mb_state.dist_ref = 0.5;
			count -= 1;
		} else if (count > 0){
			if (mb_state.dist_ref - mb_state.dist <= 0.15) {
				mb_state.dist_ref += 0.5;
				count -= 1;
			}
		} else {
			mb_state.dist_ref = dist_goal;
		}
			dist_error = mb_state.dist_ref - mb_state.dist;
			D2_u = rc_filter_march(&D2, dist_error*40);
			theta_error = -D2_u + mb_state.theta - 0.03;
			D1_u = rc_filter_march(&D1, theta_error);
			gamma_error = mb_odometry.psi_ref + mb_odometry.psi;
			// gamma_error = mb_state.gamma_ref - mb_state.gamma;
			D3_u = rc_filter_march(&D3, gamma_error*5);
			mb_state.left_cmd = D1_u - D3_u;
			mb_state.right_cmd = D1_u + D3_u;
	} else if (controller == 3) {
		dist_error = mb_state.dist_ref + (mb_state.dist_ref!=0)*mb_state.dist_prev + setpoint_dist - mb_state.dist;
		// dist_error = 0 - mb_state.dist;
		D2_u = rc_filter_march(&D2, dist_error*50);
		theta_error = -D2_u + mb_state.theta - 0.03;
		D1_u = rc_filter_march(&D1, theta_error);
		gamma_error = mb_odometry.psi_ref + (mb_odometry.psi_ref!=0)*mb_odometry.psi_prev - setpoint_psi + mb_odometry.psi;
		// gamma_error = mb_state.gamma_ref - mb_state.gamma;
		// if (mb_odometry.psi_ref!=0) {
		// 	flag5 = 1.0;
		// }
		// gamma_error = flag5*1.0 + mb_odometry.psi;
		D3_u = rc_filter_march(&D3, gamma_error*10);
		mb_state.left_cmd = D1_u - D3_u;
		mb_state.right_cmd = D1_u + D3_u;
		// printf("%f\t\t%f\t\t%f\n",mb_odometry.psi,flag5,gamma_error);
	} else if (controller == 4) {
		D4_u = rc_filter_march(&D4, mb_state.phi_dot-(rc_dsm_ch_normalized(3)>0.02)*speed+(rc_dsm_ch_normalized(3)<-0.02)*speed);
		if (rc_dsm_ch_normalized(3)<0.02 && rc_dsm_ch_normalized(3)>-0.02) {
			D4_u = 0;
		}
		dist_error = D4_u + (D4_u!=0)*mb_state.dist_prev + setpoint_dist - mb_state.dist;
		D2_u = rc_filter_march(&D2, dist_error*60);
		theta_error = -D2_u + mb_state.theta - 0.03;
		D1_u = rc_filter_march(&D1, theta_error);
		gamma_error = mb_odometry.psi_ref + (mb_odometry.psi_ref!=0)*mb_odometry.psi_prev - setpoint_psi + mb_odometry.psi;
		// gamma_error = mb_state.gamma_ref - mb_state.gamma;
		D3_u = rc_filter_march(&D3, gamma_error*5);
		mb_state.left_cmd = D1_u - D3_u;
		mb_state.right_cmd = D1_u + D3_u;
	} else if (controller == 5) {
		if (mb_state.dist>=10.4) {
			mb_state.dist_ref = 10.4;
			mb_state.dist_prev = 0;
			flag3 = 1;
			// printf("\r");
			printf("10.4 meters now!!\n");
		} else if (mb_state.dist>=9.5) {
			mb_state.dist_ref = 0.0;
			// printf("\r");
			printf("9.5 meters now!!\n");
		} else if (mb_state.dist>=9) {
			mb_state.dist_ref = 0.01;
			// printf("\r");
			printf("9 meters now!!\n");
		} else if (mb_state.dist>=8.5) {
			mb_state.dist_ref = 0.03;
			// printf("\r");
			printf("8.5 meters now!!\n");
		} else if (mb_state.dist>=6.5) {
			mb_state.dist_ref = 0.05;
			// printf("\r");
			printf("6.5 meters now!!\n");
		} else if (mb_state.dist>=5) {
			mb_state.dist_ref = 0.15;
			// printf("\r");
			printf("5 meters now!!\n");
		} else if (mb_state.dist>=3) {
			mb_state.dist_ref = 0.25;
			// printf("\r");
			printf("3 meters now!!\n");
		} else if (mb_state.dist<=0.5) {
			mb_state.dist_ref = 0.9;
		}
		else {
			mb_state.dist_ref = 0.45;
		}
		if (mb_state.dist>0.9 && mb_state.dist<1.1) {
			printf("\r");
			printf("1 meter now!!");
		}
		if (flag3) {
			mb_state.dist_ref = 10.4;
			mb_state.dist_prev = 0;
			stop = 50;
		}
		// printf("%f\n",mb_state.dist);
		dist_error = mb_state.dist_ref + mb_state.dist_prev - mb_state.dist;
		D2_u = rc_filter_march(&D2, dist_error*stop);
		theta_error = -D2_u + mb_state.theta - 0.03;
		D1_u = rc_filter_march(&D1, theta_error);
		gamma_error = mb_odometry.psi;
		// gamma_error = - mb_state.gamma;
		D3_u = rc_filter_march(&D3, gamma_error*7);
		mb_state.left_cmd = D1_u - D3_u;
		mb_state.right_cmd = D1_u + D3_u;
	} else if (controller == 6) {
		if ((mb_state.dist-position_dist>=1)&&(flag3==0)) {
			position_dist += 1;
			flag3 = 1;
			position_now = 1;
			// printf("1 meters now!!-----%f-----%d-----%d*****%f-----%d\n",position_dist,flag3,num_loop,dist_error,stop);
		} else if ((mb_state.dist-position_dist>=0.95)&&(flag3==0)) {
			mb_state.dist_ref = 0.0;
			position_now = 0.95;
			// printf("0.95 meters now!!-----%f-----%d-----%d*****%f-----%d\n",position_dist,flag3,num_loop,dist_error,stop);
		} else if ((mb_state.dist-position_dist>=0.85)&&(flag3==0)) {
			mb_state.dist_ref = 0.0;
			position_now = 0.85;
			// printf("0.85 meters now!!-----%f-----%d-----%d*****%f-----%d\n",position_dist,flag3,num_loop,dist_error,stop);
		} else if ((mb_state.dist-position_dist>=0.75)&&(flag3==0)) {
			mb_state.dist_ref = 0.025;
			position_now = 0.75;
			// printf("0.75 meters now!!-----%f-----%d-----%d*****%f-----%d\n",position_dist,flag3,num_loop,dist_error,stop);
		} else if ((mb_state.dist-position_dist>=0.5)&&(flag3==0)) {
			mb_state.dist_ref = 0.075;
			position_now = 0.5;
			// printf("0.5 meters now!!-----%f-----%d-----%d*****%f-----%d\n",position_dist,flag3,num_loop,dist_error,stop);
		} else if ((mb_state.dist-position_dist>=0.25)&&(flag3==0)) {
			mb_state.dist_ref = 0.1;
			position_now = 0.25;
			// printf("0.25 meters now!!-----%f-----%d-----%d*****%f-----%d\n",position_dist,flag3,num_loop,dist_error,stop);
		} else if ((mb_state.dist-position_dist<=0.05)&&(flag3==0)) {
			mb_state.dist_ref = 0.2;
			position_now = 0;
			// printf("Beginning!!-----%f-----%d-----%d*****%f-----%d\n",position_dist,flag3,num_loop,dist_error,stop);
		}
		else if (flag3==0) {
			mb_state.dist_ref = 0.125;
			position_now = 0.1;
			// printf("0.05 meters now!!-----%f-----%d-----%d*****%f-----%d\n",position_dist,flag3,num_loop,dist_error,stop);
		}

		if ((mb_odometry.psi-position_angle>=-PI/16)&&(flag4==0)) {
			mb_odometry.psi_ref = -2.5;
		} else if ((mb_odometry.psi-position_angle>=-PI/4)&&(flag4==0)) {
			mb_odometry.psi_ref = -2;
		} else if ((mb_odometry.psi-position_angle>=-3*PI/8)&&(flag4==0)) {
			mb_odometry.psi_ref = -1;
		} else if ((mb_odometry.psi-position_angle>=-PI/2)&&(flag4==0)) {
			mb_odometry.psi_ref = -0.5;
		} else if (flag4==0) {
			position_angle -= PI/2;
			flag4 = 1;
		}

		

		if (flag3==1) {
			mb_state.dist_ref = 0;
			mb_state.dist_prev = 0;
			stop = 50.0;
		}

		if (flag4==1) {
			mb_odometry.psi_ref = 0;
			mb_odometry.psi_prev = 0;
			mb_state.gamma_prev = 0;
			stop2 = 40.0;
			// printf("%f*****%f*****%f*****%f\n",mb_odometry.psi_ref,mb_odometry.psi,gamma_error,position_angle);
		}

		if ((flag3==1)&(flag4==1)) {
			num_loop = num_loop - 1;
		}

		if (position_dist==16) {
			position_now = 1;
			position_prev = position_now;
			flag3 = 1;
			stop = 60.0;
		}

		dist_error = (mb_state.dist_ref + mb_state.dist_prev)*(position_dist!=16) + position_dist*flag3 - mb_state.dist;//+ (position_now<position_prev)*0.6
		D2_u = rc_filter_march(&D2, dist_error*20);//*(stop+10.0)
		theta_error = -D2_u + mb_state.theta - 0.05;
		D1_u = rc_filter_march(&D1, theta_error);
		gamma_error = -(mb_odometry.psi_ref + mb_odometry.psi_prev)*(position_angle!=-8*PI) - position_angle*flag4 + mb_odometry.psi;
		// gamma_error = (mb_odometry.psi_ref + mb_state.gamma_prev)*(position_angle!=-8*PI) + position_angle*flag4 - mb_state.gamma;
		D3_u = rc_filter_march(&D3, gamma_error*stop2);
		mb_state.left_cmd = D1_u - D3_u;
		mb_state.right_cmd = D1_u + D3_u;

		if (num_loop==0) {
			num_loop = 300;
			stop = 10.0;
			position_now = 0;
			plan_change = 1;
			plan += 1;
		}

		if (plan_change) {
			if ((plan%2)==1) {
				flag3 = 0;
				
				plan_change = 0;
			}
			else {
				flag4 = 0;
				stop2 = 10.0;

				plan_change = 0;
			}
		}

	
		position_prev = position_now;
	} else if (controller == 7) {
		if (mb_state.dist>=10.4) {
			mb_state.dist_ref = 10.4;
			mb_state.dist_prev = 0;
			flag3 = 1;
			// printf("\r");
			printf("10.4 meters now!!\n");
		} else if (mb_state.dist>=10) {
			mb_state.dist_ref = 0.0;
			// printf("\r");
			printf("10 meters now!!\n");
		} else if (mb_state.dist>=9.5) {
			mb_state.dist_ref = 0.01;
			// printf("\r");
			printf("9.5 meters now!!\n");
		} else if (mb_state.dist>=8.5) {
			mb_state.dist_ref = 0.05;
			// printf("\r");
			printf("8.5 meters now!!\n");
		} else if (mb_state.dist>=6.5) {
			mb_state.dist_ref = 0.1;
			// printf("\r");
			printf("6.5 meters now!!\n");
		} else if (mb_state.dist>=5) {
			mb_state.dist_ref = 0.25;
			// printf("\r");
			printf("5 meters now!!\n");
		} else if (mb_state.dist>=3) {
			mb_state.dist_ref = 0.4;
			// printf("\r");
			printf("3 meters now!!\n");
		} else if (mb_state.dist<=0.5) {
			mb_state.dist_ref = 1;
		}
		else {
			mb_state.dist_ref = 0.5;
		}
		if (mb_state.dist>0.9 && mb_state.dist<1.1) {
			printf("\r");
			printf("1 meter now!!");
		}
		if (flag3) {
			mb_state.dist_ref = 10.4;
			mb_state.dist_prev = 0;
			stop = 50;
		}
		// printf("%f\n",mb_state.dist);
		dist_error = mb_state.dist_ref + mb_state.dist_prev - mb_state.dist;
		D2_u = rc_filter_march(&D2, dist_error*stop);
		theta_error = -D2_u + mb_state.theta - 0.03;
		D1_u = rc_filter_march(&D1, theta_error);
		gamma_error = mb_odometry.psi;
		// gamma_error = - mb_state.gamma;
		D3_u = rc_filter_march(&D3, gamma_error*7);
		mb_state.left_cmd = D1_u - D3_u;
		mb_state.right_cmd = D1_u + D3_u;
	} else if (controller == 8) {
		mb_state.theta_ref_LQR = 0;
		mb_state.theta_dot_ref_LQR = 0;
		mb_state.phi_ref_LQR = 0;
    	mb_state.phi_dot_ref_LQR = 0; 
    	theta_error = mb_state.theta_ref_LQR - mb_state.theta_LQR;
		theta_dot_error = mb_state.theta_dot_ref_LQR - mb_state.theta_dot_LQR;
		phi_error = mb_state.phi_ref_LQR - mb_state.phi_LQR;
		phi_dot_error = mb_state.phi_dot_ref_LQR - mb_state.phi_dot_LQR;
		mb_state.left_cmd = K1*theta_error + K2*theta_dot_error + K3*phi_error + K4*phi_dot_error;
		mb_state.right_cmd = K1*theta_error + K2*theta_dot_error + K3*phi_error + K4*phi_dot_error;
	}

    if (mb_state.left_cmd > 1.0) mb_state.left_cmd = 1.0;
	if (mb_state.left_cmd < -1.0) mb_state.left_cmd = -1.0;
	if (mb_state.right_cmd > 1.0) mb_state.right_cmd = 1.0;
	if (mb_state.right_cmd < -1.0) mb_state.right_cmd = -1.0;
	if (rc_dsm_ch_normalized(1)>-0.5) {
		mb_motor_set(LEFT_MOTOR,mb_state.left_cmd);
		mb_motor_set(RIGHT_MOTOR,mb_state.right_cmd);
	}
	else {
		mb_motor_set(LEFT_MOTOR,0.0);
		mb_motor_set(RIGHT_MOTOR,0.0);
	}

   	mb_state.phi_prev = mb_state.phi;
   	mb_state.gamma_prev = mb_state.gamma;
   	mb_state.dist_prev = mb_state.dist;
   	mb_odometry.psi_prev = mb_odometry.psi;

   	prev_time = curr_time;

	XBEE_getData();
	double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	double tb_array[3] = {0, 0, 0};
	rc_quaternion_to_tb_array(q_array, tb_array);
	mb_state.opti_x = xbeeMsg.x;
	mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_roll = tb_array[0];
	mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
	
	
   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);

}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){

	while(1){
		if(rc_dsm_is_new_data()){
			// TODO: Handle the DSM data from the Spektrum radio reciever
			// You may should implement switching between manual and autonomous mode
			// using channel 5 of the DSM data.
			
			if (rc_dsm_ch_normalized(5)>0.2) {
				if ((rc_dsm_ch_normalized(4)<0.05)&&(rc_dsm_ch_normalized(4)>-0.05)) {
					mb_state.gamma_ref = 0;
				}
				else {
					mb_state.gamma_ref = rc_dsm_ch_normalized(4)*45;
				}
				if ((rc_dsm_ch_normalized(3)<0.05)&&(rc_dsm_ch_normalized(3)>-0.05)) {
					mb_state.phi_ref = 0;
				}
				else {
					mb_state.phi_ref = rc_dsm_ch_normalized(3)*8;
				}

				mb_setpoints.manual_ctl = 1;
				if ((mb_state.phi_ref==0)&&(flag1==0)) {
					flag1 = 1;
					setpoint_phi = mb_state.phi;
				}
				if ((mb_state.phi_ref!=0)&&(flag1==1)) {
					flag1 = 0;
					setpoint_phi = 0;
				}
				if ((mb_state.gamma_ref==0)&&(flag2==0)) {
					flag2 = 1;
					setpoint_gamma = mb_state.gamma;
				}
				if ((mb_state.gamma_ref!=0)&&(flag2==1)) {
					flag2 = 0;
					setpoint_gamma = 0;
				}
			}
			else if ((rc_dsm_ch_normalized(5)<0.2)&&(rc_dsm_ch_normalized(5)>-0.2)) {
				if ((rc_dsm_ch_normalized(4)<0.02)&&(rc_dsm_ch_normalized(4)>-0.02)) {
					mb_odometry.psi_ref = 0;
				}
				else {
					mb_odometry.psi_ref = rc_dsm_ch_normalized(4)*10;
				}
				if ((rc_dsm_ch_normalized(3)<0.02)&&(rc_dsm_ch_normalized(3)>-0.02)) {
					mb_state.dist_ref = 0;
				}
				else {
					mb_state.dist_ref = rc_dsm_ch_normalized(3)/15;
				}

				mb_setpoints.manual_ctl = 0;
				if ((mb_state.dist_ref==0)&&(flag1==0)) {
					flag1 = 1;
					setpoint_dist = mb_state.dist;
				}
				if ((mb_state.dist_ref!=0)&&(flag1==1)) {
					flag1 = 0;
					setpoint_dist = 0;
				}
				if ((mb_odometry.psi_ref==0)&&(flag2==0)) {
					flag2 = 1;
					setpoint_psi = mb_odometry.psi;
				}
				if ((mb_odometry.psi_ref!=0)&&(flag2==1)) {
					flag2 = 0;
					setpoint_psi = 0;
				}
			}
			else {
				mb_setpoints.manual_ctl = -1;
				if ((rc_dsm_ch_normalized(4)<0.05)&&(rc_dsm_ch_normalized(4)>-0.05)) {
					mb_odometry.psi_ref = 0;
				}
				else {
					mb_odometry.psi_ref = rc_dsm_ch_normalized(4)*20;
				}
				if ((rc_dsm_ch_normalized(3)<0.05)&&(rc_dsm_ch_normalized(3)>-0.05)) {
					mb_state.dist_ref = 0;
				}
				else {
					mb_state.dist_ref = rc_dsm_ch_normalized(3)/10;
				}

				if ((mb_state.dist_ref==0)&&(flag1==0)) {
					flag1 = 1;
					setpoint_dist = mb_state.dist;
				}
				if ((mb_state.dist_ref!=0)&&(flag1==1)) {
					flag1 = 0;
					setpoint_dist = 0;
				}
				if ((mb_odometry.psi_ref==0)&&(flag2==0)) {
					flag2 = 1;
					setpoint_psi = mb_odometry.psi;
				}
				if ((mb_odometry.psi_ref!=0)&&(flag2==1)) {
					flag2 = 0;
					setpoint_psi = 0;
				}
			}
		}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}




/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){

	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |            MOCAP            |");
			printf("\n");
			printf("    θ    |");
			printf("    φ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\n");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.phi);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_state.opti_x);
			printf("%7.3f  |", mb_state.opti_y);
			printf("%7.3f  |", mb_state.opti_yaw);
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
		// usleep(1E6/MSG_RATE_HZ);
	}
	return NULL;


} 