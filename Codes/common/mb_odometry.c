/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

static double phi_left_prev = 0.0;
static double phi_right_prev = 0.0;
static double theta = 0.0;
static double x = 0.0;
static double y = 0.0;
static double distance = 0.0;

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
/* TODO */
	mb_odometry -> x = x;
	mb_odometry -> y = y;
	mb_odometry -> psi = theta;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state, int mode){
/* TODO */

	double SL, SR, dd, dx, dy, dtheta;
	double VR, VL, R, CRx, CRy;
	double phi_left = rc_encoder_eqep_read(LEFT_MOTOR);
	double phi_right = rc_encoder_eqep_read(RIGHT_MOTOR);
	double dtheta_gyro, dtheta_odo;

	SL = (phi_left - phi_left_prev)*ENC_1_POL*WHEEL_DIAMETER*PI/ENCODER_RES;
	SR = (phi_right - phi_right_prev)*ENC_2_POL*WHEEL_DIAMETER*PI/ENCODER_RES;
	VL = SL/DT;
	VR = SR/DT;
	
	if (mode == 1) {
		dd = (SL+SR)/2.0;
		dtheta = (SR-SL)/WHEEL_BASE;
		dx = dd*cos(theta+dtheta/2.0);
		dy = dd*sin(theta+dtheta/2.0);
		x += dx;
		y += dy;
		theta += dtheta;
		// theta = mb_clamp_radians(theta);
	}
	else if (mode == 2){
		if (VR==VL) { 
			dd = (SL+SR)/2.0;
			dtheta = (VR-VL)/WHEEL_BASE*DT;
			dx = dd*cos(theta+dtheta/2.0);
			dy = dd*sin(theta+dtheta/2.0);
			x += dx;
			y += dy;
			theta += dtheta;
			theta = mb_clamp_radians(theta);
		}
		else {
			dtheta = (VR-VL)/WHEEL_BASE*DT;
			R = (WHEEL_BASE/2.0)*(VR+VL)/(VR-VL);
			CRx = x-R*sin(theta);
			CRy = y+R*cos(theta);
			x = cos(dtheta)*(x-CRx)-sin(dtheta)*(y-CRy)+CRx;
			y = sin(dtheta)*(x-CRx)+cos(dtheta)*(y-CRy)+CRy;
			theta += dtheta;
			theta = mb_clamp_radians(theta);
		}
	}else {
		dd = (SL+SR)/2.0;
		dtheta_gyro = mpu_data.gyro[2]*DEG_TO_RAD*DT;
		dtheta_odo = (VR-VL)/WHEEL_BASE*DT;
		if (fabs(dtheta_gyro - dtheta_odo)>0.125*DEG_TO_RAD) {
			dtheta = dtheta_gyro;
		} else {
			dtheta = dtheta_odo;
		}
		dx = dd*cos(theta+dtheta/2.0);
		dy = dd*sin(theta+dtheta/2.0);
		x += dx;
		y += dy;
		distance += dd;
		theta += dtheta;
		theta = mb_clamp_radians(theta);
	}
	mb_state->dist += (SL+SR)/2.0;
	phi_left_prev = phi_left;
	phi_right_prev = phi_right;
	mb_odometry ->x = x;
	mb_odometry ->y = y;
	mb_odometry ->psi = theta;
}


float mb_clamp_radians(float angle){

	if (angle<-PI) {
		angle += 2*PI;
	}
	if (angle>PI) {
		angle -= 2*PI;
	}
    return angle;
}