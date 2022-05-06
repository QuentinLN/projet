#include "ch.h"
#include "hal.h"
#include <math.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <compute_angle.h>



//simple PI regulator implementation
int16_t pi_regulator(float angle, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = angle - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 800;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread

        speed_correction = pi_regulator(get_angle(), 0);
        //chprintf((BaseSequentialStream *)&SD3, "%SPEED=%.d SPEEDCORRECTION=%.d,SPEEDCORRECTION=%.2f ,angle=%.d  \r\n\n",
        //		speed, speed_correction,speed_correction,get_angle_degre());

        //applies the speed from the PI regulator and the correction for the rotation
        //right_motor_set_speed((speed - ROTATION_COEFF * speed_correction)/5);
    	//left_motor_set_speed((speed + ROTATION_COEFF * speed_correction)/5);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));

    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
