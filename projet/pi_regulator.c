#include "ch.h"
#include "hal.h"
#include <math.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <compute_angle.h>
#include <src_prox.h>



/* Simple PI regulator implementation adapted from TP3 */
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

/* Thread associated to the PIregulator that manages the motor (code adapted form TP4) */
static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 160;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors in function of the different thread (proximity sensors and compute angle)

        if(isDetectedindirection(0)==1){
        	//right_motor_set_speed(speed);
        	//left_motor_set_speed(-speed);
        	//chThdSleepMilliseconds(200);
        } else if(isDetectedindirection(7)==1){
        	//right_motor_set_speed(-speed);
        	//left_motor_set_speed(speed);
        	//chThdSleepMilliseconds(200);

        } else if(isDetectedindirection(1)==1){
        	//right_motor_set_speed(speed);
        	//left_motor_set_speed(-speed);
        	//chThdSleepMilliseconds(200);
    	} else if(isDetectedindirection(6)==1){
    		//right_motor_set_speed(-speed);
        	//left_motor_set_speed(speed);
    		//chThdSleepMilliseconds(200);
    	} else{
            //applies the speed from the PI regulator and the correction for the rotation
            //right_motor_set_speed((speed - ROTATION_COEFF * speed_correction));
        	//left_motor_set_speed((speed + ROTATION_COEFF * speed_correction));
    	}
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));

    }
}

/* Starts the thread associated to the PIregulator that manages the motor */
void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
