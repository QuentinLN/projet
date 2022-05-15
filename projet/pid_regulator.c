/*
 * pid_regulator.c
 *
 *  Created on: 14 mai 2022
 *  Author: Quentin Le Nézet
 *
 *  Description:
 *	Regulateur avec correcteur PID du miniprojet Robot Climber
 *
 */





#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <pid_regulator.h>
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include <chprintf.h>

#include "motors.h"
#include <compute_angle.h>
#include "receive.h"
#include "send.h"
#include "src_prox.h"
#include "leds.h"
#include "bus.h"
#include "main.h"


static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);
static float send_tab[2*NB_SAMPLES] = {0};
static reg_param_t reg_param;


/* Thread associated to the PIDregulator that manages the motor (code adapted form TP4) */
static THD_WORKING_AREA(waPIDRegulator, 256);
static THD_FUNCTION(PIDRegulator, arg) {

	chRegSetThreadName(__FUNCTION__);
  (void)arg;

	systime_t time;

    float speed = 160.f;
    float rightspeed = 0.f;
    float leftspeed = 0.f;
    float speed_correction = 0.f;
	float angle_error= 0.f;
	float angle_error_prev = 0.f;

	while(1) {
    for(uint16_t i=0; i<NB_SAMPLES; ++i) {
    	time = chVTGetSystemTime();


		angle_error_prev = angle_error;
		angle_error = get_angle();
		reg_param.derivative = angle_error_prev - angle_error; // Compute the derivative
		reg_param.integral += angle_error; // Compute the integral

		// Limit the integral value if the command is saturated
		if((reg_param.integral * reg_param.ki) > MOTOR_SPEED_LIMIT)
			reg_param.integral = MOTOR_SPEED_LIMIT / reg_param.ki;
		else if((reg_param.integral * reg_param.ki) < - MOTOR_SPEED_LIMIT)
			reg_param.integral = - MOTOR_SPEED_LIMIT / reg_param.ki;


		  // Compute the speed command
		speed_correction = reg_param.kp * angle_error + reg_param.kd * reg_param.derivative + reg_param.ki * reg_param.integral;

		if(isDetectedindirection(0)==1){
			right_motor_set_speed(speed);
			left_motor_set_speed(-speed);
			chThdSleepMilliseconds(500);
			reg_param.integral=0;
			reg_param.derivative =0;
		} else if(isDetectedindirection(7)==1){
			right_motor_set_speed(-speed);
			left_motor_set_speed(speed);
			chThdSleepMilliseconds(500);
			reg_param.integral=0;
			reg_param.derivative =0;
		} else if(isDetectedindirection(1)==1){
			right_motor_set_speed(speed);
			left_motor_set_speed(-speed);
			chThdSleepMilliseconds(500);
			reg_param.integral=0;
			reg_param.derivative =0;
		} else if(isDetectedindirection(6)==1){
			right_motor_set_speed(-speed);
			left_motor_set_speed(speed);
			chThdSleepMilliseconds(500);
			reg_param.integral=0;
			reg_param.derivative =0;
		} else{
			// limits the speed to the motors max speed
			rightspeed=speed - speed_correction;
			leftspeed=speed + speed_correction;
			if(rightspeed > MOTOR_SPEED_LIMIT)
				rightspeed = MOTOR_SPEED_LIMIT;
			else if(rightspeed < -MOTOR_SPEED_LIMIT)
				rightspeed = -MOTOR_SPEED_LIMIT;
			if(leftspeed > MOTOR_SPEED_LIMIT)
				leftspeed = MOTOR_SPEED_LIMIT;
			else if(leftspeed < -MOTOR_SPEED_LIMIT)
				leftspeed = -MOTOR_SPEED_LIMIT;

			//applies the speed from the PI regulator and the correction for the rotation
			right_motor_set_speed(rightspeed);
			left_motor_set_speed(leftspeed);
		}

		send_tab[i] = angle_error;
		send_tab[NB_SAMPLES+i] = speed_correction;


		//10Hz
		chThdSleepUntilWindowed(time, time + MS2ST(100));
    	}
    chBSemSignal(&sendToComputer_sem);
	}
}

static THD_WORKING_AREA(waPIDRegulatorReader, 256);
static THD_FUNCTION(PIDRegulatorReader, arg) {

	chRegSetThreadName(__FUNCTION__);
  (void) arg;

	float data[4];
	while(true) {
		ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, data, 4);

		reg_param.kp = data[0];
		reg_param.kd = data[1];
		reg_param.ki = data[2];
    // Reset de l'intégrale au cas ou
		reg_param.integral = 0.f;

		chThdSleepMilliseconds(1000);
	}
}


// Thread to send the angle datas to the computer
static THD_WORKING_AREA(waPIDRegulatorSender, 256);
static THD_FUNCTION(PIDRegulatorSender, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while(true) {
		chBSemWait(&sendToComputer_sem);

		SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, 2*NB_SAMPLES);
	}
}

// Start the 3 thread of regulator
void pid_regulator_start(void){
  reg_param.kp = 2;
  reg_param.kd = 0.4 ;
  reg_param.ki = 0.05;
  reg_param.integral = 0.0;

	chThdCreateStatic(waPIDRegulator, sizeof(waPIDRegulator), HIGHPRIO, PIDRegulator, NULL);
	chThdCreateStatic(waPIDRegulatorReader, sizeof(waPIDRegulatorReader), NORMALPRIO, PIDRegulatorReader, NULL);
	chThdCreateStatic(waPIDRegulatorSender, sizeof(waPIDRegulatorSender), LOWPRIO, PIDRegulatorSender, NULL);
}

