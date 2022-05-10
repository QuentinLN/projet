#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <i2c_bus.h>
#include <leds.h>
#include <usbcfg.h>
#include <motors.h>
#include <chprintf.h>
#include <pid_regulator.h>
#include <src_prox.h>

#include "main.h"
#include "motors.h"
#include "chprintf.h"
#include "send.h"
#include "compute_angle.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void timer11_start(void){
    //General Purpose Timer configuration
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
}


/* Start a thread that blinks the front led to show the direction of the epuck */
static THD_WORKING_AREA(waThdFrontLed, 128);
static THD_FUNCTION(ThdFrontLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();
        palTogglePad(GPIOD, GPIOD_LED_FRONT);
        chThdSleepUntilWindowed(time, time + MS2ST(1000));
    }
}

int main(void)
{
    /* System init */
    halInit();
    chSysInit();
    mpu_init();

    /* Start the serial communication */
    serial_start();
    timer11_start();
    i2c_start();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    /* Start the imu */
    imu_start();

    /* Calibration of the imu */
    calibrate_acc();

    /*  Start the proximity sensors */
    proximity_start();

    /* Calibration of the ambient IR intensity */
    calibrate_ir();

    /* Start to compute the datas from the IR sensors */
    proximity_compute_start();

    /* Init the motors */
	motors_init();

	/* Starts the threads for the pi regulator and compute angle and front_led */
	compute_angle_start();
	pid_regulator_start();
	chThdCreateStatic(waThdFrontLed, sizeof(waThdFrontLed), NORMALPRIO, ThdFrontLed, NULL);



    while(1)
    {
        /*
      	 *
      	 *
      	 * DO NOTHING
      	 *
      	 *
      	 */


    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
