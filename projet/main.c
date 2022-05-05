#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <msgbus/messagebus.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

#include <main.h>
#include <motors.h>
#include <chprintf.h>

#include <pi_regulator.h>


#include <math.h>
#include <i2c_bus.h>
#include <sensors/imu.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static float angle_degre=0;
extern parameter_namespace_t parameter_root;



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

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

void printangle(imu_msg_t *imu_values){
	float *accel = imu_values->acceleration;


    //clock wise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)

    float threshold = 0.2f;
    float thresholddegre = 10;
    if((sqrtf(fabs(accel[X_AXIS])*fabs(accel[X_AXIS]) + fabs(accel[Y_AXIS])*fabs(accel[Y_AXIS])) > threshold )&& fabs(atan2(accel[X_AXIS], accel[Y_AXIS]))*360/(2*M_PI)-thresholddegre>0){


    angle_degre=360/(2*M_PI)*atan2(accel[X_AXIS], accel[Y_AXIS]);
    }else{
    	angle_degre=0;
    }
    //chprintf((BaseSequentialStream *)&SD3, "%Angle>10%.2f carre%.2f  Angle%.2f\r\n\n",
    //		angle_degre,sqrtf(fabs(accel[X_AXIS])*fabs(accel[X_AXIS]) + fabs(accel[Y_AXIS])*fabs(accel[Y_AXIS])),fabs(atan2(accel[X_AXIS], accel[Y_AXIS]))*360/(2*M_PI)-thresholddegre);

}



static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

float get_angle_degre(void){
	return angle_degre;
}

void show_gravity(imu_msg_t *imu_values){

    //we create variables for the led in order to turn them off at each loop and to
    //select which one to turn on
    uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
    //threshold value to not use the leds when the robot is too horizontal
    float threshold = 0.98;
    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;
    //variable to measure the time some functions take
    //volatile to not be optimized out by the compiler if not used
    volatile uint16_t time = 0;


    if(fabs(accel[X_AXIS]) > threshold || fabs(accel[Y_AXIS]) > threshold){

        chSysLock();
        //reset the timer counter
        GPTD11.tim->CNT = 0;
        //clock wise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)
        float angle = atan2(accel[X_AXIS], accel[Y_AXIS]);
        //by reading time with the debugger, we can know the computational time of atan2 function
        time = GPTD11.tim->CNT;
        chSysUnlock();

        //rotates the angle by 45 degrees (simpler to compare with PI and PI/2 than with 5*PI/4)
        angle += M_PI/4;

        //if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
        //so we correct it
        if(angle > M_PI){
            angle = -2 * M_PI + angle;
        }

        if(angle >= 0 && angle < M_PI/2){
            led5 = 1;
        }else if(angle >= M_PI/2 && angle < M_PI){
            led7 = 1;
        }else if(angle >= -M_PI && angle < -M_PI/2){
            led1 = 1;
        }else if(angle >= -M_PI/2 && angle < 0){
            led3 = 1;
        }
    }

    //to see the duration on the console
    chprintf((BaseSequentialStream *)&SD3, "time = %dus\n",time);
    //we invert the values because a led is turned on if the signal is low
    palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);

}

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
    serial_start();
    timer11_start();
    i2c_start();
    imu_start();

    mpu_init();

	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	chThdCreateStatic(waThdFrontLed, sizeof(waThdFrontLed), NORMALPRIO, ThdFrontLed, NULL);

    /** Inits the Inter Process Communication bus. */
    	imu_msg_t imu_values;
        messagebus_init(&bus, &bus_lock, &bus_condvar);
        messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
        imu_compute_offset(imu_topic,NB_SAMPLES_OFFSET);


        while(1){
            //wait for new measures to be published
            messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
            printangle(&imu_values);
            show_gravity(&imu_values);
            chThdSleepMilliseconds(100);
        }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
