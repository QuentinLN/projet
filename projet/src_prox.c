#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <src_prox.h>
#include "sensors/proximity.h"
#include "leds.h"

#include "bus.h"

static proximity_group_t proximity_sensors;

// Thread to compute the proximity sensors datas
static THD_WORKING_AREA(waProximitySensorCompute, 256);
static THD_FUNCTION(ProximitySensorCompute, arg) {

	(void) arg;
	// Initalization of the proximity_sensors datas
	proximity_struct_init(IR_DETEC_LIMIT);

	while(true) {
    // FRONT_RIGHT
    if(get_calibrated_prox(0)> proximity_sensors.detec_limit) {
      proximity_sensors.IR0_detec = 1;
    } else {
      proximity_sensors.IR0_detec = 0;
    }
    if(get_calibrated_prox(1)> proximity_sensors.detec_limit) {
      proximity_sensors.IR1_detec = 1;
    } else {
      proximity_sensors.IR1_detec = 0;
    }
    if(get_calibrated_prox(2)> proximity_sensors.detec_limit) {
      proximity_sensors.IR2_detec = 1;
    } else {
      proximity_sensors.IR2_detec = 0;
    }
    if(get_calibrated_prox(3)> proximity_sensors.detec_limit) {
      proximity_sensors.IR3_detec = 1;
    } else {
      proximity_sensors.IR3_detec = 0;
    }
    if(get_calibrated_prox(4)> proximity_sensors.detec_limit) {
      proximity_sensors.IR4_detec = 1;
    } else {
      proximity_sensors.IR4_detec = 0;
    }
    if(get_calibrated_prox(5)> proximity_sensors.detec_limit) {
      proximity_sensors.IR5_detec = 1;
    } else {
      proximity_sensors.IR5_detec = 0;
    }
    if(get_calibrated_prox(6)> proximity_sensors.detec_limit) {
      proximity_sensors.IR6_detec = 1;
    } else {
      proximity_sensors.IR6_detec = 0;
    }
    if(get_calibrated_prox(7)> proximity_sensors.detec_limit) {
      proximity_sensors.IR7_detec = 1;
    } else {
      proximity_sensors.IR7_detec = 0;
    }
    if(proximity_sensors.IR0_detec == 1||proximity_sensors.IR1_detec == 1||proximity_sensors.IR2_detec == 1||proximity_sensors.IR3_detec == 1||proximity_sensors.IR6_detec == 1||proximity_sensors.IR7_detec == 1)
    {
        palWritePad(GPIOB, GPIOB_LED_BODY, 1);
    }else{
        palWritePad(GPIOB, GPIOB_LED_BODY, 0);
    }
	chThdSleepMilliseconds(200);
	}
}

void proximity_compute_start(void)
{
	// Start the thread to compute proximity sensors data
	chThdCreateStatic(waProximitySensorCompute, sizeof(waProximitySensorCompute), NORMALPRIO, ProximitySensorCompute, NULL);
}

void proximity_struct_init(uint16_t limit)
{
	// Initialize the proximity sensors structure values
	proximity_sensors.detec_limit = limit;
	proximity_sensors.IR0_detec = 0;
	proximity_sensors.IR1_detec = 0;
	proximity_sensors.IR2_detec = 0;
	proximity_sensors.IR3_detec = 0;
	proximity_sensors.IR4_detec = 0;
	proximity_sensors.IR5_detec = 0;
	proximity_sensors.IR6_detec = 0;
	proximity_sensors.IR7_detec = 0;
}

// Return 1 if something has been detected in the direction
int isDetectedindirection(int direction)
{

	switch(direction)
	{
	case 0:
		return proximity_sensors.IR0_detec;
		break;

	case 1:
		return proximity_sensors.IR1_detec;
		break;

	case 2:
		return proximity_sensors.IR2_detec;
		break;

	case 3:
		return proximity_sensors.IR3_detec;
		break;

	case 4:
		return proximity_sensors.IR4_detec;
		break;

	case 5:
		return proximity_sensors.IR5_detec;
		break;

	case 6:
		return proximity_sensors.IR6_detec;
		break;

	case 7:
		return proximity_sensors.IR7_detec;
		break;
	}
	return 0;
}
