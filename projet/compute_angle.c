#include <ch.h>
#include <chvt.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <pi_regulator.h>
#include "sensors/imu.h"
#include "sensors/proximity.h"

#include "motors.h"
#include "compute_angle.h"

#include "bus.h"
#include "send.h"

#define STANDARD_GRAVITY    9.80665f
// Le temps de rafraichissement de l'IMU est quasi-constant (determiné empiriquement)
// donc le temps est directement fixé à une constante pour réduire le nombre d'instructions

static compute_angle_t compute_angle_values;

float get_angle()
{
  return compute_angle_values.angle;
}

static THD_WORKING_AREA(waEstimator, 256);
static THD_FUNCTION(Estimator, arg) {

  messagebus_topic_t* imu_topic = (messagebus_topic_t*)arg;
  imu_msg_t imu_values;

  float angle_rad;
  float angle;

  while(true) {
    // Attente des valeurs des prochaines valeurs de l'IMU
    messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));


    // Angle de l'accéleromètre
    //  Accéléromètre --> Bruit élevé donc on ajoute un treshold

    float threshold = 0.5f;
    float thresholddegre = 10;
    if((sqrtf(fabs(imu_values.acceleration[0])*fabs(imu_values.acceleration[0]) + fabs(imu_values.acceleration[1])*fabs(imu_values.acceleration[1])) > threshold )&& fabs(atan2(imu_values.acceleration[0], imu_values.acceleration[1]))*360/(2*M_PI)-thresholddegre>0)
    {
    	uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
    	angle_rad=atan2(imu_values.acceleration[0], imu_values.acceleration[1]);
    	angle=360/(2*M_PI)*angle_rad;

        //rotates the angle by 45 degrees (simpler to compare with PI and PI/2 than with 5*PI/4)
        angle_rad += M_PI/4;

        //if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
        //so we correct it
        if(angle_rad > M_PI)
        {
            angle_rad = -2 * M_PI + angle;
        }

        if(angle_rad >= 0 && angle_rad < M_PI/2)
        {
            led5 = 1;
        }else if(angle_rad >= M_PI/2 && angle_rad < M_PI)
        {
            led7 = 1;
        }else if(angle_rad >= -M_PI && angle_rad < -M_PI/2)
        {
            led1 = 1;
        }else if(angle_rad >= -M_PI/2 && angle_rad < 0)
        {
            led3 = 1;
        }
		//to see the duration on the console
		//chprintf((BaseSequentialStream *)&SD3, "time = %dus\n",time);
		//we invert the values because a led is turned on if the signal is low
		palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
		palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
		palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
		palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);
    }else
    {
    	angle=0;
    }
  }
}

void compute_angle_start(void){
  messagebus_topic_t *imu_topic;
  imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

  compute_angle_values.angle = 0.f;

  chThdCreateStatic(waEstimator, sizeof(waEstimator), NORMALPRIO+2, Estimator, (void*)imu_topic);
}
