#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>
#include <sensors/mpu9250.h>
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"



/* Constants for the different parts of the project */


#define ROTATION_THRESHOLD		10
#define GOAL_ROTATION			0f
#define MAX_ROTATION			25.0f
#define ERROR_THRESHOLD			3.0f	//[degre] because of the noise of the IMU


#define NB_SAMPLES_OFFSET       200
#define RES_2G                  2.0f
#define MAX_INT16               32768.0f

/* Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;


#ifdef __cplusplus
}
#endif

#endif
