/*
 * pid_regulator.h
 *
 *  Created on: 14 mai 2022
 *  Author: Quentin Le N?zet
 *
 *
 */



#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#define NB_SAMPLES 256

typedef struct {
  float kp;
  float kd;
  float ki;

  float integral;
  float derivative;

} reg_param_t;

//start the PID regulator threads
void pid_regulator_start(void);


#endif /* PID_REGULATOR_H */
