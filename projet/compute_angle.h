/*
 * compute_angle.h
 *
 *  Created on: 14 mai 2022
 *  Author: Quentin Le Nézet
 */

#ifndef COMPUTE_ANGLE_H
#define COMPUTE_ANGLE_H

typedef struct
{
  float angle;
} compute_angle_t;

/* Start the thread that computes angle and manages led to show gravity */
void compute_angle_start(void);
/* Function that returns angle */
float get_angle(void);

#endif /* COMPUTE_ANGLE_H */
