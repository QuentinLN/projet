#ifndef ESTIMATOR_H
#define ESTIMATOR_H

typedef struct
{
  float angle;
} compute_angle_t;

/* Start the thread that computes angle and manages led to show gravity */
void compute_angle_start(void);
/* Function that returns angle */
float get_angle(void);

#endif /* ESTIMATOR_H */
