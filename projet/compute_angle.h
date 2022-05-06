#ifndef ESTIMATOR_H
#define ESTIMATOR_H

typedef struct
{
  float angle;
} compute_angle_t;

void compute_angle_start(void);
float get_angle(void);

#endif /* ESTIMATOR_H */
