/*
 * src_prox.h
 *
 *  Created on: 14 mai 2022
 *  Author: Quentin Le Nézet
 *
 *
 */



#ifndef PROXIMITY_SENSOR_H_
#define PROXIMITY_SENSOR_H_

#define IR_DETEC_LIMIT 200

typedef struct
{
	uint16_t detec_limit;
	uint8_t IR0_detec;
	uint8_t IR1_detec;
	uint8_t IR2_detec;
	uint8_t IR3_detec;
	uint8_t IR4_detec;
	uint8_t IR5_detec;
	uint8_t IR6_detec;
	uint8_t IR7_detec;
} proximity_group_t;

void proximity_compute_start(void);
void proximity_struct_init(uint16_t limit);
int isDetectedindirection(int direction);


#endif /* PROXIMITY_SENSOR_H_ */
