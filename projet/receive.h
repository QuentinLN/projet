/*
 * receive.h
 *
 *  Created on: 14 mai 2022
 *  Author: Quentin Le Nézet
 *
 *
 */


#ifndef RECEIVE_DATA_H
#define RECEIVE_DATA_H

uint16_t ReceiveFloatFromComputer(BaseSequentialStream* in, float* data, uint16_t size);

#endif /* RECEIVE_H */
