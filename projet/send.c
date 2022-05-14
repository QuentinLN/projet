/*
 * send.c
 *
 *  Created on: 14 mai 2022
 *  Author: Quentin Le Nézet
 *
 *  Description:
 *	Envoie les données d'un ordinateur externe
 *
 */






#include <ch.h>
#include <hal.h>
#include <usbcfg.h>

#include "send.h"

// Fonctionn du TP4
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size)
{
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}

// Fonction du TP4
void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

