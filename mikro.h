/*
 * mikro.h
 *
 *  Created on: Dec 11, 2025
 *      Author: gothi
 */

#ifndef INC_MIKRO_H_
#define INC_MIKRO_H_



#endif /* INC_MIKRO_H_ */

#include "main.h"
#include "stm32f4xx_it.h"




//Audio Processing Function
void process_audio_buffer(uint32_t offset, I2S_HandleTypeDef* hi2s2);
void HAL (I2S_HandleTypeDef* hi2s2);
void getSamples ();
void LED_ansteuern ();
void safeValues();
void convert();


