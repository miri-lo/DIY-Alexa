/*
 * funk.c
 *
 *  Created on: Dec 8, 2025
 *      Author: gothi
 */

#include "funk.h"
#include "main.h"
#include "string.h"

#include <stdbool.h>

extern TIM_HandleTypeDef* TIMER;
extern bool stecker_an;

void transmit(int numHighPulses, int numLowPulses) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

	delay350Microseconds(numHighPulses);


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

	delay350Microseconds(numLowPulses);
	//HAL_Delay(numLowPulses*0.35);
}

void sendSequence(char* bitSequenz) {

	HAL_TIM_Base_Start(TIMER);
	for (int i = 0; i <10; i++) {

		for (int j = 0; j < strlen(bitSequenz); j++) {
			transmit(1, 3);

			int temp = (int)bitSequenz[j]-48;
			if (temp==1) {
				transmit(1,3);
			}
			else {
				transmit(3,1);
			}
		}
		transmit(1,31);
	}
	HAL_TIM_Base_Stop(TIMER);
}

void delay350Microseconds(uint32_t n)
{

	for (int i = 0; i < n; i++) {
		__HAL_TIM_SET_COUNTER(TIMER, 0);
		while(__HAL_TIM_GET_COUNTER(TIMER) < 350);
	}

}

void stecker_schalten () {
	if (stecker_an) {
		sendSequence("111111000010"); 		// aus
		stecker_an = false;
	}
	else {
		sendSequence("111111000001"); 		//an
		stecker_an = true;
	}
}

void steckerAn() {
		sendSequence("111111000001"); 		//an
		stecker_an = true;
	}



