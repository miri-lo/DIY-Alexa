/*
 * mikro.c
 *
 *  Created on: Dec 11, 2025
 *      Author: gothi
 */
#include "mikro.h"
#include "string.h"
#include "main.h"
#include "funk.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#define PCM_SAMPLES_PER_SEC 16000

// DMA Input Buffer size in 16-bit Half Words (for 1 second):
// 16000 * 4 (*2 for 2 16 bit and *2 for unused right channel) * 2 (Two Halves of Buffer) = 128000
#define I2S_DMA_BUF_SIZE 1000 	//PCM_SAMPLES_PER_SEC * 8;
#define DMA_HALF_SIZE  I2S_DMA_BUF_SIZE / 2
#define MAX_MIKRO 20000
const uint32_t STEP = MAX_MIKRO / 10;

#define SCHWELLWERT 8000


// DMA Input Buffer: Stores the raw 16-bit data read by DMA.
static volatile uint16_t inputBuffer[I2S_DMA_BUF_SIZE*2] = {0};

// Processed Output Array: Stores the final 16-bit mono PCM samples.
static int16_t pcmSamples[PCM_SAMPLES_PER_SEC] = {0};
static int16_t safedValues[PCM_SAMPLES_PER_SEC] = {0};
uint32_t recordIndex =0;

// Index tracker and state flag
//extern volatile bool recording_finished;
volatile uint32_t pcm_index = 0;
volatile bool stecker_an = false;
volatile bool recordingFinished = false;
static uint32_t currentVolume = 0;
volatile bool schwellwert_passed = false;
volatile uint32_t counter = 0;

uint32_t timer;
uint32_t timerStart;

extern UART_HandleTypeDef* display;
extern I2S_HandleTypeDef mikro;


/*typedef enum {
    AUDIO_IDLE,
    AUDIO_RECORDING
} AudioState;

AudioState audioState = AUDIO_IDLE;*/


void HAL (I2S_HandleTypeDef* hi2s2) {
	timerStart = HAL_GetTick;
	HAL_I2S_Receive_DMA(hi2s2, (uint16_t*)inputBuffer, I2S_DMA_BUF_SIZE);

}


void getSamples () {
	char buffer[16]; // temporary buffer for converting int16_t to string
	int16_t max_volume = 0;
		uint32_t TimeBegin_ms = HAL_GetTick();
		int numLED;
	    for (int i = 0; i < PCM_SAMPLES_PER_SEC; i++)
	    {
	        // Convert sample to string
	        char len = sprintf(buffer, "%d\r\n", pcmSamples[i]);




	        // Transmit string over UART
	        //HAL_UART_Transmit(display, (uint8_t*)buffer, len, HAL_MAX_DELAY);


	        //HAL_Delay(500);
	        numLED = abs(pcmSamples[i]);
	        LED_ansteuern(numLED);
	        //delay350Microseconds(1);
	        // Check if it surpasses the threshold



	        if (abs(pcmSamples[i]) > SCHWELLWERT)
	        {
	        // max_volume = abs(pcmSamples[i]);
	        	pcm_index = 0;
	        	char* start = "Aufnahme beginnt";
	        	HAL_UART_Transmit(display, start, strlen(start), HAL_MAX_DELAY);
	        	safeValues();
	        	sprintf(buffer, "%d\r\n", safedValues);
	        	HAL_UART_Transmit(display, buffer, strlen(buffer), HAL_MAX_DELAY);
	        	char* end = "Aufnahme beendet \r\n";
	        	HAL_UART_Transmit(display, end, strlen(end), HAL_MAX_DELAY);
	         stecker_schalten();
	         continue;
	         //HAL_Delay(500);
	         //HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);


	    	    }
	    }


	uint32_t TimePassed_ms = HAL_GetTick() - TimeBegin_ms;
	LED_ansteuern(MAX_MIKRO);
	 /*for (int i = 0; i < I2S_DMA_BUF_SIZE*2; i++) {
		inputBuffer[i] = 0;
	}*/

}

//brief Processes Half of raw I2S data from the DMA buffer.
	//param offset Starting index in inputBuffer (0 for half, DMA_HALF_SIZE for full).

void safeValues() {
	// TODO

}


	/*void process_audio_buffer(uint32_t offset, I2S_HandleTypeDef* hi2s2)
	{
	    // The number of 16-bit Half Words to process in this Half
		const uint32_t half_size = DMA_HALF_SIZE;

	    // Loop through the Half, stepping by 4 (2*16bit words for the sample + 2*16bit words for the right channel).
		pcm_index = 0;
		int counter = 0;
		for (uint32_t i = 0; i < half_size; i += 4) {

			//if (pcm_index < PCM_SAMPLES_PER_SEC) {
			uint32_t currentIndex = pcm_index % PCM_SAMPLES_PER_SEC;

			uint32_t mergedValue = (inputBuffer[offset + i] << 16) | inputBuffer[offset + i + 1];

			int16_t finalSample = (int16_t)(mergedValue >> 14);

			pcmSamples[currentIndex] = finalSample;

			uint32_t absVol = abs(finalSample);
			if(absVol > currentVolume) {
				currentVolume = absVol;
			}
		pcm_index++;

			if(pcm_index % 100 == 0) {

				if(absVol > SCHWELLWERT && !recordingFinished) {
					recording_finished = true;
														timer = HAL_GetTick() - timerStart;
														//char* message[];
															//sprintf(message, timer);
															HAL_UART_Transmit(display, timer, strlen(timer), HAL_MAX_DELAY);
					stecker_schalten();
				}
			}

			//pcm_index++;

			if(pcm_index % 500 == 0) {
				currentVolume = 0;
			}
		//}
			//else {

			//}
		}

	}*/
void process_audio_buffer(uint32_t offset, I2S_HandleTypeDef* hi2s2)
	{
		timerStart = HAL_GetTick();
	    // The number of 16-bit Half Words to process in this Half
		const uint32_t half_size = DMA_HALF_SIZE;

	    // Loop through the Half, stepping by 4 (2*16bit words for the sample + 2*16bit words for the right channel).

		for (uint32_t i = 0; i < half_size; i += 4) {
			if (pcm_index >= PCM_SAMPLES_PER_SEC) {
				pcm_index = 0;
			}

	        if (pcm_index < PCM_SAMPLES_PER_SEC) {

	            // Reconstruct 32-bit Frame

	        	uint32_t mergedValue = (inputBuffer[offset + i ] << 16) | inputBuffer[offset + i + 1];
	            // Extract 18-bit Sample and convert to signed 16-bit integer.
	            int16_t finalSample = (int16_t)(mergedValue >> 14);

	            //  Store the clean Mono Sample

	            //check if schwellwert passed
	            pcmSamples[pcm_index] = finalSample;
	            if (finalSample >= SCHWELLWERT && schwellwert_passed == false) {
	            	int temp = pcm_index - 4000;
	            	if (temp < 0) {
	            		temp = 16000 + temp;
	            	}
	            	for (int j = 0; j < 4000; j++) {
	            		safedValues[j] = pcmSamples[temp];
	            		temp = (temp + 1) % PCM_SAMPLES_PER_SEC;
	            		counter = j;
	            	}
	            	schwellwert_passed = true;
	            }
	            if (schwellwert_passed) {
	            	safedValues[counter] = pcmSamples[pcm_index];
	            	counter++;
	            }
	            if (counter >= PCM_SAMPLES_PER_SEC) {
	            	convert();
	            }
	            // safe 1600 values (4000 before 16000-4000 after)
	            // pass those values into mfcc (new function)



	        	/*uint16_t low  = inputBuffer[offset + i];
	        	        uint16_t high = inputBuffer[offset + i + 1];

	        	        uint32_t frame32 = ((uint32_t)high << 16) | low;

	        	        // discard lower 14 bits → keep signed 18-bit
	        	        int32_t sample18 = ((int32_t)frame32) >> 14;

	        	        pcmSamples[pcm_index++] = (int16_t)sample18;
	            */pcm_index++;
	        }
	        else {
	                        // Stop recording once 16000 samples have been collected.
	                        recording_finished = true;
	                        //pcm_index = 0;
	                        //LED_ansteuern(10);
	                        //HAL_I2S_DMAStop(hi2s2);
	                        //getSamples(); // Exit the loop
	                        break;
	        }
	                }
		timer = HAL_GetTick() - timerStart;
	}


/*
void process_audio_buffer(uint32_t offset, I2S_HandleTypeDef* hi2s2)
{
    const uint32_t half_size = DMA_HALF_SIZE;

    for (uint32_t i = 0; i < half_size; i += 4) {


        uint32_t mergedValue =
            (inputBuffer[offset + i] << 16) |
             inputBuffer[offset + i + 1];

        int16_t sample = (int16_t)(mergedValue >> 14);
        uint32_t absVol = abs(sample);


        if (audioState == AUDIO_IDLE) {

            if (absVol > SCHWELLWERT) {
                audioState = AUDIO_RECORDING;
                recordIndex = 0;
            }
        }


        if (audioState == AUDIO_RECORDING) {

            if (recordIndex < PCM_SAMPLES_PER_SEC) {
                safedValues[recordIndex++] = sample;
            }

            if (recordIndex >= PCM_SAMPLES_PER_SEC) {
                audioState = AUDIO_IDLE;
                recording_finished = true;


                stecker_schalten();
            }
        }
    }
}

*/
	uint32_t getCurrentVolume(void) {
		return currentVolume;
	}


/*
void process_audio_buffer(uint32_t offset, I2S_HandleTypeDef* hi2s2)
{
    // process half-buffer
    for (uint32_t i = 0; i < DMA_HALF_SIZE; i += 4) {

        if (pcm_index >= PCM_SAMPLES_PER_SEC) {
            recording_finished = true;
            HAL_I2S_DMAStop(hi2s2);
            return;
        }

        // LEFT channel is stored in halfwords [0] and [1]
        uint16_t low  = inputBuffer[offset + i];
        uint16_t high = inputBuffer[offset + i + 1];

        // Merge into 32-bit I2S frame
        uint32_t frame32 = ((uint32_t)high << 16) | low;

        // Shift according to spec: discard lower 14 bits
        int32_t sample18 = ((int32_t)frame32) >> 14;

        // Convert 18-bit signed → 16-bit signed PCM
        // (sample18 already sign-extended because >> is arithmetic)
        int16_t finalSample = (int16_t)sample18;

        pcmSamples[pcm_index++] = finalSample;
    }
}
*/

	void LED_ansteuern (int messwert) {
		int lights = (messwert * 10)/MAX_MIKRO;
		if(lights > 10) {
			lights = 10;
		}
		if(lights < 0) {
			lights = 0;
		}
		LightUp(lights);
		//delay350Microseconds(1);


	}

	void convert() {
		//TODO convert safeValues to mfcc


	}



