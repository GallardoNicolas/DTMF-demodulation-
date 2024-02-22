/*
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    TP FINAL.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "arm_math.h"


#define ADC16_BASE 			ADC0
#define ADC16_CHANNEL_GROUP	0U
#define PIT_BASEADDR 		PIT
#define SAMPLERATE		    (uint32_t)14470
#define BUFFERSIZE 		    (uint32_t)4096
#define BLOCKSIZE			1
#define TONE1				697
#define TONE2				770
#define TONE3 				852
#define TONE4 				941
#define TONE5 				1209
#define TONE6				1336
#define TONE7				1447


float32_t hanning_window[BUFFERSIZE];	 //ventaneo de hanning
float32_t input_signal[BUFFERSIZE];		//señal de microfono o software
float32_t fft_input[BUFFERSIZE];		//buffer con los datos de entrada luego de hacer ventaneo
float32_t fft_magnitud[BUFFERSIZE/2]; //buffer con la magnitud del espectro
float32_t fft_output[BUFFERSIZE]; //Buffer con FFT
float32_t max_value=0;			//valor maximo
uint32_t max_indice=0;			//indice de posicion
float32_t max_value2=0;			//valor maximo
uint32_t max_indice2=0;
arm_rfft_fast_instance_f32 fftinit;
uint16_t fftlength = 4096;
float32_t Bo[BUFFERSIZE];
float32_t sintone1[BUFFERSIZE];
float32_t sintone2[BUFFERSIZE];
float32_t sintone3[BUFFERSIZE];
float32_t sintone4[BUFFERSIZE];
float32_t sintone5[BUFFERSIZE];
float32_t sintone6[BUFFERSIZE];
float32_t sintone7[BUFFERSIZE];
uint8_t boton=0;
uint8_t inter=0;;


void GenSin(void);
void Gentonos(void);
int main(void) {

    /* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	BOARD_InitButtonsPins();
	BOARD_InitPins();
	BOARD_InitLEDsPins();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif
    //hanning window
   /* for(int i=0; i<fftlength; i++) {
    hanning_window[i] =
    0.5f * (1.0f - cosf(2.0f * PI * i / (float)(fftlength - 1)));
    }*/
    arm_rfft_fast_init_f32(&fftinit, fftlength);
    Gentonos();
    while(inter==0){
    inter=1;
    GenSin();




    //arm_mult_f32(input_signal, hanning_window, fft_input, fftlength);//a la señal de entrada la ventaneo
    arm_rfft_fast_f32(&fftinit, &Bo[0], &fft_output[0], 0);
    arm_cmplx_mag_f32(&fft_output[0], &fft_magnitud[0], BUFFERSIZE/2 );
    fft_magnitud[0] = 0;
    fft_magnitud[1] = 0;
    arm_max_f32(&fft_magnitud[0], BUFFERSIZE/2, &max_value, &max_indice);
    fft_magnitud[max_indice] = 0;
    arm_max_f32(&fft_magnitud[0], BUFFERSIZE/2, &max_value2, &max_indice2);

    if(max_indice==197){
    	if(max_indice2==342){
    		PRINTF("el boton presionado es : 1\r\n");
    	}else if(max_indice2==378){
    		PRINTF("el boton presionado es : 2\r\n");
    	}else if(max_indice2==410){
    		PRINTF("el boton presionado es : 3\r\n");
    	}}
    if(max_indice==218){
    	  if(max_indice2==342){
      		PRINTF("el boton presionado es : 4\r\n");
 	   	}else if(max_indice2==378){
       		PRINTF("el boton presionado es : 5\r\n");
       	}else if(max_indice2==410){
       		PRINTF("el boton presionado es : 6\r\n");
    	    	}}
     if(max_indice==241){
     	  if(max_indice2==342){
     		 PRINTF("el boton presionado es : 7\r\n");
    	 }else if(max_indice2==378){
    		 PRINTF("el boton presionado es : 8\r\n");
    	 }else if(max_indice2==410){
    		 PRINTF("el boton presionado es : 9\r\n");
    	      	}}
     if(max_indice==266){
    	  if(max_indice2==342){
    		  PRINTF("el boton presionado es : *\r\n");
     	 }else if(max_indice2==378){
     		PRINTF("el boton presionado es : 0\r\n");
     	 }else if(max_indice2==410){
     		PRINTF("el boton presionado es : #\r\n");
     	     	}
     }
     while(inter==1){}
}  }

/* La frecuencia de muestreo es 20 [k/s] */

//void PIT_CHANNEL_0_IRQHANDLER(void) {
//  uint32_t intStatus;
//  /* Reading all interrupt flags of status register */
 // intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0);
 // PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, intStatus);

  //Adquisicion de datos del microfono
  //input_signal[0] = (float32_t)(ADC16_GetChannelConversionValue(ADC16_BASE, ADC16_CHANNEL_GROUP)); //Adquirimos los datos del adc y casteamos
//}

/* PORTA_IRQn interrupt handler */
void GPIOA_IRQHANDLER(void) {
  /* Get pin flags */
  uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOA);

  /* Place your interrupt code here */
boton++;
inter=0;
if(boton==12)
	boton=0;
  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOA, pin_flags);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}




void GenSin(void){

	switch (boton){
	case 0:
		arm_add_f32(&sintone4, &sintone6, Bo, fftlength);
		break;
	case 1:
		arm_add_f32(&sintone1, &sintone5, Bo, fftlength);
		break;
	case 2:
		arm_add_f32(&sintone1, &sintone6, Bo, fftlength);
			break;
	case 3:
		arm_add_f32(&sintone1, &sintone7, Bo, fftlength);
			break;
	case 4:
		arm_add_f32(&sintone2, &sintone5, Bo, fftlength);
			break;
	case 5:
		arm_add_f32(&sintone2, &sintone6, Bo, fftlength);
			break;
	case 6:
		arm_add_f32(&sintone2, &sintone7, Bo, fftlength);
			break;
	case 7:
		arm_add_f32(&sintone3, &sintone5, Bo, fftlength);
			break;
	case 8:
		arm_add_f32(&sintone3, &sintone6, Bo, fftlength);
			break;
	case 9:
		arm_add_f32(&sintone3, &sintone7, Bo, fftlength);
			break;
	case 10:
		arm_add_f32(&sintone4, &sintone5, Bo, fftlength);
			break;
	case 11:
		arm_add_f32(&sintone4, &sintone7, Bo, fftlength);
			break;
	}
}

void Gentonos(void)
{
	for(int i=0; i<fftlength; i++) {
	sintone1[i] = arm_sin_f32(2.0f*PI*TONE1*i/SAMPLERATE);
	sintone2[i]= arm_sin_f32(2.0f*PI*TONE2*i/SAMPLERATE);
	sintone3[i]= arm_sin_f32(2.0f*PI*TONE3*i/SAMPLERATE);
	sintone4[i]= arm_sin_f32(2.0f*PI*TONE4*i/SAMPLERATE);

	sintone5[i]= 0.8f*arm_sin_f32(2.0f*PI*TONE5*i/SAMPLERATE);
	sintone6[i]= 0.8f*arm_sin_f32(2.0f*PI*TONE6*i/SAMPLERATE);

	sintone7[i]= 0.8f*arm_sin_f32(2.0f*PI*TONE7*i/SAMPLERATE);
	}
}
