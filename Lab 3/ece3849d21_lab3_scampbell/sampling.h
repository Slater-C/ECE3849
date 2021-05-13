/*
 * sampling.h
 *
 *  Created on: Apr 13, 2021
 *      Author: Slater Campbell
 */

#ifndef SAMPLING_H_
#define SAMPLING_H_


#define ADC_SAMPLING_RATE 2000000   // [samples/sec] desired ADC sampling rate
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates
#define ADC_INT_PRIORITY 0 // priorities are in steps of 32, button int is priority 32. ADC must be higher priority
                           // so zero is used.

#define ADC_BUFFER_SIZE 2048                             // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro

#define PWM_GEN_PERIOD 258
#define SYSTEM_CLOCK_FREQ 120000000


void ADC_ISR(void);
void sampleInit(void);
void Timer0_ISR(void);
void freqCaptureInit();
void pwmInit(void);
void PWM_ISR(void);


#endif /* SAMPLING_H_ */
