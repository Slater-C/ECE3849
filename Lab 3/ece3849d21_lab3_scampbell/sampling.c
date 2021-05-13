/*
 * sampling.c
 *
 *  Created on: Apr 13, 2021
 *      Author: Slater Campbell
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
//#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "audio_waveform.h"

#include "inc/tm4c1294ncpdt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "driverlib/udma.h"
#include "sampling.h"

#define DMA

#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required
tDMAControlTable gDMAControlTable[64];     // uDMA control table (global)

// ADC ISR
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;  // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];           // circular buffer
volatile uint32_t gADCErrors = 0;                        // number of missed ADC deadlines
volatile bool gDMAPrimary = true; // is DMA occurring in the primary channel?

// Frequency timer ISR
volatile uint32_t signalPeriod = 0;

// PWM globals
uint32_t gPWMSample = 0;            // PWM sample counter
uint32_t gSamplingRateDivider = 20; // sampling rate divider

/*  SIGNAL ADC FUNCTIONS  */

void ADC_ISR(void)
{
    #ifndef DMA

        ADC1_ISC_R = ADC_ISC_IN0; // clear ADC1 sequence0 interrupt flag in the ADCISC register

        if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
            gADCErrors++;                   // count errors
            ADC1_OSTAT_R = ADC_OSTAT_OV0;   // clear overflow condition
        }
        gADCBuffer[
                gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
                ] = ADC1_SSFIFO0_R;  // read sample from the ADC1 sequence 0 FIFO

    #endif

    #ifdef DMA

        ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS0); // clear the ADC1 sequence 0 DMA interrupt flag

        // Check the primary DMA channel for end of transfer, and restart if needed.
        if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) == UDMA_MODE_STOP) {
            uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
            UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
            (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2); // restart the primary channel (same as setup)

            gDMAPrimary = false;    // DMA is currently occurring in the alternate buffer
        }

        // Check the alternate DMA channel for end of transfer, and restart if needed.
        // Also set the gDMAPrimary global.
        if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) == UDMA_MODE_STOP) {
            uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
            UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
            (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2); // restart the primary channel (same as setup)

            gDMAPrimary = true;    // DMA is currently occurring in the primary buffer
        }

        // The DMA channel may be disabled if the CPU is paused by the debugger.
        if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10)) {
            uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);  // re-enable the DMA channel
        }

    #endif
}



void sampleInit(void){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);          // GPIO setup for analog input AIN3

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; //round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCSequenceDisable(ADC1_BASE, 0);      // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);    // specify the "Always" trigger. Used relative priority of zero, changed button sequence to priority 1.
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_END | ADC_CTL_IE);// in the 0th step, sample channel 3 (AIN3)
                                                                                      // enable interrupt, and make it the end of sequence
    ADCSequenceEnable(ADC1_BASE, 0);           // enable the sequence.  it is now sampling
    #ifndef DMA
        ADCIntEnable(ADC1_BASE, 0);            // enable sequence 0 interrupt in the ADC1 peripheral
    #endif


    #ifdef DMA

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        uDMAEnable();
        uDMAControlBaseSet(gDMAControlTable);

        uDMAChannelAssign(UDMA_CH24_ADC1_0); // assign DMA channel 24 to ADC1 sequence 0
        uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);

        // primary DMA channel = first half of the ADC buffer
        uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
        UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
        (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);

        // alternate DMA channel = second half of the ADC buffer
        uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
        UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
        (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);

        uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);

    #endif

    #ifdef DMA
        ADCSequenceDMAEnable(ADC1_BASE, 0);             // enable DMA for ADC1 sequence 0
        ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0);     // enable ADC1 sequence 0 DMA interrupt
    #endif

}

/*  SIGNAL FREQUENCY MEASUREMENT FUNCTIONS  */

void freqCaptureInit(){

    // configure GPIO PD0 as timer input T0CCP0 at BoosterPack Connector #1 pin 14
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff);   // use maximum load value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff); // use maximum prescale value
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);

}

void Timer0_ISR(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT); // clear interrupt flag

    static uint32_t prevTimerValue = 0;
    uint32_t timerValue = TimerValueGet(TIMER0_BASE, TIMER_A);
    signalPeriod = (timerValue - prevTimerValue) & 0xFFFFFF;

    prevTimerValue = timerValue;
}



/*  PWM AUDIO FUNCTIONS  */

void pwmInit(void){

    // configure M0PWM5, at GPIO PG1

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1); // PG1 = M0PWM5
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    // configure the PWM0 peripheral, gen 2, output 5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWM_GEN_PERIOD);  // initialize to a constant 258 clock cycle period
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, roundf((float)PWM_GEN_PERIOD*0.5f)); // 50% duty cycle
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    gSamplingRateDivider = (SYSTEM_CLOCK_FREQ)/(AUDIO_SAMPLING_RATE * PWM_GEN_PERIOD);

    /*
    This function creates a PWM signal to be controlled with audio.
    */

}

void PWM_ISR(void)
{
    PWMGenIntClear(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO); // clear PWM interrupt flag
    //PWM0_ISC_R = PWM_0_ISC_INTCNTZERO;                    // clear PWM interrupt flag with direct register access

    int i = (gPWMSample++) / gSamplingRateDivider; // waveform sample index
    PWM0_2_CMPB_R = 1 + gWaveform[i]; // write directly to the PWM compare B register
    if (i > gWaveformSize) { // if at the end of the waveform array
        PWMIntDisable(PWM0_BASE, PWM_INT_GEN_2); // disable these interrupts
        gPWMSample = 0; // reset sample index so the waveform starts from the beginning
    }
}
