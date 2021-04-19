// ECE 3849 Lab 1 source code fragments for copying and pasting

// Step 1: Signal source

// includes / defines

#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz

// initialization in main()

// configure M0PWM2, at GPIO PF2, which is BoosterPack 1 header C1 (2nd from right) pin 2
// configure M0PWM3, at GPIO PF3, which is BoosterPack 1 header C1 (2nd from right) pin 3
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3); // PF2 = M0PWM2, PF3 = M0PWM3
GPIOPinConfigure(GPIO_PF2_M0PWM2);
GPIOPinConfigure(GPIO_PF3_M0PWM3);
GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

// configure the PWM0 peripheral, gen 1, outputs 2 and 3
SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, roundf((float)gSystemClock/PWM_FREQUENCY));
PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f)); // 40% duty cycle
PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
PWMGenEnable(PWM0_BASE, PWM_GEN_1);


// Step 2: ADC sampling

SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO?);
GPIOPinTypeADC(...);          // GPIO setup for analog input AIN3

SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // initialize ADC peripherals
SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
// ADC clock
uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; //round up
ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
ADCSequenceDisable(...);      // choose ADC1 sequence 0; disable before configuring
ADCSequenceConfigure(...);    // specify the "Always" trigger
ADCSequenceStepConfigure(...);// in the 0th step, sample channel 3 (AIN3)
                              // enable interrupt, and make it the end of sequence
ADCSequenceEnable(...);       // enable the sequence.  it is now sampling
ADCIntEnable(...);            // enable sequence 0 interrupt in the ADC1 peripheral
IntPrioritySet(...);          // set ADC1 sequence 0 interrupt priority
IntEnable(...);               // enable ADC1 sequence 0 interrupt in int. controller


// ADC ISR

#define ADC_BUFFER_SIZE 2048                             // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;  // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];           // circular buffer
volatile uint32_t gADCErrors = 0;                        // number of missed ADC deadlines

void ADC_ISR(void)
{
    <...>; // clear ADC1 sequence0 interrupt flag in the ADCISC register
    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
        gADCErrors++;                   // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0;   // clear overflow condition
    }
    gADCBuffer[
               gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
               ] = <...>;               // read sample from the ADC1 sequence 0 FIFO
}


// Step 3: Trigger search

int RisingTrigger(void)   // search for rising edge trigger
{
    // Step 1
    int x = gADCBufferIndex - <...>/* half screen width; don’t use a magic number */;
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--) {
        if (    gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET &&
                <...>/* next older sample */ < ADC_OFFSET)
            break;
    }
    // Step 3
    if (x == x_stop) // for loop ran to the end
        x = <...>; // reset x back to how it was initialized
    return x;
}

// Step 4: ADC sample scaling

int y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)sample - ADC_OFFSET));

float fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv);


// Step 5: Button command processing

const char * const gVoltageScaleStr[] = {
    "100 mV", "200 mV", "500 mV", "  1 V", "  2 V"
};

