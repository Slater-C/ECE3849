/*
 * ECE 3849 Lab 2
 *
 * Slater Campbell    4/26/2021
 */


/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"

/* Program Header Files */
#include "driverlib/sysctl.h"
#include "Crystalfontz128x128_ST7735.h"
#include "driverlib/timer.h"
#include <stdio.h>
#include "buttons.h"
#include "sampling.h"
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

/* Program Definitions */
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define ADC_OFFSET 2048 //ADC offset is half the size of ADC resolution

#define VIN_RANGE 3.3       // ADC uses 3.3V and GND Vref
#define PIXELS_PER_DIV 20   // 20 pixels of the LCD per grid square/voltage division
#define ADC_BITS 12         // ADC is 12 bit, giving range 0-4095

/* Global Variables */
uint32_t gSystemClock = 120000000; // [Hz] system clock frequency

extern volatile int32_t gADCBufferIndex;  // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];     // circular buffer for ADC

volatile uint16_t rawWaveBuffer[LCD_HORIZONTAL_MAX];    // circular buffer for one screen width
volatile int displayWaveBuffer[LCD_HORIZONTAL_MAX];    // circular buffer for scaled pixels of the wave
float voltageScale = 1;
bool triggerMode = 0;
bool spectrumMode = 0;

volatile uint16_t spectrumWaveBuffer[1024];    // circular buffer for FFT spectrum calculations
volatile float out_dB[128];

/* Function Definitions */
void signalInit(void);
int risingTrigger(void);
int fallingTrigger(void);
void copyBuffer(int triggerIndex);
void scaleWave(float fVoltsPerDiv);
void drawWave(tContext *sContext);
void drawBackground(tContext *sContext, tRectangle *rectFullScreen, bool spectrum);
void modeIcon(tContext *sContext, bool triggerMode, bool spectrumMode);
void adjustVoltageScale(bool adj);
void displaySettings(tContext *sContext, bool spectrumMode);
void buttonSchedule_func(void);
void copySpectrumBuffer(void);


/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    // hardware initialization goes here
    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    sampleInit();
    signalInit();
    ButtonInit();


    /* Start BIOS */
    BIOS_start();

    return (0);
}

void waveformTask_func(UArg arg1, UArg arg2)
{
    // High priority task, finds a trigger and copies the waveform from the ADC buffer.
    // Signals the processing task.

    IntMasterEnable();
    int trigger;

    while (true) {

        Semaphore_pend(waveSem0, BIOS_WAIT_FOREVER);

        if(!spectrumMode){
            if(!triggerMode){
                trigger = risingTrigger();
            }
            else if(triggerMode){
                trigger = fallingTrigger();
            }
            copyBuffer(trigger);
        }

        else if (spectrumMode){
            copySpectrumBuffer();
        }

        Semaphore_post(processSem0);

    }
}

void processingTask_func(UArg arg1, UArg arg2)
{
    // Lowest priority task, takes the copied waveform and processes/scales it.
    // Signals the display task and waveform task.

    #define PI 3.14159265358979f
    #define NFFT 1024 // FFT length
    #define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg; // Kiss FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT]; // complex waveform and spectrum buffers
    int i;

    static float w[NFFT]; // window function
    for (i = 0; i < NFFT; i++) {
        // Blackman window
        w[i] = 0.42f - 0.5f * cosf(2*PI*i/(NFFT-1)) + 0.08f * cosf(4*PI*i/(NFFT-1));
    }

    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT

    while (true) {

        Semaphore_pend(processSem0, BIOS_WAIT_FOREVER);

        if(!spectrumMode){
            scaleWave(voltageScale);
        }

        else if (spectrumMode){
            for (i = 0; i < NFFT; i++) { // generate an input waveform
                in[i].r = spectrumWaveBuffer[i] * w[i]; // copy real part of waveform to local buffer
                in[i].i = 0; // imaginary part of waveform
            }
            kiss_fft(cfg, in, out); // compute FFT
            // convert first 128 bins of out[] to dB for display
            for (i = 0; i < 128; i++) { // generate an input waveform
                out_dB[i] = 10 * log10f(out[i].r * out[i].r + out[i].i * out[i].i);
            }

        }


        Semaphore_post(waveSem0);
        Semaphore_post(displaySem0);
    }

}

void displayTask_func(UArg arg1, UArg arg2)
{
    // Low priority. Displays the processed waveform and settings.
    // Does not signal any task.


    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    while (true) {

        Semaphore_pend(displaySem0, BIOS_WAIT_FOREVER);

        drawBackground(&sContext, &rectFullScreen, spectrumMode);
        drawWave(&sContext);
        modeIcon(&sContext, triggerMode, spectrumMode);
        displaySettings(&sContext, spectrumMode);
        GrFlush(&sContext); // flush the frame buffer to the LCD
    }
}

void buttonTask_func(UArg arg1, UArg arg2)
{
    // High priority. Calls the function that reads the buttons.

    while (true){
        Semaphore_pend(buttonSem0, BIOS_WAIT_FOREVER);
        ButtonRoutine();
    }
}

void userInputTask_func(UArg arg1, UArg arg2)
{
    // Medium priority. Pends on the mailbox and processes inputs from it.

    while(true){

        char msg = 0;
        Mailbox_pend(buttonMailbox, &msg, BIOS_WAIT_FOREVER);

        if(msg != 0){
            if(msg == 'D'){   //joystick down
                //voltage scale down
                adjustVoltageScale(1);
            }
            else if(msg == 'U'){  //joystick up
                //voltage scale up
                adjustVoltageScale(0);
            }
            else if(msg == 'S'){  //Joystick press (select)
                //toggle trigger mode
                triggerMode = !triggerMode;
            }
            else if(msg == 'M'){    // User button 1
                //toggle scope mode
                spectrumMode = !spectrumMode;
            }
        }
        Semaphore_post(displaySem0);    // Signal display for update
    }
}



void buttonSchedule_func(void)
{
    // Called by the clock. Posts to the button reading semaphore.

    Semaphore_post(buttonSem0);
}

/*  OSCILLOSCOPE FUNCTIONS  */

int risingTrigger(void)   // search for rising edge trigger
{
    // Step 1
    int32_t localBufferIndex = gADCBufferIndex;
    int x = localBufferIndex - (LCD_HORIZONTAL_MAX/2); // half screen width; don’t use a magic number
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--) {
        if (    gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET &&
                gADCBuffer[ADC_BUFFER_WRAP(x-1)]/* next older sample */ < ADC_OFFSET)
            break;
    }
    // Step 3
    if (x == x_stop) // for loop ran to the end
        x = localBufferIndex; // reset x back to how it was initialized.
                              // use a local copy, because the global one will have changed.
    return x;

    /*
    This function takes no arguments, and returns the location of the nearest trigger that will
    produce a valid display.
    First, the global buffer index used by the ADC ISR is atomically read into a local variable.
    Then, trigger search begins half a display width behind that index, so if a trigger is found immediately,
    it will have enough surrounding data to fill the screen. Trigger search is done by traversing the global buffer
    backwards, looking for an index where the value is above the ADC_OFFSET, and where the index before it is below
    ADC_OFFSET. When this location is found, it is returned by the function. If no valid trigger can be found
    after traversing half of the global buffer, then the local buffer index that was saved at the start will be used
    as the trigger. This results in an unsynchronized waveform.
    */
}

int fallingTrigger(void)   // search for rising edge trigger
{
    // Step 1
    int32_t localBufferIndex = gADCBufferIndex;
    int x = localBufferIndex - (LCD_HORIZONTAL_MAX/2); // half screen width; don’t use a magic number
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--) {
        if (    gADCBuffer[ADC_BUFFER_WRAP(x)] <= ADC_OFFSET &&
                gADCBuffer[ADC_BUFFER_WRAP(x-1)]/* next older sample */ > ADC_OFFSET)
            break;
    }
    // Step 3
    if (x == x_stop) // for loop ran to the end
        x = localBufferIndex; // reset x back to how it was initialized.
                              // use a local copy, because the global one will have changed.
    return x;

    /*
    This function takes no arguments, and returns the location of the nearest trigger that will
    produce a valid display.
    First, the global buffer index used by the ADC ISR is atomically read into a local variable.
    Then, trigger search begins half a display width behind that index, so if a trigger is found immediately,
    it will have enough surrounding data to fill the screen. Trigger search is done by traversing the global buffer
    backwards, looking for an index where the value is below the ADC_OFFSET, and where the index before it is above
    ADC_OFFSET. When this location is found, it is returned by the function. If no valid trigger can be found
    after traversing half of the global buffer, then the local buffer index that was saved at the start will be used
    as the trigger. This results in an unsynchronized waveform.
    */
}


void copyBuffer(int triggerIndex){
    uint16_t xPixel;
    int32_t bufferIndex = ADC_BUFFER_WRAP(triggerIndex - (LCD_HORIZONTAL_MAX/2));    // Start half a screen width left of the buffer
    for (xPixel = 0; xPixel < LCD_HORIZONTAL_MAX; xPixel++){
        rawWaveBuffer[xPixel] = gADCBuffer[ADC_BUFFER_WRAP(bufferIndex + xPixel)];   // Move pixel by pixel from the left edge of the screen to the right
        // xPixel is the X coordinate the sample will take.
        // xPixel is also the index of the buffer. Will take the values 0 to LCD_HORIZONTAL_MAX-1.
    }

    /*
    This function takes the newly found trigger as it's only argument. It then saves half a screen width
    of data on either side of the trigger from the global ADC buffer to a separate global buffer, used only by
    the main() loop. This is accomplished by taking the trigger, subtracting half the display width, and then
    wrapping that value, giving the index of what will be the leftmost pixel of the screen. Then, it traverses
    pixel by pixel for a full screen width, copying the raw ADC output to rawWaveBuffer[] used by main();.
    */
}

void scaleWave(float fVoltsPerDiv){
    uint16_t xPixel;
    float fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv);
    for (xPixel = 0; xPixel < LCD_HORIZONTAL_MAX; xPixel++){
        int yPixel = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)rawWaveBuffer[xPixel] - ADC_OFFSET));
        displayWaveBuffer[xPixel] = yPixel;
    }

    /*
    This function takes an argument of the voltage per division in the form of a float. It then uses the raw ADC output
    in rawWaveBuffer[] to produce a scaled output in displayWaveBuffer[], where the index is the x coordinate and the value
    is the y coordinate. Each time it is called, it calculates the scale factor based on the fVoltsPerDiv argument and then
    uses that scale factor for the entirety of rawWaveBuffer[].
    */
}

void drawWave(tContext *sContext){
    uint16_t xPixel;

    GrContextForegroundSet(sContext, ClrChartreuse);
    if(!spectrumMode){
        for (xPixel = 0; xPixel < (LCD_HORIZONTAL_MAX-1); xPixel++){
            GrLineDraw(sContext, xPixel, displayWaveBuffer[xPixel], xPixel+1, displayWaveBuffer[xPixel+1]);
        }
    }
    if(spectrumMode){
        for (xPixel = (LCD_HORIZONTAL_MAX-1); xPixel > 0 ; xPixel--){
            GrLineDraw(sContext, xPixel, 128 - out_dB[xPixel], xPixel+1, 128-out_dB[xPixel+1]);
        }
    }

    /*
    This function takes an argument of type tContext, which is a pointer to the display buffer used in the graphics drivers.
    It then uses the global displayWaveBuffer[], which contains scaled values filled by scaleWave(), and draws a line between each coordinate
    to make a smooth display.
    */
}

void drawBackground(tContext *sContext, tRectangle *rectFullScreen, bool spectrum){
    GrContextForegroundSet(sContext, ClrBlack);
    GrRectFill(sContext, rectFullScreen); // fill screen with gray
    GrContextForegroundSet(sContext, ClrDarkSlateGray);

    uint8_t xOffset = 4;
    uint8_t yOffset = 4;
    if(spectrum){
        xOffset = 0;
        yOffset = 8;
    }

    uint8_t line;
    for (line = 0; line < 7; line++){
        GrLineDrawH(sContext, 0, LCD_HORIZONTAL_MAX, yOffset+(20*line));
        GrLineDrawV(sContext, xOffset+(20*line), 0, LCD_VERTICAL_MAX);
    }

    if(spectrum){
        GrContextForegroundSet(sContext, ClrSlateGray);
        GrLineDrawH(sContext, 0, LCD_HORIZONTAL_MAX, LCD_VERTICAL_MAX-1);
        GrLineDrawV(sContext, 0, 0, LCD_VERTICAL_MAX);
    }
    else if(!spectrum){
        GrContextForegroundSet(sContext, ClrSlateGray);
        GrLineDrawH(sContext, 0, LCD_HORIZONTAL_MAX, LCD_VERTICAL_MAX/2);
        GrLineDrawV(sContext, LCD_HORIZONTAL_MAX/2, 0, LCD_VERTICAL_MAX);
    }


    /*
    This function takes two arguments: one is a pointer to the display buffer used by the graphics driver, and the other
    is a pointer to type tRectangle that sets the background size of the screen. The function then fills the background with black,
    then draws slate gray grid lines separated by 20 pixels. Next, it draws the X/Y axis lines on top in a lighter color.
    This function should be called before drawWave() and any of the UI draw functions, that way the grid lines will be rendered underneath.
    */
}

void modeIcon(tContext *sContext, bool triggerMode, bool spectrumMode){
    #define X_POS 104 // X coordinate of bottom left corner of icon
    #define Y_POS 7   // Y coordinate of bottom left corner of icon


    if(!spectrumMode){

        tRectangle iconBackground = {X_POS-1, Y_POS-7, X_POS+13, Y_POS+2};
        GrContextForegroundSet(sContext, ClrDarkRed);
        GrRectFill(sContext, &iconBackground);


        GrContextForegroundSet(sContext, ClrWhite);
        if(!triggerMode){
            GrLineDrawH(sContext, X_POS, X_POS+5, Y_POS+1);             //Lower horizontal line
            GrLineDrawH(sContext, X_POS+7, X_POS+12, Y_POS-6);          //Upper horizontal line
            GrLineDrawV(sContext, X_POS+6, Y_POS+1, Y_POS-6);           //Vertical line
            GrLineDraw(sContext, X_POS+6, Y_POS-4, X_POS+8, Y_POS-2);   //Arrow right side
            GrLineDraw(sContext, X_POS+6, Y_POS-4, X_POS+4, Y_POS-2);   //Arrow left side
        }

        if(triggerMode){
            GrLineDrawH(sContext, X_POS, X_POS+5, Y_POS-6);             //Upper horizontal line
            GrLineDrawH(sContext, X_POS+7, X_POS+12, Y_POS+1);          //Lower horizontal line
            GrLineDrawV(sContext, X_POS+6, Y_POS+1, Y_POS-6);           //Vertical line
            GrLineDraw(sContext, X_POS+6, Y_POS-1, X_POS+8, Y_POS-3);   //Arrow right side
            GrLineDraw(sContext, X_POS+6, Y_POS-1, X_POS+4, Y_POS-3);   //Arrow left side
        }
    }

    if(spectrumMode){
        tRectangle iconBackground = {X_POS-1, Y_POS-7, X_POS+18, Y_POS+2};
        GrContextForegroundSet(sContext, ClrDarkRed);
        GrRectFill(sContext, &iconBackground);
        GrContextForegroundSet(sContext, ClrWhite);

        GrStringDraw(sContext, "FFT", /*length*/ -1, /*x*/ X_POS, /*y*/ Y_POS-6, /*opaque*/ false);
    }

    /*
    This function takes two arguments: one is a pointer to the display buffer used by the graphics driver, and the other is a boolean
    containing the current trigger mode being used. It then draws a red rectangle, and on top of it an icon for the respective trigger
    mode is drawn. This icon is composed of a series of line draw operations.
    */
}

void adjustVoltageScale(bool adj){
    static int mode = 0;
    float voltageScales[5] = {2, 1, 0.5, 0.2, 0.1};

    if(adj && (mode < 4)){
        mode++;
    }
    if(!adj && (mode > 0)){
        mode--;
    }

    voltageScale = voltageScales[mode];

    /*
    This function takes a boolean argument requesting to change the voltage scale in a certain direction. If that direction is valid, it
    will change the voltage scale between the preset values in an array, and write them to a global float.
    */
}

void displaySettings(tContext *sContext, bool spectrumMode){

    GrContextForegroundSet(sContext, ClrWhite);
    GrContextBackgroundSet(sContext, ClrNavy);

    if(!spectrumMode){
        char str[30];   // Voltage setting string buffer

        if(voltageScale < 1){
            snprintf(str, sizeof(str), "%d mV", (int)(voltageScale * 1000)); // convert time to string
        }
        else{
            snprintf(str, sizeof(str), "%d V", (int)voltageScale); // convert time to string
        }
        GrStringDrawCentered(sContext, str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2, /*y*/ 4, /*opaque*/ true);
        GrStringDraw(sContext, "20 us", /*length*/ -1, /*x*/ 4, /*y*/ 1, /*opaque*/ true);
    }

    if(spectrumMode){
        GrStringDrawCentered(sContext, "20 dBV", /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2, /*y*/ 4, /*opaque*/ true);
        GrStringDraw(sContext, "20 kHz", /*length*/ -1, /*x*/ 4, /*y*/ 1, /*opaque*/ true);

    }

    /*
    This function takes an argument of type tContext, which is a pointer to the display buffer used in the graphics drivers. It then formats
    the current voltage scale, adds the appropriate prefix for mV or V, and prints it to the screen with a background. It also prints the
    current timescale, but this is a constant value currently.
    */

}



/*  SPECTRUM (FFT) FUNCTIONS  */

void copySpectrumBuffer(void){
    int32_t localBufferIndex = gADCBufferIndex;

    uint16_t indexOffset;
    localBufferIndex = ADC_BUFFER_WRAP(localBufferIndex - NFFT);    // Start half a screen width left of the buffer
    for (indexOffset = 0; indexOffset < NFFT; indexOffset++){
        spectrumWaveBuffer[indexOffset] = gADCBuffer[ADC_BUFFER_WRAP(localBufferIndex + indexOffset)];
    }

    /*
        This function copies the most recent 1024 samples from the ADC buffer to a separate global variable, to be processed by the FFT code.
    */


}


void signalInit(void){

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

    /*
    This function creates the square wave signal. It was provided in the starter code.
    */

}
