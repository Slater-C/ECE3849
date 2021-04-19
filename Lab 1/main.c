
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
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

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define ADC_OFFSET 2048 //ADC offset is half the size of ADC resolution

#define VIN_RANGE 3.3       // ADC uses 3.3V and GND Vref
#define PIXELS_PER_DIV 20   // 20 pixels of the LCD per grid square/voltage division
#define ADC_BITS 12         // ADC is 12 bit, giving range 0-4095

void signalInit(void);
int risingTrigger(void);
int fallingTrigger(void);
void copyBuffer(int triggerIndex);
void scaleWave(float fVoltsPerDiv);
void drawWave(tContext *sContext);
void drawBackground(tContext *sContext, tRectangle *rectFullScreen);
void triggerIcon(tContext *sContext, bool triggerMode);
void adjustVoltageScale(bool adj);
void displaySettings(tContext *sContext);
void cpu_load_init(void);
void displayCPUuse(tContext *sContext);
uint32_t cpu_load_count(void);

uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 0; // time in hundredths of a second

extern volatile int32_t gADCBufferIndex;  // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];     // circular buffer for ADC

extern volatile char fifo[FIFO_SIZE];
extern int fifo_get(char* data);

volatile uint16_t rawWaveBuffer[LCD_HORIZONTAL_MAX];    // circular buffer for one screen width
volatile int displayWaveBuffer[LCD_HORIZONTAL_MAX];    // circular buffer for scaled pixels of the wave
float voltageScale = 1;

// CPU load counters
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;


int main(void)
{
    IntMasterDisable();

    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    // full-screen rectangle
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    sampleInit();
    ButtonInit();
    signalInit();
    cpu_load_init();

    count_unloaded = cpu_load_count();
    IntMasterEnable();

    //Fifo char
    char c;

    //Trigger config
    bool triggerMode = 0;
    

    while (true) {
        

        if(fifo_get(&c)){    // Read out whole buffer
            if(c == 68){   //joystick down
                //voltage scale down
                adjustVoltageScale(1);
            }
            else if(c == 85){  //joystick up
                //voltage scale up
                adjustVoltageScale(0);
            }
            else if(c == 83){  //Joystick press (select)
                //change trigger
                triggerMode = !triggerMode;
            }
        }

        drawBackground(&sContext, &rectFullScreen);

        int trigger;
        if(!triggerMode){
            trigger = risingTrigger();
        }
        else if(triggerMode){
            trigger = fallingTrigger();
        }
        
        copyBuffer(trigger);
        scaleWave(voltageScale);
        drawWave(&sContext);
        triggerIcon(&sContext, triggerMode);
        displaySettings(&sContext);

        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load
        displayCPUuse(&sContext);

        GrFlush(&sContext); // flush the frame buffer to the LCD
    }
}

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
    for (xPixel = 0; xPixel < (LCD_HORIZONTAL_MAX-1); xPixel++){
        GrLineDraw(sContext, xPixel, displayWaveBuffer[xPixel], xPixel+1, displayWaveBuffer[xPixel+1]);
    }

    /*
    This function takes an argument of type tContext, which is a pointer to the display buffer used in the graphics drivers.
    It then uses the global displayWaveBuffer[], which contains scaled values filled by scaleWave(), and draws a line between each coordinate
    to make a smooth display.
    */
}

void drawBackground(tContext *sContext, tRectangle *rectFullScreen){
    GrContextForegroundSet(sContext, ClrBlack);
    GrRectFill(sContext, rectFullScreen); // fill screen with gray

    GrContextForegroundSet(sContext, ClrDarkSlateGray);
    uint8_t line;
    for (line = 0; line < 6; line++){
        GrLineDrawH(sContext, 0, LCD_HORIZONTAL_MAX, 4+(20*line));
        GrLineDrawV(sContext, 4+(20*line), 0, LCD_VERTICAL_MAX);
    }

    GrContextForegroundSet(sContext, ClrSlateGray);
    GrLineDrawH(sContext, 0, LCD_HORIZONTAL_MAX, LCD_VERTICAL_MAX/2);
    GrLineDrawV(sContext, LCD_HORIZONTAL_MAX/2, 0, LCD_VERTICAL_MAX);

    /*
    This function takes two arguments: one is a pointer to the display buffer used by the graphics driver, and the other
    is a pointer to type tRectangle that sets the background size of the screen. The function then fills the background with black,
    then draws slate gray grid lines separated by 20 pixels. Next, it draws the X/Y axis lines on top in a lighter color.
    This function should be called before drawWave() and any of the UI draw functions, that way the grid lines will be rendered underneath.
    */
}

void triggerIcon(tContext *sContext, bool triggerMode){
    #define X_POS 104 // X coordinate of bottom left corner of icon
    #define Y_POS 7   // Y coordinate of bottom left corner of icon
    
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

void displaySettings(tContext *sContext){
    char str[30];   // Voltage setting string buffer
    
    if(voltageScale < 1){
        snprintf(str, sizeof(str), "%d mV", (int)(voltageScale * 1000)); // convert time to string
    }
    else{
        snprintf(str, sizeof(str), "%d V", (int)voltageScale); // convert time to string
    }

    GrContextForegroundSet(sContext, ClrWhite);
    GrContextBackgroundSet(sContext, ClrNavy);
    GrStringDrawCentered(sContext, str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2, /*y*/ 4, /*opaque*/ true);
    GrStringDraw(sContext, "20 us", /*length*/ -1, /*x*/ 4, /*y*/ 1, /*opaque*/ true);

    /*
    This function takes an argument of type tContext, which is a pointer to the display buffer used in the graphics drivers. It then formats
    the current voltage scale, adds the appropriate prefix for mV or V, and prints it to the screen with a background. It also prints the 
    current timescale, but this is a constant value currently.
    */

}

void displayCPUuse(tContext *sContext){
    char str[50];   // CPU usage string buffer
    GrContextForegroundSet(sContext, ClrWhite);
    GrContextBackgroundSet(sContext, ClrDarkSlateGray);
    
    snprintf(str, sizeof(str), "CPU: %3.2f%%", cpu_load*100); // convert time to string
    GrStringDrawCentered(sContext, str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2, /*y*/ 120, /*opaque*/ true);

    /*
    This function takes an argument of type tContext, which is a pointer to the display buffer used in the graphics drivers. It then formats
    the current CPU usage, and writes it to the bottom of the display with a background.
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

void cpu_load_init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock/100 - 1); // 10 msec interval

    /*
    This function configures Timer3 in one shot mode, and loads the timer to a ten millisecond countdown.
    */
}

uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;

    /*
    This function counts for 10 msec, while being interrupted by the ADC ISR and button ISR. The difference in
    counts found here and found at the start of main will tell the current CPU load. 
    */
}

