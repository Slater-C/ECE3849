/**
 * main.c
 *
 * ECE 3849 Lab 0 Starter Project
 * Gene Bogdanov    10/18/2017
 *
 * This version is using the new hardware for B2017: the EK-TM4C1294XL LaunchPad with BOOSTXL-EDUMKII BoosterPack.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "buttons.h"

uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 8345; // time in hundredths of a second

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

    uint32_t time;  // local copy of gTime
    char str[50];   // string buffer (for time)
    char str2[50];  // string buffer (for button state)
    // full-screen rectangle
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    ButtonInit();
    IntMasterEnable();

    while (true) {
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black

        uint32_t rollover = 0;
        uint8_t minute = 0;
        uint8_t second = 0;
        uint16_t hSecond = 0;
        time = gTime; // read shared global only once

        minute = ((time - (time % 6000)) / 6000);   // convert time into multiple variables containing minutes, seconds,
        rollover = time % 6000;                     // and hundredths of a second

        second = ((rollover - (rollover % 100)) / 100);
        rollover = rollover % 100;

        hSecond = rollover; //hundredths of a second

        snprintf(str, sizeof(str), "Time = %02u:%02u:%02u", minute, second, hSecond); // convert time to string
        GrContextForegroundSet(&sContext, ClrYellow); // yellow text
        GrStringDraw(&sContext, str, /*length*/ -1, /*x*/ 0, /*y*/ 0, /*opaque*/ false);


        char output[8] = {'0'}; // output buffer for binary buttonstate
        uint32_t buffer = 0;

        int i;
        for(i = 0; i <= 8; i++){
            buffer = ((gButtons >> i) & 0x1);   // take current gButtons value, preserve one bit, place it at bit 0 and mask out other bits
            output[8-i] = buffer + '0';         // write the single bit from above into the output buffer at correct position (8-i flips the order)
        }
        snprintf(str2, sizeof(str2), "Buttons = %s", output); // convert buttonstate to string  //print buttonstate char array to a formatted string
        GrStringDraw(&sContext, str2, /*length*/ -1, /*x*/ 0, /*y*/ 20, /*opaque*/ false);
        GrFlush(&sContext); // flush the frame buffer to the LCD
    }
}
