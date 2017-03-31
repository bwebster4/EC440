/**************************************************************************
 * Joystick Example Version 3 (March 2017)
 *
 * This is a (highly) modified version of the EDU Booster Pack
 * Joystick example from the TI SDK.
 *
 * Operation:
 * The application uses the EDU MKII Joystick and Crystalfont display
 * The Joysick is periodically sampled (by the MSP432 ADC14).
 * The resulting x,y values (between 0 and 2**14-1) are rescaled to 7 bits
 * and used as x,y coordinates on the 128x128 display for drawing a small circle.
 * As in the original Joystick example, the ADC14 readings are also written on the screen.
 *
 * Parameters and Details
 * ======================
 *
 * Connections (from the EDU MKII):
 * ------------------------------
 * Joystick X analog input: P6.0 (A15) [A15 is the default tertiary function of P6.0]
 * Joystick Y analog input: P4.4 (A9)
 * Joystick Button input: P4.1 (not used here, yet)
 *
 * The Crystalfont128x128 display is connected to an SPI communication interface
 * and several gpio pins:
 * LCD SPI CLK:  P1.5 (UCB0CLK)
 * LCD RST:      P5.7
 * LCD SPI MOSI: P1.6 (UCB0SIMO)
 * LCD SPI CS:   P5.0
 * LCD RS PIN:   P3.7
 *
 * The display is managed by a Graphics Library (grlib.h) and
 * an LCD driver for the Crystalfont128x128 display.
 * See the graphics driver library documentation in the MSP432 SDK
 * and the LCD driver code that is part of this project.
 *
 * Timing parameters
 * -----------------
 * The MCLK and SMLK are driven by the DCO at 10MHz (still at VCORE0 voltage levels)
 * We use this faster than default speed mainly to be sure that graphics library
 * has enough headroom for CPU utilization (not sure if this is necessary).
 *
 * The ADC14 could be run in automatic mode to take samples repeatedly (this was
 * done in original TI example), but we choose to trigger individual readings rapidly
 * on a human time scale but not on the CPU time scale by using the WDT with divisor 8K.
 *
 * Software Overview:
 *
 * The WDT interrupt triggers a single ADC14 sequence of channels measurement.
 * The completion of the final ADC channel measurement triggers the ADC14 interrupt
 * The ADC interrupt saves the joystick readings (including the button, and
 * sets a global 'print_flag' indicating a refresh of the screen is needed.
 *
 * After any interrupt, the main program wakes up, checks the print flag and,
 * if it is set, refreshes the display:
 * It computes the screen coordinates for the analog measurements:
 *         xscreen = x/128
 *         yscreen = 127-y/128
 * and draws a solid circle whose color depends on whether the joystick button is pressed.
 **************************************************************************************/
/**************************************************************************
 * Joystick Example Version 3 (March 2017)
 *
 * This is a (highly) modified version of the EDU Booster Pack
 * Joystick example from the TI SDK.
 *
 * Operation:
 * The application uses the EDU MKII Joystick and Crystalfont display
 * The Joysick is periodically sampled (by the MSP432 ADC14).
 * The resulting x,y values (between 0 and 2**14-1) are rescaled to 7 bits
 * and used as x,y coordinates on the 128x128 display for drawing a small circle.
 * As in the original Joystick example, the ADC14 readings are also written on the screen.
 *
 * Parameters and Details
 * ======================
 *
 * Connections (from the EDU MKII):
 * ------------------------------
 * Joystick X analog input: P6.0 (A15) [A15 is the default tertiary function of P6.0]
 * Joystick Y analog input: P4.4 (A9)
 * Joystick Button input: P4.1 (not used here, yet)
 *
 * The Crystalfont128x128 display is connected to an SPI communication interface
 * and several gpio pins:
 * LCD SPI CLK:  P1.5 (UCB0CLK)
 * LCD RST:      P5.7
 * LCD SPI MOSI: P1.6 (UCB0SIMO)
 * LCD SPI CS:   P5.0
 * LCD RS PIN:   P3.7
 *
 * The display is managed by a Graphics Library (grlib.h) and
 * an LCD driver for the Crystalfont128x128 display.
 * See the graphics driver library documentation in the MSP432 SDK
 * and the LCD driver code that is part of this project.
 *
 * Timing parameters
 * -----------------
 * The MCLK and SMLK are driven by the DCO at 10MHz (still at VCORE0 voltage levels)
 * We use this faster than default speed mainly to be sure that graphics library
 * has enough headroom for CPU utilization (not sure if this is necessary).
 *
 * The ADC14 could be run in automatic mode to take samples repeatedly (this was
 * done in original TI example), but we choose to trigger individual readings rapidly
 * on a human time scale but not on the CPU time scale by using the WDT with divisor 8K.
 *
 * Software Overview:
 *
 * The WDT interrupt triggers a single ADC14 sequence of channels measurement.
 * The completion of the final ADC channel measurement triggers the ADC14 interrupt
 * The ADC interrupt saves the joystick readings (including the button, and
 * sets a global 'print_flag' indicating a refresh of the screen is needed.
 *
 * After any interrupt, the main program wakes up, checks the print flag and,
 * if it is set, refreshes the display:
 * It computes the screen coordinates for the analog measurements:
 *         xscreen = x/128
 *         yscreen = 127-y/128
 * and draws a solid circle whose color depends on whether the joystick button is pressed.
 **************************************************************************************/
// Full set of include files including graphics and driver libraries
// and also the LCD driver which is part of the project itself
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/********************************************
 * Global Variables shared between handlers
 ********************************************/

uint16_t resultsBuffer[2];           // latest readings from the analog inputs
uint16_t buttonPressed;              // 1 if joystick button is pressed
uint16_t print_flag;                 // flag to signal main to redisplay - set by ADC14
uint16_t receive_flag;
uint16_t receiveBuffer[2];
uint16_t count;
uint16_t needsTransmit;

/***************************************************************
 * WDT system
 * The WDT is used to trigger sampling the ADC every 8192 cycles
 * (interval = 8192/10MHz = .8192ms)
 ***************************************************************/

void WDT_A_IRQHandler(void)
{
    MAP_ADC14_toggleConversionTrigger(); // trigger the next conversion
}

void init_WDT(){
    MAP_WDT_A_initIntervalTimer(WDT_A_CLOCKSOURCE_SMCLK,WDT_A_CLOCKITERATIONS_8192);
    MAP_WDT_A_startTimer(); // start the timer
}

/*************************************************
 * ADC14 Subsystem
 *************************************************/
/*
 * ADC Interrupt handler
 * This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer
 */
void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    /* ADC_MEM1 conversion completed */
    if(status & ADC_INT1)
    {
        /* Store ADC14 conversion results */
        resultsBuffer[0] = MAP_ADC14_getResult(ADC_MEM0);
        resultsBuffer[1] = MAP_ADC14_getResult(ADC_MEM1);

        /* Determine if JoyStick button is pressed */
        buttonPressed = (P4IN & GPIO_PIN1)?0:1;

        print_flag=1;  // signal main to refresh the display
    }
}

/*
 * ADC Setup
 */
void init_ADC(){
    /* Configure Pin 6.0 (A15) and 4.4 (A9) to be analog inputs ('tertiary function') */
    /* see the port 4 and port 6 pinout details of the MSP432p401r */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8)
     * drive from the internal ASD oscillator
     * with predivider 64 and divider 8, no routing to internal pins
     */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, ADC_NOROUTE);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
     * Basic operation is sequence mode with
     *   ADC-MEM0 -- A15
     *   ADC_MEM1 -- A9
     *
     *   NO automatic repeats
     */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false); // use MEM...MEM1 channels
    // configure each memory channel:

    MAP_ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

    /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
     * is complete and enabling conversions
     */
    MAP_ADC14_enableInterrupt(ADC_INT1);

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Enable conversions (must be triggered using ASC14_toggle_conversion()) */
    MAP_ADC14_enableConversion();
    //MAP_ADC14_toggleConversionTrigger();

}

/********************************************************
 * DISPLAY Section
 ********************************************************/

// Color parameters for drawing on the screen - see grlib.h
#define TEXTCOL GRAPHICS_COLOR_YELLOW
#define BACKCOL GRAPHICS_COLOR_BLACK
#define DOTCOL GRAPHICS_COLOR_LIGHT_GREEN
#define DOTCOL_PRESSED GRAPHICS_COLOR_RED
#define RADIUS 2

// Graphics Globals (used by put_dot and ADC14 handler)

Graphics_Context g_sContext;    // graphics context for grlib
uint16_t xscreen, yscreen;      // current screen location coordinates


// Draw a dot (small circle) on the screen at position x,y of color dotcolor
// also ERASE previous dot (remembered in globals xscreen, yscreen)
void put_dot(uint16_t x,uint16_t y, uint32_t dotcolor){
    // erase previous dot
    Graphics_setForegroundColor(&g_sContext, BACKCOL);
    Graphics_fillCircle(&g_sContext,xscreen,yscreen,RADIUS);
    // draw the requested circle
    Graphics_setForegroundColor(&g_sContext, dotcolor);
    Graphics_fillCircle(&g_sContext,x,y,RADIUS);
    xscreen=x;
    yscreen=y;
}

// text printout of joystick readings on the screen
void print_current_results(uint16_t *results){
    char string[8];

    Graphics_setForegroundColor(&g_sContext, TEXTCOL);
    sprintf(string, "X: %5d", results[0]);
    Graphics_drawString(&g_sContext,(int8_t *)string,8,20,116,OPAQUE_TEXT);
    sprintf(string, "Y: %5d", results[1]);
    Graphics_drawString(&g_sContext,(int8_t *)string,8,76,116,OPAQUE_TEXT);
}

void init_display(){
    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,&g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, TEXTCOL);
    Graphics_setBackgroundColor(&g_sContext, BACKCOL);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawString(&g_sContext,"J:",AUTO_STRING_LENGTH,0,116,OPAQUE_TEXT);
    xscreen=0;
    yscreen=0;  // just use origin, first write is a background
}


const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                     // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

/**********************************
 * Main function
 **********************************/

void main(void)
{
    // Variables for refresh of display
    unsigned dotcolor;          // color of the dot to display
    uint16_t xdisplay,ydisplay; // screen coordinates to disolay

    /* Halting WDT and disabling master interrupts */
    MAP_CS_setDCOFrequency(10000000); // 10 MHz


    /* Selecting P3.2 and P3.3 in UART mode */
   MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,
                GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
//   MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,
//               GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    init_WDT();

    init_display(); // setup the display
    print_flag=0;   //clear print flag until there is a result
    receive_flag=0;
    count = 0;
    needsTransmit = 0;
    init_ADC();

    MAP_Interrupt_disableSleepOnIsrExit();   // Specify that after an interrupt, the CPU wakes up
    MAP_Interrupt_enableMaster();

    /* Enable Interrupts at the NVIC level */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableInterrupt(INT_WDT_A);


/* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
//    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableMaster();



    while(1)
    {
        MAP_PCM_gotoLPM0();
        __no_operation(); //  For debugger
        if (print_flag | receive_flag)
        {
            dotcolor=buttonPressed ? DOTCOL_PRESSED: DOTCOL;
            xdisplay=resultsBuffer[0]/128;
            ydisplay=127-resultsBuffer[1]/128;

//            print_current_results(resultsBuffer);

            if(print_flag){
                uint8_t data = resultsBuffer[0] >> 7;
                MAP_UART_transmitData(EUSCI_A0_BASE, data);
                needsTransmit = 1;
            }
            uint16_t tempBuffer[2];
            tempBuffer[0] = resultsBuffer[0] >> 7;
            tempBuffer[1] = resultsBuffer[1] >> 7;
            print_current_results(tempBuffer);

            put_dot(xdisplay,ydisplay,dotcolor);

            print_flag = 0;
            receive_flag = 0;

        }
    }
}


/* EUSCI A0 UART ISR - Echoes data back to PC host */
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        //MAP_UART_transmitData(EUSCI_A0_BASE, MAP_UART_receiveData(EUSCI_A0_BASE));
        switch(count){
        case 0:
            receiveBuffer[0] = MAP_UART_receiveData(EUSCI_A0_BASE);
            break;
        case 1:
            receiveBuffer[0] += MAP_UART_receiveData(EUSCI_A0_BASE) << 8;
            break;
        case 2:
            receiveBuffer[1] = MAP_UART_receiveData(EUSCI_A0_BASE);
            break;
        case 3:
            receiveBuffer[1] += MAP_UART_receiveData(EUSCI_A0_BASE) << 8;
            receive_flag = 1;
            break;
        }

        count++;
        if(count > 3)
            count = 0;

        MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    }

    if((status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG) && needsTransmit){
        uint8_t data =  resultsBuffer[1] >> 7;
        MAP_UART_transmitData(EUSCI_A0_BASE, data);
        needsTransmit = 0;
    }

}
