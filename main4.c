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

/********************************************
 * Global Variables shared between handlers
 ********************************************/
#define PERIOD 100
#define DUTY_CYCLE_STEP 10
#define INITIAL_DUTY_CYCLE 10
#define AUDIO_COUNT 50

/* Volume and Pitch Adjustments */
uint16_t volume_period = 16;
uint16_t pitch_period = 12;
uint16_t set_volume_period = 16;
uint16_t set_pitch_period = 12;


/* ADC results buffer */
uint16_t resultsBuffer[2];           // latest readings from the analog inputs
uint16_t setBuffer[2];
uint16_t buttonPressed;              // 1 if joystick button is pressed
volatile uint16_t print_flag;                 // flag to signal main to redisplay - set by ADC14


// Variables for refresh of display
unsigned dotcolor;          // color of the dot to display
uint16_t xdisplay,ydisplay; // screen coordinates to disolay

// Color parameters for drawing on the screen - see grlib.h
#define TEXTCOL GRAPHICS_COLOR_YELLOW
#define BACKCOL GRAPHICS_COLOR_BLACK
#define DOTCOL GRAPHICS_COLOR_LIGHT_GREEN
#define DOTCOL_PRESSED GRAPHICS_COLOR_RED
#define RADIUS 2

// Graphics Globals (used by put_dot and ADC14 handler)

Graphics_Context g_sContext;    // graphics context for grlib
uint16_t xscreen, yscreen;      // current screen location coordinates


void put_dot(uint16_t x,uint16_t y, uint32_t dotcolor);

// text printout of joystick readings on the screen
uint16_t vol_and_pitch[2];
void print_current_results(uint16_t *results);

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

// TA0CCR0 Interrupt Handler

// Variables used for time related activities by the handler

volatile unsigned int counter; // down counter for toggling pwm output mode
//volatile unsigned int ad_counter = 2048;
//volatile unsigned int change_counter = 0;

void TA0_0_Handler(){

    volume_period = (resultsBuffer[0] >> 9) + 1;
    pitch_period = (resultsBuffer[1] >> 10) + 4;

    if(buttonPressed){
        set_volume_period = volume_period;
        set_pitch_period = pitch_period;
        setBuffer[0] = resultsBuffer[0];
        setBuffer[1] = resultsBuffer[1];
    }

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);

    if(buttonPressed)
        MAP_Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_4, volume_period);
    else MAP_Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_4, set_volume_period);

    if (--counter == 0){
        TIMER_A0->CCTL[4]^=TIMER_A_CCTLN_OUTMOD_7; // toggle between mode 0 and mode 7 reset/set
        if(buttonPressed)
            counter=pitch_period;
        else counter=set_pitch_period;


    }



}


/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_10,          // 10MHz
        PERIOD-1,                               //
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE ,   // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};
const Timer_A_CompareModeConfig ccr0_Config ={
        TIMER_A_CAPTURECOMPARE_REGISTER_0,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
        TIMER_A_OUTPUTMODE_OUTBITVALUE,
        PERIOD-1
};

const Timer_A_CompareModeConfig ccr4_Config ={
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_RESET_SET,
        INITIAL_DUTY_CYCLE
};

void init_timer(){              // initialization and start of timer
    /* Configuring Timer_A1 for Up Mode */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);

    // configure channels
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &ccr0_Config);
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &ccr4_Config);

    counter=AUDIO_COUNT;

    MAP_Timer_A_registerInterrupt(TIMER_A0_BASE,TIMER_A_CCR0_INTERRUPT,TA0_0_Handler);

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE); // start TA0 in up mode
}

/********************************************************
 * DISPLAY Section
 ********************************************************/

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

    sprintf(string, "v: %5d", results[0]);
    Graphics_drawString(&g_sContext,
                                    (int8_t *)string,
                                    8,
                                    20,
                                    116,
                                    OPAQUE_TEXT);

    sprintf(string, "p: %5d", results[1]);
    Graphics_drawString(&g_sContext,
                                    (int8_t *)string,
                                    8,
                                    76,
                                    116,
                                    OPAQUE_TEXT);
}



void init_display(){
	/*
	 * All init code for the display
	 *
	 */
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
    Graphics_drawString(&g_sContext,
                                    "J:",
                                    AUTO_STRING_LENGTH,
                                    0,
                                    116,
                                    OPAQUE_TEXT);
    xscreen=0;
    yscreen=0;  // just use origin, first write is a background
    setBuffer[0] = 8000;
    setBuffer[1] = 8000;
}





/**********************************
 * Main function
 **********************************/

void main(void)
    {


    /* Halting WDT and disabling master interrupts */
	MAP_CS_setDCOFrequency(10000000); // 10 MHz

	 init_WDT();

    init_display(); // setup the display
    print_flag=0;   //clear print flag until there is a result
    init_ADC();

    MAP_Interrupt_disableSleepOnIsrExit();   // Specify that after an interrupt, the CPU wakes up
    MAP_Interrupt_enableMaster();

    /* Enable Interrupts at the NVIC level */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableInterrupt(INT_WDT_A);

    init_timer();

    while(1)
    {
//        MAP_PCM_gotoLPM0();
        __no_operation(); //  For debugger
        if (print_flag){
            print_flag=0;
            dotcolor=buttonPressed ? DOTCOL_PRESSED: DOTCOL;

            if(buttonPressed){
                vol_and_pitch[0] = volume_period;
                vol_and_pitch[1] = pitch_period;
                xdisplay=resultsBuffer[0]/128;
                ydisplay=127-resultsBuffer[1]/128;
            }else{
                vol_and_pitch[0] = set_volume_period;
                vol_and_pitch[1] = set_pitch_period;
                xdisplay=setBuffer[0]/128;
                ydisplay=127-setBuffer[1]/128;
            }

            print_current_results(vol_and_pitch);
            put_dot(xdisplay,ydisplay,dotcolor);

        }
    }
}

