//******************ECE 132*************************
//Names: Ethan Shotwell, Matyas Negash
//Lab section: Friday
//*************************************************
//Date Started: 4/18/2025
//Date of Last Modification:
//Lab Assignment: Project 2
//Lab Due Date:
//*************************************************
//Purpose of program: Create an automated lazy susan via a servo motor that turns it based on user input with a pot and button
//Program Inputs: IR sensor, user switch 1 on Tiva, Potentiometer
//Program Outputs: Green LED, Red LED, Servo Motor
//*************************************************

//libraries
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/watchdog.h"

//function declarations
void setup_adc_potentiometer();
void setup_watchdog();
void pwm_setup(void);
void portB_input_setup(int input_pin);
void portB_output_setup(int pin);
void handler_portB(void);

//variable declarations
float selection;
unsigned int INPUT;
//global variables for PWM
float duty_cycle = .02;
int divider;
int ulPeriod;
volatile bool g_bWatchdogFeed = 1;
//global variables for LED FSM
const int T = 1000;
int input;
int currentState = 0;

//FSM for LEDs
struct state
{
    unsigned char out[2]; //two outputs
    unsigned long wait; //delay
    unsigned char next[4]; //four input options {0, 1, 2, 3}
};
//typedef
typedef struct state stype;
//variable declaration and initialization of FSM
stype fsm[11] = {
    {{0,0}, T, {0, 1, 2, 3}},  //None (0)
    {{1,0}, T, {0, 1, 2, 3}},  //L1 (1)
    {{0,1}, T, {0, 1, 2, 3}},  //L2 (2)
    {{1,1}, T, {0, 1, 2, 3}},  //Both (3)

};


int main(void){
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set up Clock

    
    //sensor 1 setup (PB4)
    portB_input_setup(0x10);

    //LED 1 Setup (PB0)
    portB_output_setup(0x01);
    GPIO_PORTB_DATA_R |= 0x01; //initialize first LED on
    //LED 2 Setup (PB1)
    portB_output_setup(0x02);
    GPIO_PORTB_DATA_R |= 0x02; //initialize second LED on
    
    //INTERRUPTS PORT B SETUP
    //triggering behavior of GPIO pin B4 interrupt
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    //Registering function as interrupt handler
    GPIOIntRegister(GPIO_PORTB_BASE, handler_portB);
    //Enabling interrupt of GPIO pin B4
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_4);

    
    //PWM SETUP
    pwm_setup();
    
    setup_adc_potentiometer();
    while (1){
        // read in ADC:
        ADCProcessorTrigger(ADC0_BASE, 0); //Trigger the ADC conversion for sequence 0
        ADCSequenceDataGet(ADC0_BASE, 0, &INPUT); //Retrieve the ADC conversion result from sequence 0 and store it in INPUT

        // step 1 - selection ranges from 0 to 2 depending on ADC Input
        selection = round((INPUT * 3.3f) / 4.095f / 1650.0f);
    }
    
}

void setup_adc_potentiometer()
{
    // Peripheral setup for ADC0
    // This will read the value off Port E Pin 3
    // The resolution of the measurement is .805 mV ( With a 12-bit ADC (4096 levels), each count represents about 0.805 mV)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0); // The ADC0 is configured
    // The ADC will support up to 8 samples to be taken
    // The ADC will be triggered to get a sample based on the processor trigger condition
    // The ADC will have a priority that is the highest compared to other samplings

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0,
    ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    // For step 0 the step configuration is being setup
    // The step configuration is for Channel 0 to be read and this to be the sample that is the last sample in the sequence read
    // It is configured that the ADC interrupt is allowed to happen

    ADCSequenceEnable(ADC0_BASE, 0); // Enable ADC0 with sample sequence number 0
}

void setup_watchdog()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0))
    {
    }
    IntEnable(INT_WATCHDOG);
    if (WatchdogLockState(WATCHDOG0_BASE) == true)
    {
        WatchdogUnlock(WATCHDOG0_BASE);
    }
    WatchdogIntEnable(WATCHDOG0_BASE);
    WatchdogIntTypeSet(WATCHDOG0_BASE, WATCHDOG_INT_TYPE_INT);
    //every 1 sec if directly from crystal
    WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet() / 3);
//    WatchdogResetEnable(WATCHDOG0_BASE);
    WatchdogLock(WATCHDOG0_BASE);
    WatchdogEnable(WATCHDOG0_BASE);

}
void WatchdogIntHandler(void)
{
    // Clear the watchdog interrupt.
    if (g_bWatchdogFeed)
    {
        WatchdogIntClear(WATCHDOG0_BASE);
    }
}
void feed_watchdog(){
    g_bWatchdogFeed = 0;
}

void pwm_setup(void)
{
    // Set the clock
    // This Function is on page 493 in the reference manual
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //for 80 MHz

    // Configure PWM Clock to match system
    SysCtlPWMClockSet(SYSCTL_PWMDIV_32);

    // Enable the peripherals used by this program.
    // This SysCtlPeripheralEnable function is on page 509 in the reference manual
    // Function call to enable Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Function call to enable the appropriate PWM Module
    // The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // Need to set the divider to something to get the desired period
    // For 15khz frequency, divider should be 15000, which gives us a period of 5333 clock ticks for a 80Mhz clock speed
    divider=1600;
    ulPeriod = 80000000 /divider;

    // Configure PF2 Pins as PWM
    // Information about GPIOPinConfigure is on page 268
    GPIOPinConfigure(GPIO_PF2_M1PWM6);

    // GPIOPinTypePWM performs the task:
    // Information on this function is on page 279
    // Pins F2 are set up as PWM
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 );


    // Configure PWM Options
    // PWM_GEN_3 Covers M1PWM6 and M1PWM7
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the Period (expressed in clock ticks)
    // Use the PWMGenPeriodSet function
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ulPeriod);


    // Use the PWMPulseWidthSet function to set the duty cycle for the signal
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, (ulPeriod * duty_cycle) - 2); //subtract 2 ticks from width to prevent LED from turning off at 100% duty cycle

    // Enable the PWM generator
    // PWMGenEnable information is found on page 427
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    // Turn on the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
}

//Port B Input Setup Function
void portB_input_setup(int input_pin)
{
    // Enable clock for GPIO Port B
    // The value is 0x02, b00000010
    SYSCTL_RCGCGPIO_R |= 0x02;

    // Setting the pin of Port B to an Input
    GPIO_PORTB_DIR_R &= ~(input_pin);

    // Enable digital function for the pin
    GPIO_PORTB_DEN_R |= input_pin;

    // Enable Pull Up Resistor for the pin
    GPIO_PORTB_PUR_R |= input_pin;

}

//portB_output_setup
//void function
//parameters: pin number in hex
void portB_output_setup(int pin)
{
    // Enable clock for GPIO Port F
    // The value is 0x20, b00000010
    SYSCTL_RCGCGPIO_R |= 0x02;

    // Setting the pin of Port B to an Output
    GPIO_PORTB_DIR_R |= pin;

    // Enable digital function for the pin
    GPIO_PORTB_DEN_R |= pin;
}

//interrupt handler port B
void handler_portB(void)
{
    //Turn off interrupts while performing function
    IntMasterDisable();

    //capture which GPIO pin was triggered
    uint32_t status = GPIOIntStatus(GPIO_PORTB_BASE, true);

    // Check if PB4 triggered the interrupt
    if (status & GPIO_PIN_4)
    {

    }

    //Delay for Debounce of Sensors
    SysCtlDelay(1000000);

    //Clear the interrupt flag
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_4);

    //Turn interrupts back on
    IntMasterEnable();
}
