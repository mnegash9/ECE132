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


//function declarations
void portF_input_setup(int input_pin);
void handler_portF(void);
void pwm_setup(void);

//Global Variables
float duty_cycle = .02;
int divider;
int ulPeriod;


//main function
//purpose:
int main(void)
{
    //switch 1 setup (PF4)
    portF_input_setup(0x10);


    //INTERRUPTS PORT F SETUP
    //triggering behavior of GPIO pin F4 interrupt
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    //Registering function as interrupt handler
    GPIOIntRegister(GPIO_PORTF_BASE, handler_portF);
    //Enabling interrupt of GPIO pin F4
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);


    //PWM SETUP
    pwm_setup();


    //enable global interrupts
    IntMasterEnable();


    //infinite loop to keep program running
    while(1);

    return 0;
}



//Defining Functions

//Port F Input Setup Function
void portF_input_setup(int input_pin)
{
    // Enable clock for GPIO Port F
    // The value is 0x20, b00100000
    SYSCTL_RCGCGPIO_R |= 0x20;

    //Tiva Switch 2 Initialization
    GPIO_PORTF_LOCK_R|=0x4C4F434B;
    GPIO_PORTF_CR_R |= input_pin;

    // Setting the pin of Port F to an Input
    GPIO_PORTF_DIR_R &= ~(input_pin);

    // Enable digital function for the pin
    GPIO_PORTF_DEN_R |= input_pin;

    // Enable Pull Up Resistor for the pin
    GPIO_PORTF_PUR_R |= input_pin;

}


//interrupt handler port F
void handler_portF(void)
{
    //Turn off interrupts while performing function
    IntMasterDisable();

    duty_cycle = duty_cycle + .001;
    if (duty_cycle > .13)
    {
        duty_cycle = 0.13;
    }

    //Clear the interrupt flag
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);

    //Turn interrupts back on
    IntMasterEnable();
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
