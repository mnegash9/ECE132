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

//Variable Declarations
#define GPIO_PA0_U0RX           0x00000001
#define GPIO_PA1_U0TX           0x00000401

//function declarations
void portF_input_setup(int input_pin);
void portB_input_setup(int input_pin);
void portB_output_setup(int pin);
void handler_portB(void);
void handler_portF(void);

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

    //INTERRUPTS PORT B SETUP
    //triggering behavior of GPIO pin B4 interrupt
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    //Registering function as interrupt handler
    GPIOIntRegister(GPIO_PORTB_BASE, handler_portB);
    //Enabling interrupt of GPIO pin B4
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_4);


    //UART Setup-----------------------------------------------------------------------------------------------------
    //The following block of code is used to configure UART
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set up Clock
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART hardware
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable Pin hardware
    GPIOPinConfigure(GPIO_PA0_U0RX); // Configure GPIO pin for UART RX line
    GPIOPinConfigure(GPIO_PA1_U0TX); // Configure GPIO Pin for UART TX line
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Set Pins for UART
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, // Configure UART to 8N1 at 115200bps
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    //------------------------------------------------------------------------------------------------------------------------


    //ADC Setup-----------------------------------------------------------------------------------------------------------
    //Peripheral setup for ADC0
    //This will read the voltage off Port E Pin 3
    //This ADC will have the ability to read a voltage between 0mV and 3300 mV
    //The resolution of the measurement is 0.8 mV ((Vp - Vn)/4096)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR,0); //The ADC0 is configured
    //The ADC will support up to 8 samples to be taken
    //The ADC will be triggered to get a sample based on the condition ADCProcessorTrigger()
    //The ADC will have a priority that is higher than other samplings

    ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    //For Step 0 the step configuration is being setup
    //The step configuration is for Channel 0 to be read and this to be the sample that is first in the sequence read
    //It is configured that the ADC interrupt is allowed to happen

    ADCSequenceEnable(ADC0_BASE, 0); //Enable ADC0 with sample sequence number 0
    //----------------------------------------------------------------------------------------------------------


    //PWM SETUP -----------------------------------------------------------------------------------------------
    // Set the clock
    // This Function is on page 493 in the reference manual
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //for 80 MHz

    // Configure PWM Clock to match system
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // Enable the peripherals used by this program.
    // This SysCtlPeripheralEnable function is on page 509 in the reference manual
    // Function call to enable Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Function call to enable the appopriate PWM Module
    // The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // Need to set the divider to something to get the disired period
    // With a divider of 12000, the frequencey is about 12kHz
    // For 15khz frequency, divider should be 15000, which gives us a period of 5333 clock ticks for a 80Mhz clock speed
    divider=15000;
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
    //------------------------------------------------------------------------------------------------------------------

    //enable global interrupts
    IntMasterEnable();



    //infinite loop to keep program running
    while(1)
    {
        //read in ADC:
        ADCProcessorTrigger(ADC0_BASE,0);//Triggers ADC conversion which means it will read from the pin
        ADCSequenceDataGet(ADC0_BASE,0,&INPUT);//Gets the captured data for a sample sequence stores it in INPUT

        int voltage = (INPUT/4096.0)*3300;
    }

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

    /* Check if PB4 triggered the interrupt
    if (status & GPIO_PIN_4)
    {

    }
    */

    //Clear the interrupt flag
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_4);

    //Turn interrupts back on
    IntMasterEnable();
}


//interrupt handler port F
void handler_portF(void)
{
    //Turn off interrupts while performing function
    IntMasterDisable();



    //Clear the interrupt flag
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);

    //Turn interrupts back on
    IntMasterEnable();
}
