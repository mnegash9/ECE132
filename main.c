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
void portB_input_setup(int input_pin);
void portB_output_setup(int pin);
void handler_portB(void);

//main function
//purpose:
int main(void)
{
    //sensor 1 setup (PB4)
    portB_input_setup(0x10);

    //LED 1 Setup (PB0)
    portB_output_setup(0x01);
    GPIO_PORTB_DATA_R |= 0x01; //initialize first LED on
    //LED 2 Setup (PB1)
    portB_output_setup(0x02);

    //INTERRUPTS PORT B SETUP
    //triggering behavior of GPIO pin B4 interrupt
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    //Registering function as interrupt handler
    GPIOIntRegister(GPIO_PORTB_BASE, handler_portB);
    //Enabling interrupt of GPIO pin B4
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_4);

    return 0;
}



//Defining Functions

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
