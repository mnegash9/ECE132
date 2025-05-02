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

//definitions for UART
#define GPIO_PA0_U0RX           0x00000001
#define GPIO_PA1_U0TX           0x00000401

//function declarations
void setup_adc_potentiometer();
void setup_watchdog();
void setup_uart();
void pwm_setup(void);
void portB_input_setup(int input_pin);
void portB_output_setup(int pin);
void handler_portB(void);
void portF_input_setup(int input_pin);
void handler_portF(void);
void motor_move(void);
void led_update(bool ir, bool motor);
void feed_watchdog();

//global variables for ADC
int selection;
unsigned int INPUT;
//global variables for UART
int mess_len;
int i;
//global variables for PWM
float duty_cycle = .021;
int divider;
int ulPeriod;
//global variable for watchdog
volatile bool g_bWatchdogFeed = 1;
//global variables for LED FSM
const int T = 10;
int input;
int currentState = 0;
//global variables for IR Sensor
volatile bool ir_triggered = false;

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
stype fsm[4] = {
    {{0,0}, T, {0, 1, 2, 3}},  //None (0)
    {{1,0}, T, {0, 1, 2, 3}},  //L1 (1)
    {{0,1}, T, {0, 1, 2, 3}},  //L2 (2)
    {{1,1}, T, {0, 1, 2, 3}},  //Both (3)

};


int main(void){
    // Set the clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //for 50 MHz

    //switch 1 setup
    portF_input_setup(0x10);

    //sensor 1 setup (PB4)
    portB_input_setup(0x10);

    //LED 1 Setup (PB0)
    portB_output_setup(0x01);
    GPIO_PORTB_DATA_R &= ~(0x01); //initialize first LED off
    //LED 2 Setup (PB1)
    portB_output_setup(0x02);
    GPIO_PORTB_DATA_R &= ~(0x02); //initialize second LED off

    //INTERRUPTS PORT B SETUP
    //triggering behavior of GPIO pin B4 interrupt
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    //Registering function as interrupt handler
    GPIOIntRegister(GPIO_PORTB_BASE, handler_portB);
    //Enabling interrupt of GPIO pin B4
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_4);

    //INTERRUPTS PORT F SETUP
    //triggering behavior of GPIO pin F4 interrupt
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    //Registering function as interrupt handler
    GPIOIntRegister(GPIO_PORTF_BASE, handler_portF);
    //Enabling interrupt of GPIO pin F4
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);

    //set priority for interrupts
    IntPrioritySet(INT_GPIOB, 0x00);  // high priority
    IntPrioritySet(INT_GPIOF, 0x20);  // low priority


    //PWM SETUP
    pwm_setup();


    //ADC Setup
    setup_adc_potentiometer();
    //UART Setup
    setup_uart();


    //enable global interrupts
    IntMasterEnable();

    //Console Output
    char message[] = "Position: ";
    mess_len = sizeof(message) / sizeof(message[0]);

    while (1)
    {
        // read in ADC:
        ADCProcessorTrigger(ADC0_BASE, 0); //Trigger the ADC conversion for sequence 0
        ADCSequenceDataGet(ADC0_BASE, 0, &INPUT); //Retrieve the ADC conversion result from sequence 0 and store it in INPUT

        // step 1 - selection ranges from 0 to 2 depending on ADC Input
        selection = round((INPUT * 3.3f) / 4.095f / 1650.0f);
        // Step 2 - output to UART
        for (i = 0; i < mess_len; i++)
        {
            UARTCharPut(UART0_BASE, message[i]);
        }
        UARTCharPut(UART0_BASE, selection + '0');
        UARTCharPut(UART0_BASE, '\r');

        //update LED states
        led_update(ir_triggered, false);
        if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4))
        {
            ir_triggered = false;  // sensor cleared
        }
        //feed watchdog
        feed_watchdog();

        SysCtlDelay(10);

    }

}

void setup_adc_potentiometer(void)
{
    // Peripheral setup for ADC0
    // This will read the value off Port E Pin 3
    // The resolution of the measurement is .805 mV ( With a 12-bit ADC (4096 levels), each count represents about 0.805 mV)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0); // The ADC0 is configured
    // The ADC will support up to 8 samples to be taken
    // The ADC will be triggered to get a sample based on the processor trigger condition
    // The ADC will have a priority that is the highest compared to other samplings

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
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
    //WatchdogResetEnable(WATCHDOG0_BASE);
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
void feed_watchdog()
{
    g_bWatchdogFeed = 1;
}

void setup_uart()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);         // Enable UART hardware
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);          // Enable Pin hardware
    GPIOPinConfigure(GPIO_PA0_U0RX);      // Configure GPIO pin for UART RX line
    GPIOPinConfigure(GPIO_PA1_U0TX);      // Configure GPIO Pin for UART TX line
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Set Pins for UART
    UARTConfigSetExpClk(
    UART0_BASE,
                        SysCtlClockGet(),
                        115200, // Configure UART to 8N1 at 115200bps
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void pwm_setup(void)
{
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
    // For 50hz frequency, divider should be 1600 (since 50*32 = 1600)
    divider=1600;
    ulPeriod = 50000000/divider;

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
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, (ulPeriod * duty_cycle));

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

    //Delay for Debounce of Sensors
    SysCtlDelay(100000);

    ir_triggered = true;

    //Clear the interrupt flag
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_4);

    //Turn interrupts back on
    IntMasterEnable();
}

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
    //move the motor to new position based on selection
    motor_move();

    //Clear the interrupt flag
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
}


void motor_move(void)
{
    IntMasterEnable();

    float sub_cycle;
    float old_cycle = duty_cycle;

    if (selection == 0)
    {
       duty_cycle = .021;
    }
    if (selection == 1)
    {
       duty_cycle = .07;
    }
    if (selection == 2)
    {
        duty_cycle = .12;
    }
    else;

    led_update(ir_triggered, true);

    if (old_cycle > duty_cycle)
    {
        int i;
        for(i=0; i<100; i++)
        {
            while(ir_triggered)
            {
                led_update(ir_triggered, true);
                if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4))
                {
                    ir_triggered = false;  // sensor cleared
                }
                led_update(ir_triggered, true);
            }
            sub_cycle = (old_cycle - duty_cycle)/100;
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, (ulPeriod * (old_cycle - (sub_cycle*i))));
            SysCtlDelay(500000);
        }
    }
    if (old_cycle < duty_cycle)
    {
        int i;
        for(i=0; i<100; i++)
        {
            while(ir_triggered)
            {
                led_update(ir_triggered, true);
                if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4))
                {
                    ir_triggered = false;  // sensor cleared
                }
                led_update(ir_triggered, true);
            }
            sub_cycle = (duty_cycle - old_cycle)/100;
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, (ulPeriod * (old_cycle + (sub_cycle*i))));
            SysCtlDelay(500000);
        }
    }

    //feed watchdog
    feed_watchdog();

}

void led_update(bool ir, bool motor)
{
    int input;

    if (ir == true & motor == true)
    {
        input = 3;
    }
    else if (ir == false & motor == true)
    {
        input = 2;
    }
    else if (ir == true & motor == false)
    {
        input = 1;
    }
    else if (ir == false & motor == false)
    {
        input = 0;
    }

    currentState = fsm[currentState].next[input];

    if(fsm[currentState].out[0] == 1)
    {
        GPIO_PORTB_DATA_R |= 0x01;
    }
    if(fsm[currentState].out[1] == 1)
    {
        GPIO_PORTB_DATA_R |= 0x02;
    }
    if(fsm[currentState].out[0] == 0)
    {
        GPIO_PORTB_DATA_R &= ~(0x01);
    }
    if(fsm[currentState].out[1] == 0)
    {
        GPIO_PORTB_DATA_R &= ~(0x02);
    }

    SysCtlDelay(fsm[currentState].wait);
}
