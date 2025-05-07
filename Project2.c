//******************ECE 132*************************
//Names: Ethan Shotwell, Matyas Negash
//Lab section: Friday
//*************************************************
//Date Started: 5/6/2025
//Date of Last Modification:
//Lab Assignment: Project 2
//Lab Due Date: 5/9/2025
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
void WatchdogIntHandler(void);

//global variables for ADC
int selection; //determines potentiometer selection
unsigned int INPUT; //stores ADC value
//global variables for PWM
float duty_cycle = 0.21; //duty cycle
int divider;
int ulPeriod;
//global variable for watchdog
volatile bool g_bWatchdogFeed = 1; //feeds watchdog
bool watchdog_running; //checks state of watchdog
//global variables for LED FSM
const int T = 10; //delay for FSM
int input; //FSM input
int currentState = 0; //current state of FSM
//global variables for IR Sensor
volatile bool ir_triggered = false; //checks if IR sensor was triggered

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
    IntPrioritySet(INT_WATCHDOG, 0x00);  // high priority
    IntPrioritySet(INT_GPIOB, 0x10);    //medium priority
    IntPrioritySet(INT_GPIOF, 0x20);  // low priority


    //PWM SETUP
    pwm_setup();

    //ADC Setup
    setup_adc_potentiometer();

    //UART Setup
    setup_uart();

    //watchdog setup
    setup_watchdog();
    watchdog_running = WatchdogRunning(WATCHDOG0_BASE); //check if watchdog is running

    //enable global interrupts
    IntMasterEnable();

    //message length for UART
    int mess_len;
    //Console Output
    char message[] = "Position: ";
    mess_len = sizeof(message) / sizeof(message[0]); //finds length

    while (1)
    {
        // read in ADC:
        ADCProcessorTrigger(ADC0_BASE, 0); //Trigger the ADC conversion for sequence 0
        ADCSequenceDataGet(ADC0_BASE, 0, &INPUT); //Retrieve the ADC conversion result from sequence 0 and store it in INPUT

        // step 1 - selection ranges from 0 to 2 depending on ADC Input
        selection = round((INPUT * 3.3f) / 4.095f / 1650.0f);
        // Step 2 - output to UART
        int i;
        for (i = 0; i < mess_len; i++)
        {
            UARTCharPut(UART0_BASE, message[i]); //outputs each character in array
        }
        UARTCharPut(UART0_BASE, selection + '0'); //outputs selection determined by potentiometer
        UARTCharPut(UART0_BASE, '\r'); //returns cursor position to beginning

        //update LED states
        led_update(ir_triggered, false);
        if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4))
        {
            ir_triggered = false;  // sensor cleared
        }

        //feed watchdog
        feed_watchdog();

        //small delay
        SysCtlDelay(50);

    }

}


//Functions

//potentiometer ADC setup
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

//watchdog setup
void setup_watchdog()
{
    //enable peripheral for watchdog
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    //waits for peripheral to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0))
    {
    }
    //enables watchdog interrupt
    IntEnable(INT_WATCHDOG);
    //unlocks watchdog
    if (WatchdogLockState(WATCHDOG0_BASE) == true)
    {
        WatchdogUnlock(WATCHDOG0_BASE);
    }
    //enables watchdog interrupt
    WatchdogIntEnable(WATCHDOG0_BASE);
    //sets watchdog interrupt type
    WatchdogIntTypeSet(WATCHDOG0_BASE, WATCHDOG_INT_TYPE_INT);
    //sets reload to around every 10 sec
    WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet() * 10);
    //set handler for watchdog
    WatchdogIntRegister(WATCHDOG0_BASE, WatchdogIntHandler);
    //reset system when watchdog is not fed
    WatchdogResetEnable(WATCHDOG0_BASE);
    //locks watchdog
    WatchdogLock(WATCHDOG0_BASE);
    //enables watchdog
    WatchdogEnable(WATCHDOG0_BASE);

}

//watchdog interrupt handler
void WatchdogIntHandler(void)
{

    // Clear the watchdog interrupt.
    if (g_bWatchdogFeed)
    {
        WatchdogIntClear(WATCHDOG0_BASE);
    }
    else
    {
        int mess_len;
        //Console Output
        UARTCharPut(UART0_BASE, '\n');
        UARTCharPut(UART0_BASE, '\r');
        char message[] = "SYSTEM RESETTING, CHECK IR SENSOR!";
        mess_len = sizeof(message) / sizeof(message[0]);

        int i;
        for (i = 0; i < mess_len; i++)
        {
            UARTCharPut(UART0_BASE, message[i]); //outputs every character in array
        }
        UARTCharPut(UART0_BASE, '\n'); //new line
        UARTCharPut(UART0_BASE, '\r'); //reset cursor
    }

    //primes watchdog for next trigger
    g_bWatchdogFeed = 0;

}

//feed watchdog
void feed_watchdog()
{
    //feed watchdog
    g_bWatchdogFeed = 1;
}

//UART setup
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

//PWM setup
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

    //change status of IR trigger
    ir_triggered = true;

    //output warning message
    int mess_len;
    //Console Output
    UARTCharPut(UART0_BASE, '\n');
    UARTCharPut(UART0_BASE, '\r');
    char message[] = "OBJECT IN PATH, PLEASE CHECK!";
    mess_len = sizeof(message) / sizeof(message[0]);

    int i;
    for (i = 0; i < mess_len; i++)
    {
        UARTCharPut(UART0_BASE, message[i]); //output every character of array
    }
    UARTCharPut(UART0_BASE, '\n');
    UARTCharPut(UART0_BASE, '\r');

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

//controls motor logic using PWM
void motor_move(void)
{
    //makes sure function can be interrupted by IR sensor
    IntMasterEnable();

    //sub cycle represents small increments of duty_cycle
    float sub_cycle;
    //old duty cycle is used to calculate duty cycle difference between it and the new duty cycle
    float old_cycle = duty_cycle;

    //change duty_cycle depending on what the potentiometer selects
    if (selection == 0)
    {
       duty_cycle = .021; //first position
    }
    if (selection == 1)
    {
       duty_cycle = .07; //second position
    }
    if (selection == 2)
    {
        duty_cycle = .12; //third position
    }
    else;

    //update LED states
    led_update(ir_triggered, true);

    //create incremental steps in PWM for motor to slow turning speed
    //check if new duty_cycle is bigger or smaller
    if (old_cycle > duty_cycle)
    {
        int i;
        for(i=0; i<100; i++)
        {
            //if ir sensor is triggered, stop motor
            while(ir_triggered)
            {
                //update LED states
                led_update(ir_triggered, true);
                //clear trigger when IR sensor is not detected
                if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4))
                {
                    ir_triggered = false;  // sensor cleared
                }
                //update LED states
                led_update(ir_triggered, true);
            }
            //create sub_cycle increments (100 increments)
            sub_cycle = (old_cycle - duty_cycle)/100;
            //set new PWM duty_cycle
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, (ulPeriod * (old_cycle - (sub_cycle*i))));
            //delay to slow down motor
            SysCtlDelay(500000);
        }
    }
    if (old_cycle < duty_cycle)
    {
        int i;
        for(i=0; i<100; i++)
        {
            //if ir sensor is triggered, stop motor
            while(ir_triggered)
            {
                //update LED states
                led_update(ir_triggered, true);
                //clear trigger when IR sensor is not detected
                if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4))
                {
                    ir_triggered = false;  // sensor cleared
                }
                //update LED states
                led_update(ir_triggered, true);
            }
            //create sub_cycle increments (100 increments)
            sub_cycle = (duty_cycle - old_cycle)/100;
            //set new PWM duty_cycle
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, (ulPeriod * (old_cycle + (sub_cycle*i))));
            //delay to slow down motor
            SysCtlDelay(500000);
        }
    }

    //feed watchdog
    feed_watchdog();

}

//function to update LEDs
void led_update(bool ir, bool motor)
{
    //input for FSM
    int input;

    //Determines state of FSM depending on motor function and IR sensor detection
    if (ir == true & motor == true)
    {
        input = 3; //both LEDs on
    }
    else if (ir == false & motor == true)
    {
        input = 2; //motor LED on
    }
    else if (ir == true & motor == false)
    {
        input = 1; //IR LED on
    }
    else if (ir == false & motor == false)
    {
        input = 0; //both LEDs off
    }

    //change current state of FSM to next state
    currentState = fsm[currentState].next[input];

    //depending on FSM output states, turn LEDs on or off
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

    //delay for FSM
    SysCtlDelay(fsm[currentState].wait);
}
