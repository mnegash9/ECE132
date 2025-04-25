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

//function declarations
void setup_adc_potentiometer();
void setup_watchdog();
//variable declarations
float selection;
unsigned int INPUT;

int main(void){
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set up Clock

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


