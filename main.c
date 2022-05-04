/***********************************************************************************************************************
* File Name    : main.c
* Project      : RTES Final Course Project
* Version      : 4.0
* Description  : This is the main function that holds all the initialisation functions and holds the main() which is the
*                entry point to the function
* Author       : Sudarshan Jagannathan & Nihal Thirunakarasu referred from
*                https://github.com/akobyl/TM4C129_FreeRTOS_Demo
* Creation Date: 4.19.22
***********************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "main.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"

// TivaWare includes
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "user_task.h"

//Timer headers
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"

/***********************************************************************************************************************
 * Global variables
***********************************************************************************************************************/
uint32_t output_clock_rate_hz;
uint32_t ui32Period, pom = 0;
SemaphoreHandle_t xsemS[NUMBER_SERVICES];

#if (PROJECT == 1)
//For x1 frequency.
#define FREQ_MULTIPLIER      1
#endif

#define SERVICE_1_PERIOD     (10000)
#define SERVICE_2_PERIOD     (20000)
#define SERVICE_3_PERIOD     (80000)

int service_Freq_arr[NUMBER_SERVICES-1] = {10, 1, 1};


/***********************************************************************************************************************
 * Function Prototypes
***********************************************************************************************************************/
void PWM_Init(void);
void setPMW(uint8_t duty);
void GPIO_Init(void);

/***********************************************************************************************
* Name          : main
* Description   : used to create the user task creation function and start the scheduler along
*                 with clock configuration.
* Parameters    : None
* Return        : function return status
***********************************************************************************************/
int main(void)
{
    int dist;
    // Initialize system clock to 120 MHz
    uint32_t output_clock_rate_hz;
    output_clock_rate_hz = ROM_SysCtlClockFreqSet(
                               (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                               SYSTEM_CLOCK);
    ASSERT(output_clock_rate_hz == SYSTEM_CLOCK);

    /* UART configuration */
    UARTStdioConfig(0, 57600, SYSTEM_CLOCK);

    /* TimerA configuration*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ui32Period = ((output_clock_rate_hz / SCALING_FACTOR)/*/1000->ms*/);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

//     Initialize the GPIO pins for the Launchpad
    PinoutSet(false, false);

    GPIO_Init();

    PWM_Init();

    setPMW(99);
    TimerEnable(TIMER0_BASE, TIMER_A);
    IntMasterEnable();

    vStartUserTasks();

    vTaskStartScheduler();
    
    // Code should never reach this point
    return 0;
}

/***********************************************************************************************
* Name          : GPIO Init
* Description   : This routine initialises the GPIO pins to input or output mode. It also sets
*                 the pins in PWM Mode
* Parameters    : None
* Return        : None
***********************************************************************************************/
void GPIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
    {
    }

    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4); // PortNPin4-Trigger
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_5);  // PortPPin5-Echo
    GPIOIntEnable(GPIO_PORTN_BASE, GPIO_INT_PIN_5);
}

/***********************************************************************************************
* Name          : PWM Init
* Description   : This routine initialises PWM Mode by setting the clock and the pin to output
*                 the PWM signal
* Parameters    : None
* Return        : None
***********************************************************************************************/
void PWM_Init(void)
{
    float PWM_FREQ;
    float CPU_FREQ;
    float pwm_word;

    PWM_FREQ = 10000; //10khz
    CPU_FREQ = 120000000;
    pwm_word = (1/PWM_FREQ)*CPU_FREQ;
    float var = 1000-1;

    SysCtlClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_DB_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, pwm_word);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2)*var)/1000);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
}

/***********************************************************************************************
* Name          : setPMW
* Description   : This function sets the PWM duty cycle for the PWM pin. The duty cycle value
*                 passed is in percentage.
* Parameters    : Duty is the percentage of duty cycle.
* RETURN        : None
***********************************************************************************************/
void setPMW(uint8_t duty)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2)*((duty*1000)/100))/1000);
}

/***********************************************************************************************
* Name          : Timer0AIntHandler
* Description   : The timer callback function which executes after every timer expiry for Q3.
*                 This also dispatched the two tasks by releasing task specific semaphores.
* Parameters    : Timer handler of the timer to which the callback is registered.
* RETURN        : None
***********************************************************************************************/
void Timer0AIntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    pom++;

    if((pom % SERVICE_1_PERIOD) == 0)
        xSemaphoreGive(xsemS[1]);

    if((pom % SERVICE_2_PERIOD) == 0)
        xSemaphoreGive(xsemS[2]);

    if((pom % SERVICE_3_PERIOD) == 0)
        xSemaphoreGive(xsemS[3]);
}

/*  ASSERT() Error function
 *
 *  failed ASSERTS() from driverlib/debug.h are executed in this function
 */
void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished

    while (1)
    {
    }
}
