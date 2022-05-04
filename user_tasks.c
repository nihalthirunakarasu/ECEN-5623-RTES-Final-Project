/***********************************************************************************************************************
* File Name    : user_tasks.c
* Project      : RTES Final Course Project
* Version      : 4.0
* Description  : This is the main function that holds all the initialisation functions and holds the main() which is the
*                entry point to the function
* Author       : Sudarshan Jagannathan & Nihal Thirunakarasu referred from
*                https://github.com/akobyl/TM4C129_FreeRTOS_Demo
* Creation Date: 4.22.22
***********************************************************************************************************************/
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"


#include <stdio.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"
#include "user_task.h"

#include "semphr.h"

/***********************************************************************************************************************
 * Global variables
***********************************************************************************************************************/
int idx = 0, jdx = 1;

#define DIST_SENSOR_TIMEOUT     5000 // 30ms
#define WARNING_LOW (50)

static void vService_1(void* pvParameters);
static void vService_2(void* pvParameters);
static void vService_3(void* pvParameters);

static void calculate_wcet(uint32_t start, uint32_t end,uint32_t *WCET,uint8_t id);
static int getDistance(void);

//static void delay(int n);

uint32_t WCET_arr[NUMBER_SERVICES];
uint32_t ServiceNumber_arr[NUMBER_SERVICES] = {0, 0, 0, 0, 0, 0, 0};

QueueHandle_t xQueue1;
int var1 = 0;
int start_time = 0;
int g_distance_sensed = 0;

SemaphoreHandle_t xsemS1,xsemS2;

/***********************************************************************************************
* Name          : calculate_wcet
* Description   : The function calculates the WCET of the start and stop time and stores it in
*                 an array.
* Parameters    : Start time
*                 Stop time
*                 Pointer to a variable
*                 ID to pass the service number
* Return        : Function return status
***********************************************************************************************/
void calculate_wcet(uint32_t start, uint32_t end, uint32_t *WCET, uint8_t id)
{

    uint32_t Exec_t = 0;

    ServiceNumber_arr[id-1]++;

    Exec_t = (end - start)*(10);

    if(Exec_t > *WCET)
    {
        *WCET = Exec_t;
        taskENTER_CRITICAL();
        WCET_arr[id-1] = Exec_t;

        taskEXIT_CRITICAL();
    }

}

/***********************************************************************************************
* Name          : vStartUserTasks
* Description   : This function creates the threads for each of the tasks. This function also
*                 assigns the priorities to the threads that are created.
* Parameters    : None
* Return        : None
***********************************************************************************************/
void vStartUserTasks(void)
{
    int i;

    TimerHandle_t xTimerCheck = NULL;
    TaskHandle_t xServiceHandle = NULL;
    BaseType_t xReturn;

    /* Led configuration */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);


    //Timer created properly
    if(xTimerCheck != NULL){
        xTimerStart( xTimerCheck, 0);
    }

    /* Semaphore creation*/
    for(i=0;i<=NUMBER_SERVICES;i++){

        xsemS[i] = xSemaphoreCreateBinary();
        //check for error
        if(xsemS[i] == NULL){
            UARTprintf("Bin semaphore creation error!\n");
            for(;;);
        }
    }

    /* Create timer user task1 here*/
    xReturn = xTaskCreate( vService_1,
                           (const char *)"user_timer_task1",
                           (configMINIMAL_STACK_SIZE * 2),
                           ( void * ) 1,
                           ( tskIDLE_PRIORITY + 4 ),
                           &xServiceHandle );

    if(xReturn != pdPASS){//User timer task creation failed
        vTaskDelete(xServiceHandle);
    }

    /* Create timer user task2 here*/
    xReturn = xTaskCreate( vService_2,
                           (const char *)"user_timer_task2",
                           (configMINIMAL_STACK_SIZE * 2),
                           ( void * ) 1,
                           ( tskIDLE_PRIORITY + 2 ),
                           &xServiceHandle );

    if(xReturn != pdPASS){//User timer task creation failed
        vTaskDelete(xServiceHandle);
    }

    /* Create timer user task3 here*/
    xReturn = xTaskCreate( vService_3,
                           (const char *)"user_timer_task3",
                           (configMINIMAL_STACK_SIZE * 2),
                           ( void * ) 1,
                           ( tskIDLE_PRIORITY + 3 ),
                           &xServiceHandle );

    if(xReturn != pdPASS){//User timer task creation failed
        vTaskDelete(xServiceHandle);
    }
}

/***********************************************************************************************
 * Name          : vService_1
 * Description   : This task gets the distance value measured by the ultra-sound sensor
 * Parameters    : Thread parameters
 * RETURN        : None
 ***********************************************************************************************/
static void vService_1(void* pvParameters){

    TickType_t xStartTime,xEndTime;

//    taskENTER_CRITICAL();
//    UARTprintf("---Starting Service_1---\n");
//    taskEXIT_CRITICAL();

    uint32_t WCET = 0;
    int sensed_distance = 0;

    unsigned long long S1Cnt=0;

    while(1)
    {
        if( xSemaphoreTake(xsemS[1], portMAX_DELAY) == pdPASS){
        }

        xStartTime = pom;

#if (PROJECT == 1)

        sensed_distance = getDistance();
       taskENTER_CRITICAL();
      g_distance_sensed = sensed_distance;
       taskEXIT_CRITICAL();
#endif

        xEndTime = pom;

        calculate_wcet(xStartTime,xEndTime, &WCET, 1);

        UARTprintf("==Service_1 WCET time:%dms==\n", (xEndTime-xStartTime)/100);
    }
}

/***********************************************************************************************
 * Name          : vService_2
 * Description   : This task handles the buzzer operations
 * Parameters    : Thread parameters
 * RETURN        : None
 ***********************************************************************************************/
static void vService_2(void* pvParameters){

    TickType_t xStartTime,xEndTime;

    uint32_t WCET = 0;

    unsigned long long S2Cnt=0;

    while(1){

        if( xSemaphoreTake(xsemS[2], portMAX_DELAY) == pdPASS){
        }

        xStartTime = pom;

        S2Cnt++;

#if (PROJECT == 1)
        if( (g_distance_sensed >= WARNING_LOW) && (g_distance_sensed <= 70) )
        {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);//GPIO_high
        }

        else
        {
            GPIOPinWrite(GPIO_PORTN_BASE, (uint32_t) GPIO_PIN_3, (uint32_t) ~GPIO_PIN_3);
        }

        xEndTime = pom;

        calculate_wcet(xStartTime,xEndTime, &WCET, 2);
#endif

        UARTprintf("==Service_2 WCET time:%dms==\n", (xEndTime-xStartTime)/100);
    }
}

/***********************************************************************************************
 * Name          : vService_3
 * Description   : This task handles the acceleration and deceleration of the system based on
 *                 the distance value
 * Parameters    : Thread parameters
 * RETURN        : None
 ***********************************************************************************************/
static void vService_3(void* pvParameters){

    TickType_t xStartTime,xEndTime;

    uint32_t WCET = 0;
    uint8_t current_PWM = 100;
    unsigned long long S3Cnt=0;

    while(1)
    {
        if( xSemaphoreTake(xsemS[3], portMAX_DELAY) == pdPASS){;
        }

        xStartTime = pom;

        S3Cnt++;

        if( (g_distance_sensed >= 0) && (g_distance_sensed < 50) )
        {
            while(current_PWM > 30)
            {
                current_PWM -= 5;
                setPMW(current_PWM);
            }
        }
        else if( (g_distance_sensed >= 50) && (g_distance_sensed < 70) )
        {
            while(current_PWM > 70)
            {
                current_PWM -= 5;
                setPMW(current_PWM);
            }
        }
        else if( (g_distance_sensed > 70) )
        {
            while(current_PWM < 90)
            {
                current_PWM += 5;
                setPMW(current_PWM);
            }
        }

        xEndTime = pom;

        calculate_wcet(xStartTime,xEndTime, &WCET, 3);

        UARTprintf("==Service_3 WCET time:%dms==\n", (xEndTime-xStartTime)/100);
    }
}

/***********************************************************************************************
 * Name          : getDistance
 * Description   : This function obtains the distance measured by the ultra-sonic sensor
 * Parameters    : None
 * RETURN        : None
 ***********************************************************************************************/
static int getDistance(void)
{
    int i, start_pom=0, stop_pom=0, delta_pom = 0, dist = 0, temp_pom = 0;

    temp_pom = pom;

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0x00);
    i=5;
    while(i--);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4);
    i=100;
    while(i--);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0x00);

    while(!(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_5)));
    start_pom=pom;
    while((GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_5)));
    stop_pom = pom;

    dist = ((stop_pom - start_pom)*(1000000/SCALING_FACTOR))/58;

    return dist;
}
//[EOF]
