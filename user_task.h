/***********************************************************************************************************************
* File Name    : user_tasks.h
* Project      : RTES Final Course Project
* Version      : 4.0
* Description  : This is the main function that holds all the initialisation functions and holds the main() which is the
*                entry point to the function
* Author       : Sudarshan Jagannathan & Nihal Thirunakarasu referred from
*                https://github.com/akobyl/TM4C129_FreeRTOS_Demo
* Creation Date: 4.22.22
***********************************************************************************************************************/
#ifndef USER_TASK_H_
#define USER_TASK_H_
#include "semphr.h"

#define PROJECT    1


// Keep this 1000 for interrupt to hit every ms
#if (PROJECT == 1)
//For x1 frequency.
#define SCALING_FACTOR      100000
#endif

#define PROGRAM_TIMEOUT 30 // The program will terminate after 30 sec

#define NUMBER_SERVICES         8

void vStartUserTasks(void);

extern uint32_t pom;
extern SemaphoreHandle_t xsemS[];
extern uint32_t WCET_arr[NUMBER_SERVICES];
extern uint32_t ServiceNumber_arr[NUMBER_SERVICES];


#endif /* USER_TASK_H_ */
