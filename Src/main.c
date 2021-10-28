/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#define RECEIVER // Either set the board as TRANSMITTER, RECEIVER or ANALOG_MEASURE

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Buffer Size */
#define BUFFER_SIZE 20
/* Task Stack Size */
#define APP_TASK_START_STK_SIZE 128u
#define MOISTURE_DETECTION_STK_SIZE 128u
#define BLINK_TASK_STK_SIZE 128u
#define RECEIVE_TASK_STK_SIZE 128u
#define TRANSMIT_TASK_STK_SIZE 128u
#define TRANSMIT_TO_PC_TASK_STK_SIZE 128u
#define READ_ANALOG_TASK_STK_SIZE 128u
/* Task Priority */
#define APP_TASK_START_PRIO 1u
#define MOISTURE_DETECTION_PRIO 4u
#define BLINK_TASK_PRIO 5u
#define RECEIVE_TASK_PRIO 2u
#define TRANSMIT_TASK_PRIO 2u
#define TRANSMIT_TO_PC_TASK_PRIO 3u
#define READ_ANALOG_TASK_PRIO 6u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Ring Buffer */
struct ring_buffer rb;
unsigned char buffer[BUFFER_SIZE];
char message[BUFFER_SIZE];
int voltage;
/* Task Control Block */
static OS_TCB AppTaskStartTCB;
static OS_TCB MoistureDetectionTCB;
static OS_TCB BlinkTaskTCB;
static OS_TCB ReceiveTaskTCB;
static OS_TCB TransmitTaskTCB;
static OS_TCB TransmitToPCTaskTCB;
static OS_TCB ReadAnalogTaskTCB;
/* Task Stack */
static CPU_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
static CPU_STK MoistureDetectionStk[MOISTURE_DETECTION_STK_SIZE];
static CPU_STK BlinkTaskStk[BLINK_TASK_STK_SIZE];
static CPU_STK ReceiveTaskStk[RECEIVE_TASK_STK_SIZE];
static CPU_STK TransmitTaskStk[TRANSMIT_TASK_STK_SIZE];
static CPU_STK TransmitToPCTaskStk[TRANSMIT_TO_PC_TASK_STK_SIZE];
static CPU_STK ReadAnalogTaskStk[READ_ANALOG_TASK_STK_SIZE];
/* Semaphore */
OS_SEM sem;
OS_SEM startBlink;
OS_SEM endBlink;
OS_SEM startAnalog;
OS_SEM endAnalog;
OS_SEM comms_busy;
OS_SEM transmitUART3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void AppTaskStart(void *p_arg);
static void BlinkTask(void *p_arg);
static void MoisturePercentageTask();
static void ReceiveTask(void *p_arg);
static void TransmitTask(void *p_arg);
static void TransmitToPCTask(void *p_arg);
static void ReadAnalogTask(void *p_arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int main(void)
{
  /* To store error code */
  OS_ERR os_err;

  /* Initialize uC/OS-III */
  OSInit(&os_err);

  if (os_err != OS_ERR_NONE)
  {
    while (DEF_TRUE)
      ;
  }

  OSSemCreate(
      (OS_SEM *)&startBlink,
      (CPU_CHAR *)"Semaphore",
      (OS_SEM_CTR)0,
      (OS_ERR *)&os_err);
  
  OSSemCreate(
      (OS_SEM *)&endBlink,
      (CPU_CHAR *)"Semaphore",
      (OS_SEM_CTR)0,
      (OS_ERR *)&os_err);
  
  OSSemCreate(
      (OS_SEM *)&startAnalog,
      (CPU_CHAR *)"Semaphore",
      (OS_SEM_CTR)0,
      (OS_ERR *)&os_err);
    
  OSSemCreate(
      (OS_SEM *)&endAnalog,
      (CPU_CHAR *)"Semaphore",
      (OS_SEM_CTR)0,
      (OS_ERR *)&os_err);
  
  OSSemCreate(
      (OS_SEM *)&comms_busy,
      (CPU_CHAR *)"Semaphore",
      (OS_SEM_CTR)0,
      (OS_ERR *)&os_err);
      
  OSSemCreate(
      (OS_SEM *)&transmitUART3,
      (CPU_CHAR *)"Semaphore",
      (OS_SEM_CTR)0,
      (OS_ERR *)&os_err);

  OSTaskCreate(
      /* pointer to task control block */
      (OS_TCB *)&AppTaskStartTCB,
      /* task name can be displayed by debuggers */
      (CPU_CHAR *)"App Task Start",
      /* pointer to the task */
      (OS_TASK_PTR)AppTaskStart,
      /* pointer to an OPTIONAL data area */
      (void *)0,
      /* task priority: the lower the number, the higher the priority */
      (OS_PRIO)APP_TASK_START_PRIO,
      /* pointer to task's stack base addr */
      (CPU_STK *)&AppTaskStartStk[0],
      /* task's stack limit to monitor and ensure that the stack 
       * doesn't overflow (10%) */
      (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
      /* task's stack size */
      (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
      /* max number of message that the task can receive through 
       * internal message queue (5) */
      (OS_MSG_QTY)5u,
      /* amount of clock ticks for the time quanta 
       * when round robin is enabled */
      (OS_TICK)0u,
      /* pointer to an OPTIONAL user-supplied memory location 
       * use as a TCB extension */
      (void *)0,
      /* contain task-specific option 
       * OS_OPT_TASK_STK_CHK: allow stack checking 
       * OS_OPT_TASK_STK_CLR: stack needs to be cleared */
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      /* pointer to a variable that will receive an error code */
      (OS_ERR *)&os_err);

  if (os_err != OS_ERR_NONE)
  {
    while (DEF_TRUE)
      ;
  }

  /* Start Mulitasking */
  OSStart(&os_err);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void AppTaskStart(void *p_arg)
{
  OS_ERR os_err;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC_Init();
  init_buffer(&rb, buffer, BUFFER_SIZE);

  OSTaskCreate(
      (OS_TCB *)&BlinkTaskTCB,
      (CPU_CHAR *)"Blink Task",
      (OS_TASK_PTR)BlinkTask,
      (void *)0,
      (OS_PRIO)BLINK_TASK_PRIO,
      (CPU_STK *)&BlinkTaskStk[0],
      (CPU_STK_SIZE)BLINK_TASK_STK_SIZE / 10,
      (CPU_STK_SIZE)BLINK_TASK_STK_SIZE,
      (OS_MSG_QTY)5u,
      (OS_TICK)0u,
      (void *)0,
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      (OS_ERR *)&os_err);
    
#ifdef ANALOG_MEASURE

  OSTaskCreate(
      (OS_TCB *)&ReadAnalogTaskTCB,
      (CPU_CHAR *)"Read Analog Task",
      (OS_TASK_PTR)ReadAnalogTask,
      (void *)0,
      (OS_PRIO)READ_ANALOG_TASK_PRIO,
      (CPU_STK *)&ReadAnalogTaskStk[0],
      (CPU_STK_SIZE)READ_ANALOG_TASK_STK_SIZE / 10,
      (CPU_STK_SIZE)READ_ANALOG_TASK_STK_SIZE,
      (OS_MSG_QTY)5u,
      (OS_TICK)0u,
      (void *)0,
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      (OS_ERR *)&os_err);

#endif
#ifdef RECEIVER

  OSTaskCreate(
      (OS_TCB *)&ReceiveTaskTCB,
      (CPU_CHAR *)"Receive Task",
      (OS_TASK_PTR)ReceiveTask,
      (void *)0,
      (OS_PRIO)RECEIVE_TASK_PRIO,
      (CPU_STK *)&ReceiveTaskStk[0],
      (CPU_STK_SIZE)RECEIVE_TASK_STK_SIZE / 10,
      (CPU_STK_SIZE)RECEIVE_TASK_STK_SIZE,
      (OS_MSG_QTY)5u,
      (OS_TICK)0u,
      (void *)0,
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      (OS_ERR *)&os_err);
    
#endif
#ifdef TRANSMITTER
  
  OSTaskCreate(
      (OS_TCB *)&TransmitTaskTCB,
      (CPU_CHAR *)"Transmit Task",
      (OS_TASK_PTR)TransmitTask,
      (void *)0,
      (OS_PRIO)TRANSMIT_TASK_PRIO,
      (CPU_STK *)&TransmitTaskStk[0],
      (CPU_STK_SIZE)TRANSMIT_TASK_STK_SIZE / 10,
      (CPU_STK_SIZE)TRANSMIT_TASK_STK_SIZE,
      (OS_MSG_QTY)5u,
      (OS_TICK)0u,
      (void *)0,
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      (OS_ERR *)&os_err);

  OSTaskCreate(
    (OS_TCB *)&MoistureDetectionTCB,
    (CPU_CHAR *)"Moisture Detection",
    (OS_TASK_PTR)MoisturePercentageTask,
    (void *)0,
    (OS_PRIO)MOISTURE_DETECTION_PRIO,
    (CPU_STK *)&MoistureDetectionStk[0],
    (CPU_STK_SIZE)MOISTURE_DETECTION_STK_SIZE / 10,
    (CPU_STK_SIZE)MOISTURE_DETECTION_STK_SIZE,
    (OS_MSG_QTY)5u,
    (OS_TICK)0u,
    (void *)0,
    (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
    (OS_ERR *)&os_err
  );

  OSTaskCreate(
      (OS_TCB *)&ReadAnalogTaskTCB,
      (CPU_CHAR *)"Read Analog Task",
      (OS_TASK_PTR)ReadAnalogTask,
      (void *)0,
      (OS_PRIO)READ_ANALOG_TASK_PRIO,
      (CPU_STK *)&ReadAnalogTaskStk[0],
      (CPU_STK_SIZE)READ_ANALOG_TASK_STK_SIZE / 10,
      (CPU_STK_SIZE)READ_ANALOG_TASK_STK_SIZE,
      (OS_MSG_QTY)5u,
      (OS_TICK)0u,
      (void *)0,
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      (OS_ERR *)&os_err);

#endif

  OSSemPost(
      (OS_SEM*)&comms_busy,
      (OS_OPT)OS_OPT_POST_1,
      (OS_ERR*)&os_err
    );
}

static void ReadAnalogTask(void *p_arg){
  HAL_ADC_Start(&hadc);
  OS_ERR os_err;

  while(DEF_TRUE){
    OSSemPend(
      (OS_SEM*)&startAnalog,
      (OS_TICK)0,
      (OS_OPT)OS_OPT_PEND_BLOCKING,
      (CPU_TS*)NULL,
      (OS_ERR*)&os_err
    );
    uint16_t raw_value;

    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);

    raw_value = HAL_ADC_GetValue(&hadc);
    voltage = ((float)raw_value) / 4096 * 3300;
    
    OSSemPost(
      (OS_SEM*)&endAnalog,
      (OS_OPT)OS_OPT_POST_1,
      (OS_ERR*)&os_err
    );
  }
}

static void TransmitTask(void *p_arg){
  OS_ERR os_err;

  while(DEF_TRUE){
    OSSemPend(
      (OS_SEM*)&transmitUART3,
      (OS_TICK)0,
      (OS_OPT)OS_OPT_PEND_BLOCKING,
      (CPU_TS*)NULL,
      (OS_ERR*)&os_err
    );
    OSSemPend(
      (OS_SEM*)&comms_busy,
      (OS_TICK)0,
      (OS_OPT)OS_OPT_PEND_BLOCKING,
      (CPU_TS*)NULL,
      (OS_ERR*)&os_err
    );
    HAL_UART_Transmit(&huart3, message, BUFFER_SIZE, 100);
    OSSemPost(
      (OS_SEM*)&comms_busy,
      (OS_OPT)OS_OPT_POST_1,
      (OS_ERR*)&os_err
    );
    
    OSSemPost(
      (OS_SEM*)&startBlink,
      (OS_OPT)OS_OPT_POST_1,
      (OS_ERR*)&os_err
    );
    OSSemPend(
      (OS_SEM*)&endBlink,
      (OS_TICK)0,
      (OS_OPT)OS_OPT_PEND_BLOCKING,
      (CPU_TS*)NULL,
      (OS_ERR*)&os_err
    );
  }
}

static void ReceiveTask(void *p_arg){
  OS_ERR os_err;

  while(DEF_TRUE){
    OSSemPend(
      (OS_SEM*)&comms_busy,
      (OS_TICK)0,
      (OS_OPT)OS_OPT_PEND_BLOCKING,
      (CPU_TS*)NULL,
      (OS_ERR*)&os_err
    );

    HAL_UART_Receive(&huart3, (uint8_t*)message, sizeof(message), 100);
    unsigned char cleaned_MSG[strlen(message)];

    for(int i = 0; i < strlen(message); i++){
      cleaned_MSG[i] = message[i];
    }

    HAL_UART_Transmit(&huart2, cleaned_MSG, sizeof(cleaned_MSG), 100);
    // append_string_to_buffer(&rb, message);

    // unsigned char MSG_2[7];
    // pop_string_from_buffer(&rb, MSG_2, sizeof(MSG_2));
    // HAL_UART_Transmit(&huart2, MSG_2, sizeof(MSG_2), 100);

    OSSemPost(
      (OS_SEM*)&comms_busy,
      (OS_OPT)OS_OPT_POST_1,
      (OS_ERR*)&os_err
    );
  }
}

static void BlinkTask(void *p_arg)
{
  OS_ERR os_err;
  while (DEF_TRUE)
  {
    OSSemPend(
      (OS_SEM*)&startBlink,
      (OS_TICK)0,
      (OS_OPT)OS_OPT_PEND_BLOCKING,
      (CPU_TS*)NULL,
      (OS_ERR*)&os_err
    );

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);

    OSSemPost(
      (OS_SEM*)&endBlink,
      (OS_OPT)OS_OPT_POST_1,
      (OS_ERR*)&os_err
    );
  }
}

static void MoisturePercentageTask(){
  OS_ERR os_err;
  int minimum = 1200; // Minimal possible voltage, this means that the sensor is currently dipped in water
  int maximum = 2900; // Maximal possible voltage, this means that the sensor is currently measuring the air
  double max_value = maximum - minimum; // Maximal possible delta-Voltage

  while(DEF_TRUE){
    double delta_voltage;
    OSSemPost(
      (OS_SEM*)&startAnalog,
      (OS_OPT)OS_OPT_POST_1,
      (OS_ERR*)&os_err
    );
    OSSemPend(
      (OS_SEM*)&endAnalog,
      (OS_TICK)0,
      (OS_OPT)OS_OPT_PEND_BLOCKING,
      (CPU_TS*)NULL,
      (OS_ERR*)&os_err
    );
    
    delta_voltage = voltage - minimum;
    double percentage = ((double)(max_value - delta_voltage) / max_value * 100.00);
    if(voltage < 0){
      sprintf(message, "SensorVal:100.0\r\n");
    } else if(voltage > maximum){
      sprintf(message, "SensorVal:00.00\r\n");
    } else {
      sprintf(message, "SensorVal:%d.%d\r\n", (int)percentage, (int)((percentage - (double)(int)percentage) * 100));
    }

    OSSemPost(
      (OS_SEM*)&transmitUART3,
      (OS_OPT)OS_OPT_POST_1,
      (OS_ERR*)&os_err
    );
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/