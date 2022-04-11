/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Demo includes. */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum
{
    TECLA,
    LED
} tecla_led_state_t;

#define LED_RATE_MS 1000

#define KEYS_INVALID_TIME   -1
#define DEBOUNCE_TIME_MS   40

typedef enum
{
    STATE_BUTTON_UP,
    STATE_BUTTON_DOWN,
    STATE_BUTTON_FALLING,
    STATE_BUTTON_RISING
} keys_ButtonState_t;

typedef struct
{
    keys_ButtonState_t state;   //variables

    TickType_t time_down;		//timestamp of the last High to Low transition of the key
    TickType_t time_up;		    //timestamp of the last Low to High transition of the key
    TickType_t time_diff;	    //variables
} t_key_data;

t_key_data keys_data;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void vPrintString( const char *pcString );

TickType_t get_diff( void );
void clear_diff( void );

// Prototipo de funcion de la tarea
void zeros (uint32_t * vector, uint32_t longitud);
void task_tecla_led( void* taskParmPtr );


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void vPrintString( const char *pcString )
{
	/* Print the string, using a critical section as a crude method of mutual
	exclusion. */
	taskENTER_CRITICAL();

		HAL_UART_Transmit(&huart3, (uint8_t *)pcString, (uint16_t) strlen((char *)pcString), 10);

	taskEXIT_CRITICAL();
}


TickType_t get_diff( void )
{
    TickType_t tiempo;

    tiempo = keys_data.time_diff;

    return tiempo;
}


void clear_diff( void )
{
    keys_data.time_diff = KEYS_INVALID_TIME;
}

void zeros (uint32_t * vector, uint32_t longitud)
{char buffer1 [50];
int i=0;

    vPrintString( "INGRESO A LA FUNCION ZEROS \r\n" );
    vPrintString( "SE INICIALIZARON LOS VALORES DEL VECTOR EN CEROS \r\n" );
    //muestra el vector inicializado con cero sus elemeentos
    while(i<longitud)
    {
    	vector[i]=0;
    sprintf( buffer1, "Vector [%d] = %lu \r\n", i,vector[i],longitud);
    HAL_UART_Transmit(&huart3, (uint8_t*) buffer1, strlen(buffer1), 1000);
    i++;
    }
}



// Implementacion de funcion de la tarea
void task_tecla_led( void* taskParmPtr )
{
    // ---------- CONFIGURACIONES ------------------------------
	char buffer [50];
	uint32_t vector[10]={1,2,3,4};
		uint32_t longitud=4;
		int index=0;



		  //muestra el vector con los valores originales
		    while(index<longitud)
		{
		  if(index==0) vPrintString( "VALORES ORIGINALES DEL VECTOR \r\n" );

		  sprintf( buffer, "Vector [%d] = %lu \r\n", index,vector[index]);
		  HAL_UART_Transmit(&huart3, (uint8_t*) buffer, strlen(buffer), 1000);
		  index++;

		}
         zeros (vector,longitud);

	tecla_led_state_t tecla_led_state = TECLA;

	TickType_t dif;

	keys_data.state          = STATE_BUTTON_UP;  		// Set initial state
    keys_data.time_down      = KEYS_INVALID_TIME;
    keys_data.time_up        = KEYS_INVALID_TIME;
    keys_data.time_diff      = KEYS_INVALID_TIME;

    vPrintString( "      Task: task_tecla_led\r\n" );

	// ---------- REPETIR POR SIEMPRE --------------------------
    while( 1 )
    {
    	if( tecla_led_state == TECLA)
    	{
    		switch( keys_data.state )
    		{
            	case STATE_BUTTON_UP:
            		/* CHECK TRANSITION CONDITIONS */
            		if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin))
            		{
            			keys_data.state = STATE_BUTTON_FALLING;
            			vPrintString( "            keys_data.state: STATE_BUTTON_FALLING\r\n" );
            		}
            		break;

            	case STATE_BUTTON_FALLING:
            		/* ENTRY */

            		/* CHECK TRANSITION CONDITIONS */
            		if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin))
            		{
            			keys_data.state = STATE_BUTTON_DOWN;
            			vPrintString( "            keys_data.state: STATE_BUTTON_DOWN\r\n" );

            			/* ACCION DEL EVENTO !*/
            			keys_data.time_down = xTaskGetTickCount();
            		}
            		else
            		{
            			keys_data.state = STATE_BUTTON_UP;
                    	vPrintString( "            keys_data.state: STATE_BUTTON_UP\r\n" );
            		}

            		/* LEAVE */
            		break;

            	case STATE_BUTTON_DOWN:
            		/* CHECK TRANSITION CONDITIONS */
            		if (!HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin))
            		{
            			keys_data.state = STATE_BUTTON_RISING;
            			vPrintString( "            keys_data.state: STATE_BUTTON_RISING\r\n" );
            		}
            		break;

            	case STATE_BUTTON_RISING:
            		/* ENTRY */

            		/* CHECK TRANSITION CONDITIONS */

            		if (!HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin))
            		{
            			tecla_led_state = LED;

            			keys_data.state = STATE_BUTTON_UP;
            			vPrintString( "            keys_data.state: STATE_BUTTON_UP\r\n" );

            			/* ACCION DEL EVENTO ! */
            			keys_data.time_up    = xTaskGetTickCount();
            			keys_data.time_diff  = keys_data.time_up - keys_data.time_down;
            		}
            		else
            		{
            			keys_data.state = STATE_BUTTON_DOWN;
            			vPrintString( "            keys_data.state: STATE_BUTTON_DOWN\r\n" );
            		}

            		/* LEAVE */
            		break;

            	default:
            		keys_data.state = STATE_BUTTON_UP;
            		vPrintString( "            keys_data.state: STATE_BUTTON_UP\r\n" );

            		break;
    		}
    		// Envia la tarea al estado bloqueado durante DEBOUNCE_TIME_MS
    		vTaskDelay( DEBOUNCE_TIME_MS / portTICK_RATE_MS );
    	}

    	if( tecla_led_state == LED)
    	{
            dif = get_diff();

            if( dif != KEYS_INVALID_TIME )
            {
                if( dif > LED_RATE_MS )
                {
                    dif = LED_RATE_MS;
                }
    			tecla_led_state = TECLA;

        		sprintf( buffer, "            led_state: Encendido - dif %u\r\n", (unsigned int)dif );
        		vPrintString( buffer );
            	HAL_GPIO_WritePin( LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET );

                // Envia la tarea al estado bloqueado durante dif
                vTaskDelay( dif / portTICK_RATE_MS );

            	vPrintString( "            led_state: Apagado\r\n" );
            	HAL_GPIO_WritePin( LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET );

                clear_diff();
            }
            else
            {
                // Envia la tarea al estado bloqueado durante LED_RATE_MS
                vTaskDelay( LED_RATE_MS / portTICK_RATE_MS );
            }
    	}
    }
}

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main( void )
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
	vPrintString( "Main: Ejercicio 01 -Language C - AdM.\r\n" );

  /* USER CODE END 2 */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

    // Crear tareas en freeRTOS
    BaseType_t res =
    	xTaskCreate (
              task_tecla_led,					// Funcion de la tarea a ejecutar
              ( const char * )"task_tecla_led",	// Nombre de la tarea como String amigable para el usuario
              configMINIMAL_STACK_SIZE*2,		// Cantidad de stack de la tarea
              NULL,								// Parametros de tarea
              tskIDLE_PRIORITY+1,				// Prioridad de la tarea
              NULL								// Puntero a la tarea creada en el sistema
          );

    // Gestion de errores
    //if(res == pdFAIL)
    //{
    //	gpioWrite( LEDR, ON );
    //	vPrintString( "Error al crear las tareas.\r\n" );
    //	while(TRUE);						// VER ESTE LINK: https://pbs.twimg.com/media/BafQje7CcAAN5en.jpg
    //}
    configASSERT( res == pdPASS);	// gestion de errores

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  // ---------- REPETIR POR SIEMPRE --------------------------
  //while( TRUE )
  //{
  // Si cae en este while 1 significa que no pudo iniciar el scheduler
  //}
  configASSERT( 0 );

  // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
  // directamenteno sobre un microcontroladore y no es llamado por ningun
  // Sistema Operativo, como en el caso de un programa para PC.
}


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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

