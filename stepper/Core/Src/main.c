/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// NOTE: The following variables are hard coded
enum MODES{START, ACCELERATE, CRUISE, DECCELERATE, STOP};
const double DIST_PER_STEP = 0.0028; // mm/step
const uint32_t MIN_VELOCITY = 14000;
const uint32_t MAX_VELOCITY= 10000;
const uint32_t RAMP_UP_STEPS = 500;
const uint32_t STEP_SIZE = (MIN_VELOCITY - MAX_VELOCITY)/RAMP_UP_STEPS;

uint8_t mode = STOP;

volatile uint32_t t1Count=0;
volatile uint32_t cum_ticks = 0;
volatile uint32_t curr_ticks = 0;
uint32_t curr_velocity = 14000; // steps
uint32_t debug_speed=MIN_VELOCITY;

// NOTE: startStepper handles the initialization for these variables
uint32_t dist_ticks = 0;
uint32_t cruise_ticks = 0;
uint32_t acc_ticks = 0;
uint32_t dec_ticks = 0;

sque usb={0};

com SRCom={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	switch(mode){
		case STOP:
			htim->Instance->ARR = debug_speed;
			htim->Instance->CCR1 = 0;
			curr_ticks = 0;
			break;
		case START:
			htim->Instance->ARR = debug_speed;
			htim->Instance->CCR1 = debug_speed/2;
			if(curr_ticks++ >= dist_ticks){
				mode = STOP;
			}
	}

//	t1Count++;
//
//	switch(mode){
//		case STOP:
//			htim->Instance->ARR = MIN_VELOCITY;
//			htim->Instance->CCR1 = 0;
//			break;
//		case START:
//			cum_ticks = 0;
//			curr_ticks = 0;
//			mode = ACCELERATE;
//			break;
//		case ACCELERATE:
//			if(curr_velocity > MAX_VELOCITY && ++curr_ticks <= acc_ticks){
//				curr_velocity -= STEP_SIZE;
//			}
//			else if (cruise_ticks > 0){
//				mode = CRUISE;
//				curr_ticks = 0;
//			}
//			else{
//				mode = DECCELERATE;
//				curr_ticks = 0;
//			}
//			break;
//		case CRUISE:
//			if(++curr_ticks >= cruise_ticks){
//				mode = DECCELERATE;
//				curr_ticks = 0;
//			}
//			break;
//		case DECCELERATE:
//			if(curr_velocity < MIN_VELOCITY && ++curr_ticks <= dec_ticks){
//				curr_velocity += STEP_SIZE;
//			}
//			else{
//				mode = STOP;
//			}
//			break;
//	}
//
//	if(mode != STOP){
//		cum_ticks++;
//		htim->Instance->ARR = curr_velocity;
//		htim->Instance->CCR1 = curr_velocity/2;
//	}
}

/**
 * startStepper - Converts dist (mm) into ticks and starts stepper in dir.
 *
 * @dir: direction of stepper (0: Clockwise, 1: Anti-clockwise)
 *
 * @dist: Amount of distance to move in mm
 */
void startStepper(uint8_t dir, double dist){

	// Convert mm to ticks
	dist_ticks = dist/DIST_PER_STEP;

	// Cruise Decision
	if(dist_ticks >= 2*RAMP_UP_STEPS){
		cruise_ticks = (dist_ticks - 2*RAMP_UP_STEPS);
		dec_ticks = acc_ticks = RAMP_UP_STEPS;
	}
	else{
		cruise_ticks = 0;
		dec_ticks = acc_ticks = dist_ticks/2;
	}

	// Direction to move
	if(dir==0){
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	}

	// Give signal to timer to start stepper
	mode = START;
}

/**
 * debugStepper - Moves stepper in dir for n ticks
 *
 * @dir: direction of stepper (0: Clockwise, 1: Anti-clockwise)
 *
 * @ticks: Amount of ticks to move
 */
void debugStepper(uint8_t dir, int ticks, uint32_t vel){

	dist_ticks = ticks;
	debug_speed = vel;

	// Cruise Decision
//	if(dist_ticks >= 2*RAMP_UP_STEPS){
//		cruise_ticks = (dist_ticks - 2*RAMP_UP_STEPS);
//		dec_ticks = acc_ticks = RAMP_UP_STEPS;
//	}
//	else{
//		cruise_ticks = 0;
//		dec_ticks = acc_ticks = dist_ticks/2;
//	}

	// Direction to move
	if(dir==0){
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	}

	// Give signal to timer to start stepper
	mode = START;
}

void SendString(char* buf){
	CDC_Transmit_FS(buf, strlen(buf));
}

void ProcessCommand(char *buf) {
	char *mt=NULL;
	char msg[128] = {0};
	if ((mt=strstr(buf,"$START:")) !=NULL) {
		int dir=0;
		int dist=0;
		int parsed = sscanf(&mt[7],"%d %d",&dir, &dist);
		if (parsed == 2) {
			startStepper((uint8_t)dir, (double)dist);
			sprintf(msg,"Stepper Started\r\n");
		} else {
			sprintf(msg,"Invalid START\r\n");
		}
		SendString(msg);
	}
	else if ((mt=strstr(buf,"$RESET:")) !=NULL) {
		SendString("Resetting in 5 seconds\r\n");
		HAL_Delay(5000);
		NVIC_SystemReset();
	}
	else if((mt=strstr(buf,"$DEBUG:")) != NULL){
		int dir=0;
		int ticks=0;
		uint32_t vel=0;
		int parsed = sscanf(&mt[7],"%d %d %lu",&dir, &ticks, &vel);
		if (parsed == 3){
			debugStepper(dir, ticks, vel);
            sprintf(msg, "Stepper Debug Started: dir=%d ticks=%d vel=%lu\r\n", dir, ticks, vel);
		}
		else{
			 sprintf(msg, "Invalid DEBUG\r\n");
		}
		SendString(msg);
	}
}

uint8_t read(char* tmp){
	if(usb.head==usb.tail){
		return 0;
	}
	char a = usb.buf[usb.head++];
	*tmp = a;
	usb.head%=BUFSIZE;
	return 1;
}

char* ComProcess() {
  char tmp=0;
  while (read(&tmp)) {
    switch(SRCom.state) {
      case 0x00:
        if ('$'==tmp) {
          memset(SRCom.buf,0,sizeof(SRCom.buf));
          SRCom.bpointer=0;
          SRCom.buf[SRCom.bpointer++]=tmp;
          SRCom.state=1;
        }
      break;
      case 0x01:
//        tmp=receive(chn,10);
        if ((tmp>=0x20) && (tmp<=0x7f)) {
          SRCom.buf[SRCom.bpointer++]=tmp;
          if (SRCom.bpointer>=(BUFSIZE-1)) {
            // Discard everything
            SRCom.bpointer=0;
            SRCom.state=0;
          }
        } else
        {
          if ('\n'==tmp) {
            SRCom.buf[SRCom.bpointer]=0;
			SRCom.state=2;
            return SRCom.buf;
          }
        }
      break;
      case 0x02:
      break;
    }
  }
  return NULL;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
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
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // Stepper motor won't be rotating in the beginning
  htim1.Instance->PSC=11;
  htim1.Instance->ARR=curr_velocity;
  htim1.Instance->CCR1=0;

  // Start Timer 1 Callback
  HAL_TIM_Base_Start_IT(&htim1);

  // Start PWM on TIM2 CH1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


//  HAL_Delay(10000);

  char* a = NULL;
  while (1)
  {
	  if((a=ComProcess())!=NULL){
		  ProcessCommand(SRCom.buf);
		  SRCom.state = 0;
	  }
  }
  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
