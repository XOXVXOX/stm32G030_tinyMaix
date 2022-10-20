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
#include "usart.h"
#include "led.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tinymaix.h"
#include "mnist_valid_q.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t pic[28*28]={
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,116,125,171,255,255,150, 93,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,169,253,253,253,253,253,253,218, 30,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,169,253,253,253,213,142,176,253,253,122,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0, 52,250,253,210, 32, 12,  0,  6,206,253,140,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0, 77,251,210, 25,  0,  0,  0,122,248,253, 65,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0, 31, 18,  0,  0,  0,  0,209,253,253, 65,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,117,247,253,198, 10,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 76,247,253,231, 63,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,128,253,253,144,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,176,246,253,159, 12,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 25,234,253,233, 35,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,198,253,253,141,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0, 78,248,253,189, 12,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0, 19,200,253,253,141,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,134,253,253,173, 12,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,248,253,253, 25,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,248,253,253, 43, 20, 20, 20, 20,  5,  0,  5, 20, 20, 37,150,150,150,147, 10,  0,
  0,  0,  0,  0,  0,  0,  0,  0,248,253,253,253,253,253,253,253,168,143,166,253,253,253,253,253,253,253,123,  0,
  0,  0,  0,  0,  0,  0,  0,  0,174,253,253,253,253,253,253,253,253,253,253,253,249,247,247,169,117,117, 57,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,118,123,123,123,166,253,253,253,155,123,123, 41,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};
#define IMG_L 28
#define IMG_CH 1
#define CLASS_NUM (10)
char* labels[CLASS_NUM] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};



static tm_err_t layer_cb(tm_mdl_t* mdl, tml_head_t* lh)
{   //dump middle result
    int h = lh->out_dims[1];
    int w = lh->out_dims[2];
    int ch= lh->out_dims[3];
    mtype_t* output = TML_GET_OUTPUT(mdl, lh);
    #if 0
    TM_PRINTF("Layer %d callback ========\r\n", mdl->layer_i);
    for(int y=0; y<h; y++){
        TM_PRINTF("[");
        for(int x=0; x<w; x++){
            TM_PRINTF("[");
            for(int c=0; c<ch; c++){
            #if TM_MDL_TYPE == TM_MDL_FP32
                TM_PRINTF("%.3f,", output[(y*w+x)*ch+c]);
            #else
                TM_PRINTF("%.3f,", TML_DEQUANT(lh,output[(y*w+x)*ch+c]));
            #endif
            }
            TM_PRINTF("],");
        }
        TM_PRINTF("],\r\n");
    }
    TM_PRINTF("\r\n");
    #endif
    return TM_OK;
}

static void parse_output(tm_mat_t* outs)
{
    tm_mat_t out = outs[0];
    float* data  = out.dataf;
    float maxp = 0;
    int maxi = -1;
    for(int i = 0; i < CLASS_NUM; i++){
        printf("%d: %.3f\r\n", i, data[i]);
        if(data[i] > maxp) {
            maxi = i;
            maxp = data[i];
        }
    }
    TM_PRINTF("### Predict output is: Class %d, Lable %s, prob %.3f\r\n", maxi, labels[maxi], maxp);
    return;
}

static uint8_t mdl_buf[MDL_BUF_LEN];



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
  LED_Init();
  USART1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  printf("%d\r\n",uwTick);

	  TM_DBGT_INIT();
    TM_PRINTF("mnist demo\r\n");
    tm_mdl_t mdl;

    for(int i=0; i<28*28; i++){
        TM_PRINTF("%3d,", pic[i]);
        if(i%28==27)printf("\r\n");
    }

    tm_mat_t in_uint8 = {3,IMG_L,IMG_L,IMG_CH, (mtype_t*)pic};
    tm_mat_t in = {3,IMG_L,IMG_L,IMG_CH, NULL};
    tm_mat_t outs[1];
    tm_err_t res;
//    tm_stat((tm_mdlbin_t*)mdl_data); 

    res = tm_load(&mdl, mdl_data, mdl_buf, layer_cb, &in);
    if(res != TM_OK) {
        TM_PRINTF("tm model load err %d\r\n", res);
        return -1;
    }

#if TM_MDL_TYPE != TM_MDL_FP32
    res = tm_preprocess(&mdl, TMPP_UINT2INT, &in_uint8, &in); 
#else
    res = tm_preprocess(&mdl, TMPP_UINT2FP01, &in_uint8, &in); 
#endif
    TM_DBGT_START();
    res = tm_run(&mdl, &in, outs);
    TM_DBGT("tm_run");
    if(res==TM_OK) parse_output(outs);  
    else TM_PRINTF("tm run error: %d\r\n", res);
    tm_unload(&mdl);      
		
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
void SystemClock_Config(void)	// 该函数由cubeMX生成，例程里所有外设时钟都放在这个函数
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
