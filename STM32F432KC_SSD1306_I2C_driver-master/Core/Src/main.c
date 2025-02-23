/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <ssd1306_tests.h>
#include "main.h"
#include "ssd1306_tests.h"
#include "ssd1306.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SSD1306_USE_I2C
#define __DEBUG 1
#define BUFFERSIZE 100
#define I2CBUF	12
#define debug_print(x) 	do { if ( __DEBUG ) { strcpy(uartBuffer, x); HAL_UART_Transmit(&huart2, (unsigned char*) uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY); }} while (0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint8_t SSD1306_ADDRESS = 0x3C << 1;
const uint8_t RANDOM_REG = 0x0F;

char uartBuffer[BUFFERSIZE] = "";
uint8_t I2CBuffer[I2CBUF] = {0};
HAL_StatusTypeDef returnValue = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
char height;
char width;
bool blockShape[3][4];
}block;
char blockcounter = 0;



typedef struct {
	int bufferheight;// = 4;
	int blockheight; //= 0;
	int blockwidth; // = 0;
    int yCoordUnderBrick;// = 0; //TrackHeight Of bootom of brick
    int XCoordLeftBrick;// = 0; //Tack ypos of brick from lest side
	int height;// = 16 + bufferheight;
	int width;// = 8;
	int score;// = 0;
	 int randomcounter;
	bool playingfield[20][8];     /*= {{0,0,0,0,0,0,0,0}  //BUFF
                                       ,{0,0,0,0,0,0,0,0}  //BUFF
                                    ,{0,0,0,0,0,0,0,0}  //BUFF
                                       ,{0,0,0,0,0,0,0,0}  //BUFF
                                       ,{0,0,0,0,0,0,0,0}  //1
   		                         ,{0,0,0,0,0,0,0,0}  //2
                                     ,{0,0,0,0,0,0,0,0}  //3
                                     ,{0,0,0,0,0,0,0,0}  //4
	                                 ,{0,0,0,0,0,0,0,0}  //5
	                                 ,{0,0,0,0,0,0,0,0}  //6
	                                 ,{0,0,0,0,0,0,0,0}  //6
	                                 ,{0,0,0,0,0,0,0,0}  //7
	                                 ,{0,0,0,0,0,0,0,0}  //8
	                                 ,{0,0,0,0,0,0,0,0}  //9
	                                 ,{0,0,0,0,0,0,0,0}  //10
	                                 ,{0,0,0,0,0,0,0,0}  //11
		                         ,{0,0,0,0,0,0,0,0}  //12
		                         ,{0,0,0,0,0,0,0,0}  //13
		                         ,{0,0,0,0,0,0,0,0}  //14
		                         ,{0,0,0,0,0,0,0,0}  //15
		                         ,{0,0,0,0,0,0,0,0}};*///16

	//For when block is being placed (has to do with die checking)
	bool ghostBlockField[20][8];/*= {{0,0,0,0,0,0,0,0}  //BUFF
                                   ,{0,0,0,0,0,0,0,0}  //BUFF
                                          ,{0,0,0,0,0,0,0,0}  //BUFF
                                          ,{0,0,0,0,0,0,0,0}  //BUFF
	                                  ,{0,0,0,0,0,0,0,0}  //1
    		                          ,{0,0,0,0,0,0,0,0}  //2
                                          ,{0,0,0,0,0,0,0,0}  //3
                                          ,{0,0,0,0,0,0,0,0}  //4
	                                  ,{0,0,0,0,0,0,0,0}  //5
	                                  ,{0,0,0,0,0,0,0,0}  //6
	                                  ,{0,0,0,0,0,0,0,0}  //6
	                                  ,{0,0,0,0,0,0,0,0}  //7
	                                  ,{0,0,0,0,0,0,0,0}  //8
	                                  ,{0,0,0,0,0,0,0,0}  //9
	                                  ,{0,0,0,0,0,0,0,0}  //10
	                                  ,{0,0,0,0,0,0,0,0}  //11
		                          ,{0,0,0,0,0,0,0,0}  //12
		                          ,{0,0,0,0,0,0,0,0}  //13
		                          ,{0,0,0,0,0,0,0,0}  //14
		                          ,{0,0,0,0,0,0,0,0}  //15
		                          ,{0,0,0,0,0,0,0,0}};//16 */


} tetrisgame;
void FillArr(tetrisgame *t);
int* checkline(tetrisgame *t);
bool checkdead(tetrisgame *t);
void printLCD(tetrisgame *t);
//Deze twee werken in conjunctie samen
void dropblock(tetrisgame *t, int ycoord,int randomvar);
void placeblock(tetrisgame *t,int yCoord);
void newblock(tetrisgame *t, int ycoordinate,int randomvar);
void ssd1306_Init();
void init() {
	 /* tetrisgame tgame;
	  tgame.bufferheight =4;
	  tgame.height =20;
	  tgame.width =8;
	  FillArr(&tgame);

	char blockcounter = 0;*/
	//ssd1306_TestAll();
	/*while(1)
	{
	     // ssd1306_DrawPixel(10,10,White);
	      //ssd1306_TestBorder();

		  //newblock(&tgame,blockcounter);
		//  printLCD(&tgame);
		// placeblock(&tgame);
		 //checkline(&tgame);
		  ssd1306_DrawPixel(10, 10, White);
		  ssd1306_UpdateScreen();
		 //checkdead(&tgame);

		  HAL_Delay(250);
		  // blockcounter++;

	}*/
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
	  tetrisgame tgame;
	  tgame.bufferheight =0;
	  tgame.height =20;
	  tgame.width =8;
	  tgame.yCoordUnderBrick =3;
	  tgame.XCoordLeftBrick =2;
	  tgame.randomcounter = 0;

int randomvararr[9] ={4,1,3,2,3,3,1,0,0 };
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  FillArr(&tgame);
int ycoordinate = 3;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  debug_print("Hello from STM32L432KC dev board\r\n");

  init();
  newblock(&tgame,ycoordinate, randomvararr[tgame.randomcounter]);
  while (1)
  {

    /* USER CODE END WHILE */


	  dropblock(&tgame, ycoordinate,randomvararr[tgame.randomcounter]);
		// placeblock(&tgame,ycoordinate);

		  printLCD(&tgame);
		  ssd1306_UpdateScreen();

			HAL_Delay(500);
    /* USER CODE BEGIN 3 */


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/*void dropblock(tetrisgame *t, int ycoord)
{bool blockhit = 0;
	for(int i = 0; i < 8;i++){
		t->ghostBlockField[t->yCoordUnderBrick+1][i] =t->ghostBlockField[t->yCoordUnderBrick][i];
		t->ghostBlockField[t->yCoordUnderBrick][i] = 0;
		if(((t->playingfield[t->yCoordUnderBrick+2][i] == 1 && t->ghostBlockField[t->yCoordUnderBrick+1][i]) == 1) ||t->yCoordUnderBrick > 13 ){
		        blockhit = 1;
		      }
	}*/

void dropblock(tetrisgame *t, int ycoord,int randomvar)
{
    bool blockhit = 0;
    for(int a = t->yCoordUnderBrick; a >= t->yCoordUnderBrick - 3; a--){
        for(int i = 0; i < 8;i++){
            t->ghostBlockField[a+1][i] =t->ghostBlockField[a][i];
            t->ghostBlockField[a][i] = 0;
            if(((t->playingfield[a+2][i] == 1 && t->ghostBlockField[a+1][i]) == 1) ||t->yCoordUnderBrick > 13 ){
            blockhit = 1;
         }
      }
    }
if(blockhit==1)
{
	placeblock(t,t->yCoordUnderBrick);
		//t->yCoordUnderBrick =3;
		newblock(t,t->yCoordUnderBrick,randomvar );
}

else
{
	 t->yCoordUnderBrick= t->yCoordUnderBrick+1;


}
}

void placeblock(tetrisgame *t,int yCoord)
{
for(int y = 0;y<=15; y++)
{
		for(int x = 0 ;x<8; x++)
		{
			t->playingfield[y][x] = t->playingfield[y][x] | t->ghostBlockField[y][x];
		}
}
}
/*
void placeblock(tetrisgame *t,int yCoord) {
	bool placevalue = 0;



	if(t->yCoordUnderBrick > 19) {
		placevalue = 1;
	}
	else {
		/*for(int x = t->yCoordUnderBrick; x <= (int)t->height + (int)t->yCoordUnderBrick; x++) {
			for(int y = t->XCoordLeftBrick; y <= (int)t->width + (int)t->XCoordLeftBrick; y++) {
		        if(t->playingfield[x+1][y] == 1 && t->ghostBlockField[x][y] == 1) { placevalue = 1; }
		    }*/
/*
		  for(int x =0; x <8;x++ ) {
		      for(int y =0;y<19;y++ ) {
			  if( t->playingfield[y+1][x] == 1&&t->ghostBlockField[y][x] == 1){
				  	  placevalue = 1;

			  }
			  // x++;
		      }
		  }

		}
	if(placevalue == 1) {

		//Place block
		for(int y = t->yCoordUnderBrick; y <= (int)t->height + (int)t->yCoordUnderBrick; y++) {
			for(int x = t->XCoordLeftBrick; x <= (int)t->width + (int)t->XCoordLeftBrick; x++) {
				t->playingfield[x][y] = t->ghostBlockField[x][y] & t->playingfield[x][y];
	     	    }
			}
		//Clear Ghost Arr
		  for(int x =0; x <20;x++ ) {
		      for(int y =0;y<8;y++ ) {
			  t->ghostBlockField[x][y]= 0;
			 // x++;
		      }
		  }
		  	t->yCoordUnderBrick = 3;
		  	 t->XCoordLeftBrick = 2;

		  newblock(t,yCoord);
	    }



	else {

		for(int i = 0; i < 8;i++){
			t->ghostBlockField[t->yCoordUnderBrick+1][i] =t->ghostBlockField[t->yCoordUnderBrick][i];
			t->ghostBlockField[t->yCoordUnderBrick][i] = 0;
		}

		 t->yCoordUnderBrick = t->yCoordUnderBrick+1;
        }
        }

*/



void printLCD(tetrisgame *t) {
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
        for(int x = 0; x < t->width; x++) {
            for(int y = t->bufferheight-1; y < (int)t->height; y++) {
                if(t->playingfield[y][x] == 1) {
                    for(int a = (x * 8); a < (x * 8)+7; a++)
                    {
                    	for(int b = (y * 8); b < (y*8)+7; b++)
                    	{
                    		ssd1306_DrawPixel(b,a,White);
                    	}
                    }
                }
                if(t->ghostBlockField[y][x] == 1) {
                    for(int a = (x * 8); a < (x * 8)+7; a++)
                    {
                    	for(int b = (y * 8); b < (y*8)+7; b++)
                    	{
                    		ssd1306_DrawPixel(b,a,White);
                    	}
                 }

               }
            }

        }
        ssd1306_UpdateScreen();
    }




void FillArr(tetrisgame *t)
{
  for(int x =0; x <8;x++ ) {
      for(int y =0;y<20;y++ ) {
	  t->playingfield[y][x]= 0;
	  t->ghostBlockField[y][x]= 0;
	 // x++;
      }
  }
}
void newblock(tetrisgame *t, int ycoordinate,int randomvar)
{

     block currentblock;
     block longBlock = {.height =1,.width =4, .blockShape = {{0,0,0,0},{0,0,1,1},{0,0,1,1}}};
     block blitzBlock = {.height = 2,.width = 2, .blockShape = {{0,1,1,0},{0,1,1,0},{0,0,0,0}}};
     block Lblock = {.height =3,.width =2,.blockShape = {{1,0,0,0},{1,0,0,0},{1,0,0,0}}};
     block Tblock = {.height = 3,.width =3,.blockShape = {{0,1,1,0},{0,1,1,0},{0,1,1,0}}};
     switch (blockcounter)
     {
     case 0 : currentblock = longBlock; break;
     case 1 : currentblock = blitzBlock; break;
     case 2 : currentblock = Lblock; break;
     case 3 : currentblock = Tblock; break;
     }
     blockcounter ++;
     if(blockcounter >3){
    	 blockcounter = 0;
     }
t-> randomcounter ++;
if(t->randomcounter>9){
	 blockcounter = 0;
}
        for(int x=0; x< 4; x++){
            for(int y= 0; y<3; y++){

                t->ghostBlockField [y][x+randomvar] = currentblock.blockShape[y][x];
                t->yCoordUnderBrick = 3;
                t->XCoordLeftBrick = currentblock.height;
                //increment pointer for next element fetch//
                //blocks++;
            }
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
