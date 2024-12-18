/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    GPIO_TypeDef* DacCsPort;
	uint16_t DacCsPin;
	SPI_HandleTypeDef *spi_port;
	uint32_t spi_clock_speed;
} DAC_Handle;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ADC Defines */
#define VREF            (3.3f)
#define ADC_MAX_RESULT  ((1 << 12)) // 10 bit ADC
#define MCU_CLOCK       (51200000U)

/* FFT Defines */
#define FFT_BUFFER_SIZE             (1024U)  /* This is the max power of 2, to fit in the RAM. The used array can be reduced. */
#define FREQ_COMPONENTS_THRESHOLD   (0.05f)
#define MAX_FREQ_PEAKS              (19U)

/* DAC register defines begin */
#define REG4_DIVGAIN  ((uint8_t)0x04)
#define REG5_RESET    ((uint8_t)0x05)
#define REG8_DAC      ((uint8_t)0x08)
#define REG3_CON      ((uint8_t)0x03)
#define REG2_SYNC     ((uint8_t)0x02)

#define SET5_RESET_1  ((uint8_t)0x0A)
#define SET5_RESET_2  ((uint8_t)0x00)

#define SET3_CON_1    ((uint8_t)0x00)
#define SET3_CON_2    ((uint8_t)0x01)

#define SET2_SYNC_1   ((uint8_t)0x00)
#define SET2_SYNC_2   ((uint8_t)0x00)

#define SET4_GAIN1    ((uint8_t)0x00)
#define SET4_GAIN2    ((uint8_t)0x01)
#define SET4_DIV1     ((uint8_t)0x00)
#define SET4_DIV2     ((uint8_t)0x01)
/* DAC register defines end */

/* DAC config defines begin */
#define NUM_OF_SAMPLES_DAC ((uint16_t)1024)
#define SPI_TIMEOUT        (10u)
/* DAC config defines end */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* DAC variables */
DAC_Handle dacHandle;

/* Defs to variables, Configuring DAC */
uint8_t dacResetRegData[3] = {REG5_RESET,SET5_RESET_2,SET5_RESET_1}; // Fix
uint8_t dacDivGainRegData[3] = {REG4_DIVGAIN,SET4_DIV2,SET4_GAIN2}; // Can be set
uint8_t dacConfigRefRegData[3] = {REG3_CON,SET3_CON_2,SET3_CON_1}; // Fix
uint8_t dacSyncRegData[3] = {REG2_SYNC,SET2_SYNC_2,SET2_SYNC_1}; // Fix
/* DAC config variables end */

/* Data to send - Multisine: sum(Vi*sin(2pi*f0*i*t)) */
uint8_t sinBuffer[2*NUM_OF_SAMPLES_DAC] = {
#include "dacMultiSine.txt"
};
volatile uint8_t sinBufferTemp[3] = {REG8_DAC,0x00,0x00};
volatile uint16_t sinCounter = 0;
volatile uint8_t sinDataCounter = 0;
/* Signal generation variables end */


/* Measurement variables begin */

// ADC variables
uint8_t ADC_FLAG_STATE_FSM = 0;


// FFT variables
arm_rfft_fast_instance_f32 fftHandler;
uint32_t fftBuffIn[FFT_BUFFER_SIZE*2];
float32_t fftBuffInVoltsSig[FFT_BUFFER_SIZE];
float32_t fftBuffInVoltsRef[FFT_BUFFER_SIZE];
float32_t fftBuffInVoltsImp[FFT_BUFFER_SIZE];
float32_t fftBuffOutRef[FFT_BUFFER_SIZE];
float32_t fftBuffOutImp[FFT_BUFFER_SIZE];
float32_t fftBuffOutSig[FFT_BUFFER_SIZE];
float32_t fftBuffOutZpN[FFT_BUFFER_SIZE];
float32_t fftBuffAbsRef[FFT_BUFFER_SIZE/2];
float32_t fftBuffAbsImp[FFT_BUFFER_SIZE/2];
float32_t fftBuffAbsSig[FFT_BUFFER_SIZE/2];
float32_t fftBuffAbsZpN[FFT_BUFFER_SIZE/2];

uint16_t freqIndex = 0;

/* Collecting the peaks in an array */
uint16_t freqPeakIndeces[MAX_FREQ_PEAKS] = {0x00,0x00,0x00,0x00,0x00};
float32_t freqPeaks[MAX_FREQ_PEAKS] = {0.0f,0.0f,0.0f,0.0f,0.0f};

uint32_t SAMPLING_FREQ = 0; // --> Should be set properly, according to the CubeMX settings!!!

/* Measurement variables end */


/* Config variables begin */
uint8_t send_serial = 1; // send on serial or not
volatile uint8_t  send_data = 0;
uint16_t data_ind = 0; // instead of i in main for loop, to live watch
/* Config variables end */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void dacBegin(DAC_Handle *dac, SPI_HandleTypeDef *port, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint32_t cspeed);
void writeREG(DAC_Handle *dac, uint8_t *command);
void writeDAC(DAC_Handle *dac, uint8_t *data);
void setClockSpeed(DAC_Handle *dac, uint32_t cspeed);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Redirect printf to UASRT2, redefine __write */
size_t __write_own(int handle, const unsigned char * buffer, size_t size)
{
  size_t nChars = 0;

  if (buffer == 0)
  {
    /*
     * This means that we should flush internal buffers.  Since we
     * don't we just return.  (Remember, "handle" == -1 means that all
     * handles should be flushed.)
     */
    return 0;
  }

  for (/* Empty */; size != 0; --size)
  {
    HAL_UART_Transmit(&huart2, (uint8_t*) (buffer), 1, HAL_MAX_DELAY);
    buffer++;
    ++nChars;
  }

  return nChars;

}

/* DAC handling functions begin*/
void dacBegin(DAC_Handle *dac, SPI_HandleTypeDef *port, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint32_t cspeed)
{
  dac->DacCsPort = cs_port;
  dac->DacCsPin = cs_pin;
  dac->spi_port = port;
  dac->spi_clock_speed = cspeed;
}

void setClockSpeed(DAC_Handle *dac, uint32_t cspeed)
{
	dac->spi_clock_speed = cspeed;
}

void writeREG(DAC_Handle *dac, uint8_t* command)
{
	HAL_GPIO_WritePin(dac->DacCsPort, dac->DacCsPin, GPIO_PIN_SET); //SW NSS
        HAL_Delay(1);
  	HAL_GPIO_WritePin(dac->DacCsPort, dac->DacCsPin, GPIO_PIN_RESET); //SW NSS
  	while(HAL_SPI_GetState(dac->spi_port) != HAL_SPI_STATE_READY){}
  	HAL_SPI_Transmit(dac->spi_port, (uint8_t*) (command), 3, SPI_TIMEOUT);
        HAL_Delay(1);
  	HAL_GPIO_WritePin(dac->DacCsPort, dac->DacCsPin, GPIO_PIN_SET); //SW NSS
}

void writeDAC(DAC_Handle *dac, uint8_t* data)
{
	while(HAL_SPI_GetState(dac->spi_port) != HAL_SPI_STATE_READY){}
        HAL_GPIO_WritePin(dac->DacCsPort, dac->DacCsPin, GPIO_PIN_RESET); //SW NSS
        HAL_SPI_Transmit(dac->spi_port, (uint8_t*) (data), 3, SPI_TIMEOUT);
        HAL_GPIO_WritePin(dac->DacCsPort, dac->DacCsPin, GPIO_PIN_SET); //SW NSS

}
/* DAC handling functions end*/

/* ADC conversion to Volts */
float32_t ADC_convertSampleToVolts(uint16_t sample) {
	// The measured voltage applied to P1.7 is given by:
	//
	//                           Vref (mV)
	//   measurement (mV) =   --------------- * result (bits)
	//                       (2^12) (bits)
	return ((float32_t) sample * VREF) / ADC_MAX_RESULT;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  /* Calculate the sampling freq from registers and oscillator freq */
  SAMPLING_FREQ = ((uint32_t)MCU_CLOCK)/((htim3.Init.Prescaler+1)*(htim3.Init.Period+1)); // 1 kHz sampling freq is too high for this settings
  
  /* Initializing DAC */
  dacBegin(&dacHandle, &hspi1, SPI_SW_NSS_GPIO_Port, SPI_SW_NSS_Pin,(uint32_t)500000); // 16 Mhz Oscillator - 8 Mhz CLK - 16 SPI prescaler = 500 kHz

  // setREG5_Reset, Soft Reset
  writeREG(&dacHandle, (uint8_t*)(&dacResetRegData));
  HAL_Delay(100);
  
  // setREG4_DivGain, DIV = 2, GAIN = 2
  writeREG(&dacHandle, (uint8_t*)(&dacDivGainRegData));
  HAL_Delay(100);

  // setREG3_Internal_Ref_Off
  writeREG(&dacHandle, (uint8_t*)(&dacConfigRefRegData));
  HAL_Delay(100);

  // setREG2_SyncMode_Off
  writeREG(&dacHandle, (uint8_t*)(&dacSyncRegData));
  HAL_Delay(100);
  
  /* Start TIM2(for DAC sine) and TIM3(for ADC), init FFT vector and start DMA for FFT */
  arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE);
  send_data = 1;
  
  HAL_TIM_Base_Init(&htim3);
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, fftBuffIn, FFT_BUFFER_SIZE*2); // Start DMA for FFT
  /* ADC with DMA must be init and started before TIM2 IT-s, because they interrupt the init method!!!! */
  HAL_TIM_Base_Init(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/* Processing signals */
	if (ADC_FLAG_STATE_FSM == 1) {
		// Reset state machine
		ADC_FLAG_STATE_FSM = 0;

		/* ADC_IN8 (Rank 1 - 1. in the array) - input signal, ADC_IN9 (Rank 2 - 2. in the array) - Rn signal */

		// Calculate FFT
		for (int i = 0; i < FFT_BUFFER_SIZE; i++) {
			fftBuffInVoltsSig[i] = ADC_convertSampleToVolts((uint16_t) (fftBuffIn[2 * i])); // Rn signal
			fftBuffInVoltsRef[i] = ADC_convertSampleToVolts((uint16_t) (fftBuffIn[2 * i + 1])); // Input signal
			fftBuffInVoltsImp[i] = ADC_convertSampleToVolts((uint16_t) (fftBuffIn[2 * i]-fftBuffIn[2 * i + 1])); // Impedance signal
		}

		/* FFT */
		freqIndex = 0;
		arm_rfft_fast_f32(&fftHandler, fftBuffInVoltsRef, fftBuffOutRef, 0);
		arm_rfft_fast_f32(&fftHandler, fftBuffInVoltsImp, fftBuffOutImp, 0);
		arm_rfft_fast_f32(&fftHandler, fftBuffInVoltsSig, fftBuffOutSig, 0);
                

		/* Calculating spectres first component */
		for (uint16_t i = 0; i < FFT_BUFFER_SIZE-2; i+=2){

			fftBuffAbsSig[freqIndex] = (float32_t)(sqrt((double)(fftBuffOutSig[i]*fftBuffOutSig[i])+(double)(fftBuffOutSig[i+1]*fftBuffOutSig[i+1]))/FFT_BUFFER_SIZE*2); // Calculating the amplitude characteristics
			fftBuffAbsRef[freqIndex] = (float32_t)(sqrt((double)(fftBuffOutRef[i]*fftBuffOutRef[i])+(double)(fftBuffOutRef[i+1]*fftBuffOutRef[i+1]))/FFT_BUFFER_SIZE*2); // Calculating the amplitude characteristics
			fftBuffAbsImp[freqIndex] = (float32_t)(sqrt((double)(fftBuffOutImp[i]*fftBuffOutImp[i])+(double)(fftBuffOutImp[i+1]*fftBuffOutImp[i+1]))/FFT_BUFFER_SIZE*2); // Calculating the amplitude characteristics
			fftBuffOutZpN[i] = ((fftBuffOutImp[i]*fftBuffOutRef[i])+(fftBuffOutImp[i+1]*fftBuffOutRef[i+1]))/((fftBuffOutRef[i]*fftBuffOutRef[i])+(fftBuffOutRef[i+1]*fftBuffOutRef[i+1]));
			fftBuffOutZpN[i+1] = ((fftBuffOutRef[i]*fftBuffOutImp[i+1])-(fftBuffOutImp[i]*fftBuffOutRef[i+1]))/((fftBuffOutRef[i]*fftBuffOutRef[i])+(fftBuffOutRef[i+1]*fftBuffOutRef[i+1]));
			fftBuffAbsZpN[freqIndex] = (float32_t)(sqrt((double)(fftBuffOutZpN[i]*fftBuffOutZpN[i])+(double)(fftBuffOutZpN[i+1]*fftBuffOutZpN[i+1]))/FFT_BUFFER_SIZE*2);

			freqIndex++;
		}
		fftBuffAbsRef[0] = fftBuffAbsRef[0]/2; // For the single sided FFT, but we do not need it.
		fftBuffAbsImp[0] = fftBuffAbsImp[0]/2;
		fftBuffAbsSig[0] = fftBuffAbsSig[0]/2;
                fftBuffAbsZpN[0] = fftBuffAbsZpN[0]/2;

                
                /* 
                // Printing out the absolute spectres 
		for (uint16_t i = 0; i < FFT_BUFFER_SIZE/2; i++){ 
			printf("%.5f,%.5f,%.5f,%.5f\r\n",fftBuffAbsSig[i],fftBuffAbsImp[i],fftBuffAbsRef[i],fftBuffAbsZpN[i]);
		}
                
                // Printing out the complex spectres 
		for (uint16_t i = 0; i < FFT_BUFFER_SIZE-2; i+=2){ 
			printf("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",fftBuffOutSig[i],fftBuffOutSig[i+1],fftBuffOutImp[i],fftBuffOutImp[i+1],fftBuffOutRef[i],fftBuffOutRef[i+1],fftBuffOutZpN[i],fftBuffOutZpN[i+1]);
		}
                */
                
                for (int i = 0; i < MAX_FREQ_PEAKS; i++) {
                    freqPeaks[i] = 0.0f;
                    freqPeakIndeces[i] = 0xFFFF;
                }
                
                // Iterate through the original array
                for (int i = 1; i < FFT_BUFFER_SIZE/2; i++) {
                    // Check if the current element is larger than the smallest in the largest array
                    for (int j = 0; j < MAX_FREQ_PEAKS; j++) {
                        if (fftBuffAbsSig[i] > freqPeaks[j]) {
                            // Shift the rest of the array and the indices array
                            for (int k = MAX_FREQ_PEAKS - 1; k > j; k--) {
                                freqPeaks[k] = freqPeaks[k - 1];
                                freqPeakIndeces[k] = freqPeakIndeces[k - 1];
                            }
                            // Insert the new element and its index
                            freqPeaks[j] = fftBuffAbsSig[i];
                            freqPeakIndeces[j] = i;
                            break;
                        }
                    }
                }
                
                // Printing out the complex spectres 
		for (uint16_t i = 0; i < FFT_BUFFER_SIZE-2; i+=2){ 
			printf("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",fftBuffOutSig[i],fftBuffOutSig[i+1],fftBuffOutImp[i],fftBuffOutImp[i+1],fftBuffOutRef[i],fftBuffOutRef[i+1],fftBuffOutZpN[i],fftBuffOutZpN[i+1]);
		}
                
                             
                /* Printing out the impeadance value at the peak indeces  */
                for (uint8_t i = 0; i < MAX_FREQ_PEAKS; i++){ 
			printf("%d,%.5f,%.5f,%.5f\r\n",freqPeakIndeces[i],fftBuffOutZpN[2*freqPeakIndeces[i]],fftBuffOutZpN[2*freqPeakIndeces[i]+1],fftBuffAbsZpN[freqPeakIndeces[i]]);
		}
                 
                
		HAL_ADC_Start_DMA(&hadc1, fftBuffIn, FFT_BUFFER_SIZE*2); // Start DMA for FFT
		__enable_irq();


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 499;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 499;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SW_NSS_GPIO_Port, SPI_SW_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA6 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SW_NSS_Pin */
  GPIO_InitStruct.Pin = SPI_SW_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SW_NSS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	__disable_irq();
	HAL_ADC_Stop_DMA(hadc);
	ADC_FLAG_STATE_FSM = 1;
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
