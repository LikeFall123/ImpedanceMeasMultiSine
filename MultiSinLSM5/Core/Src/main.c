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
#define VREF         (3.3)
#define ADC_MAX_RESULT  ((1 << 12)) // 10 bit ADC

/* Global Defines */
#define REF_RESISTANCE   (1000.0L)

/* LSM Defines */
#define NUM_OF_SAMPLES   (450U)
#define NUM_LSM_PARAMS   (5U)

/* FFT Defines */
#define FFT_BUFFER_SIZE   (2048U)

/* DAC register defines begin */
#define REG4_DIVGAIN  ((uint8_t)0x04)
#define REG5_RESET    ((uint8_t)0x05)
#define REG8_DAC      ((uint8_t)0x08)
#define REG3_CON      ((uint8_t)0x03)
#define REG2_SYNC     ((uint8_t)0x02)

#define SET5_RESET_1 ((uint8_t)0x0A)
#define SET5_RESET_2 ((uint8_t)0x00)

#define SET3_CON_1 ((uint8_t)0x00)
#define SET3_CON_2 ((uint8_t)0x01)

#define SET2_SYNC_1 ((uint8_t)0x00)
#define SET2_SYNC_2 ((uint8_t)0x00)

#define SET4_GAIN1 ((uint8_t)0x00)
#define SET4_GAIN2 ((uint8_t)0x01)
#define SET4_DIV1  ((uint8_t)0x00)
#define SET4_DIV2  ((uint8_t)0x01)
/* DAC register defines end */

/* DAC config defines begin */
#define NUM_OF_SAMPLES_DAC ((uint16_t)1000)
#define SPI_TIMEOUT        (10U)
#define MCU_CLOCK          (84000000U)
/* DAC config defines end */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

enum ImpedanceType {
  LR_SERIES,
  LR_PARALLEL,
  CR_SERIES,
  CR_PARALLEL
};

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

/* Data to send - Multisine: 1 Hz (0.7/2 V)+ 10 Hz (0.3/2 V)*/

uint8_t sinBuffer[2*NUM_OF_SAMPLES_DAC] = {
#include "dacMultiSine.txt"
};

volatile uint8_t sinBufferTemp[3] = {REG8_DAC,0x00,0x00};
volatile uint8_t sinBufferSwitch = 0;
volatile uint16_t sinCounter = 0;
volatile uint8_t sinDataCounter = 0;
/* Signal generation variables end */


/* Measurement variables begin */
// LSM matrices
float32_t mat_DT[NUM_LSM_PARAMS][NUM_OF_SAMPLES]; //calculating
float32_t mat_DT_D[NUM_LSM_PARAMS][NUM_LSM_PARAMS]; //calculating
float32_t mat_DT_D_inv[NUM_LSM_PARAMS][NUM_LSM_PARAMS]; //calculating
float32_t vect_ref_sig[NUM_OF_SAMPLES][1];
float32_t vect_input_sig[NUM_OF_SAMPLES][1];
float32_t vect_ref_lsm[NUM_LSM_PARAMS][1];
float32_t vect_input_lsm[NUM_LSM_PARAMS][1];
float32_t lsm_temp[NUM_LSM_PARAMS][1];

arm_matrix_instance_f32 mat_DT_instance; //calculating
arm_matrix_instance_f32 mat_DT_D_instance; //calculating
arm_matrix_instance_f32 mat_DT_D_inv_instance; //calculating
arm_matrix_instance_f32 vect_ref_instance;
arm_matrix_instance_f32 vect_input_instance;
arm_matrix_instance_f32 vect_ref_lsm_instance;
arm_matrix_instance_f32 vect_input_lsm_instance;
arm_matrix_instance_f32 lsm_temp_instance;

arm_status arm_mult_ref;
arm_status arm_mult_input;
arm_status arm_inverse;

// LSM result - For two sines
float32_t ref_sig_amp_1 = 0;
float32_t input_sig_amp_1 = 0;
float32_t ref_sig_phase_1 = 0;
float32_t input_sig_phase_1 = 0;

float32_t ref_sig_amp_2 = 0;
float32_t input_sig_amp_2 = 0;
float32_t ref_sig_phase_2 = 0;
float32_t input_sig_phase_2 = 0;

// ADC variables
uint32_t adc_values[NUM_OF_SAMPLES * 2];
volatile uint8_t ADC_FLAG_STATE_FSM = 0;

// Calculated variables - for two sines
float32_t phaseShift_1 = (0UL);
float32_t realImp_1 = (0UL);
float32_t imagImp_1 = (0UL);
float32_t absImp_1 = (0UL);
float32_t impedance_sig_amp_1 = 0;
float32_t impedance_sig_phase_1 = 0;

float32_t phaseShift_2 = (0UL);
float32_t realImp_2 = (0UL);
float32_t imagImp_2 = (0UL);
float32_t absImp_2 = (0UL);
float32_t impedance_sig_amp_2 = 0;
float32_t impedance_sig_phase_2 = 0;
float32_t resistance_1 = 0.0;
float32_t reactance_1 = 0.0;
float32_t resistance_2 = 0.0;
float32_t reactance_2 = 0.0;

// FFT variables
arm_rfft_fast_instance_f32 fftHandler;
uint32_t fftBuffIn[FFT_BUFFER_SIZE*2];
float32_t fftBuffInVolts[FFT_BUFFER_SIZE*2];
float32_t fftBuffOut[FFT_BUFFER_SIZE];
float32_t fftBuffAbs[FFT_BUFFER_SIZE/2];
float32_t peakVal = 0.0f;
uint16_t peakHz = 0;
float32_t peakHzCorrected = 0.0;

uint16_t freqIndex = 0;
uint16_t freqIndexPeak_1 = 0;
uint16_t freqIndexPeak_2 = 0;
uint16_t k = 0;
float32_t curVal = 0;
float32_t delta = 0;
uint8_t FFT_FLAG = 1;

float32_t SIGNAL_FREQ_1 = 15.0;
float32_t SIGNAL_FREQ_2 = 15.0;
uint32_t SAMPLING_FREQ = 0; // --> Should be set properly, according to the CubeMX settings!!!

/* Measurement variables end */

/* Config variables begin */
uint8_t send_serial = 1; // send on serial or not
volatile uint8_t  send_data = 0;
uint16_t data_ind = 0; // insted of i in main for loop, to live watch
enum ImpedanceType imp_type = CR_SERIES;
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

/* Measurement functions begin */
void fill_mat_DT_D() {
	int i = 0;

	/* 5x5 D'*D matrix elements */
	float32_t cos1_sum = 0.0;
	float32_t cos1_square_sum = 0.0;
	float32_t sin1_sum = 0.0;
	float32_t sin1_square_sum = 0.0;
	float32_t cos2_sum = 0.0;
	float32_t cos2_square_sum = 0.0;
	float32_t sin2_sum = 0.0;
	float32_t sin2_square_sum = 0.0;

	float32_t cos1_sin1_sum = 0.0;
	float32_t cos1_sin2_sum = 0.0;
	float32_t cos2_sin1_sum = 0.0;
	float32_t cos2_sin2_sum = 0.0;

	float32_t cos1_cos2_sum = 0.0;
	float32_t sin1_sin2_sum = 0.0;

	float32_t temp_cos_1 = 0.0;
	float32_t temp_sin_1 = 0.0;
	float32_t temp_cos_2 = 0.0;
	float32_t temp_sin_2 = 0.0;

	for (i = 0; i < NUM_OF_SAMPLES; i++) {
		temp_cos_1 = (float32_t) (cos(2.0 * PI * SIGNAL_FREQ_1 * (((double) i) / (double)SAMPLING_FREQ)));
		temp_sin_1 = (float32_t) (sin(2.0 * PI * SIGNAL_FREQ_1 * (((double) i) / (double)SAMPLING_FREQ)));
		temp_cos_2 = (float32_t) (cos(2.0 * PI * SIGNAL_FREQ_2 * (((double) i) / (double)SAMPLING_FREQ)));
		temp_sin_2 = (float32_t) (sin(2.0 * PI * SIGNAL_FREQ_2 * (((double) i) / (double)SAMPLING_FREQ)));

		/* Diagonal and last row */
		cos1_sum = cos1_sum + temp_cos_1;
		cos1_square_sum = cos1_square_sum + temp_cos_1*temp_cos_1;
		sin1_sum = sin1_sum + temp_sin_1;
		sin1_square_sum = sin1_square_sum + temp_sin_1*temp_sin_1;
		cos2_sum = cos2_sum + temp_cos_2;
		cos2_square_sum = cos2_square_sum + temp_cos_2*temp_cos_2;
		sin2_sum = sin2_sum + temp_sin_2;
		sin2_square_sum = sin2_square_sum + temp_sin_2*temp_sin_2;

		/* Above the main diagonal */
		cos1_sin1_sum = cos1_sin1_sum + temp_cos_1*temp_sin_1;
		cos1_sin2_sum = cos1_sin2_sum + temp_cos_1*temp_sin_2;
		cos2_sin1_sum = cos2_sin1_sum + temp_cos_2*temp_sin_1;
		cos2_sin2_sum = cos2_sin2_sum + temp_cos_2*temp_sin_2;

		cos1_cos2_sum = cos1_cos2_sum + temp_cos_1*temp_cos_2;
		sin1_sin2_sum = sin1_sin2_sum + temp_sin_1*temp_sin_2;

		/* Init D' for multipling it with the measured signal */
		mat_DT[0][i] = (float32_t) temp_cos_1;
		mat_DT[1][i] = (float32_t) temp_sin_1;
		mat_DT[2][i] = (float32_t) temp_cos_2;
		mat_DT[3][i] = (float32_t) temp_sin_2;
		mat_DT[4][i] = (float32_t) 1;
	}

	mat_DT_D[0][0] = cos1_square_sum;
	mat_DT_D[0][1] = cos1_sin1_sum;
	mat_DT_D[0][2] = cos1_cos2_sum;
	mat_DT_D[0][3] = cos1_sin2_sum;
	mat_DT_D[0][4] = cos1_sum;

	mat_DT_D[1][0] = cos1_sin1_sum;
	mat_DT_D[1][1] = sin1_square_sum;
	mat_DT_D[1][2] = cos2_sin1_sum;
	mat_DT_D[1][3] = sin1_sin2_sum;
	mat_DT_D[1][4] = sin1_sum;

	mat_DT_D[2][0] = cos1_cos2_sum;
	mat_DT_D[2][1] = cos2_sin1_sum;
	mat_DT_D[2][2] = cos2_square_sum;
	mat_DT_D[2][3] = cos2_sin2_sum;
	mat_DT_D[2][4] = cos2_sum;

	mat_DT_D[3][0] = cos1_sin2_sum;
	mat_DT_D[3][1] = sin1_sin2_sum;
	mat_DT_D[3][2] = cos2_sin2_sum;
	mat_DT_D[3][3] = sin2_square_sum;
	mat_DT_D[3][4] = sin2_sum;

	mat_DT_D[4][0] = cos1_sum;
	mat_DT_D[4][1] = sin1_sum;
	mat_DT_D[4][2] = cos2_sum;
	mat_DT_D[4][3] = sin2_sum;
	mat_DT_D[4][4] = NUM_OF_SAMPLES;

}

void init_vectors(){
	arm_mat_init_f32(&mat_DT_instance, NUM_LSM_PARAMS, NUM_OF_SAMPLES, &mat_DT[0][0]);
	arm_mat_init_f32(&mat_DT_D_instance, NUM_LSM_PARAMS, NUM_LSM_PARAMS, &mat_DT_D[0][0]);
	arm_mat_init_f32(&mat_DT_D_inv_instance, NUM_LSM_PARAMS, NUM_LSM_PARAMS, &mat_DT_D_inv[0][0]);

	arm_inverse = arm_mat_inverse_f32(&mat_DT_D_instance, &mat_DT_D_inv_instance);

	arm_mat_init_f32(&vect_ref_instance, NUM_OF_SAMPLES, 1, &vect_ref_sig[0][0]);
	arm_mat_init_f32(&vect_input_instance, NUM_OF_SAMPLES, 1, &vect_input_sig[0][0]);
	arm_mat_init_f32(&vect_ref_lsm_instance, NUM_LSM_PARAMS, 1, &vect_ref_lsm[0][0]);
	arm_mat_init_f32(&vect_input_lsm_instance, NUM_LSM_PARAMS, 1, &vect_input_lsm[0][0]);
	arm_mat_init_f32(&lsm_temp_instance, NUM_LSM_PARAMS, 1, &lsm_temp[0][0]);
}
/* Measurement functions end */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* Without using buffer */
  //uint16_t u16sineTemp = 0;
  //uint8_t u8sineTempLSB = 0;
  //uint8_t u8sineTempMSB = 0;

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
  
  /* Start TIM2(LL for DAC sine) and TIM3(for ADC), init FFT vector and start DMA for FFT */
  /* Start TIM2 for sending data*/
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

		// Calculate freq
		if (FFT_FLAG == 1) {
			for (int i = 0; i < FFT_BUFFER_SIZE; i++) {
				fftBuffInVolts[i] = ADC_convertSampleToVolts((uint16_t) (fftBuffIn[2 * i]));
				//printf("ADC1:%.5f,ADC3:%.5f,ADCdiff:%.5f\r\n",fftBuffInVolts[i],0,0);
                                //printf("%.5f\r\n",fftBuffInVolts[i]);
			}

			/* FFT - detecting two frequencies */
			peakVal = 0.0f;
			peakHz = 0;
			freqIndex = 0;
			arm_rfft_fast_f32(&fftHandler, fftBuffInVolts, fftBuffOut, 0);

			/* Detecting first component */
			for (uint16_t i = 0; i < FFT_BUFFER_SIZE-2; i+=2){

				curVal = (float32_t)(sqrt((double)(fftBuffOut[i]*fftBuffOut[i])+(double)(fftBuffOut[i+1]*fftBuffOut[i+1]))/FFT_BUFFER_SIZE*2); // Calculating the amplitude characteristics
				fftBuffAbs[freqIndex] = curVal;
				if ((curVal > peakVal)&&(i>0)){
					peakVal = curVal;
					freqIndexPeak_1 = freqIndex;
					//peakHz = (uint16_t)(freqIndex * SAMPLING_FREQ/((float32_t)FFT_BUFFER_SIZE)); // We don't need it!
				}
				freqIndex++;
			}
			fftBuffAbs[0] = fftBuffAbs[0]/2; // For the single sided FFT, but we do not need it.

			/* Detecting the second component */
			peakVal = 0.0f;
			for (uint16_t i = 1; i < FFT_BUFFER_SIZE; i++){ // Skip DC? Start from i = 1 (in the abs)
				if ((fftBuffAbs[i] > peakVal)&&(i != freqIndexPeak_1)){
					peakVal = fftBuffAbs[i];
					freqIndexPeak_2 = i;
					//peakHz = (uint16_t)(freqIndex * SAMPLING_FREQ/((float32_t)FFT_BUFFER_SIZE));
				}
			}

			/* Correcting the first freq (Jacobsen (3)) */
			k = freqIndexPeak_1;
			delta = -((fftBuffOut[2*(k+1)]-fftBuffOut[2*(k-1)])*(2*fftBuffOut[2*k]-fftBuffOut[2*(k-1)]-fftBuffOut[2*(k+1)])
							+ (fftBuffOut[2*(k+1)+1]-fftBuffOut[2*(k-1)+1])*(2*fftBuffOut[2*k+1]-fftBuffOut[2*(k-1)+1]-fftBuffOut[2*(k+1)+1]))
									/(float32_t)(pow((double)(2*fftBuffOut[2*k]-fftBuffOut[2*(k-1)]-fftBuffOut[2*(k+1)]),2)+(float32_t)pow((double)(2*fftBuffOut[2*k+1]-fftBuffOut[2*(k-1)+1]-fftBuffOut[2*(k+1)+1]),2));
			peakHzCorrected = ((float32_t)freqIndexPeak_1 + (float32_t)(delta))*((float32_t)SAMPLING_FREQ)/((float32_t)FFT_BUFFER_SIZE);
			SIGNAL_FREQ_1 = peakHzCorrected;

			/* Correcting the second freq (Jacobsen (3)) */
			k = freqIndexPeak_2;
			delta = -((fftBuffOut[2*(k+1)]-fftBuffOut[2*(k-1)])*(2*fftBuffOut[2*k]-fftBuffOut[2*(k-1)]-fftBuffOut[2*(k+1)])
							+ (fftBuffOut[2*(k+1)+1]-fftBuffOut[2*(k-1)+1])*(2*fftBuffOut[2*k+1]-fftBuffOut[2*(k-1)+1]-fftBuffOut[2*(k+1)+1]))
									/((float32_t)pow((double)(2*fftBuffOut[2*k]-fftBuffOut[2*(k-1)]-fftBuffOut[2*(k+1)]),2)+(float32_t)pow((double)(2*fftBuffOut[2*k+1]-fftBuffOut[2*(k-1)+1]-fftBuffOut[2*(k+1)+1]),2));
			peakHzCorrected = ((float32_t)freqIndexPeak_2 + (float32_t)(delta))*((float32_t)SAMPLING_FREQ)/((float32_t)FFT_BUFFER_SIZE);
			SIGNAL_FREQ_2 = peakHzCorrected;
                        
			fill_mat_DT_D(); // initialize (D'*D)^-1*D'
			init_vectors(); // init LSM matrices
			HAL_ADC_Start_DMA(&hadc1, adc_values, NUM_OF_SAMPLES * 2); // start DMA for processing
			FFT_FLAG=0; // end calculating freq
			__enable_irq();
		}

		// processing signals
		if (FFT_FLAG == 0) {
			for (data_ind = 0; data_ind < NUM_OF_SAMPLES; data_ind++) {
				vect_input_sig[data_ind][0] = ADC_convertSampleToVolts((uint16_t) (adc_values[2 * data_ind]));
				vect_ref_sig[data_ind][0] = ADC_convertSampleToVolts((uint16_t) (adc_values[2 * data_ind + 1]));
				/*if (send_serial){
                                  printf("%.5f,%.5f\r\n",(vect_input_sig[data_ind][0]),vect_ref_sig[data_ind][0]);
				} */                           
			}

			// (DT*D)^-1 multiplication with (DT*sig) - with temporary variable
			arm_mat_mult_f32(&mat_DT_instance, &vect_ref_instance, &lsm_temp_instance);
			arm_mult_ref = arm_mat_mult_f32(&mat_DT_D_inv_instance, &lsm_temp_instance, &vect_ref_lsm_instance);
			arm_mat_mult_f32(&mat_DT_instance, &vect_input_instance, &lsm_temp_instance);
			arm_mult_input = arm_mat_mult_f32(&mat_DT_D_inv_instance, &lsm_temp_instance, &vect_input_lsm_instance);

			if ((arm_mult_ref != ARM_MATH_SUCCESS)|| (arm_mult_input != ARM_MATH_SUCCESS)) {
				printf("Multiplication error!");
			} else {
				/* For 2 components! */
				ref_sig_amp_1 = (float32_t)(sqrt(pow(vect_ref_lsm[0][0], 2) + pow(vect_ref_lsm[1][0], 2)));
				ref_sig_phase_1 = (float32_t)(atan2(-vect_ref_lsm[1][0], vect_ref_lsm[0][0]) + PI / 2);
				ref_sig_amp_2 = (float32_t)(sqrt(pow(vect_ref_lsm[2][0], 2) + pow(vect_ref_lsm[3][0], 2)));
				ref_sig_phase_2 = (float32_t)(atan2(-vect_ref_lsm[3][0], vect_ref_lsm[2][0]) + PI / 2);

				input_sig_amp_1 = (float32_t)(sqrt(pow(vect_input_lsm[0][0], 2) + pow(vect_input_lsm[1][0], 2)));
				input_sig_phase_1 = (float32_t)(atan2(-vect_input_lsm[1][0], vect_input_lsm[0][0]) + PI / 2);
				input_sig_amp_2 = (float32_t)(sqrt(pow(vect_input_lsm[2][0], 2) + pow(vect_input_lsm[3][0], 2)));
				input_sig_phase_2 = (float32_t)(atan2(-vect_input_lsm[3][0], vect_input_lsm[2][0]) + PI / 2);

				impedance_sig_amp_1 = (float32_t)(sqrt(pow((double)ref_sig_amp_1, 2) + pow((double)input_sig_amp_1, 2) - 2 * (double)ref_sig_amp_1 * (double)input_sig_amp_1 * cos((double)(input_sig_phase_1 - ref_sig_phase_1))));
				impedance_sig_amp_2 = (float32_t)(sqrt(pow((double)ref_sig_amp_2, 2) + pow((double)input_sig_amp_2, 2) - 2 * (double)ref_sig_amp_2 * (double)input_sig_amp_2 * cos((double)(input_sig_phase_2 - ref_sig_phase_2))));
				impedance_sig_phase_1 = atan2((input_sig_amp_1 * sin(input_sig_phase_1) - ref_sig_amp_1 * sin(ref_sig_phase_1)),(input_sig_amp_1 * cos(input_sig_phase_1) - ref_sig_amp_1 * cos(ref_sig_phase_1)));
				impedance_sig_phase_2 = atan2((input_sig_amp_2 * sin(input_sig_phase_2) - ref_sig_amp_2 * sin(ref_sig_phase_2)),(input_sig_amp_2 * cos(input_sig_phase_2) - ref_sig_amp_2 * cos(ref_sig_phase_2)));
                                
				absImp_1 = impedance_sig_amp_1 / ref_sig_amp_1 * REF_RESISTANCE;
				phaseShift_1 =  impedance_sig_phase_1 - ref_sig_phase_1;
				realImp_1 = absImp_1*cos(phaseShift_1);
				imagImp_1 = absImp_1*sin(phaseShift_1);
                                
				absImp_2 = impedance_sig_amp_2 / ref_sig_amp_2 * REF_RESISTANCE;
				phaseShift_2 =  impedance_sig_phase_2 - ref_sig_phase_2;
				realImp_2 = absImp_2*cos(phaseShift_2);
				imagImp_2 = absImp_2*sin(phaseShift_2);
                                
                                /* Print the measured components to serial - according to the selected type.  */
				if (send_serial){
                                    switch (imp_type){                                   
                                      case LR_SERIES:
                                        resistance_1 = realImp_1;
                                        reactance_1 = (float32_t)(imagImp_1/(2*PI*SIGNAL_FREQ_1));
                                        resistance_2 = realImp_2;
                                        reactance_2 = (float32_t)(imagImp_2/(2*PI*SIGNAL_FREQ_2));
                                        //printf("fs=%d f1=%.3f R=%.6f L=%.6f, f2=%.3f R=%.6f L=%.6f\r\n",SAMPLING_FREQ,SIGNAL_FREQ_1,resistance_1,reactance_1, SIGNAL_FREQ_2,resistance_2,reactance_2);
                                        printf("%d,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f\r\n",SAMPLING_FREQ,SIGNAL_FREQ_1,resistance_1,reactance_1, SIGNAL_FREQ_2,resistance_2,reactance_2);                                        
                                        break;
                                      case LR_PARALLEL:
                                        resistance_1 = absImp_1*sqrt(1+pow(tan(phaseShift_1),2));
                                        reactance_1 = resistance_1/(2*PI*SIGNAL_FREQ_1*(float32_t)tan(phaseShift_1));
                                        resistance_2 = absImp_2*sqrt(1+pow(tan(phaseShift_2),2));;
                                        reactance_2 = resistance_2/(2*PI*SIGNAL_FREQ_2*(float32_t)tan(phaseShift_2));
                                        //printf("fs=%d f1=%.3f R=%.6f L=%.6f, f2=%.3f R=%.6f L=%.6f\r\n",SAMPLING_FREQ,SIGNAL_FREQ_1,resistance_1,reactance_1, SIGNAL_FREQ_2,resistance_2,reactance_2);
                                        printf("%d,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f\r\n",SAMPLING_FREQ,SIGNAL_FREQ_1,resistance_1,reactance_1, SIGNAL_FREQ_2,resistance_2,reactance_2);                                                                              
                                        break;
                                      case CR_SERIES:
                                        resistance_1 = realImp_1;
                                        reactance_1 = (float32_t)(1/(imagImp_1*2*PI*SIGNAL_FREQ_1));
                                        resistance_2 = realImp_2;
                                        reactance_2 = (float32_t)(1/(imagImp_2*2*PI*SIGNAL_FREQ_1));
                                        //printf("fs=%d f1=%.3f R=%.6f L=%.6f, f2=%.3f R=%.6f L=%.6f\r\n",SAMPLING_FREQ,SIGNAL_FREQ_1,resistance_1,reactance_1, SIGNAL_FREQ_2,resistance_2,reactance_2);
                                        printf("%d,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f\r\n",SAMPLING_FREQ,SIGNAL_FREQ_1,resistance_1,reactance_1, SIGNAL_FREQ_2,resistance_2,reactance_2);
                                        break;
                                      case CR_PARALLEL:
                                        resistance_1 = absImp_1*sqrt(1+pow(tan(phaseShift_1),2));
                                        reactance_1 = ((float32_t)tan(phaseShift_1))/(2*PI*SIGNAL_FREQ_1*resistance_1);
                                        resistance_2 = absImp_2*sqrt(1+pow(tan(phaseShift_2),2));;
                                        reactance_2 = ((float32_t)tan(phaseShift_2))/(2*PI*SIGNAL_FREQ_2*resistance_2);
                                        //printf("fs=%d f1=%.3f R=%.6f L=%.6f, f2=%.3f R=%.6f L=%.6f\r\n",SAMPLING_FREQ,SIGNAL_FREQ_1,resistance_1,reactance_1, SIGNAL_FREQ_2,resistance_2,reactance_2);
                                        printf("%d,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f\r\n",SAMPLING_FREQ,SIGNAL_FREQ_1,resistance_1,reactance_1, SIGNAL_FREQ_2,resistance_2,reactance_2);
                                        break;
                                    }
				}
			}
			HAL_ADC_Start_DMA(&hadc1, adc_values, NUM_OF_SAMPLES * 2);
			__enable_irq();
		}

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 1, 0);
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
