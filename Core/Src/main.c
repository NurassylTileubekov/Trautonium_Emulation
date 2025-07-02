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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_CS43L22.h"
#include <math.h>
#include "Keypad4X4.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
#define BUF_SIZE    52
#define PI M_PI
#define SAMPLE_RATE 48828.0f
#define CurrFreqADC ADC1val[0]
#define CurrAmpADC ADC1val[1]

#define F1VolADC ADC2val[0]
#define F1FreqADC ADC2val[1]
#define F1ResADC ADC2val[2]
#define F2VolADC ADC2val[3]
#define F2FreqADC ADC2val[4]
#define F2ResADC ADC2val[5]
#define MasterVolADC ADC2val[6]

#define SMOOTH_LEN 3

typedef struct {
	float a0;
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
}biquad_coeffs;

typedef struct {
	float y_prev;
	float y_prev_prev;
	float x_prev;
	float x_prev_prev;
}biquad_state;

typedef struct {
	float f_0;
	float Q;
	float vol;
	float FreqRange;
	uint8_t mode;
}filter_par;

int16_t i2sBuf[BUF_SIZE];
int16_t F1Buf[BUF_SIZE];
int16_t F2Buf[BUF_SIZE];
float currentFreq = 55.0f;
float currentAmp = 0.05f;
float MasterVol = 1.00f;
float OscillatorVol = 1.0f;
float cutoffFreq = 100.0f;
uint8_t preFilSwMode = 0;
uint8_t currentOct = 4;
uint8_t fixedScaleMode = 0;
uint8_t fixedScale = 0;
uint8_t EnbFMode = 0;
volatile uint16_t ADC1val[2];
volatile uint16_t ADC2val[7];

biquad_state SttF1 = {0.0f};
biquad_state SttF2 = {0.0f};

filter_par F1_par = {0.0f, 0.0f, 0.0f, 640.0f, 0};
filter_par F2_par = {0.0f, 0.0f, 0.0f, 640.0f, 0};

extern char key;
char old_key;

float targetFreq;
float portamentoSpeed = 0.01f;

uint32_t Enc1_counter = 0;
uint32_t Enc2_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float softClip(float x, float threshold, float gain) {
    const float clipStart = threshold * 0.95f; // Start soft clipping at 95% of range
    const float range = threshold - clipStart;

    if (x > clipStart) {
        float t = (x - clipStart) / range;
        return clipStart + range * tanhf(t * gain); // smooth compression
    } else if (x < -clipStart) {
        float t = (-x - clipStart) / range;
        return -clipStart - range * tanhf(t * gain); // symmetric
    } else {
        return x;
    }
}
float polyBLEP(float t, float phaseInc) {
	float dt = phaseInc / 2 * PI;
    if (t < dt) {
        t /= dt;
        return t + t - t * t - 1.0f;
    } else if (t > 1.0f - dt) {
        t = (t - 1.0f) / dt;
        return t * t + t + t + 1.0f;
    } else {
        return 0.0f;
    }
}
static float noiseGate(float sample, float threshold) {
    if (fabsf(sample) < threshold) {
        return 0.0f;
    }
    return sample;
}
static float movingAverage(float sample, float *smoothBuffer, uint8_t *smoothIndex) {
    smoothBuffer[*smoothIndex] = sample;
    *smoothIndex = (*smoothIndex + 1) % SMOOTH_LEN;

    float sum = 0.0f;
    for (int i = 0; i < SMOOTH_LEN; i++) {
        sum += smoothBuffer[i];
    }
    return sum / SMOOTH_LEN;
}
static void fillSawBlock(int16_t *buf, int len, float freq, float vol) {
    static float phase = 0.0f;
    static float smoothBuffer[SMOOTH_LEN] = {0};
    static uint8_t smoothIndex;
    float phaseInc = freq / SAMPLE_RATE;

    for (int i = 0; i < len; i = i + 2) {
        float t = phase;
        float value = 2.0f * t - 1.0f;
        value -= polyBLEP(t, phaseInc); // apply PolyBLEP correction
        if(vol > 1.0){
        	value = value * 32768.0f * (1.0f + (vol - 1.0f) * 8);
        }
        else{
        	value = value * 32768.0f * vol;
        }

		value = noiseGate(value, 1e-4f);
		value = movingAverage(value, smoothBuffer, &smoothIndex);

		// Apply soft clipping only if value exceeds int16 range
		value = softClip(value, 32768.0f, 0.2f);
		// Clamp just in case (safety net)
		if (value > 32767.0f) value = 32767.0f;
		if (value < -32768.0f) value = -32768.0f;

        buf[i] = (int16_t)value;
        buf[i+1] = (int16_t)value;
        phase += phaseInc;
        if (phase >= 1.0f) phase -= 1.0f;
    }
}
static void IIR_LowPassFilter(int16_t *buf, int len, float freq_cutoff){
	float dt = 1.0f / SAMPLE_RATE;
	float RC = 1 / (2 * PI * freq_cutoff);
	float a = dt / (RC + dt);
	static float y_prev = 0.0f;
	for (int i = 0; i < len; i = i + 2) {
		float x = (float)buf[i];
		y_prev = (a * x + (1 - a) * y_prev);
		if (y_prev < -32768.0f) y_prev = -32768.0f;
		if (y_prev > 32767.0f) y_prev = 32767.0f;
		buf[i] = (int16_t)y_prev;
		buf[i+1] = (int16_t)y_prev;
	    }
}

static void IIR_2ndOrderFilter(int16_t *in_buf, int16_t *out_buf, int len, filter_par *F_par, biquad_state *Stt){
	float omega_0 = 2* PI * (F_par->f_0/SAMPLE_RATE);
	float alpha = sinf(omega_0) / (2 * F_par -> Q);
	biquad_coeffs par;
	if(F_par ->mode == 0){
		//LPF mode selected
		par.b0 = (1.0f - cosf(omega_0)) / 2.0f;
		par.b1 = 1.0f - cosf(omega_0);
		par.b2 = par.b0;
		par.a0 = 1.0f + alpha;
		par.a1 = -2.0f * cosf(omega_0);
		par.a2 = 1.0f - alpha;

	}
	else if(F_par -> mode == 1){
		//BPF mode selected
		par.b0 = alpha;
		par.b1 = 0;
		par.b2 = -alpha;
		par.a0 = 1.0f + alpha;
		par.a1 = -2.0f * cosf(omega_0);
		par.a2 = 1.0f - alpha;

	}
	par.b0 /= par.a0;
	par.b1 /= par.a0;
    par.b2 /= par.a0;
	par.a1 /= par.a0;
	par.a2 /= par.a0;
    for(uint8_t i = 0; i < len; i = i + 2){
    	float y = 0.0f;
    	float x = (float)in_buf[i];
    	y = par.b0 * x + par.b1 * (Stt -> x_prev) + par.b2 * (Stt -> x_prev_prev) - par.a1 * (Stt -> y_prev) - par.a2 * (Stt -> y_prev_prev);
    	Stt -> x_prev_prev = Stt -> x_prev;
    	Stt -> x_prev = x;
    	Stt -> y_prev_prev = Stt -> y_prev;
    	Stt -> y_prev = y;
		if(F_par ->vol > 1.0){
			y = y * (1.0f + (F_par ->vol - 1.0f) * 8);
		}
		else{
			y = y * F_par ->vol;
		}
		// Apply soft clipping only if value exceeds int16 range
		y = softClip(y, 32768.0f, 0.2f);
		// Clamp just in case (safety net)
		if (y > 32767.0f) y = 32767.0f;
		if (y < -32768.0f) y = -32768.0f;
		if(F_par ->vol > 0.01f){
			out_buf[i] = (int16_t)y;
			out_buf[i+1] = (int16_t)y;
		}
		else{
			out_buf[i] = 0;
			out_buf[i+1] = 0;
		}

    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4) {
		HAL_TIM_Base_Stop_IT(&htim4); // Stop debounce timer

		// Now read stable switch state
		if (HAL_GPIO_ReadPin(GPIOA, PreFilterSwitch_Pin) == GPIO_PIN_SET) {
			// Button released or in high state
			preFilSwMode = (preFilSwMode + 1) % 3;
			switch(preFilSwMode){
				  case 1:
					  cutoffFreq = 100.0f;
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
					  break;
				  case 2:
					  cutoffFreq = 500.0f;
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
					  break;
				  default:
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
					  break;
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	if(GPIO_PIN == PreFilterSwitch_Pin){
		HAL_TIM_Base_Start_IT(&htim4);
	}

	//1x4 keypad
	if(GPIO_PIN == OCT1_Pin){
		currentOct = 1;
	}
	else if(GPIO_PIN == OCT2_Pin){
		currentOct = 2;
	}
	else if(GPIO_PIN == OCT3_Pin){
		currentOct = 3;
	}
	else if(GPIO_PIN == OCT4_Pin){
		currentOct = 4;
	}
}
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	Enc1_counter = __HAL_TIM_GET_COUNTER(&htim3);
	if (Enc1_counter > 60000)
	{
	  __HAL_TIM_SET_COUNTER(&htim3,0);
	  Enc1_counter=0;
	}
	if (Enc1_counter > 100)
	{
	  __HAL_TIM_SET_COUNTER(&htim3,100);
	  Enc1_counter=100;
	}
	portamentoSpeed = (100 - Enc1_counter)/10000.0f;
	if(portamentoSpeed > 0.01f){
		portamentoSpeed = 0.01f;
	}
	else if(portamentoSpeed < 0.0001f){
		portamentoSpeed = 0.0001f;
	}

	Enc2_counter = __HAL_TIM_GET_COUNTER(&htim2);
	if (Enc2_counter > 60000)
	{
	  __HAL_TIM_SET_COUNTER(&htim2,0);
	  Enc2_counter=0;
	}
	if (Enc2_counter > 200)
	{
	  __HAL_TIM_SET_COUNTER(&htim2,200);
	  Enc2_counter=200;
	}
	OscillatorVol = Enc2_counter / 160.0f;
    // Refill first half [0 .. BUF_SIZE/2)
	if(!fixedScaleMode){
		currentFreq = 110 * powf(4.0, (4095 - CurrFreqADC)/4095.0f) * powf(2, currentOct - 1);
	}
	else{
		fixedScale = (uint8_t)((4095 - CurrFreqADC)*24/4095.0f + 0.5);
		if(currentAmp < 0.01f){
			currentFreq = 110 * powf(4.0, fixedScale/24.0f) * powf(2, currentOct - 1);
		}
		else{
			targetFreq = 110 * powf(4.0, fixedScale/24.0f) * powf(2, currentOct - 1);
			currentFreq += (targetFreq - currentFreq) * portamentoSpeed;
		}
	}

	currentAmp = ((CurrAmpADC - 300)*1.0)/3400;
	if(currentAmp > 1.0f){
		currentAmp = 1.0f;
	}
	if(currentAmp < 0.01f){
		currentAmp = 0.0f;
		 for (int i = 0; i < BUF_SIZE/2; i = i + 2) {
			 i2sBuf[i] = 0;
			 i2sBuf[i+1] = 0;
		 }
	}
	else{
    fillSawBlock(i2sBuf + 0, BUF_SIZE/2, currentFreq, OscillatorVol);
    if(preFilSwMode != 0) IIR_LowPassFilter(i2sBuf + 0, BUF_SIZE/2, cutoffFreq);
    switch(EnbFMode){
    case 1:
    	F1_par.vol = ((uint8_t)((F1VolADC / 4095.0f) * 100))/80.0f;
		F1_par.f_0 = F1_par.FreqRange * ((uint8_t)((F1FreqADC / 4095.0f) * 100))/100.0f;
		F1_par.Q = 14.0f * ((uint8_t)((F1ResADC / 4095.0f) * 100))/100.0f + 1.0f;
    	IIR_2ndOrderFilter(i2sBuf, i2sBuf, BUF_SIZE/2, &F1_par, &SttF1);
    	break;
    case 2:
    	F2_par.vol = ((uint8_t)((F2VolADC / 4095.0f) * 100))/80.0f;
		F2_par.f_0 = F2_par.FreqRange * ((uint8_t)((F2FreqADC / 4095.0f) * 100))/100.0f;
		F2_par.Q = 14.0f * ((uint8_t)((F2ResADC / 4095.0f) * 100))/100.0f + 1.0f;
    	IIR_2ndOrderFilter(i2sBuf, i2sBuf, BUF_SIZE/2, &F2_par, &SttF2);
    	break;
    case 3:
    	F1_par.vol = ((uint8_t)((F1VolADC / 4095.0f) * 100))/80.0f;
		F1_par.f_0 = F1_par.FreqRange * ((uint8_t)((F1FreqADC / 4095.0f) * 100))/100.0f;
		F1_par.Q = 14.0f * ((uint8_t)((F1ResADC / 4095.0f) * 100))/100.0f + 1.0f;
		F2_par.vol = ((uint8_t)((F2VolADC / 4095.0f) * 100))/80.0f;
		F2_par.f_0 = F2_par.FreqRange * ((uint8_t)((F2FreqADC / 4095.0f) * 100))/100.0f;
		F2_par.Q = 14.0f * ((uint8_t)((F2ResADC / 4095.0f) * 100))/100.0f +1.0f;
    	IIR_2ndOrderFilter(i2sBuf, F1Buf, BUF_SIZE/2, &F1_par, &SttF1);
    	IIR_2ndOrderFilter(i2sBuf, F2Buf, BUF_SIZE/2, &F2_par, &SttF2);
    	for(int i = 0; i < BUF_SIZE/2; i = i + 2){
    		float mixed = (float)F1Buf[i] + (float)F2Buf[i];
    		if (mixed > 32767.0f) mixed = 32767.0f;
    		if (mixed < -32768.0f) mixed = -32768.0f;
    		i2sBuf[i] = (int16_t)mixed;
    		i2sBuf[i+1] = (int16_t)mixed;
    	}
    	break;
    default:
    	break;
    }
    MasterVol = ((uint8_t)((MasterVolADC/4095.0f) * 100))/100.0f;
	for(int i = 0; i < BUF_SIZE/2; i = i + 2){
		i2sBuf[i] = (int16_t)(i2sBuf[i] * MasterVol * currentAmp);
		i2sBuf[i+1] = i2sBuf[i] ;
	}
	}
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	Enc1_counter = __HAL_TIM_GET_COUNTER(&htim3);
	if (Enc1_counter > 60000)
	{
	  __HAL_TIM_SET_COUNTER(&htim3,0);
	  Enc1_counter=0;
	}
	if (Enc1_counter > 100)
	{
	  __HAL_TIM_SET_COUNTER(&htim3,100);
	  Enc1_counter=100;
	}
	portamentoSpeed = (100 - Enc1_counter)/10000.0f;
	if(portamentoSpeed > 0.01f){
		portamentoSpeed = 0.01f;
	}
	else if(portamentoSpeed < 0.0001f){
		portamentoSpeed = 0.0001f;
	}

	Enc2_counter = __HAL_TIM_GET_COUNTER(&htim2);
	if (Enc2_counter > 60000)
	{
	  __HAL_TIM_SET_COUNTER(&htim2,0);
	  Enc2_counter=0;
	}
	if (Enc2_counter > 200)
	{
	  __HAL_TIM_SET_COUNTER(&htim2,200);
	  Enc2_counter=200;
	}
	OscillatorVol = Enc2_counter / 160.0f;
	// Refill second half [BUF_SIZE/2 .. BUF_SIZE)
	if(!fixedScaleMode){
		currentFreq = 110 * powf(4.0, (4095 - CurrFreqADC)/4095.0f) * powf(2, currentOct - 1);
	}
	else{
		fixedScale = (uint8_t)((4095 - CurrFreqADC)*24/4095.0f + 0.5);
		if(currentAmp < 0.01f){
			currentFreq = 110 * powf(4.0, fixedScale/24.0f) * powf(2, currentOct - 1);
		}
		else{
			targetFreq = 110 * powf(4.0, fixedScale/24.0f) * powf(2, currentOct - 1);
			currentFreq += (targetFreq - currentFreq) * portamentoSpeed;
		}
	}

	currentAmp = ((CurrAmpADC - 300)*1.0)/3400;
	if(currentAmp > 1.0f){
		currentAmp = 1.0f;
	}
	if(currentAmp < 0.01f){
		currentAmp = 0.0f;
		for (int i = BUF_SIZE/2; i < BUF_SIZE; i = i + 2) {
			 i2sBuf[i] = 0;
			 i2sBuf[i+1] = 0;
		}
	}
	else{
    fillSawBlock(i2sBuf + BUF_SIZE/2, BUF_SIZE/2, currentFreq, OscillatorVol);
    if(preFilSwMode != 0) IIR_LowPassFilter(i2sBuf + BUF_SIZE/2, BUF_SIZE/2, cutoffFreq);
    switch(EnbFMode){
        case 1:
        	F1_par.vol = ((uint8_t)((F1VolADC / 4095.0f) * 100))/80.0f;
			F1_par.f_0 = F1_par.FreqRange * ((uint8_t)((F1FreqADC / 4095.0f) * 100))/100.0f;
			F1_par.Q = 14.0f * ((uint8_t)((F1ResADC / 4095.0f) * 100))/100.0f + 1.0f;;
        	IIR_2ndOrderFilter(i2sBuf + BUF_SIZE/2, i2sBuf + BUF_SIZE/2, BUF_SIZE/2, &F1_par, &SttF1);
        	break;
        case 2:
        	F2_par.vol = ((uint8_t)((F2VolADC / 4095.0f) * 100))/80.0f;
			F2_par.f_0 = F2_par.FreqRange * ((uint8_t)((F2FreqADC / 4095.0f) * 100))/100.0f;
			F2_par.Q = 14.0f * ((uint8_t)((F2ResADC / 4095.0f) * 100))/100.0f + 1.0f;
        	IIR_2ndOrderFilter(i2sBuf + BUF_SIZE/2, i2sBuf + BUF_SIZE/2, BUF_SIZE/2, &F2_par, &SttF2);
        	break;
        case 3:
        	F1_par.vol = ((uint8_t)((F1VolADC / 4095.0f) * 100))/80.0f;
			F1_par.f_0 = F1_par.FreqRange * ((uint8_t)((F1FreqADC / 4095.0f) * 100))/100.0f;
			F1_par.Q = 14.0f * ((uint8_t)((F1ResADC / 4095.0f) * 100))/100.0f + 1.0f;
			F2_par.vol = ((uint8_t)((F2VolADC / 4095.0f) * 100))/80.0f;
			F2_par.f_0 = F2_par.FreqRange * ((uint8_t)((F2FreqADC / 4095.0f) * 100))/100.0f;
			F2_par.Q = 14.0f * ((uint8_t)((F2ResADC / 4095.0f) * 100))/100.0f + 1.0f;
        	IIR_2ndOrderFilter(i2sBuf + BUF_SIZE/2, F1Buf + BUF_SIZE/2, BUF_SIZE/2, &F1_par, &SttF1);
        	IIR_2ndOrderFilter(i2sBuf + BUF_SIZE/2, F2Buf + BUF_SIZE/2, BUF_SIZE/2, &F2_par, &SttF2);
        	for(int i = BUF_SIZE/2; i < BUF_SIZE; i = i + 2){
        		float mixed = (float)F1Buf[i] + (float)F2Buf[i];
        		if (mixed > 32767.0f) mixed = 32767.0f;
        		if (mixed < -32768.0f) mixed = -32768.0f;
        		i2sBuf[i] = (int16_t)mixed;
        		i2sBuf[i+1] = (int16_t)mixed;
        	}
        	break;
        default:
        	break;
        }
    MasterVol = ((uint8_t)((MasterVolADC/4095.0f) * 100))/100.0f;
	for(int i = BUF_SIZE/2; i < BUF_SIZE; i = i + 2){
		i2sBuf[i] = (int16_t)(i2sBuf[i] * MasterVol * currentAmp);
		i2sBuf[i+1] = i2sBuf[i] ;
	}
	}

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  CS43_Init(hi2c1, MODE_I2S);
  CS43_SetVolume(50);
  CS43_Enable_RightLeft(CS43_RIGHT_LEFT);
  fillSawBlock(i2sBuf, BUF_SIZE, currentFreq, currentAmp);
  HAL_I2S_Transmit_DMA(&hi2s3, i2sBuf, BUF_SIZE);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC1val, 2);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&ADC2val, 7);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim2,160);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  key = Get_Key();
	  if(key != old_key){
	  		switch(key){
	  		case '1':
	  			EnbFMode = 0;
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
	  			break;
	  		case '2':
	  			EnbFMode = 1;
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
	  				break;
	  		case '3':
	  			EnbFMode = 2;
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
	  				break;
	  		case 'A':
	  			EnbFMode = 3;
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
	  				break;
	  		case '4':
	  			F1_par.mode = 0;
	  				break;
	  		case '5':
	  			F1_par.mode = 1;
	  				break;
	  		case '6':
	  			F1_par.FreqRange = 640.0f;
	  				break;
	  		case 'B':
	  			F1_par.FreqRange = 6000.0f;
	  				break;
	  		case '7':
	  			F2_par.mode = 0;
	  				break;
	  		case '8':
	  			F2_par.mode = 1;
	  				break;
	  		case '9':
	  			F2_par.FreqRange = 640.0f;
	  				break;
	  		case 'C':
	  			F2_par.FreqRange = 6000.0f;
	  				break;
	  		case '*':
	  			fixedScaleMode = 0;
	  				break;
	  		case '0':
	  			fixedScaleMode = 1;
	  				break;
	  		case '#':
	  				break;
	  		case 'D':
	  				break;
	  		default:
	  			break;
	  		}
	  		old_key = key;
	  	}
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
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
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 7;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 840-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, KC3_Pin|KC2_Pin|KC1_Pin|KC0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PreFilterSwitch_Pin */
  GPIO_InitStruct.Pin = PreFilterSwitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PreFilterSwitch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KR3_Pin KR2_Pin KR1_Pin KR0_Pin */
  GPIO_InitStruct.Pin = KR3_Pin|KR2_Pin|KR1_Pin|KR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : KC3_Pin KC2_Pin KC1_Pin KC0_Pin */
  GPIO_InitStruct.Pin = KC3_Pin|KC2_Pin|KC1_Pin|KC0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : OCT2_Pin OCT1_Pin OCT4_Pin OCT3_Pin */
  GPIO_InitStruct.Pin = OCT2_Pin|OCT1_Pin|OCT4_Pin|OCT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
