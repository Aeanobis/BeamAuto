#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static FLASH_EraseInitTypeDef EraseInitStruct;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM21_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Flash_Write(uint32_t Flash_Address, uint32_t Flash_Data);
uint32_t Flash_Read(uint32_t Flash_Address);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define EEPROM_ADDRESS 0x08080000
#define I2C_ADDRESS 0x40
#define PI (3.141592653589793)

#define FLASH_USER_START_ADDR   (0x08080000 )             /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (FLASH_USER_START_ADDR + 1023)   /* End @ of user Flash area */


/********************
 * Charging States
 *******************/
#define charging_state 0
#define cleaning_state 1
#define manual_state 2

volatile uint8_t transferDirection, transferRequested;
uint16_t hTxNumData = 0, hRxNumData = 0;

uint32_t Address = 0x08080000, PAGEError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

int targetDeviceReady = 0;
bool TxEnded = false;
bool RxEnded = false;
bool I2CFound = false;

bool chargeComplete = 0;
bool chargeSatisfactory;

bool ledON = false;

int current_state = 1;

int batteryVoltage;

/************************
 * Register definitions
 ************************/
//register shit
static const uint8_t REG_CONFIG = 0x00;
static const uint8_t REG_CNTRL1 = 0x01;
static const uint8_t REG_CNTRL2 = 0x02;

// LED controller channels
static const uint8_t REG_D0_CTRL = 0x07;
static const uint8_t REG_D1_CTRL = 0x08;
static const uint8_t REG_D2_CTRL = 0x09;
static const uint8_t REG_D3_CTRL = 0x0a;
static const uint8_t REG_D4_CTRL = 0x0b;
static const uint8_t REG_D5_CTRL = 0x0c;
static const uint8_t REG_D6_CTRL = 0x0d;
static const uint8_t REG_D7_CTRL = 0x0e;
static const uint8_t REG_D8_CTRL = 0x0f;

// Direct PWM control registers
static const uint8_t REG_D0_PWM  = 0x16;
static const uint8_t REG_D1_PWM  = 0x17;
static const uint8_t REG_D2_PWM  = 0x18;
static const uint8_t REG_D3_PWM  = 0x19;
static const uint8_t REG_D4_PWM  = 0x1a;
static const uint8_t REG_D5_PWM  = 0x1b;
static const uint8_t REG_D6_PWM  = 0x1c;
static const uint8_t REG_D7_PWM  = 0x1d;
static const uint8_t REG_D8_PWM  = 0x1e;

// Drive current registers
static const uint8_t REG_D0_I_CTL = 0x22;
static const uint8_t REG_D1_I_CTL  = 0x23;
static const uint8_t REG_D2_I_CTL  = 0x24;
static const uint8_t REG_D3_I_CTL  = 0x25;
static const uint8_t REG_D4_I_CTL  = 0x26;
static const uint8_t REG_D5_I_CTL  = 0x27;
static const uint8_t REG_D6_I_CTL  = 0x28;
static const uint8_t REG_D7_I_CTL  = 0x29;
static const uint8_t REG_D8_I_CTL  = 0x2A;

//Charge Pump and friends
static const uint8_t REG_MISC = 0x2F;
static const uint8_t REG_PC1      = 0x30;
static const uint8_t REG_PC2      = 0x31;
static const uint8_t REG_PC3      = 0x32;
static const uint8_t REG_MISC_2 = 0x33;
static const uint8_t REG_STATUS_IRQ = 0x3C;
static const uint8_t REG_INT_GPIO   = 0x3D;
static const uint8_t REG_GLOBAL_VAR = 0x3E;
static const uint8_t REG_RESET      = 0x3F;

static const uint8_t REG_ENGINE_A_VAR = 0x42;
static const uint8_t REG_ENGINE_B_VAR = 0x43;
static const uint8_t REG_ENGINE_C_VAR = 0x44;

static const uint8_t REG_MASTER_FADE_1 = 0x46;
static const uint8_t REG_MASTER_FADE_2 = 0x47;
static const uint8_t REG_MASTER_FADE_3 = 0x48;
static const uint8_t  REG_MASTER_FADE_PWM = 0x4A;

static const uint8_t REG_PROG1_START = 0x4B;
static const uint8_t REG_PROG2_START = 0x4C;
static const uint8_t REG_PROG3_START = 0x4D;
static const uint8_t REG_PROG_PAGE_SEL = 0x4F;

// Memory is more confusing - there are 6 pages, sel by addr 4f
static const uint8_t REG_PROG_MEM_BASE = 0x50;
//static const uint8_t REG_PROG_MEM_SIZE = 0x;//
static const uint8_t REG_PROG_MEM_END  = 0x6f;

static const uint8_t REG_ENG1_MAP_MSB = 0x70;
static const uint8_t REG_ENG1_MAP_LSB = 0x71;
static const uint8_t REG_ENG2_MAP_MSB = 0x72;
static const uint8_t REG_ENG2_MAP_LSB = 0x73;
static const uint8_t REG_ENG3_MAP_MSB = 0x74;
static const uint8_t REG_ENG3_MAP_LSB = 0x75;

static const uint8_t REG_GAIN_CHANGE = 0x76;
static const uint8_t PWM_CONFIG = 0x80;


/********************
 * i2c Buffer address
 *******************/
uint8_t aTxBuffer[2];
uint8_t aRxBuffer[1];
int Run;

bool button = 0;
volatile uint32_t counter = 0;

uint32_t button_processed = 0, lastDebounceTime;

uint32_t minimalTime = 3000;
uint32_t cleanTime = 5000;
int delayTime = 7500;
bool buttonHoldBreak = false;


uint32_t data;

bool buttonPressed = false;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWREx_EnableUltraLowPower();
	HAL_PWREx_EnableFastWakeUp();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM21_CLK_ENABLE();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */
  int countPress = 0;
  buttonHoldBreak = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	  {
		  /* Clear Standby flag */
		  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		  //current_state = Flash_Read(Address);
		  if(isCharging() == 1){
			  current_state = charging_state;
		  }
	  }

	  GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  HAL_Delay(10);


	  if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == SET)){
		  int startTime = HAL_GetTick();
		  while(((HAL_GetTick() - startTime) < minimalTime+500) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == SET)){
			  if(((HAL_GetTick() - startTime) > minimalTime) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == SET)){
			  			  if(current_state == cleaning_state){
			  				  current_state = manual_state;
			  			  } else if (current_state == manual_state){
			  				  current_state = cleaning_state;
			  			  }
			  			  break;
			  		  }
		  }


	  }

	  switch(current_state){
	  case 0: {
		  int isChargingInt = isCharging();
		  int isChargeSatisfactoryInt = isChargeSatisfactory();

		  //charging
		  if (isChargingInt == 0 & isChargeSatisfactoryInt == 1) {
			  ledOff();
			  Disable();
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
			  m_disable();
			  led_disable();
			  boost_disable();
			  current_state = cleaning_state;
			  Flash_Write(EEPROM_ADDRESS,current_state);

		  }
		  else if (isChargingInt == 1 && isChargeSatisfactoryInt == 0) {
				  //boost_enable();
				  LED_Enable();
				  ledCharge();
		  }
		  else if (isChargingInt == 1 && isChargeSatisfactoryInt == 1) {
				  ledOff();
				  Disable();
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
				  m_disable();
				  led_disable();
				  boost_disable();
				  StandbyMode();
				  //Flash_Write(EEPROM_ADDRESS,current_state);
			  }
	  }
	  break;
	  case 1:{
		  //cleaning
//		  GPIO_InitTypeDef GPIO_InitStruct;
//		  GPIO_InitStruct.Pin = GPIO_PIN_0;
//		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//		  GPIO_InitStruct.Pull = GPIO_NOPULL;
//		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		  int isChargingInt = isCharging();
		  int isChargeSatisfactoryInt = isChargeSatisfactory();

		  if(isChargingInt == 1){
			  current_state = charging_state;
		  }
		  if(isChargeSatisfactoryInt == 0){
			  LED_Enable();
			  redFlickerBeat(255);
		  }
		  if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == SET) && isChargeSatisfactoryInt == 1){
			  int startTime;
			  boost_enable();
			  LED_Enable();
			  for(int i = 0; i < 4; i++){
				  startTime = HAL_GetTick();
				  while((HAL_GetTick() - startTime) < delayTime){

					  loadingAnimationTopCW(90); // replace with direction function for LEDs
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET); //Motor reset
					  m_enable();
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET); //Motor ON
				  }
				  if((HAL_GetTick() - startTime) > delayTime){
					  ledOff();
					  //Disable();
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
					  m_disable();
					  //led_disable();
					  HAL_Delay(1500);
				  }


			  }
		  }
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)== RESET ){

			  ledOff();
			  Disable();
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
			  m_disable();
			  boost_disable();
			  Flash_Write(EEPROM_ADDRESS,current_state);
			  StandbyMode();
		  }
	  }
	  break;
	  case 2:{
		  //manual mode
//		  int startTime = HAL_GetTick();
//		  if(((HAL_GetTick() - startTime) < minimalTime+2000) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == SET)){
//			  //			  if(current_state == cleaning_state){
//			  //				  current_state = manual_state;
//			  //			  }
//			  if(current_state == manual_state){
//				  current_state = cleaning_state;
//			  }
//			  if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == SET))
//			  break;
//		  }


		  bool clicked = false;
		  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == RESET ){
			  LED_Enable();
			  ledBreathe2(90,50);
			  //redFlickerBeat(90,0);
			  uint16_t val = getBatteryVoltage();
//			  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == SET){
//			  clicked = true;
//			  break;
//			  }
		  }
		  Flash_Write(EEPROM_ADDRESS,current_state);
				  StandbyMode();

//		  while(clicked == true){
//
//		  }

	  }

	  }
  }


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7500-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2097-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM21 init function */
static void MX_TIM21_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 0;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5 
                           PA6 PA7 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Flash_Write(uint32_t Flash_Address, uint32_t DATA_32){
	HAL_FLASH_Unlock();

	Address = FLASH_USER_START_ADDR;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
	EraseInitStruct.NbPages     = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

	HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);

	while (Address < FLASH_USER_END_ADDR)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32) == HAL_OK)
		{
			Address = Address + 4;
		}
		HAL_FLASH_Lock();
	}
}

uint32_t Flash_Read(uint32_t Flash_Address)
{
	uint32_t Flash_Data;
	Flash_Data = *(uint32_t*) Flash_Address;
	return Flash_Data;
}

void HAL_SYSTICK_Callback(void)
{
	HAL_IncTick();
}

GPIO_IRQ_HANDLER(void){
	lastDebounceTime = HAL_GetTick();
	button_processed = 0;
}

void cleanTeeth(void){
	m_enable();
}

int isChargeSatisfactory(){
	int val = getBatteryVoltage();
	//batteryMeasureDisable();
	if(val > 200){
		return 1;
	}
	else{
		return 0;
	}
}

void m_enable(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
}

void m_disable(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}

void led_enable(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
}

void led_disable(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

void boost_enable(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
}

void boost_disable(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}

void batteryMeasureEnable(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
}
void batteryMeasureDisable(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

int isCharging(void){
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8) == RESET){
		return 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8) == SET){
		return 0;
	}
}

void ADC_IRQHandler(void) {
	HAL_ADC_IRQHandler(&hadc);
}

int getBatteryVoltage(){
	batteryMeasureEnable();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1000);
	uint16_t batteryVoltage = HAL_ADC_GetValue(&hadc);
	//do something in between
	uint16_t val = batteryVoltage;
	HAL_ADC_Stop(&hadc);
	return val;
}

void StandbyMode(void){
	//HAL_DBGMCU_EnableDBGStandbyMode();
	//MX_GPIO_Deinit();
	/* This procedure come from the STM32F030 Errata sheet*/
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	//	m_disable();
	//	led_disable();
	//	boost_disable();
	//	batteryMeasureDisable();
	//__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	/* Clear PWR wake up Flag */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	/* Enable WKUP pin */
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_EnterSTANDBYMode();
}

void MX_GPIO_Deinit(void){
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Configure all GPIO as analog to reduce current consumption on non used IOs */
	/* Enable GPIOs clock */
	/* Warning : Reconfiguring all GPIO will close the connection with the debugger */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* Disable GPIOs clock */
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
}


void writeReg(uint8_t MemAddress, uint8_t pData){
	uint8_t aTxBuff[2] = {MemAddress, pData};
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)I2C_ADDRESS<<1, aTxBuff,2,100);
}

void readReg(uint16_t* const MemAddress, uint8_t *pData){
	while(HAL_I2C_Master_Transmit_IT(&hi2c1, (uint16_t)I2C_ADDRESS<<1, *MemAddress,1) != HAL_OK){
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			Error_Handler();
		}
	}
	while(!TxEnded){
	}
	TxEnded = false;


	while(HAL_I2C_Master_Receive_IT(&hi2c1, (uint16_t)I2C_ADDRESS<<1, *pData, 1) != HAL_OK)
	{
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			Error_Handler();
		}
	}
	while(!RxEnded){
	}
	RxEnded = false;
}

void LED_Enable(void){
	//enable
	led_enable();
	writeReg(REG_CONFIG, 0x40);
	writeReg(REG_MISC,0x50);
}

void Disable(void){
	writeReg(REG_CONFIG,0x00);
	led_disable();
}

void delay_ms(uint32_t t) {
	__HAL_TIM_SetCounter(&htim2, 0);
	HAL_TIM_Base_Start(&htim2);
	while(__HAL_TIM_GetCounter(&htim2)< t);
	HAL_TIM_Base_Stop(&htim2);
}


void loadingAnimationTopCW(uint16_t brightness){
	//writeReg(REG_D6_PWM, brightness);
	uint8_t led0 = REG_D0_PWM;
	uint8_t led1 = REG_D1_PWM;
	uint8_t led2 = REG_D2_PWM;
	uint8_t led3 = REG_D4_PWM;
	uint8_t led4 = REG_D5_PWM;
	uint8_t led5 = REG_D8_PWM;
	int val1, val2;
	uint32_t delay_time = 100;
	val1 = brightness;
	val2 = brightness -50;
	writeReg(led0, val1);
	writeReg(led1, 0);
	writeReg(led2, 0);
	writeReg(led3, 0);
	writeReg(led4, 0);
	writeReg(led5, 0);
	HAL_Delay(delay_time);
	writeReg(led0, 0);
	writeReg(led1, val1);
	writeReg(led2, 0);
	writeReg(led3, 0);
	writeReg(led4, 0);
	writeReg(led5, 0);
	HAL_Delay(delay_time);
	writeReg(led0, 0);
	writeReg(led1, 0);
	writeReg(led2, val1);
	writeReg(led3, 0);
	writeReg(led4, 0);
	writeReg(led5, 0);
	HAL_Delay(delay_time);
	writeReg(led0, 0);
	writeReg(led1, 0);
	writeReg(led2, 0);
	writeReg(led3, val1);
	writeReg(led4, 0);
	writeReg(led5, 0);
	HAL_Delay(delay_time);
	writeReg(led0, 0);
	writeReg(led1, 0);
	writeReg(led2, 0);
	writeReg(led3, 0);
	writeReg(led4, val1);
	writeReg(led5, 0);
	HAL_Delay(delay_time);
	writeReg(led0, 0);
	writeReg(led1, 0);
	writeReg(led2, 0);
	writeReg(led3, 0);
	writeReg(led4, 0);
	writeReg(led5, val1);
	HAL_Delay(delay_time);
}

void loadingAnimationTopCCW(uint16_t brightness){
	uint8_t led0 = REG_D0_PWM;
	uint8_t led1 = REG_D1_PWM;
	uint8_t led2 = REG_D2_PWM;
	uint8_t led3 = REG_D4_PWM;
	uint8_t led4 = REG_D5_PWM;
	uint8_t led5 = REG_D8_PWM;
	int val1, val2;
	val1 = brightness;
	val2 = brightness -50;

	writeReg(led4, val1);
	writeReg(led5, val2);
	HAL_Delay(100);
	writeReg(led5, 0);
	writeReg(led4, val2);
	writeReg(led3,val1);
	HAL_Delay(100);
	writeReg(led4, 0);
	writeReg(led3, val2);
	writeReg(led2, val1);
	HAL_Delay(100);
	writeReg(led3,0);
	writeReg(led2, val2);
	writeReg(led1, val1);
	HAL_Delay(100);
	writeReg(led2, 0);
	writeReg(led1, val2);
	writeReg(led0, val1);
	HAL_Delay(100);
	writeReg(led1, 0);
	writeReg(led0, val2);
	writeReg(led5, val1);
	HAL_Delay(100);
	writeReg(led0, 0);
	writeReg(led5, 0);
	//delay_ms(50);
}

void ledOff(void){
	writeReg(REG_D0_PWM, 0);
	writeReg(REG_D1_PWM, 0);
	writeReg(REG_D2_PWM, 0);
	writeReg(REG_D3_PWM, 0);
	writeReg(REG_D4_PWM, 0);
	writeReg(REG_D5_PWM, 0);
	writeReg(REG_D6_PWM, 0);
	writeReg(REG_D7_PWM, 0);
	writeReg(REG_D8_PWM, 0);
}

void ledCharge(void){
	writeReg(REG_D0_PWM, 0);
	writeReg(REG_D1_PWM, 0);
	writeReg(REG_D2_PWM, 0);
	writeReg(REG_D3_PWM, 255);
	writeReg(REG_D4_PWM, 0);
	writeReg(REG_D5_PWM, 0);
	writeReg(REG_D6_PWM, 0);
	writeReg(REG_D7_PWM, 0);
	writeReg(REG_D8_PWM, 0);
}

void redFlickerBeat(int maxVal){
	int count = 0;
	while(count < 5){
	writeReg(REG_D3_PWM, maxVal);
	HAL_Delay(75);
	writeReg(REG_D3_PWM, 0);
	count++;
	}

	HAL_Delay(150);
}


void ledBreathe(int maxVal, int delayTime){
	ledOff();
	float LED_brightness;
	int count = 0;
	while(count < 1){
		for(int i = 0; i < 255; i++){
			LED_brightness = (exp(sin(PI/2*i)) - 0.36787944)*108.0;
		}
		writeReg(REG_D0_PWM, 0);
		writeReg(REG_D1_PWM, 0);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, 0);
		writeReg(REG_D5_PWM, 0);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		HAL_Delay(delayTime);
		writeReg(REG_D0_PWM, 0);
		writeReg(REG_D1_PWM, LED_brightness);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, LED_brightness);
		writeReg(REG_D5_PWM, 0);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		HAL_Delay(delayTime);
		writeReg(REG_D0_PWM, LED_brightness);
		writeReg(REG_D1_PWM, LED_brightness);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, LED_brightness);
		writeReg(REG_D5_PWM, LED_brightness);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		HAL_Delay(delayTime);
		writeReg(REG_D0_PWM, LED_brightness);
		writeReg(REG_D1_PWM, LED_brightness);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, LED_brightness);
		writeReg(REG_D5_PWM, LED_brightness);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, LED_brightness);
		HAL_Delay(delayTime);

		HAL_Delay(500);

		writeReg(REG_D0_PWM, LED_brightness);
		writeReg(REG_D1_PWM, LED_brightness);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, LED_brightness);
		writeReg(REG_D5_PWM, LED_brightness);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		HAL_Delay(delayTime);
		writeReg(REG_D0_PWM, 0);
		writeReg(REG_D1_PWM, LED_brightness);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, LED_brightness);
		writeReg(REG_D5_PWM, 0);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		HAL_Delay(delayTime);
		writeReg(REG_D0_PWM, 0);
		writeReg(REG_D1_PWM, 0);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, 0);
		writeReg(REG_D5_PWM, 0);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		HAL_Delay(delayTime);
		writeReg(REG_D0_PWM, 0);
		writeReg(REG_D1_PWM, 0);
		writeReg(REG_D2_PWM, 0);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, 0);
		writeReg(REG_D5_PWM, 0);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);

	}
	count++;
	ledOff();
}

int millis(uint32_t t){
	__HAL_TIM_SetCounter(&htim21, 0);
	HAL_TIM_Base_Start(&htim21);
	while(__HAL_TIM_GetCounter(&htim21) < t);
	return __HAL_TIM_GetCounter(&htim21);
	HAL_TIM_Base_Stop(&htim21);
}

void ledBreathe2(int maxVal, int delayTime){
	ledOff();
	int LED_brightness;
	int count = 0;
	while(count < 1){
		LED_brightness = maxVal;

		writeReg(REG_D0_PWM, 0);
		writeReg(REG_D1_PWM, 0);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, 0);
		writeReg(REG_D5_PWM, 0);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		delay_ms(delayTime);
		writeReg(REG_D0_PWM, 0);
		writeReg(REG_D1_PWM, LED_brightness);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, LED_brightness);
		writeReg(REG_D5_PWM, 0);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		delay_ms(delayTime);
		writeReg(REG_D0_PWM, LED_brightness);
		writeReg(REG_D1_PWM, LED_brightness);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, LED_brightness);
		writeReg(REG_D5_PWM, LED_brightness);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		delay_ms(delayTime);
		writeReg(REG_D0_PWM, LED_brightness);
		writeReg(REG_D1_PWM, LED_brightness);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, LED_brightness);
		writeReg(REG_D5_PWM, LED_brightness);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, LED_brightness);
		delay_ms(delayTime+100);


		writeReg(REG_D0_PWM, LED_brightness);
		writeReg(REG_D1_PWM, LED_brightness);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, LED_brightness);
		writeReg(REG_D5_PWM, LED_brightness);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		delay_ms(delayTime);
		writeReg(REG_D0_PWM, 0);
		writeReg(REG_D1_PWM, LED_brightness);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, LED_brightness);
		writeReg(REG_D5_PWM, 0);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		delay_ms(delayTime);
		writeReg(REG_D0_PWM, 0);
		writeReg(REG_D1_PWM, 0);
		writeReg(REG_D2_PWM, LED_brightness);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, 0);
		writeReg(REG_D5_PWM, 0);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		delay_ms(delayTime);
		writeReg(REG_D0_PWM, 0);
		writeReg(REG_D1_PWM, 0);
		writeReg(REG_D2_PWM, 0);
		writeReg(REG_D3_PWM, 0);
		writeReg(REG_D4_PWM, 0);
		writeReg(REG_D5_PWM, 0);
		writeReg(REG_D6_PWM, 0);
		writeReg(REG_D7_PWM, 0);
		writeReg(REG_D8_PWM, 0);
		delay_ms(delayTime);
		count++;
	}

}



void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
	TxEnded = true;
	I2cHandle->State = HAL_I2C_STATE_READY;

}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
	RxEnded = true;
	I2cHandle->State = HAL_I2C_STATE_READY;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
	m_disable();
	Disable();
	__HAL_TIM_SetCounter(&htim2, 0);
	//HAL_TIM_Base_Stop_IT(&htim2);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
