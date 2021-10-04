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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "scpiparser.h"
#include "printf.h"
#include "dac101.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum modes {
  OFF,
  CURRENT_INIT,
  CURRENT,
  RESISTANCE_INIT,
  RESISTANCE,
  POWER_INIT,
  POWER
}Mode_t;

typedef struct load_channel_struct {
  bool active;
  uint8_t ch_num;
  uint16_t dac_value;
  uint32_t isns_ua; // sensed current in microamps
  uint32_t errors;
}LoadChannel_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_CHANNELS          8
#define ADC_SAMPLES           2
#define VIN_SAMPLE_DEPTH      4
#define VIN_FE_GAIN           15.7059
#define ADC_FS                3.3
#define CS_UA_PER_COUNT       4740
#define SCRATCH_BUF_SIZE      512
#define MAX_CURRENT_PER_CHAN  19.4
#define CS_RES                0.005
#define CS_AMP_GAIN           34.0
#define DAC_FS                1023.0
#define DAC_VREF              3.3
#define UART_BUF_SIZE         128

extern uint8_t* UserRxBufferFS;
extern uint8_t* UserTxBufferFS;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Mode_t mode = OFF;
float iset = 0.0;
float rset = 0.0;
LoadChannel_t load_channels[DAC_CHANNELS];
int channel_ind = 0;
int channel_adc_ind = 0;
float vin = 0.0;
int vin_acc = 0;
int vin_samples = 0;
uint16_t adc_samples[ADC_SAMPLES];
volatile bool adc_ready = false;
float max_current = MAX_CURRENT_PER_CHAN * DAC_CHANNELS;
volatile uint8_t uart_buf[UART_BUF_SIZE] = {};
volatile int uart_ind = 0;
volatile bool process_scpi_cmd;
volatile int i2c_error_count = 0;
uint8_t scratch_buf[SCRATCH_BUF_SIZE];
uint8_t idn_str[] = "1000W Electronic Load, (c) 2021 Adam Cordingley, acordingley.us\r\n\0";
struct scpi_parser_context ctx;
volatile uint8_t i2c_tx_data[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void switch_cs_channel(int ch_num);
void update_dac(LoadChannel_t* load);
void update_dac_setpoints(uint16_t counts);
void pause_uart_interrupts();
void resume_uart_interrupts();
void pause_i2c_interrupts();
void resume_i2c_interrupts();
scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_voltage(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_current(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_power(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_resistance(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t set_current(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t set_resistance(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_error_count(struct scpi_parser_context* context, struct scpi_token* command);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_UART_Transmit(&huart1, idn_str, strlen(idn_str), 10);

  // init channels
  for(int i = 0; i < DAC_CHANNELS; i++){
    load_channels[i].active = true;
    load_channels[i].ch_num = i;
    load_channels[i].dac_value = 0;
    load_channels[i].isns_ua = 0;
  }
  // start the I2C ball rolling
  update_dac(&(load_channels[channel_ind]));


  // init ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_samples, 2); // ADC to memory performed through DMA

  // start timer that triggers the ADC conversions
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  
  // ==== SCPI stuff ====
  struct scpi_command* source;
  struct scpi_command* measure;
  struct scpi_command* system;
  scpi_init(&ctx);

  /*
   * After initialising the parser, we set up the command tree.  Ours is
   *
   *  *IDN?         -> identify
   *  :MEASure
   *    :VOLTage?   -> get_voltage
   *    :CURRent?   -> get_current
   *    :POWer?     -> get_power
   *    :RESistance?-> get_resistance
   *  :SOURce
   *    :CURRent    -> set_current
   *    :RESistance -> set_resistance
   */
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "*IDN?", 5, "*IDN?", 5, identify);

  source = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "SOURCE", 6, "SOUR", 4, NULL);
  measure = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "MEASURE", 7, "MEAS", 4, NULL);
  system = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "SYSTEM", 6, "SYS", 3, NULL);

  scpi_register_command(measure, SCPI_CL_CHILD, "VOLTAGE?", 8, "VOLT?", 5, get_voltage);
  scpi_register_command(measure, SCPI_CL_CHILD, "CURRENT?", 8, "CURR?", 5, get_current);
  scpi_register_command(measure, SCPI_CL_CHILD, "POWER?", 6, "POW?", 4, get_power);
  scpi_register_command(measure, SCPI_CL_CHILD, "RESISTANCE?", 11, "RES?", 4, get_resistance);

  scpi_register_command(source, SCPI_CL_CHILD, "CURRENT", 7, "CURR", 4, set_current);
  scpi_register_command(source, SCPI_CL_CHILD, "RESISTANCE", 10, "RES", 3, set_resistance);

  scpi_register_command(system, SCPI_CL_CHILD, "ERRORS?", 7, "ERR?", 4, get_error_count);

  // ==== END SCPI STUFF ====

  HAL_UART_Receive_IT(&huart1, uart_buf, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // ==== CHECK FOR UART DATA ====
    // HAL_StatusTypeDef stat = HAL_UART_Receive(&huart1, uart_buf, 1, 10);
    if(uart_ind > 0 && (uart_buf[uart_ind - 1] == '\n' || uart_buf[uart_ind - 1] == '\r')){
      HAL_UART_AbortReceive(&huart1);
      uart_buf[uart_ind - 1] = '\0';
      uart_ind = 0;
      pause_i2c_interrupts();
      scpi_execute_command(&ctx, uart_buf, strlen(uart_buf));
      resume_i2c_interrupts();
      HAL_UART_Receive_IT(&huart1, uart_buf, 1);
    }

    // ==== CHECK ADC STATUS ====
    if(adc_ready){
      adc_ready = false;
      HAL_ADC_Start_DMA(&hadc1,  (uint32_t*)adc_samples, 2);
    }
    
    // // ==== CHECK FOR USB DATA ====
    // if(UserRxBufferFS[0] == ':' || UserRxBufferFS[0] == '*'){
    //   int len = strlen(UserRxBufferFS);
    //   if(len <= APP_RX_DATA_SIZE){
    //     snprintf(scratch_buf, SCRATCH_BUF_SIZE, "Executing SCPI command: %s\r\n", UserRxBufferFS);
    //     HAL_UART_Transmit(&huart1, scratch_buf, strlen(scratch_buf), 10);
    //     scpi_execute_command(&ctx, UserRxBufferFS, len);
    //     UserRxBufferFS[0] = '\0';
    //   }
      
    // }
    
    // ==== STATE MACHINE ====
    switch(mode){
      case CURRENT_INIT:{
        float ichan = iset / DAC_CHANNELS;
        uint16_t counts = (uint16_t)((DAC_FS * ichan * CS_RES * CS_AMP_GAIN)/DAC_VREF);
        update_dac_setpoints(counts);
        mode = CURRENT;
        break;
      }case RESISTANCE_INIT:{
        float itarget = vin / rset;
        float ichan = itarget / DAC_CHANNELS;
        uint16_t counts = (uint16_t)((DAC_FS * ichan * CS_RES * CS_AMP_GAIN)/DAC_VREF);
        update_dac_setpoints(counts);
        mode = RESISTANCE;
        break;
      }case POWER_INIT:{
        mode = POWER;
        break;
      }case CURRENT:{
        float ichan = iset / DAC_CHANNELS;
        uint16_t counts = (uint16_t)((DAC_FS * ichan * CS_RES * CS_AMP_GAIN)/DAC_VREF);
        update_dac_setpoints(counts);
        break;
      }case RESISTANCE:{
        float itarget = vin / rset;
        float ichan = itarget / DAC_CHANNELS;
        uint16_t counts = (uint16_t)((DAC_FS * ichan * CS_RES * CS_AMP_GAIN)/DAC_VREF);
        update_dac_setpoints(counts);
        break;
      }case POWER:{
        
        break;
      }
      case OFF:
      default:
      {
        update_dac_setpoints(0);
        break;
      }
    }
    
    // DEBUGGING
    struct scpi_error* err;
    do{
      err = scpi_pop_error(&ctx);
      if(err->id != 0){
        snprintf(scratch_buf, SCRATCH_BUF_SIZE, "%s\r\n", err->description);
        HAL_UART_Transmit(&huart1, scratch_buf, strlen(scratch_buf), 10);
      }
      free(err);
    }while(err->id != 0);

    // snprintf(uart_buf, UART_BUF_SIZE, "ISET: %.3f\r\n", iset);
    // HAL_UART_Transmit(&huart1, uart_buf, strlen(uart_buf), 10);
    
    HAL_Delay(25);
    
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
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
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
  uart_ind ++;
  HAL_UART_Receive_IT(&huart1, uart_buf + uart_ind, 1);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){

  // find the next channel to update
  channel_ind++;
  if(channel_ind >= DAC_CHANNELS){
    channel_ind = 0;
  }
  
  update_dac(&(load_channels[channel_ind]));
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  HAL_I2C_Master_Abort_IT(&hi2c1, DAC101_ADDR_BROADCAST << 1);
  // i2c_error_count ++;
  load_channels[channel_ind].errors++;

  // find the next channel to update
  channel_ind++;
  if(channel_ind >= DAC_CHANNELS){
    channel_ind = 0;
  }
  
  update_dac(&(load_channels[channel_ind]));
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
  vin_acc += adc_samples[1];
  vin_samples++;
  if(vin_samples >= VIN_SAMPLE_DEPTH){
    // compute VIN
    vin = (VIN_FE_GAIN * ADC_FS * vin_acc) / (vin_samples * 4095.0);

    // reset these things
    vin_acc = 0;
    vin_samples = 0;
  }
  load_channels[channel_adc_ind].isns_ua = adc_samples[0] * CS_UA_PER_COUNT;
  /*
  switch(mode){
    case CURRENT:{
      float ichan = iset / DAC_CHANNELS;
      uint16_t counts = (uint16_t)((DAC_FS * ichan * CS_RES * CS_AMP_GAIN)/DAC_VREF);
      update_dac_setpoints(counts);
      break;
    }case RESISTANCE:{
      float itarget = vin / rset;
      float ichan = itarget / DAC_CHANNELS;
      uint16_t counts = (uint16_t)((DAC_FS * ichan * CS_RES * CS_AMP_GAIN)/DAC_VREF);
      update_dac_setpoints(counts);
      break;
    }case POWER:{
      
      break;
    }
    case POWER_INIT:{
      break;
    }
    case RESISTANCE_INIT:{
      mode = RESISTANCE;
      break;
    }
    case CURRENT_INIT:{
      mode = CURRENT;
      break;
    }
    case OFF:
    default:
    {
      break;
    }
  }
  */
  channel_adc_ind++;
  if(channel_adc_ind >= DAC_CHANNELS){
    channel_adc_ind = 0;
  }
  switch_cs_channel(load_channels[channel_adc_ind].ch_num);
  adc_ready = true;
}


void switch_cs_channel(int ch_num){
  if((ch_num >> 2) & 1)
    CS_SEL2_GPIO_Port->ODR |= CS_SEL2_Pin;
  else
    CS_SEL2_GPIO_Port->ODR &= ~CS_SEL2_Pin;
    
  if((ch_num >> 1) & 1)
    CS_SEL1_GPIO_Port->ODR |= CS_SEL1_Pin;
  else
    CS_SEL1_GPIO_Port->ODR &= ~CS_SEL1_Pin;
    
  if(ch_num & 1)
    CS_SEL0_GPIO_Port->ODR |= CS_SEL0_Pin;
  else
    CS_SEL0_GPIO_Port->ODR &= ~CS_SEL0_Pin;
}

void update_dac(LoadChannel_t* load){
  // switch the I2C mux
  if((load->ch_num >> 2) & 1)
    I2C_SEL0_GPIO_Port->ODR |= I2C_SEL2_Pin;
  else
    I2C_SEL0_GPIO_Port->ODR &= ~I2C_SEL2_Pin;
  
  if((load->ch_num >> 1) & 1)
    I2C_SEL0_GPIO_Port->ODR |= I2C_SEL1_Pin;
  else
    I2C_SEL0_GPIO_Port->ODR &= ~I2C_SEL1_Pin;
  
  if(load->ch_num & 1)
    I2C_SEL0_GPIO_Port->ODR |= I2C_SEL0_Pin;
  else
    I2C_SEL0_GPIO_Port->ODR &= ~I2C_SEL0_Pin;

  i2c_tx_data[0] = (load->dac_value >> 6) & 0x0F;
  i2c_tx_data[1] = (load->dac_value << 2) & 0xFC;
  HAL_I2C_Master_Transmit_IT(&hi2c1, DAC101_ADDR_BROADCAST << 1, i2c_tx_data, 2);
}


void update_dac_setpoints(uint16_t counts){
  int start = channel_ind; // remember where we started the update process
  int i = start;
  do{
    load_channels[i].dac_value = counts;
    i++;
    if(i >= DAC_CHANNELS){
      i = 0;
    }
  }while(i != start);
}

float get_total_current(){
  float itotal = 0.0;
  for(int i = 0; i < DAC_CHANNELS; i++){
    if(load_channels[i].active){
      itotal += load_channels[i].isns_ua / 1.0e6f;
    }
  }
  return itotal;
}


void pause_uart_interrupts(){

}

void resume_uart_interrupts(){
  
}

void pause_i2c_interrupts(){
  HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);

}


void resume_i2c_interrupts(){
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/*
 * Respond to *IDN?
 */
scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* command){
  scpi_free_tokens(command);
  snprintf(uart_buf, UART_BUF_SIZE, "%s\r\n", idn_str);
  HAL_UART_Transmit(&huart1, uart_buf, strlen(uart_buf), 10);
  // CDC_Transmit_FS(UserTxBufferFS, strlen(UserTxBufferFS));
  return SCPI_SUCCESS;
}

/**
 * Report the voltage on the input
 */
scpi_error_t get_voltage(struct scpi_parser_context* context, struct scpi_token* command){
  snprintf(uart_buf, UART_BUF_SIZE, "%.6g\r\n", vin);
  HAL_UART_Transmit(&huart1, uart_buf, strlen(uart_buf), 10);
  // CDC_Transmit_FS(UserTxBufferFS, strlen(UserTxBufferFS));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}
scpi_error_t get_current(struct scpi_parser_context* context, struct scpi_token* command){
  float itotal = get_total_current();
  snprintf(uart_buf, UART_BUF_SIZE, "%.6g\r\n", itotal);
  HAL_UART_Transmit(&huart1, uart_buf, strlen(uart_buf), 10);
  // CDC_Transmit_FS(UserTxBufferFS, strlen(UserTxBufferFS));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
  
}
scpi_error_t get_power(struct scpi_parser_context* context, struct scpi_token* command){
  float itotal = get_total_current();
  float power = itotal * vin;
  snprintf(uart_buf, UART_BUF_SIZE, "%.6g\r\n", power);
  HAL_UART_Transmit(&huart1, uart_buf, strlen(uart_buf), 10);
  // CDC_Transmit_FS(UserTxBufferFS, strlen(UserTxBufferFS));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
  
}
scpi_error_t get_resistance(struct scpi_parser_context* context, struct scpi_token* command){
  float itotal = get_total_current();
  float resistance = vin / itotal;
  snprintf(uart_buf, UART_BUF_SIZE, "%.6g\r\n", resistance);
  HAL_UART_Transmit(&huart1, uart_buf, strlen(uart_buf), 10);
  // CDC_Transmit_FS(UserTxBufferFS, strlen(UserTxBufferFS));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}


scpi_error_t get_error_count(struct scpi_parser_context* context, struct scpi_token* command){
  snprintf(uart_buf, UART_BUF_SIZE, "--- I2C-ERRORS ---\r\n", i2c_error_count);
  HAL_UART_Transmit(&huart1, uart_buf, strlen(uart_buf), 10);
  for(int i = 0; i < DAC_CHANNELS; i++){
    snprintf(uart_buf, UART_BUF_SIZE, "\tCH%d: %d\r\n", i+1, load_channels[i].errors);
    HAL_UART_Transmit(&huart1, uart_buf, strlen(uart_buf), 10);
  }
  
  // CDC_Transmit_FS(UserTxBufferFS, strlen(UserTxBufferFS));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}
/*
scpi_error_t set_voltage(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while(args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 0, 0, MAX_VOLTAGE);
  if(output_numeric.length == 0 || (output_numeric.length == 1 && output_numeric.unit[0] == 'V') ){
    vset = output_numeric.value;
    mode = VOLTAGE_INIT;
  }else{
    struct scpi_error error;
    error.id = -200;
    error.description = "Command error;Invalid unit";
    error.length = 26;
    snprintf(scratch_buf, SCRATCH_BUF_SIZE, "%d", output_numeric.length);
    HAL_UART_Transmit(&huart1, scratch_buf, strlen(scratch_buf), 10);

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }
}
*/

scpi_error_t set_current(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while(args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 0, 0, max_current);
  if(output_numeric.length == 0 || (output_numeric.length == 1 && output_numeric.unit[0] == 'A') ){
    iset = output_numeric.value;
    mode = CURRENT_INIT;
    return SCPI_SUCCESS;
  }else{
    struct scpi_error error;
    error.id = -200;
    error.description = "Command error;Invalid unit";
    error.length = 26;
    snprintf(UserTxBufferFS, APP_TX_DATA_SIZE, "%d", output_numeric.length);
    CDC_Transmit_FS(UserTxBufferFS, strlen(UserTxBufferFS));

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }
}
scpi_error_t set_resistance(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while(args != NULL && args->type == 0)
  {
    args = args->next;
  }

  float min_r = 5.0 / max_current;
  output_numeric = scpi_parse_numeric(args->value, args->length, 0, min_r, __FLT32_MAX__);
  if(output_numeric.length == 0 || (output_numeric.length == 1 && output_numeric.unit[0] == 'O') ){
    rset = output_numeric.value;
    mode = RESISTANCE_INIT;
    return SCPI_SUCCESS;
  }else{
    struct scpi_error error;
    error.id = -200;
    error.description = "Command error;Invalid unit";
    error.length = 26;
    snprintf(UserTxBufferFS, APP_TX_DATA_SIZE, "%d", output_numeric.length);
    CDC_Transmit_FS(UserTxBufferFS, strlen(UserTxBufferFS));

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }
}

/*
scpi_error_t set_power(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while(args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 0, 0, max_current);
  if(output_numeric.length == 0 || (output_numeric.length == 1 && output_numeric.unit[0] == 'W') ){
    pset = output_numeric.value;
    mode = POWER_INIT;
  }else{
    struct scpi_error error;
    error.id = -200;
    error.description = "Command error;Invalid unit";
    error.length = 26;
    snprintf(scratch_buf, SCRATCH_BUF_SIZE, "%d", output_numeric.length);
    HAL_UART_Transmit(&huart1, scratch_buf, strlen(scratch_buf), 10);

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }
}
*/

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
    HAL_UART_Transmit(&huart1, "ERROR\r\n", 7, 10);
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
