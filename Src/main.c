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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "nRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum {
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
    RF24_PA_HIGH,
    RF24_PA_MAX,
    RF24_PA_ERROR
} rf24_pa_dbm_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _BV(x) (1<<(x))
#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
bool rf_begin(void);
uint8_t write_register(uint8_t reg, uint8_t value);
void rf_setPALevel(uint8_t level);
void rf_openWritingPipe(const uint8_t* address);
void rf_openReadingPipe(uint8_t child, const uint8_t* address);
void rf_startListening(void);
bool rf_available(uint8_t* pipe_num);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // bool role = 0;
  bool radioNumber = 0;
   uint8_t addresses[][6] = {"1Node","2Node"};
  // uint8_t addresses[][6] = {"edoN1","edoN2"};
  const uint8_t address[6] = "00001";
  
 uint8_t spi_txbuf[33];
 uint8_t spi_rxbuf[33];
 

  if(rf_begin() == true)
    {

      if  (radioNumber == 0)
	{
	   rf_openWritingPipe(addresses[0]);
	    rf_openReadingPipe(1, addresses[1]);

	  //rf_openReadingPipe(0, address);
	    //  rf_setPALevel(RF24_PA_HIGH);
	  rf_startListening();
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	  while (1)
	    {
	    if (rf_available(NULL) == 1) {
	      while(rf_available(NULL) == 1)
		{
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // NSS1 low
	      for (int i=0; i<33; i++)
		{
		  spi_txbuf[i] = 0xFF;
		}
	      spi_txbuf[0] = R_RX_PAYLOAD;
	 
	      HAL_SPI_TransmitReceive(&hspi1, spi_txbuf, spi_rxbuf, 33, 1000);
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // NSS1 low
	      write_register(NRF_STATUS, _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS));
		}
              HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); 
            }
          }
        }
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint8_t write_register(uint8_t reg, uint8_t value)
{
  uint8_t status = 0;
  uint8_t spi_txbuf[2];
  uint8_t spi_rxbuf[2];

  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_RESET ); // NSS1 low
  spi_txbuf[0] = W_REGISTER | (REGISTER_MASK & reg);
  spi_txbuf[1] = value;

  HAL_SPI_TransmitReceive(&hspi1, spi_txbuf, spi_rxbuf, 2, 1000);

  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_SET ); // NSS1 low
  status = spi_rxbuf[0];

  return status;
}
 
uint8_t write_register_buf(uint8_t reg, const uint8_t* buf, uint8_t buf_size)
{
  uint8_t status = 0;
  uint8_t spi_txbuf[2];
  uint8_t spi_rxbuf[2];
 
  spi_txbuf[0] = W_REGISTER | (REGISTER_MASK & reg);

  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_RESET ); // NSS1 low

  HAL_SPI_TransmitReceive(&hspi1, spi_txbuf, spi_rxbuf, 1, 1000);
  status = spi_rxbuf[0];
  HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, buf_size, 1000);

  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_SET ); // NSS1 low

  return status;
}

uint8_t read_register(uint8_t reg) {
  uint8_t result;
  uint8_t spi_txbuf[3];
  uint8_t spi_rxbuf[3];

  spi_txbuf[0] = R_REGISTER | (REGISTER_MASK & reg);
  spi_txbuf[1] = 0xFF;

  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_RESET ); // NSS1 low
  HAL_SPI_TransmitReceive(&hspi1, spi_txbuf, spi_rxbuf, 2, 1000);

  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_SET ); // NSS1 low

  result = spi_rxbuf[1];

  return result;
}

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum {
    RF24_1MBPS = 0,
    RF24_2MBPS,
    RF24_250KBPS
} rf24_datarate_e;


void setRetries(uint8_t delay, uint8_t count)
{
    write_register(SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}

uint32_t txDelay;
bool p_variant; /* False for RF24L01 and true for RF24L01P */
bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
uint8_t addr_width = 5;
uint8_t payload_size = 32;
uint8_t pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */
static const uint8_t child_pipe[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};
static const uint8_t child_payload_size[] = {RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5};
static const uint8_t child_pipe_enable[] = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};

bool setDataRate(rf24_datarate_e speed)
{
    bool result = false;
    uint8_t setup = read_register(RF_SETUP);

    // HIGH and LOW '00' is 1Mbs - our default
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    #if !defined(F_CPU) || F_CPU > 20000000
    txDelay = 250;
    #else //16Mhz Arduino
    txDelay=85;
    #endif
    if (speed == RF24_250KBPS) {
        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.
        setup |= _BV(RF_DR_LOW);
        #if !defined(F_CPU) || F_CPU > 20000000
        txDelay = 450;
        #else //16Mhz Arduino
        txDelay = 155;
        #endif
    } else {
        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'
        if (speed == RF24_2MBPS) {
            setup |= _BV(RF_DR_HIGH);
            #if !defined(F_CPU) || F_CPU > 20000000
            txDelay = 190;
            #else // 16Mhz Arduino
            txDelay = 65;
            #endif
        }
    }
    write_register(RF_SETUP, setup);

    // Verify our result
    if (read_register(RF_SETUP) == setup) {
        result = true;
    }
    return result;
}

void toggle_features(void)
{
  uint8_t spi_txbuf[2];
  uint8_t spi_rxbuf[2];

  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_RESET ); // NSS1 low
  spi_txbuf[0] = ACTIVATE;
  spi_txbuf[1] = 0x73;

  
  HAL_SPI_TransmitReceive(&hspi1, spi_txbuf, spi_rxbuf, 2, 1000);

  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_SET ); // NSS1 low
}

uint8_t spiTrans(uint8_t cmd)
{
  uint8_t status;

  uint8_t spi_txbuf[1];
  uint8_t spi_rxbuf[1];

  spi_txbuf[0] = cmd;

  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_RESET ); // NSS1 low
  HAL_SPI_TransmitReceive(&hspi1, spi_txbuf, spi_rxbuf, 1, 1000);
  status = spi_txbuf[0];

  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_SET ); // NSS1 low
  return status;
}

void setChannel(uint8_t channel)
{
    const uint8_t max_channel = 125;
    write_register(RF_CH, rf24_min(channel, max_channel));
}

uint8_t flush_rx(void)
{
    return spiTrans(FLUSH_RX);
}

uint8_t flush_tx(void)
{
    return spiTrans(FLUSH_TX);
}

//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void powerUp(void)
{
    uint8_t cfg = read_register(NRF_CONFIG);

    // if not powered up then power up and wait for the radio to initialize
    if (!(cfg & _BV(PWR_UP))) {
        write_register(NRF_CONFIG, cfg | _BV(PWR_UP));

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        HAL_Delay(5);
    }
}
/****************************************************************************/


bool rf_begin(void)
{
  uint8_t setup =  0;
pipe0_reading_address[0]=0;
  // don't think that the two below are actually needed. Should
  // probably be removed
  
  //ce(low); csn(high);z

  // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
  // delay(5);
  HAL_Delay(5);

  // Reset NRF_CONFIG and enable 16-bit CRC.
    write_register(NRF_CONFIG, 0x0C);


    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See documentation for a more complete explanation.
    setRetries(5, 15);

    // Reset value is MAX
    //setPALevel( RF24_PA_MAX ) ;

    // check for connected module and if this is a p nRF24l01 variant
    //
    if (setDataRate(RF24_250KBPS)) {
        p_variant = true;
    }
    setup = read_register(RF_SETUP);
    /*if( setup == 0b00001110 )     // register default for nRF24L01P
    {
      p_variant = true ;
    }*/

    // Then set the data rate to the slowest (and most reliable) speed supported by all
    // hardware.
    setDataRate(RF24_1MBPS);

    // Initialize CRC and request 2-byte (16bit) CRC
    //setCRCLength( RF24_CRC_16 ) ;

    // Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
    toggle_features();
    
    write_register(FEATURE, 0);
    write_register(DYNPD, 0);
    dynamic_payloads_enabled = false;

    // Reset current status
    // Notice reset and flush is the last thing we do
    write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    setChannel(76);

    // Flush buffers
    flush_rx();
    flush_tx();

    powerUp(); //Power up by default when begin() is called

    // Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
    // PTX should use only 22uA of power
    write_register(NRF_CONFIG, (read_register(NRF_CONFIG)) & ~_BV(PRIM_RX));

    // if setup is 0 or ff then there was no response from module
    return (setup != 0 && setup != 0xff);
}

void rf_setPALevel(uint8_t level)
{
    uint8_t setup = read_register(RF_SETUP) & 0xF8;

    if (level > 3) {                        // If invalid level, go to max PA
        level = (RF24_PA_MAX << 1) + 1;        // +1 to support the SI24R1 chip extra bit
    } else {
        level = (level << 1) + 1;            // Else set level as requested
    }

    write_register(RF_SETUP, setup |= level);    // Write it to the chip
}

void rf_openWritingPipe(const uint8_t* address)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.
    write_register_buf(RX_ADDR_P0, address, addr_width);
    write_register_buf(TX_ADDR, address, addr_width);

    //const uint8_t max_payload_size = 32;
    //write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
    write_register(RX_PW_P0, payload_size);
}

void rf_openReadingPipe(uint8_t child, const uint8_t* address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
      // memcpy(pipe0_reading_address, address, addr_width);
      for (int i=0; i<addr_width; i++)
	{
	  pipe0_reading_address[i] = address[i];
	}
    }
    if (child <= 6) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            write_register_buf(child_pipe[child], address, addr_width);
        } else {
            write_register_buf(child_pipe[child], address, 1);
        }
        write_register(child_payload_size[child], payload_size);

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(child_pipe_enable[child]));

    }
}

#define HIGH 1
#define LOW 0

void ce(uint8_t state)
{
  // push the ce to a given state
  // ce = pa11
  if (state == HIGH)
    {
      HAL_GPIO_WritePin( GPIOA, GPIO_PIN_11, GPIO_PIN_SET ); // NSS1 low
    }
  else if (state == LOW)
    {
      HAL_GPIO_WritePin( GPIOA, GPIO_PIN_11, GPIO_PIN_RESET ); // NSS1 low
    }
}
   
void closeReadingPipe(uint8_t pipe)
{
    write_register(EN_RXADDR, read_register(EN_RXADDR) & ~_BV(child_pipe_enable[pipe]));
}

void rf_startListening(void)
{
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) | _BV(PRIM_RX));
    write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
    ce(HIGH);
    // Restore the pipe0 adddress, if exists
    if (pipe0_reading_address[0] > 0) {
        write_register_buf(RX_ADDR_P0, pipe0_reading_address, addr_width);
    } else {
        closeReadingPipe(0);
    }

    // Flush buffers
    //flush_rx();
    if (read_register(FEATURE) & _BV(EN_ACK_PAY)) {
        flush_tx();
    }

    // Go!
    //delayMicroseconds(100);

    HAL_Delay(100);
}

uint8_t get_status(void) { return spiTrans(RF24_NOP); }

bool rf_available(uint8_t* pipe_num)
{
    if (!(read_register(FIFO_STATUS) & _BV(RX_EMPTY))) {

        // If the caller wants the pipe number, include that
        if (pipe_num) {
            uint8_t status = get_status();
            *pipe_num = (status >> RX_P_NO) & 0x07;
        }
        return 1;
    }

    return 0;


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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
