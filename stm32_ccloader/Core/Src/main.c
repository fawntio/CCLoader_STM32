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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Start addresses on DUP (Increased buffer size improves performance)
#define ADDR_BUF0                   0x0000 // Buffer (512 bytes)
#define ADDR_DMA_DESC_0             0x0200 // DMA descriptors (8 bytes)
#define ADDR_DMA_DESC_1             (ADDR_DMA_DESC_0 + 8)

// DMA channels used on DUP
#define CH_DBG_TO_BUF0              0x01   // Channel 0
#define CH_BUF0_TO_FLASH            0x02   // Channel 1

// Debug commands
#define CMD_CHIP_ERASE              0x10
#define CMD_WR_CONFIG               0x19
#define CMD_RD_CONFIG               0x24
#define CMD_READ_STATUS             0x30
#define CMD_RESUME                  0x4C
#define CMD_DEBUG_INSTR_1B          (0x54|1)
#define CMD_DEBUG_INSTR_2B          (0x54|2)
#define CMD_DEBUG_INSTR_3B          (0x54|3)
#define CMD_BURST_WRITE             0x80
#define CMD_GET_CHIP_ID             0x68

// Debug status bitmasks
#define STATUS_CHIP_ERASE_BUSY_BM   0x80 // New debug interface
#define STATUS_PCON_IDLE_BM         0x40
#define STATUS_CPU_HALTED_BM        0x20
#define STATUS_PM_ACTIVE_BM         0x10
#define STATUS_HALT_STATUS_BM       0x08
#define STATUS_DEBUG_LOCKED_BM      0x04
#define STATUS_OSC_STABLE_BM        0x02
#define STATUS_STACK_OVERFLOW_BM    0x01

// DUP registers (XDATA space address)
#define DUP_DBGDATA                 0x6260  // Debug interface data buffer
#define DUP_FCTL                    0x6270  // Flash controller
#define DUP_FADDRL                  0x6271  // Flash controller addr
#define DUP_FADDRH                  0x6272  // Flash controller addr
#define DUP_FWDATA                  0x6273  // Clash controller data buffer
#define DUP_CLKCONSTA               0x709E  // Sys clock status
#define DUP_CLKCONCMD               0x70C6  // Sys clock configuration
#define DUP_MEMCTR                  0x70C7  // Flash bank xdata mapping
#define DUP_DMA1CFGL                0x70D2  // Low byte, DMA config ch. 1
#define DUP_DMA1CFGH                0x70D3  // Hi byte , DMA config ch. 1
#define DUP_DMA0CFGL                0x70D4  // Low byte, DMA config ch. 0
#define DUP_DMA0CFGH                0x70D5  // Low byte, DMA config ch. 0
#define DUP_DMAARM                  0x70D6  // DMA arming register

// Commands to Bootloader
#define SBEGIN                0x01
#define SDATA                 0x02
#define SRSP                  0x03
#define SEND                  0x04
#define ERRO                  0x05
#define WAITING               0x00
#define RECEIVING             0x01

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//! Convert XREG register declaration to an XDATA integer address
//#define XREG(addr)       ((uint8_t volatile __xdata *) 0)[addr]
//#define FCTL            XREG( 0x6270 )
//#define XREG_TO_INT(a)      ((uint16_t)(&(a)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
en_state_t state = WAIT_FOR_START;
uint8_t request_data_msg = SRSP;
uint8_t Done = 0;
uint16_t addr = 0x0000;
uint8_t Verify = 0;
uint8_t revision = 0;
uint8_t extra_buf[514];
//! DUP DMA descriptor
const uint8_t dma_desc_0[8] =
{
// Debug Interface -> Buffer
		HIBYTE(DUP_DBGDATA),// src[15:8]
		LOBYTE(DUP_DBGDATA),            // src[7:0]
		HIBYTE(ADDR_BUF0),              // dest[15:8]
		LOBYTE(ADDR_BUF0),              // dest[7:0]
		0,                              // len[12:8] - filled in later
		0,                              // len[7:0]
		31,                             // trigger: DBG_BW
		0x11                            // increment destination
		};
//! DUP DMA descriptor
const uint8_t dma_desc_1[8] =
{
// Buffer -> Flash controller
		HIBYTE(ADDR_BUF0),// src[15:8]
		LOBYTE(ADDR_BUF0),              // src[7:0]
		HIBYTE(DUP_FWDATA),             // dest[15:8]
		LOBYTE(DUP_FWDATA),             // dest[7:0]
		0,                              // len[12:8] - filled in later
		0,                              // len[7:0]
		18,                             // trigger: FLASH
		0x42,                           // increment source
		};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

void receive_callback(uint8_t *Buf, uint32_t *Len);
void set_state(en_state_t new_state);
en_state_t get_state(void);
void write_debug_byte(uint8_t data);
uint8_t read_debug_byte(void);
uint8_t wait_dup_ready(void);
uint8_t debug_command(uint8_t cmd, uint8_t *cmd_bytes, uint16_t num_cmd_bytes);
void debug_init(void);
uint8_t read_chip_id(void);
void burst_write_block(uint8_t *src, uint16_t num_bytes);
void chip_erase(void);
void write_xdata_memory_block(uint16_t address, const uint8_t *values,
		uint16_t num_bytes);
void write_xdata_memory(uint16_t address, uint8_t value);
uint8_t read_xdata_memory(uint16_t address);
void read_flash_memory_block(uint8_t bank, uint16_t flash_addr,
		uint16_t num_bytes, uint8_t *values);
void write_flash_memory_block(uint8_t *src, uint32_t start_addr,
		uint16_t num_bytes);
void RunDUP(void);

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
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	//digitalWrite(LED, LOW);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_SET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		uint8_t chip_id = 0;
		uint8_t debug_config = 0;
		uint8_t flag = 0;

		switch (get_state())
		{
		case INITIALZATION:
		{
			debug_init();
			chip_id = read_chip_id();
			if (chip_id == 0)
			{
				uint8_t error_msg = ERRO;
				CDC_Transmit_FS(&error_msg, 1);
				return 1; // No chip detected, run loop again.
			}

			RunDUP();
			debug_init();

			chip_erase();
			RunDUP();
			debug_init();

			// Switch DUP to external crystal osc. (XOSC) and wait for it to be stable.
			// This is recommended if XOSC is available during programming. If
			// XOSC is not available, comment out these two lines.
			write_xdata_memory(DUP_CLKCONCMD, 0x80);
			while (read_xdata_memory(DUP_CLKCONSTA) != 0x80)
				;

			// Enable DMA (Disable DMA_PAUSE bit in debug configuration)
			debug_config = 0x22;
			debug_command(CMD_WR_CONFIG, &debug_config, 1);
			state = FLASHING;
			break;
		}
		case FLASHING:
		{

			if (flag == 0)
			{
				CDC_Transmit_FS(&request_data_msg, 1);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				flag = 1;
			}

			while (!Done)
			{

			}
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			RunDUP();
			set_state(WAIT_FOR_START);
			return 0;

			break;
		}
		default:
		{
			break;
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

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
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, DC_Pin | Reset_Pin | DD_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DC_Pin */
	GPIO_InitStruct.Pin = DC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DC_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Reset_Pin */
	GPIO_InitStruct.Pin = Reset_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(Reset_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DD_Pin */
	GPIO_InitStruct.Pin = DD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**************************************************************************//**
 * @brief    Writes a byte on the debug interface. Requires DD to be
 *           output when function is called.
 * @param    data    Byte to write
 * @return   None.
 ******************************************************************************/
void write_debug_byte(uint8_t data)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		// Set clock high and put data on DD line
		//digitalWrite(DC, HIGH);

		if (data & 0x80)
		{
			//digitalWrite(DD, HIGH);
			DC_GPIO_Port->BSRR = DC_Pin | DD_Pin;

			/*HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(DD_GPIO_Port, DD_Pin, GPIO_PIN_SET);
			 */
		}
		else
		{
			//digitalWrite(DD, LOW);
			DC_GPIO_Port->BSRR = DC_Pin | (DD_Pin << 16U);
			/*HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(DD_GPIO_Port, DD_Pin, GPIO_PIN_RESET);
			 */
		}
		data <<= 1;
		//digitalWrite(DC, LOW);
		// set clock low (DUP capture flank)
		for (uint8_t nop = 0; nop < 10; nop++)
		{
			asm("NOP");
		}

		//HAL_Delay(1);
		HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);

		for (uint8_t nop = 0; nop < 10; nop++)
		{
			asm("NOP");
		}
		//HAL_Delay(1);
	}
}

/**
 * @brief    Reads a byte from the debug interface. Requires DD to be
 *           input when function is called.
 * @return   Returns the byte read.
 ******************************************************************************/
uint8_t read_debug_byte(void)
{
	uint8_t i;
	uint8_t data = 0x00;
	for (i = 0; i < 8; i++)
	{

		//digitalWrite(DC, HIGH);
		// DC high
		HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
		data <<= 1;


		for (uint8_t nop = 0; nop < 10; nop++)
				{
					asm("NOP");
				}


		//if(HIGH == digitalRead(DD))
		if (HAL_GPIO_ReadPin(DD_GPIO_Port, DD_Pin) == GPIO_PIN_SET)
		{
			data |= 0x01;
		}
		//digitalWrite(DC, LOW);
		// DC low

		HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);

		for (uint8_t nop = 0; nop < 10; nop++)
				{
					asm("NOP");
				}

	}
	return data;
}
/**************************************************************************//**
 * @brief    Function waits for DUP to indicate that it is ready. The DUP will
 *           pulls DD line low when it is ready. Requires DD to be input when
 *           function is called.
 * @return   Returns 0 if function timed out waiting for DD line to go low
 * @return   Returns 1 when DUP has indicated it is ready.
 ******************************************************************************/

uint8_t wait_dup_ready(void)
{
// DUP pulls DD low when ready
	uint16_t count = 0;
//while ((HIGH == digitalRead(DD)) && count < 16)
	while ((HAL_GPIO_ReadPin(DD_GPIO_Port, DD_Pin) == GPIO_PIN_SET)
			&& count < 16)
	{
		// Clock out 8 bits before checking if DD is low again
		read_debug_byte();
		count++;
	}
	return (count == 16) ? 0 : 1;
}

/**************************************************************************//**
 * @brief    Issues a command on the debug interface. Only commands that return
 *           one output byte are supported.
 * @param    cmd             Command byte
 * @param    cmd_bytes       Pointer to the array of data bytes following the
 *                           command byte [0-3]
 * @param    num_cmd_bytes   The number of data bytes (input to DUP) [0-3]
 * @return   Data returned by command
 ******************************************************************************/
uint8_t debug_command(uint8_t cmd, uint8_t *cmd_bytes, uint16_t num_cmd_bytes)
{
	uint16_t i;
	uint8_t output = 0;
// Make sure DD is output
//pinMode(DD, OUTPUT);
// Send command
	write_debug_byte(cmd);
// Send bytes
	for (i = 0; i < num_cmd_bytes; i++)
	{
		write_debug_byte(cmd_bytes[i]);
	}
// Set DD as input
// pinMode(DD, INPUT);
//digitalWrite(DD, HIGH);
	HAL_GPIO_WritePin(DD_GPIO_Port, DD_Pin, GPIO_PIN_SET);
// Wait for data to be ready
	wait_dup_ready();
// Read returned byte
	output = read_debug_byte();
// Set DD as output
//pinMode(DD, OUTPUT);

	return output;
}

/**************************************************************************//**
 * @brief    Resets the DUP into debug mode. Function assumes that
 *           the programmer I/O has already been configured using e.g.
 *           ProgrammerInit().
 * @return   None.
 ******************************************************************************/
void debug_init(void)
{

// Send two flanks on DC while keeping RESET_N low
// All low (incl. RESET_N)
//digitalWrite(DD, LOW);
	HAL_GPIO_WritePin(DD_GPIO_Port, DD_Pin, GPIO_PIN_RESET);
//digitalWrite(DC, LOW);
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
// digitalWrite(RESET, LOW);
	HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);   // Wait
//digitalWrite(DC, HIGH);
// DC high
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
	HAL_Delay(10);   // Wait
//digitalWrite(DC, LOW);
// DC low
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);   // Wait
//digitalWrite(DC, HIGH);
// DC high
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
	HAL_Delay(10);   // Wait
//digitalWrite(DC, LOW);
// DC low
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);   // Wait
//digitalWrite(RESET, HIGH);
// Release RESET_N
	HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(10);   // Wait

}

/**************************************************************************//**
 * @brief    Reads the chip ID over the debug interface using the
 *           GET_CHIP_ID command.
 * @return   Returns the chip id returned by the DUP
 ******************************************************************************/
uint8_t read_chip_id(void)
{
	uint8_t id = 0;

// Make sure DD is output
//pinMode(DD, OUTPUT);
	HAL_Delay(1);
// Send command
	write_debug_byte(CMD_GET_CHIP_ID);
// Set DD as input
//pinMode(DD, INPUT);
//digitalWrite(DD, HIGH);
	HAL_GPIO_WritePin(DD_GPIO_Port, DD_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
// Wait for data to be ready
	if (wait_dup_ready() == 1)
	{
		// Read ID and revision
		id = read_debug_byte(); // ID
		revision = read_debug_byte();      // Revision (discard)
	}
// Set DD as output
//pinMode(DD, OUTPUT);

	return id;
}

/**************************************************************************//**
 * @brief    Sends a block of data over the debug interface using the
 *           BURST_WRITE command.
 * @param    src         Pointer to the array of input bytes
 * @param    num_bytes   The number of input bytes
 * @return   None.
 ******************************************************************************/
void burst_write_block(uint8_t *src, uint16_t num_bytes)
{
	uint16_t i;

// Make sure DD is output
//pinMode(DD, OUTPUT);

	write_debug_byte(CMD_BURST_WRITE | HIBYTE(num_bytes));
	write_debug_byte(LOBYTE(num_bytes));
	for (i = 0; i < num_bytes; i++)
	{
		write_debug_byte(src[i]);
	}

// Set DD as input
//pinMode(DD, INPUT);
//digitalWrite(DD, HIGH);
	HAL_GPIO_WritePin(DD_GPIO_Port, DD_Pin, GPIO_PIN_SET);
// Wait for DUP to be ready
	wait_dup_ready();
	read_debug_byte(); // ignore output
// Set DD as output
//pinMode(DD, OUTPUT);
}

/**************************************************************************//**
 * @brief    Issues a CHIP_ERASE command on the debug interface and waits for it
 *           to complete.
 * @return   None.
 ******************************************************************************/
void chip_erase(void)
{
	volatile uint8_t status;
// Send command
	debug_command(CMD_CHIP_ERASE, 0, 0);

// Wait for status bit 7 to go low
	do
	{
		status = debug_command(CMD_READ_STATUS, 0, 0);
	} while ((status & STATUS_CHIP_ERASE_BUSY_BM));
}

/**************************************************************************//**
 * @brief    Writes a block of data to the DUP's XDATA space.
 * @param    address     XDATA start address
 * @param    values      Pointer to the array of bytes to write
 * @param    num_bytes   Number of bytes to write
 * @return   None.
 ******************************************************************************/
void write_xdata_memory_block(uint16_t address, const uint8_t *values,
		uint16_t num_bytes)
{
	uint8_t instr[3];
	uint16_t i;

// MOV DPTR, address
	instr[0] = 0x90;
	instr[1] = HIBYTE(address);
	instr[2] = LOBYTE(address);
	debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

	for (i = 0; i < num_bytes; i++)
	{
		// MOV A, values[i]
		instr[0] = 0x74;
		instr[1] = values[i];
		debug_command(CMD_DEBUG_INSTR_2B, instr, 2);

		// MOV @DPTR, A
		instr[0] = 0xF0;
		debug_command(CMD_DEBUG_INSTR_1B, instr, 1);

		// INC DPTR
		instr[0] = 0xA3;
		debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
	}
}

/**************************************************************************//**
 * @brief    Writes a byte to a specific address in the DUP's XDATA space.
 * @param    address     XDATA address
 * @param    value       Value to write
 * @return   None.
 ******************************************************************************/
void write_xdata_memory(uint16_t address, uint8_t value)
{
	uint8_t instr[3];

// MOV DPTR, address
	instr[0] = 0x90;
	instr[1] = HIBYTE(address);
	instr[2] = LOBYTE(address);
	debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

// MOV A, values[i]
	instr[0] = 0x74;
	instr[1] = value;
	debug_command(CMD_DEBUG_INSTR_2B, instr, 2);

// MOV @DPTR, A
	instr[0] = 0xF0;
	debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
}

/**************************************************************************//**
 * @brief    Read a byte from a specific address in the DUP's XDATA space.
 * @param    address     XDATA address
 * @return   Value read from XDATA
 ******************************************************************************/
uint8_t read_xdata_memory(uint16_t address)
{
	uint8_t instr[3];

// MOV DPTR, address
	instr[0] = 0x90;
	instr[1] = HIBYTE(address);
	instr[2] = LOBYTE(address);
	debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

// MOVX A, @DPTR
	instr[0] = 0xE0;
	return debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
}

/**************************************************************************//**
 * @brief    Reads 1-32767 bytes from DUP's flash to a given buffer on the
 *           programmer.
 * @param    bank        Flash bank to read from [0-7]
 * @param    address     Flash memory start address [0x0000 - 0x7FFF]
 * @param    values      Pointer to destination buffer.
 * @return   None.
 ******************************************************************************/
void read_flash_memory_block(uint8_t bank, uint16_t flash_addr,
		uint16_t num_bytes, uint8_t *values)
{
	uint8_t instr[3];
	uint16_t i;
	uint16_t xdata_addr = (0x8000 + flash_addr);

// 1. Map flash memory bank to XDATA address 0x8000-0xFFFF
	write_xdata_memory(DUP_MEMCTR, bank);

// 2. Move data pointer to XDATA address (MOV DPTR, xdata_addr)
	instr[0] = 0x90;
	instr[1] = HIBYTE(xdata_addr);
	instr[2] = LOBYTE(xdata_addr);
	debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

	for (i = 0; i < num_bytes; i++)
	{
		// 3. Move value pointed to by DPTR to accumulator (MOVX A, @DPTR)
		instr[0] = 0xE0;
		values[i] = debug_command(CMD_DEBUG_INSTR_1B, instr, 1);

		// 4. Increment data pointer (INC DPTR)
		instr[0] = 0xA3;
		debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
	}
}

/**************************************************************************//**
 * @brief    Writes 4-2048 bytes to DUP's flash memory. Parameter \c num_bytes
 *           must be a multiple of 4.
 * @param    src         Pointer to programmer's source buffer (in XDATA space)
 * @param    start_addr  FLASH memory start address [0x0000 - 0x7FFF]
 * @param    num_bytes   Number of bytes to transfer [4-1024]
 * @return   None.
 ******************************************************************************/
void write_flash_memory_block(uint8_t *src, uint32_t start_addr,
		uint16_t num_bytes)
{
// 1. Write the 2 DMA descriptors to RAM
	write_xdata_memory_block(ADDR_DMA_DESC_0, dma_desc_0, 8);
	write_xdata_memory_block(ADDR_DMA_DESC_1, dma_desc_1, 8);

// 2. Update LEN value in DUP's DMA descriptors
	uint8_t len[2] =
	{ HIBYTE(num_bytes), LOBYTE(num_bytes) };
	write_xdata_memory_block((ADDR_DMA_DESC_0 + 4), len, 2);  // LEN, DBG => ram
	write_xdata_memory_block((ADDR_DMA_DESC_1 + 4), len, 2); // LEN, ram => flash

// 3. Set DMA controller pointer to the DMA descriptors
	write_xdata_memory(DUP_DMA0CFGH, HIBYTE(ADDR_DMA_DESC_0));
	write_xdata_memory(DUP_DMA0CFGL, LOBYTE(ADDR_DMA_DESC_0));
	write_xdata_memory(DUP_DMA1CFGH, HIBYTE(ADDR_DMA_DESC_1));
	write_xdata_memory(DUP_DMA1CFGL, LOBYTE(ADDR_DMA_DESC_1));

// 4. Set Flash controller start address (wants 16MSb of 18 bit address)
	write_xdata_memory(DUP_FADDRH, HIBYTE((start_addr)));  //>>2) ));
	write_xdata_memory(DUP_FADDRL, LOBYTE((start_addr)));  //>>2) ));

// 5. Arm DBG=>buffer DMA channel and start burst write
	write_xdata_memory(DUP_DMAARM, CH_DBG_TO_BUF0);
	burst_write_block(src, num_bytes);

// 6. Start programming: buffer to flash
	write_xdata_memory(DUP_DMAARM, CH_BUF0_TO_FLASH);
	write_xdata_memory(DUP_FCTL, 0x0A);  //0x06

// 7. Wait until flash controller is done
	while (read_xdata_memory(DUP_FCTL) & 0x80);
}

/**************************************************************************//**
 * @brief    Resets the DUP into normal mode.
 * @param    None.
 * @return   None.
 ******************************************************************************/
void RunDUP(void)
{

// Send two flanks on DC while keeping RESET_N low
// All low (incl. RESET_N)

//digitalWrite(DD, LOW);
	HAL_GPIO_WritePin(DD_GPIO_Port, DD_Pin, GPIO_PIN_RESET);
//digitalWrite(DC, LOW);
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
// digitalWrite(RESET, LOW);
	HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);   // Wait

// Release RESET_N
	HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(10);   // Wait

}

/**************************************************************************//**
 * @brief    Getting state for "state machine".
 * @param    None.
 * @return   State.
 ******************************************************************************/
en_state_t get_state(void)
{
	return state;
}
/**************************************************************************//**
 * @brief    Setting state for "state machine".
 * @param    New state.
 * @return   None.
 ******************************************************************************/
void set_state(en_state_t new_state)
{
	state = new_state;
	return;
}

/**************************************************************************//**
 * @brief    Setting state for "state machine".
 * @param    New state.
 * @return   None.
 ******************************************************************************/
void receive_callback(uint8_t *Buf, uint32_t *Len)
{
	switch (get_state())
	{
	case WAIT_FOR_START:
	{
		if (*Len >= 2)
		{
			if (Buf[0] == SBEGIN)
			{
				Verify = Buf[1];
				set_state(INITIALZATION);
			}
		}
		break;
	}
	case FLASHING:
	{
		static uint8_t sdata_flag = 0;
		if ((Buf[0] == SDATA) || (sdata_flag == 1))  // Incoming bytes are data
		{

			static uint16_t counter = 0;

			if (sdata_flag == 0)
			{
				for (uint16_t i = 0; i < *Len; i++)
				{
					extra_buf[i] = Buf[i + 1];
				}
				counter += *Len - 1;
				//sprintf(extra_buf + counter, Buf[1], *Len);
			}
			else
			{
				for (uint16_t i = 0; i < *Len; i++)
				{
					extra_buf[i + counter] = Buf[i];
				}
				counter += *Len;
			}


			sdata_flag = 1;
			if (counter >= 514)
			{
				uint16_t CheckSum = 0x0000;

				for (uint16_t i = 0; i < 512; i++)
				{
					CheckSum += extra_buf[i];
				}
				uint16_t CheckSum_t = extra_buf[512] << 8 | extra_buf[513];
				if (CheckSum_t != CheckSum)
				{
					uint8_t error_msg = ERRO;
					CDC_Transmit_FS(&error_msg, 1);
					chip_erase();
					return;
				}
				write_flash_memory_block(extra_buf, addr, 512); // src, address, count
				addr += (uint16_t) 128;
				CDC_Transmit_FS(&request_data_msg, 1);
				sdata_flag = 0;
				counter = 0;
			}

		}

		else if (Buf[0] == SEND)   // End receiving firmware
		{
			Done = 1;           // Exit while(1) in main function
			sdata_flag = 0;

		}
		break;
	}

	default:
	{
		break;
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
