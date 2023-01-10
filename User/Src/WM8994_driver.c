/******************************************************************************/
/****************************** Includes  *************************************/
/******************************************************************************/
#include "WM8994_driver.h"

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_i2c.h"

/******************************************************************************/
/****************************** Deifnes ***************************************/
/******************************************************************************/
#define I2C_DEVICE_ADDR						0x34u    /* 0x36u if the ADDR pin is HIGH */
#define HEADPHONE_COLD_START_UP_REG_ADDR	0x0110u
#define HEADPHONE_COLD_START_UP_REG_VAL		0x8100u

#define CONTROL_INTERFACE_REG_ADDR			0x101u
#define CONTROL_INTERFACE_DEFAULT_VALUE		0x8004u

#define I2C_SINGLE_CMD_LEN					4u
#define I2C_WRITE_TIMEOUT					1000u
/******************************************************************************/
/****************************** Private types *********************************/
/******************************************************************************/


/******************************************************************************/
/****************************** Globals ***************************************/
/******************************************************************************/

/******************************************************************************/
/****************************** Externs ***************************************/
/******************************************************************************/
extern I2C_HandleTypeDef hi2c3;
/******************************************************************************/
/********************** Private functions prototypes **************************/
/******************************************************************************/

/******************************************************************************/
/********************** Platform dependent functions **************************/
/******************************************************************************/

/******************************************************************************/
/*************************** Private functions ********************************/
/******************************************************************************/


/******************************************************************************/
/**************************** Public functions ********************************/
/******************************************************************************/
uint8_t WM8994_init(void){
	uint8_t init_data[I2C_SINGLE_CMD_LEN];
	HAL_StatusTypeDef write_res = HAL_ERROR;
	HAL_StatusTypeDef read_res = HAL_ERROR;
	uint16_t control_interface_reg = 0u;
	init_data[0] = (uint8_t)(HEADPHONE_COLD_START_UP_REG_ADDR >> 8);
	init_data[1] = (uint8_t)(HEADPHONE_COLD_START_UP_REG_ADDR);
	init_data[2] = (uint8_t)(HEADPHONE_COLD_START_UP_REG_VAL >> 8);
	init_data[3] = (uint8_t)(HEADPHONE_COLD_START_UP_REG_VAL);

	write_res = HAL_I2C_Master_Transmit(&hi2c3, I2C_DEVICE_ADDR, init_data, I2C_SINGLE_CMD_LEN, I2C_WRITE_TIMEOUT);

	/* Read the control interface reg to check that I2C works */
	init_data[0] = (uint8_t)(CONTROL_INTERFACE_REG_ADDR >> 8);
	init_data[1] = (uint8_t)(CONTROL_INTERFACE_REG_ADDR);

	write_res = HAL_I2C_Master_Transmit(&hi2c3, I2C_DEVICE_ADDR, init_data, 2u, I2C_WRITE_TIMEOUT);
	read_res = HAL_I2C_Master_Receive(&hi2c3, I2C_DEVICE_ADDR, init_data, 2u, I2C_WRITE_TIMEOUT);
	control_interface_reg = init_data[1];
	control_interface_reg |= (init_data[0] << 8u);
	if( control_interface_reg == CONTROL_INTERFACE_DEFAULT_VALUE && read_res != HAL_ERROR && write_res != HAL_ERROR ){
		return PASS;
	}
	return FAIL;
}
/*----------------------------------------------------------------------------*/

/******************************************************************************/
