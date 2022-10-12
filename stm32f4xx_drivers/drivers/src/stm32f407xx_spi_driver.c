/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Oct 9, 2022
 *      Author: UrmilShah
 */

#include "stm32f407xx_spi_driver.h"



/*
 * Peripheral Clock Setup
 */

/**
  ******************************************************************************
  * @fn   GPIO_PeriClockControl
  * @brief This function enables or disables the clock for GPIO
  * @param[in]- GPIO_RegDef_t *pGPIOx
  * @param[in] - uint8_t EnorDi
  * @return - Returns None
  *
  * @Note -
  ******************************************************************************
*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	{
		if(EnorDi == ENABLE)
			{
				if(pSPIx == SPI1)
				{
					SPI1_PCLK_EN();
				}
				else if(pSPIx == SPI2)
				{
					SPI2_PCLK_EN();
				}
				else if(pSPIx == SPI3)
				{
					SPI3_PCLK_EN();
				}
				else if(pSPIx == SPI4)
				{
					SPI4_PCLK_EN();
				}
			}
		else
			if(pSPIx == SPI1)
				{
					SPI1_PCLK_DI();
				}
				else if(pSPIx == SPI2)
				{
					SPI2_PCLK_DI();
				}
				else if(pSPIx == SPI3)
				{
					SPI3_PCLK_DI();
				}
				else if(pSPIx == SPI4)
				{
					SPI4_PCLK_DI();
				}
		}

}


/*
 * Init and De-Init
******************************************************************************
* @fn   SPI_Init
* @brief This function initializes the pin for any SPI pin
* @param[in]- SPI_Handle_t *pSPIHandle
* @return - Returns None
*
* @Note -
******************************************************************************
*/
void SPI_Init(SPI_Handle_t *pSPIHandle){

	//first configure the SPI_CR1 register
	uint32_t tempreg = 0;
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// The BIDI mode should be cleared
		tempreg &= ~(1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BiDI mode should be set
		tempreg |= (1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << 15);
		//RXONLY bit must be set
		tempreg |=  (1 << 10);
	}
	//Configure the Clock Speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3 ;
	//Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 1 ;
	//configure the clock polarity
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;
	//Configure the clock Clock phase
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;
}

/*
******************************************************************************
* @fn   SPI_Init
* @brief This function initializes the pin for any SPI pin
* @param[in]- SPI_Handle_t *pSPIHandle
* @return - Returns None
*
* @Note -
******************************************************************************
*/


void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}


/*
 *  Data Send and Receive
 *  1. Can be blocking, non blocking or DMA based API
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

}
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

}

/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

}
void SPI_IRQHandling(SPI_Handle_t *pHandle){

}


/* Other Peripheral Control APIs
*
*/


