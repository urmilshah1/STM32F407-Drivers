/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Sep 29, 2022
 *      Author: UrmilShah
 */


#include "stm32f407xx_gpio_driver.h"



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

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_EN();
			}
			else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_EN();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();
			}
			else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_EN();
			}
		}
	else
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DI();
			}
			else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_DI();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}
			else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_DI();
			}
	}


}

/*
 * Init and DeInit
 */
/**
  ******************************************************************************
  * @fn   GPIO_Init
  * @brief This function initializes the pin for any GPIO pin
  * @param[in]- GPIO_Handle_t *pGPIOHandle
  * @return - Returns None
  *
  * @Note -
  ******************************************************************************
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t temp = 0;
	// 1. Configure the mode of the pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //Setting
	}
	else
	{
		//interrupt mode will be configured later
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//Set the FTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Better to clear the RTSR bit to make sure both bits are not set
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Set the RTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the FTSR bit
			EXTI->FTSR &= ~ (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//Configure both FTSR and RTSR
			// Set the RTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Set the FTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. Configure the GPIO port selection in SYSCFG_EXTICR


		//e. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	//2. Configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	//3. Configure the pupd (pull up/pull down) settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	//4. Configure output type
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	//5. Configure the Alternate functionality
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		//configure alt function registers
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<( 4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<( 4 * temp2)); //setting
	}
	else
	{

	}
}

/**
  ******************************************************************************
  * @fn   GPIO_DeInit
  * @brief This function de-initializes the pin for any GPIO pin
  * @param[in]- GPIO_RegDef_t *pGPIOx
  * @return - Returns None
  *
  * @Note -
  ******************************************************************************
*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 * Read and Write Operations
 */

/**
  ******************************************************************************
  * @fn   GPIO_ReadFromInputPin
  * @brief This function reads from an input the GPIO pin
  * @param[in] - GPIO_RegDef_t *pGPIOx
  * @param[in] - uint8_t PinNumber
  * @return - Returns the value at the pin (0 or 1)
  *
  * @Note -
  ******************************************************************************
*/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   uint8_t value;
   value = (uint8_t)((pGPIOx->IDR>>PinNumber) & 0x00000001);
   return value;
}

/**
  ******************************************************************************
  * @fn   GPIO_ReadFromInputPort
  * @brief This function reads the data from GPIO port
  * @param[in]- GPIO_RegDef_t *pGPIOx
  * @return - Returns something
  *
  * @Note -
  ******************************************************************************
*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	 uint16_t value;
	 value = (uint16_t)pGPIOx->IDR;
	 return value;
}

/**
  ******************************************************************************
  * @fn   GPIO_WriteToOutputPin
  * @brief This function writes to the output pin
  * @param[in] - GPIO_RegDef_t *pGPIOx
  * @param[in] - uint8_t PinNumber
  * @param[in] - uint8_t Value
  * @return - Returns None
  *
  * @Note -
  ******************************************************************************
*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//Write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |=  (1 << PinNumber);
	}
	else
	{
		//Write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
/**
  ******************************************************************************
  * @fn   GPIO_WriteToOutputPort
  * @brief This function de-initializes the pin for any GPIO pin
  * @param[in] - GPIO_RegDef_t *pGPIOx
  * @param[in] - uint16_t Value
  * @return - Returns None
  *
  * @Note -
  ******************************************************************************
*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
/**
  ******************************************************************************
  * @fn   GPIO_ToggleOutputPin
  * @brief This function writes to the output port of any GPIO pin
  * @param[in] - GPIO_RegDef_t *pGPIOx
  * @param[in] -  uint8_t PinNumber
  * @return - Returns None
  *
  * @Note -
  ******************************************************************************
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);

}

/*
 * Interrupt and ISR Handling
 */
/**
  ******************************************************************************
  * @fn   GPIO_IRQConfig
  * @brief This function is used for Interrupt handling
  * @param[in]- GPIO_RegDef_t *pGPIOx
  * @param[in] - uint8_t IRQPriortiy
  * @param[in] - uint8_t EnorDi
  * @return - Returns None
  *
  * @Note -
  ******************************************************************************
*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	// Processor Side not MCU peripheral side
	// Register of the processor
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1  << IRQNumber);

		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1  << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= ( 1  << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber);

		}
		else if(IRQNumber >31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber <96)
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64));
		}
	}
}
/**
  ******************************************************************************
  * @fn   GPIO_IRQPriorityConfig
  * @brief This function sets the priority of the pin for any GPIO pin
  * @param[in] - uint8_t IRQNumber
  * @param[in] - uint8_t IRQPriority
  * @return - Returns None
  *
  * @Note -
  ******************************************************************************
*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1 Find out the IPR register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber %4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/**
  ******************************************************************************
  * @fn   GPIO_IRQHandling
  * @brief This function handles the interrupt for the pin
  * @param[in]- GPIO_RegDef_t *pGPIOx
  * @return - Returns None
  *
  * @Note -
  ******************************************************************************
*/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Implement the ISR function

	//Clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}


	//Store the address of the ISR at the vector address location corresponding to the IRQ number for which you have written the ISR
}

