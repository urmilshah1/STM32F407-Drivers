/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Sep 29, 2022
 *      Author: UrmilShah
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * ***********************************Peripheral register definition structure***********************************
 * e.g. Number of GPIO registers of Stm32f4xx family of MCUs
 *
 */
typedef struct
{
	uint8_t GPIO_PinNumber; /* !<Possible Values for the pins are @GPIO_Pin_Number > */
	uint8_t GPIO_PinMode;  /* !<Possible Values for the pins are @GPIO_Pin_Modes > */
	uint8_t GPIO_PinSpeed; /* !<Possible Values for the pins are @GPIO_Pin_Speeds > */
	uint8_t GPIO_PinPuPdControl;/* !<Possible Values for the pins are @@GPIO_Pin_PUPD > */
	uint8_t GPIO_PinOPType; /* !<Possible Values for the pins are @GPIO_Output_Type > */
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*
 * This a handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx; //This holds the base address of the GPIO port  to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; //This variable holds the GPIO pin configuration settings

}GPIO_Handle_t;


/*
 * @GPIO_Pin_Number
 * GPIO number of the pins macro
 */
#define GPIO_PIN_NO_0 		0
#define GPIO_PIN_NO_1 		1
#define GPIO_PIN_NO_2 		2
#define GPIO_PIN_NO_3 		3
#define GPIO_PIN_NO_4 		4
#define GPIO_PIN_NO_5 		5
#define GPIO_PIN_NO_6 		6
#define GPIO_PIN_NO_7 		7
#define GPIO_PIN_NO_8 		8
#define GPIO_PIN_NO_9 		9
#define GPIO_PIN_NO_10 		10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12 		12
#define GPIO_PIN_NO_13 		13
#define GPIO_PIN_NO_14 		14
#define GPIO_PIN_NO_15 		15



/*
 * @GPIO_Pin_Modes
 * GPIO modes of the pin macros
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT 	4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6

/*
 * @GPIO_Output_Type
 * GPIO Output Register type macros
 */
#define GPIO_OP_TYPE_PP 	0
#define GPIO_OP_TYPE_OD 	1

/*
 * @GPIO_Pin_Speeds
 * GPIO Register Speed type macros
 */
#define GPIO_MODE_SPEED_LOW 		0
#define GPIO_MODE_SPEED_MEDIUM 		1
#define GPIO_MODE_SPEED_HIGH 		2
#define GPIO_MODE_SPEED_V_HIGH 		3

/*
 * @GPIO_Pin_PUPD
 * GPIO pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD 		0
#define GPIO_PIN_PU      	1
#define GPIO_PIN_PD 		2



/*
 * ------------------------------------------API supported by this driver-------------------------------------
 * --------------------------------For more information about the APIs check function definition--------------------------------
*/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read and Write Operations
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Interrupt and ISR Handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
