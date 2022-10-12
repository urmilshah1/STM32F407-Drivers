/*
 * stm32f407xx.h
 *
 *  Created on: Sep 20, 2022
 *      Author: UrmilShah
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>

/* base address of Flash and SRAM memories */

#define FLASH_BASEADDR        	    0x08000000U  /*This is the base address of flash memory (page 71/1751)*/
#define SRAM1_BASEADDR				0x20000000U	 /*This is the base address of SRAM1 memory (page 71/1751)*/
#define SRAM2_BASEADDR				0x2001C000U	/*This is the base address of SRAM2 memory(page 71/1751)  */
#define ROM_BASEADDR				0x1FFF0000U  /*This is the base address of ROM (page 75/1751) */
#define SRAM 						SRAM1_BASEADDR /*This is the base address of SRAM1 memory (page 71/1751)*/

#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR         PERIPH_BASEADDR                /*This is the base address of APB1 (page 67/1751) */
#define APB2PERIPH_BASEADDR			0x40010000U  /*This is the base address of APB2 (page 66/1751) */
#define AHB1PERIPH_BASEADDR			0x40020000U  /*This is the base address of AHB1 (page 65/1751) */
#define AHB2PERIPH_BASEADDR			0x50000000U  /*This is the base address of AHB2 (page 65/1751) */


/*
 * Other Misc Macros
 */

#define __vo volatile


/*
* Processor specific NVIC ISERx register addresses
*
*/

#define NVIC_ISER0 		((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10C)

/*
* Processor specific NVIC ICERx register addresses
*
*/

#define NVIC_ICER0 		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1		((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2		((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3		((__vo uint32_t*)0xE000E18C)

/*
* Processor specific NVIC Priority register addresses
*
*/

#define NVIC_PR_BASEADDR  ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED  		4

/*
* Base Addresses for GPIO drivers
*/

#define GPIOA_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0000) /*This is the base address of GPIOA (page 65/1751) NO OFFSET*/
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400) /*This is the base address of GPIOB (page 65/1751)*/
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800) /*This is the base address of GPIOC (page 65/1751)*/
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00) /*This is the base address of GPIOD (page 65/1751)*/
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000) /*This is the base address of GPIOE (page 65/1751)*/
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400) /*This is the base address of GPIOF (page 65/1751)*/
#define GPIOG_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1800) /*This is the base address of GPIOG (page 65/1751)*/
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00) /*This is the base address of GPIOH (page 65/1751)*/
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000) /*This is the base address of GPIOI (page 65/1751)*/

#define RCC_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x3800) /*This is the base address of RCC (page 65/1751)*/

/*
* Base Addresses for Peripherals hanging on the APB1 bus
*/

#define I2C1_BASEADDR 				(APB1PERIPH_BASEADDR + 0x5400) /*This is the base address of I2C1 (page 67/1751) */
#define I2C2_BASEADDR 				(APB1PERIPH_BASEADDR + 0x5800) /*This is the base address of I2C2 (page 67/1751) */
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00) /*This is the base address of I2C3 (page 67/1751) */

#define CAN1_BASEADDR				(APB1PERIPH_BASEADDR + 0x6400) /*This is the base address of CAN1 (page 67/1751) */
#define CAN2_BASEADDR				(APB1PERIPH_BASEADDR + 0x6800) /*This is the base address of CAN2 (page 67/1751) */

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400) /*This is the base address of USART2 (page 67/1751) */
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800) /*This is the base address of USART3 (page 67/1751) */

#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00) /*This is the base address of UART4 (page 67/1751) */
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000) /*This is the base address of UART5 (page 67/1751) */

#define SPI2_BASEADDR 				(APB1PERIPH_BASEADDR + 0x3800) /*This is the base address of SPI2 (page 67/1751) */
#define SPI3_BASEADDR 				(APB1PERIPH_BASEADDR + 0x3C00) /*This is the base address of SPI3 (page 67/1751) */


/*
* Base Addresses for Peripherals hanging on the APB2 bus
*/

#define EXTI_BASEADDR  				(APB2PERIPH_BASEADDR + 0x3C00) /*This is the base address of EXTI (page 66/1751) */

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000) /*This is the base address of SPI1 (page 66/1751) */
#define SPI4_BASEADDR 				(APB2PERIPH_BASEADDR + 0x3400) /*This is the base address of SPI1 (page 66/1751) */

#define SYSCFG_BASEADDR     		(APB2PERIPH_BASEADDR + 0x3800) /*This is the base address of SYCFG (page 66/1751) */

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000) /*This is the base address of USART1 (page 66/1751) */
#define USTART6_BASEADDR  			(APB2PERIPH_BASEADDR + 0x1400) /*This is the base address of USART6 (page 66/1751) */

/*
 * ***********************************Peripheral register definition structure***********************************
 * e.g. Number of registers of GPIO peripheral interface of Stm32f4xx family of MCUs
 *
 */

typedef struct {
	__vo uint32_t MODER;  /*GPIO port mode register Address offset 0x00 (page 288/1751)*/
	__vo uint32_t OTYPER; /*GPIO port output type register Address offset 0x04 (page 288/1751)*/
	__vo uint32_t OSPEEDR; /* GPIO port output speed register (Address offset 0x08 (page 288/1751)*/
	__vo uint32_t PUPDR; /* GPIO port pull-up/pull-down register Address offset 0x0C (page 288/1751)*/
	__vo uint32_t IDR; /* GPIO port input data register Address offset 0x10 (page 288/1751)*/
	__vo uint32_t ODR; /*GPIO port output data register Address offset 0x14 (page 288/1751)*/
	__vo uint32_t BSRR; /* GPIO port bit set/reset register Address offset 0x18 (page 288/1751)*/
	__vo uint32_t LCKR; /*GPIO port configuration lock register Address offset 0x1C (page 288/1751)*/
	__vo uint32_t AFR[2]; /*GPIO alternate function low --> AFR[0] an high --> AFR[1] register ( Address offset 0x20 - 0x24 (page 288/1751)*/
}GPIO_RegDef_t;

/*
 * ***********************************Peripheral register definition structure***********************************
 * e.g. Number of RCC registers of Stm32f4xx family of MCUs
 *
 */

typedef struct {
	__vo uint32_t CR;         /*RCC clock control register (RCC_CR) Address offset 0x00 (page 210/1751)*/
	__vo uint32_t PLLCFGR;    /*RCC PLL configuration register (RCC_PLLCFGR) Address offset 0x04 (page 210/1751)*/
	__vo uint32_t CFGR;       /*RCC clock configuration register (RCC_CFGR) Address offset 0x08 (page 210/1751)*/
	__vo uint32_t CIR;        /*RCC clock interrupt register (RCC_CIR) Address offset 0x0C (page 210/1751)*/
	__vo uint32_t AHB1RSTR;   /*RCC AHB1 peripheral reset register (RCC_AHB1RSTR) Address offset 0x10 (page 210/1751)*/
	__vo uint32_t AHB2RSTR;   /*RCC AHB2 peripheral reset register (RCC_AHB2RSTR) Address offset 0x14 (page 210/1751)*/
	__vo uint32_t AHB3RSTR;   /*RCC AHB3 peripheral reset register (RCC_AHB3RSTR) Address offset 0x18 (page 210/1751)*/
	uint32_t RESERVED0;      /*RCC Reserved register Address offset 0x1C (page 210/1751)*/
	__vo uint32_t APB1RSTR;   /*RCC APB1 peripheral reset register (RCC_APB1RSTR) Address offset 0x20 (page 210/1751)*/
	__vo uint32_t APB2RSTR;   /*RCC APB2 peripheral reset register (RCC_APB2RSTR) Address offset 0x24 (page 210/1751)*/
	uint32_t RESERVED1[2];    /*RCC Reserved register Address offset 0x28 (page 210/1751)*/
	__vo uint32_t AHB1ENR;    /*RCC AHB1 peripheral clock register (RCC_AHB1ENR) Address offset 0x30 (page 210/1751)*/
	__vo uint32_t AHB2ENR;    /*RCC AHB2 peripheral clock enable register (RCC_AHB2ENR) Address offset 0x34 (page 210/1751)*/
	__vo uint32_t AHB3ENR;    /*RCC AHB3 peripheral clock enable register (RCC_AHB3ENR) Address offset 0x38 (page 210/1751)*/
	uint32_t RESERVED2;       /*RCC Reserved register Address offset 0x3C (page 210/1751)*/
	__vo uint32_t APB1ENR;    /*RCC APB1 peripheral clock enable register (RCC_APB1ENR) Address offset 0x40 (page 211/1751)*/
	__vo uint32_t APB2ENR;    /*RCC APB2 peripheral clock enable register (RCC_APB2ENR) Address offset 0x44 (page 211/1751)*/
	uint32_t RESERVED3[2];    /*RCC Reserved register Address offset 0x48- 0x4C (page 210/1751)*/
	__vo uint32_t AHB1LPENR;  /*RCC AHB1 peripheral clock enable in low power mode register (RCC_AHB1LPENR) Address offset 0x50 (page 211/1751)*/
	__vo uint32_t AHB2LPENR;   /*RCC AHB2 peripheral clock enable in low power mode register (RCC_AHB2LPENR) Address offset 0x54 (page 21/1751)*/
	__vo uint32_t AHB3LPENR;  /*RCC AHB3 peripheral clock enable in low power mode register (RCC_AHB3LPENR) Address offset 0x58 (page 211/1751)*/
	uint32_t RESERVED4;       /*RCC Reserved register Address offset 0x5C (page 211/1751)*/
	__vo uint32_t APB1LPENR;  /*RCC APB1 peripheral clock enable in low power mode register (RCC_APB1LPENR) Address offset 0x60 (page 211/1751)*/
	__vo uint32_t APB2LPENR;  /*RCC APB2 peripheral clock enabled in low power mode register (RCC_APB2LPENR) Address offset 0x64 (page 211/1751)*/
	uint32_t RESERVED5[2];    /*RCC Reserved register Address offset 0x68 -0x8C0 (page 211/1751)*/
	__vo uint32_t BDCR;      /*RCC Backup domain control register (RCC_BDCR) Address offset 0x70 (page 211/1751)*/
	__vo uint32_t CSR;       /*RCC clock control & status register (RCC_CSR) Address offset 0x74 (page 211/1751)*/
	uint32_t RESERVED6[2];    /*RCC Reserved register Address offset 0x78 -0x7C (page 211/1751)*/
	__vo uint32_t SSCGR;     /*RCC spread spectrum clock generation register (RCC_SSCGR) Address offset 0x80 (page 210/1751)*/
	__vo uint32_t PLLI2CFGR;   /*RCC PLLI2S configuration register (RCC_PLLI2SCFGR) Address offset 0x84 (page 210/1751)*/
	__vo uint32_t PLLSAICFGR; /*RCC PLL configuration register (RCC_PLLSAICFGR) Address offset 0x88 (page 210/1751)*/
	__vo uint32_t DCKCFGR;    /*RCC Dedicated Clock Configuration Register (RCC_DCKCFGR) Address offset 0x8C (page 210/1751)*/
	__vo uint32_t CKGATENR;   /*!< ,     										Address offset: 0x90 */
	__vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

/*
 * ***********************************Peripheral register definition structure***********************************
 * e.g. Number of EXTI registers of Stm32f4xx family of MCUs
 *
 */
typedef struct {
	__vo uint32_t IMR;  /*EXTI pot interrupt Masking register Address offset 0x00 (page 384/1751)*/
	__vo uint32_t EMR; /*EXTI port Event mask register Address offset 0x04 (page 384/1751)*/
	__vo uint32_t RTSR; /* EXTI port Rising Trigger selection register (Address offset 0x08 (page 385/1751)*/
	__vo uint32_t FTSR; /* EXTI port Falling Trigger selection register Address offset 0x0C (page 385/1751)*/
	__vo uint32_t SWIER; /* EXTI port software interrupt event register Address offset 0x10 (page 386/1751)*/
	__vo uint32_t PR; /* EXTI port pending register Address offset 0x14 (page 386/1751) */

}EXTI_RegDef_t;

/*
 * ***********************************Peripheral register definition structure***********************************
 * e.g. Number of SYSCFG registers of Stm32f4xx family of MCUs
 *
 */
typedef struct {
	__vo uint32_t MEMRMP; /*SSYSCFG memory remap register address offset 0x00 (page 289/1751)*/
	__vo uint32_t PMC;    /*SYSCFG peripheral mode configuration register address offset 0x04 (page 290/1751)*/
	__vo uint32_t EXTICR[4]; /*SYSCFG external interrupt configuration register 1 address offset 0x08-0x14 (page 291/1751)*/
	uint32_t RESERVED1[2]; /* SYSCFG Reserved register address offset 0x18 - 0x1C (page 294/1751) */
	__vo uint32_t CMPCR; /*Compensation cell control register address offset 0x20 (page 293/1751) */
	uint32_t RESERVED2[2]; /* SYSCFG reserved register address offset 0x24 - 0x28  (page 294/1751)*/
}SYSCFG_RegDef_t;


/*
 * ***********************************Peripheral register definition structure for SPI***********************************
 *
 */
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


/*
 * Peripheral definitions (peripheral base address type casted to xxx_RegDef_t
 */

#define GPIOA     ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB     ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC     ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD     ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE     ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF     ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG     ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH     ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI     ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC		  ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI  	  ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG    ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1 	  ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2	  ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 	  ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4 	  ((SPI_RegDef_t *)SPI4_BASEADDR)


/*
 * ---------------------------------------Enable Peripheral Clock---------------------------------------
 */

/*
 * ****************************Clock Enable macros for GPIOx Peripherals*****************************
 */

#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()   (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()   (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()   (RCC->AHB1ENR |= (1 << 8))
/*
 * ****************************Clock Enable macros for I2Cx Peripherals*****************************
 */
#define I2C1_PCLK_EN()    (RCC->APB1ENR |= ( 1<< 21))
#define I2C2_PCLK_EN()    (RCC->APB1ENR |= ( 1<< 22))
#define I2C3_PCLK_EN()    (RCC->APB1ENR |= ( 1<< 23))

/*
 * ****************************Clock Enable macros for SPIx Peripherals*****************************
 */

#define SPI1_PCLK_EN()      (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()      (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()      (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()       (RCC->APB2ENR |= (1 << 13))

/*
 * ****************************Clock Enable macros for USARTx Peripherals*****************************
 */

#define USART1_PCLK_EN()     (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()     (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()     (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()      (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()      (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()     (RCC->APB1ENR |= (1 << 5))


/*
 * ****************************Clock Enable macros for SYSCGFGx Peripherals*****************************
 */
#define SYSCFG_PCLK_EN()     (RCC->APB2ENR |= (1 << 14))



 /* -----------------------------------------------------------------------------------------------------------*/


/*
 * ---------------------------------------Disable Peripheral Clock---------------------------------------
 */
/*
 * ****************************Clock Disable macros for GPIOx Peripherals*****************************
 */

#define GPIOA_PCLK_DI()   (RCC->AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI()   (RCC->AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI()   (RCC->AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI()   (RCC->AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI()   (RCC->AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI()   (RCC->AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI()   (RCC->AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI()   (RCC->AHB1ENR &= ~( 1 << 7 ))
#define GPIOI_PCLK_DI()   (RCC->AHB1ENR &= ~( 1 << 8 ))

/*
 * ****************************Clock Disable macros for I2Cx Peripherals*****************************
 */
#define I2C1_PCLK_DI()    (RCC->APB1ENR &= ~( 1<< 21))
#define I2C2_PCLK_DI()    (RCC->APB1ENR &= ~( 1<< 22))
#define I2C3_PCLK_DI()    (RCC->APB1ENR &= ~( 1<< 23))

/*
 * ****************************Clock Disable macros for SPIx Peripherals*****************************
 */

#define SPI1_PCLK_DI()      (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 13))


/*
 * ****************************Clock Disable macros for USARTx Peripherals*****************************
 */

#define USART1_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 5))

/*
 * --------------------------------Macros to Reset the GPIO peripherals-----------------------------------------------
 */

#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)




/*
* ------------------------------Macros to Reset the SPI Peripherals------------------------------
*/
#define SPI1_REG_RESET() 			    do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET() 			    do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET() 			    do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)

/*
 * ****************************Clock Disable macros for SYSCGFGx Peripherals*****************************
 */
#define SYSCFG_PCLK_DI()     (RCC->APB2ENR |= (1 << 14))


/*
 * ****************************IRQ(Interrupt Request) Numbers of STM32F407x MCU*****************************
 *
 */
#define IRQ_NO_EXTI0     	6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40


/*
 * Generic Macros
 */

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET


/**************************************************************************
 * Bit position definitions for SPI peripherals
 **************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIODE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit position definitions SPI_CR2
 */

#define SPI_CR2_RXMAEN 		0
#define SPI_CR2_TXMAEN 		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7


/*
 * Bit position definitions SPI_SR
 */

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRC_ERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"


#endif /* INC_STM32F407XX_H_ */
