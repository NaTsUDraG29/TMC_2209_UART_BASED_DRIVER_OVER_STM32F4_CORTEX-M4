/*
 * stepper_config.h
 *
 *  Created on: Sep 13, 2025
 *      Author: KESHAVA HEGDE
 */

#ifndef INC_STEPPER_CONFIG_H_
#define INC_STEPPER_CONFIG_H_

#define STEPS_PER_REV 		200 * Microsteps
#define DIR_PORT			GPIOF
#define DIR_PIN				GPIO12
#define STEP_PORT			GPIOF
#define STEP_PIN			GPIO13
#define PITCH_LEAD			8 //mm
#define STEPS_PER_MM		STEPS_PER_REV/PITCH_LEAD


#define GPIO8				8
#define UART_GPIO			4  //software TX and RX
#define GPIO12				12 //dir
#define GPIO13				13 //step

#define GPIO_OUPUT_M		1
#define GPIO_INPUT_M		0

#define GPIOA_EN			(1 << 0U)
#define GPIOC_EN			(1 << 2U)
#define GPIOF_EN			(1 << 5U)
#define USART2_EN			(1 << 17U)

#define TIM2_EN				(1 << 0U)

#define AF_UART_RESET		(15 << 12)
#define AF_UART_SET			(7 << 12)


#define MODER(N)			N*2


#define UART_BIT_RATE		19200
#define PClk_Rate			HAL_RCC_GetHCLKFreq();
#define DELAY_PER_BIT		1/UART_BIT_RATE





#define SYNC_BYTE       0x05
#define SLAVE_ADDR	    0x00
#define SOFTWARE_RX_TO	1000
#define Rsense  		0.11f    // check your board
#define Vref   		 	0.325f

//register values
#define IHOLD_IRUN		0x10
#define CHOP_CONF		0x6C

//Define the Configurations here
#define IHOLD		0.3
#define IRUN		1
#define Microsteps	16


#endif /* INC_STEPPER_CONFIG_H_ */
