/*
 * stepper_motion.c
 *
 *  Created on: Sep 13, 2025
 *      Author: KESHAVA HEGDE
 */


#include "stepper_config.h"
#include "stepper_motion.h"


volatile Stepper_t stepper;

void stepper_set_speed(uint32_t step_pulse, TIM_TypeDef *TIM)
{
	TIM->ARR = (SystemCoreClock/step_pulse)-1;
}

void stepper_move(int32_t dist,uint32_t speed_per_min, Stepper_t *stepper)
{
	//set the direction
	if(dist > 0)
		GPIO_set(DIR_PORT, DIR_PIN);
	else
	{
		dist = - dist;
		GPIO_reset(DIR_PORT, DIR_PIN);
	}
	//set the steps it needs to generate and freq

	float freq = (speed_per_min * STEPS_PER_MM)/60.0;

	//calc total steps

	int total_steps = STEPS_PER_MM * dist;

	stepper_set_speed((uint32_t)freq, TIM2);//setting it in ARR reg of tim2

	stepper->remaining_steps = total_steps;
	stepper->stepper_freq = freq;

	TIM2->CR1 |= TIM_CR1_CEN;

	    // --- 6. Blocking wait until motion completes ---
	    while (stepper->remaining_steps > 0)
	    {
	        // optionally, you can add __WFI() to save power:
	         __WFI(); // Wait for interrupt
	    }
	    // --- 7. Stop TIM2 after motion ---
	    TIM2->CR1 &= ~TIM_CR1_CEN;

}




