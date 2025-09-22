/*
 * stepper_motion.h
 *
 *  Created on: Sep 13, 2025
 *  Author: KESHAVA HEGDE
 *
 *
 *
 *
 *  Info:
 *  Speed is given in mm/min

Lead screw pitch = e.g. 8 mm/rev

Steps per rev = motor steps × microstepping (e.g. 200 × 16 = 3200 steps/rev)

steps/mm = (steps/rev)/ (pitch (mm/rev))
	​
step frequency (Hz) = (speed (mm/min) × steps/mm )/ 60


total steps = distance (mm) × steps/mm;​
 *
 */


#ifndef INC_STEPPER_MOTION_H_
#define INC_STEPPER_MOTION_H_

#include "stm32f4xx.h"

typedef struct
{
	uint32_t remaining_steps;
	uint32_t stepper_freq;
}Stepper_t;

extern volatile Stepper_t stepper;

extern void GPIO_set(GPIO_TypeDef *GPIO, uint16_t pin);
extern void GPIO_reset(GPIO_TypeDef *GPIO, uint16_t pin);

void stepper_set_speed(uint32_t step_pulse, TIM_TypeDef *TIM);
void stepper_move(int32_t dist,uint32_t speed_per_min, Stepper_t *stepper);



#endif /* INC_STEPPER_MOTION_H_ */
