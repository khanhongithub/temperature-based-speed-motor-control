/*
 * delay_timer.h
 *
 *  Created on: Feb 19, 2022
 *      Author: acer
 */

#ifndef INC_DELAY_TIMER_H_
#define INC_DELAY_TIMER_H_

#include "stm32f1xx_hal.h"

void	DELAY_TIM_Init(TIM_HandleTypeDef *htim);
void 	DELAY_TIM_Us(TIM_HandleTypeDef *htim, uint16_t us);
void	DELAY_TIM_Ms(TIM_HandleTypeDef *htim, uint16_t ms);


#endif /* INC_DELAY_TIMER_H_ */
