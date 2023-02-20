/*
 * Motor.c
 *
 *  Created on: Feb 19, 2022
 *      Author: Nguyen_Ba_Hoang_20181486
 */

#include "Motor.h"
#include <stdio.h>
extern Motor_Name 	Motor;			//This name must = variable name in main.c
static int 			idx 		= 0;
static uint32_t 	pre_counter = 0;
static uint32_t 	old_counter = 0;
static uint16_t 	speed 		= 0;

//variable for PID
int32_t preError = 0, error = 0, sumError = 0;
float rateError = 0;
/*************************************************************************************************/

/**
  * @brief Init motor
  * @param	Timer,channel for PWM_PIN;
  * 		duty: Duty cycle of PWM
  * 		frequency: frequency of PWM
  * @retval None
  */

void Motor_Init_PWM(Motor_Name* Motor, TIM_HandleTypeDef* Timer, uint32_t Channel, 	uint32_t frequency, uint8_t duty)
{
	Motor->Timer_PWM = Timer;
	Motor->Channel_PWM = Channel;
	//Start Timer
	HAL_TIM_PWM_Start(Timer, Channel);
	Motor_Set_Duty(Motor, duty);
	Motor_Set_Frequency(Motor, frequency);
}

/**
  * @brief Init Encoder for measure motor speed
  * @param	Timer,channel for Encoder
  *
  * @retval None
  */
void Motor_Init_Encoder(Motor_Name* Motor, TIM_HandleTypeDef* Timer, uint32_t Channel)
{
	Motor->Timer_Encoder = Timer;
	Motor->Channel_Encoder = Channel;
	HAL_TIM_Encoder_Start(Timer, Channel);
}
/**
  * @brief Set duty cycle for PWM
  * @param Duty(%) 0 -> 100. % High voltage
  * @retval None
  */
void Motor_Set_Duty(Motor_Name* Motor,uint8_t duty)
{
	if(duty > DUTY_MAX)
		duty = DUTY_MAX;
	if(duty <= DUTY_MIN)
		duty = 0;

	Motor->duty = duty;
	uint16_t compare = (Motor->Timer_PWM->Init.Period + 1) * duty / 100;
	__HAL_TIM_SET_COMPARE(Motor->Timer_PWM, Motor->Channel_PWM, compare);
}

/**
  * @brief Set frequency for PWM
  * @param frequency(Hz); should  2 < f < 10000
  * @retval None
  */
void Motor_Set_Frequency(Motor_Name* Motor, uint32_t frequency)
{
	// frequency = f_timer / [(Period + 1) * (Psc + 1)]
	// -> Psc = f_timer / ((period + 1)*frequency) - 1;
	Motor->frequency = frequency;
	uint32_t period = Motor->Timer_PWM->Init.Period;
	uint32_t f_timer = HAL_RCC_GetHCLKFreq();
	TIM2->PSC = f_timer / ((period + 1) * frequency) - 1;
}

/**
  * @brief read speed
  * @param
  * @retval None
  */
uint16_t Motor_Get_Speed(Motor_Name* Motor)
{
	Motor->speed = speed;
	if(speed > SPEED_MAX)
		speed = SPEED_MAX;
	if(speed <= SPEED_MIN)
		speed = SPEED_MIN;
	return speed;
}

/**
  * @brief update speed for motor through encoder
  * @param
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SYSTICK_Callback could be implemented in the user file
   */
	if(idx == SAMPLING_TIME)	//update speed after every sampling time
	{
		pre_counter = __HAL_TIM_GET_COUNTER(Motor.Timer_Encoder);
		speed = caculate_speed(pre_counter, old_counter);
		old_counter = pre_counter;
		idx = 0;
	}
	idx++;
}

uint16_t caculate_speed(uint32_t pre_counter, uint32_t old_counter)
		{
			if(pre_counter < old_counter)
			{
				pre_counter = pre_counter + Motor.Timer_Encoder->Init.Period;
			}

			return (pre_counter - old_counter) * (1000 / SAMPLING_TIME) * 60 / 334; 	//number of pulses in one  minutes / 334 = rpm
		}

/*
uint8_t PID_Gen_Duty(uint16_t actualSpeed, uint16_t desireSpeed)
{
	error = desireSpeed - actualSpeed;
	sumError += error*T;
	rateError =(float)(error - preError)/T;
	uint8_t duty = (Kp*error + Ki*sumError + Kd*rateError)/SPEED_MAX*100;

	preError = error;
	return duty;
}

uint16_t Temp_to_Speed(uint8_t temp)
{
	return temp*SPEED_MAX/100; 		//100 độ C tương ứng với tốc độ max = 4600rpm
}
*/
uint8_t Temp_to_Duty(uint8_t temp)
{
	return temp;
}
uint8_t Manual_to_Duty(ADC_HandleTypeDef* hadc)
{
	  uint16_t manual = 0;
	  HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	  manual = HAL_ADC_GetValue(hadc);
	  //manual: 4000 -> 0 convert to
	  //duty:     10 -> 100
	  return (-0.0225*manual + 100);
}

