/*
 * Motor.h
 *
 *  Created on: Feb 19, 2022
 *      Author: Nguyen_Ba_Hoang_20181486
 *   Huong dan su dung:
		- Use HAL library
		- Config Timer with channel PWM
		- Config Timer with Encoder mode
		- add HAL_SYSTICK_IRQHandler() vào trong SysTick_Handler() (trong file stm32f1xx_it.c) để hàm callback được gọi
		- Khoi tao bien Motor : Motor_Name Motor; //Variable Name must be "Motor" because i declared extern variable in Motor.c
												  //If you want change it, you also have to change extern varialble Name in Motor.c
		- Khoi tao Motor
			Motor_Init(&Motor, &htim2, TIM_CHANNEL_4, 2000, 50);

 *
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f1xx_hal.h"
#include "main.h"

#define MODE_MOTOR_STOP				0
#define MODE_MOTOR_MUNUAL			1
#define MODE_MOTOR_TEMP				2
#define MODE_MOTOR_NONE				3

#define SPEED_MAX		4600	//rpm
#define SPEED_MIN		0
#define DUTY_MAX		100
#define DUTY_MIN		10
#define SAMPLING_TIME	200		//ms

//variable for PID
#define T		((float)SAMPLING_TIME/1000)				//s
#define Kp		1
#define Ki		5
#define Kd		0.1

typedef struct
{
	TIM_HandleTypeDef* 	Timer_PWM;
	uint32_t 			Channel_PWM;
	TIM_HandleTypeDef* 	Timer_Encoder;
	uint32_t 			Channel_Encoder;
	uint32_t 			frequency;
	uint8_t 			duty;
	uint16_t 			speed;
	uint8_t				mode;			//@MODE_MOTOR
}Motor_Name;


void Motor_Init_PWM(Motor_Name* Motor, TIM_HandleTypeDef* Timer_PWM, uint32_t Channel_PWM, 	uint32_t frequency, uint8_t duty);
void Motor_Init_Encoder(Motor_Name* Motor, TIM_HandleTypeDef* Timer, uint32_t Channel);
void Motor_Set_Duty(Motor_Name* Motor,uint8_t duty);
void Motor_Set_Frequency(Motor_Name* Motor, uint32_t frequency);
uint16_t Motor_Get_Speed(Motor_Name* Motor);
uint16_t caculate_speed(uint32_t pre_counter, uint32_t old_counter);
uint8_t PID_Gen_Duty(uint16_t actualSpeed, uint16_t desireSpeed);
uint16_t Temp_to_Speed(uint8_t temp);
uint8_t Temp_to_Duty(uint8_t temp);
uint8_t Manual_to_Duty(ADC_HandleTypeDef* hadc);



#endif /* INC_MOTOR_H_ */
