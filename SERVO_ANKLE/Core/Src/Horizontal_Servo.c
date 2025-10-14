/*
 * Vertical_Servo.c
 *
 */
#include <stdint.h>
#include "main.h"
#include "tim.h"
#include "Horizontal_Servo.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void turn_right(){
	set_Ankle_Hori_Servo(1, 60);
	set_Ankle_Hori_Servo(2, 60);
	set_Ankle_Hori_Servo(3, 110);
	set_Ankle_Hori_Servo(4, 110);
}

void turn_left(){
	set_Ankle_Hori_Servo(1, 120);
	set_Ankle_Hori_Servo(2, 120);
	set_Ankle_Hori_Servo(3, 70);
	set_Ankle_Hori_Servo(4, 70);
}

void crab_right(){
	set_Ankle_Hori_Servo(1, 70);
	set_Ankle_Hori_Servo(2, 70);
	set_Ankle_Hori_Servo(3, 70);
	set_Ankle_Hori_Servo(4, 70);
}

void crab_left(){
	set_Ankle_Hori_Servo(1, 120);
	set_Ankle_Hori_Servo(2, 120);
	set_Ankle_Hori_Servo(3, 120);
	set_Ankle_Hori_Servo(4, 120);
}

void align_Wheels(){
	set_Ankle_Hori_Servo(1, 90);
	set_Ankle_Hori_Servo(2, 90);
	set_Ankle_Hori_Servo(3, 90);
	set_Ankle_Hori_Servo(4, 90);
}

void side_move(){
	set_Ankle_Hori_Servo(1, 0);
	set_Ankle_Hori_Servo(2, 180);
	set_Ankle_Hori_Servo(3, 0);
	set_Ankle_Hori_Servo(4, 180);
}

void zero_turn(){
	set_Hip_Hori_Servo(1,45);
	set_Hip_Hori_Servo(2,45);
	set_Hip_Hori_Servo(3,45);
	set_Hip_Hori_Servo(4,45);
}

void set_Ankle_Vart_Servo(uint8_t wheelNumber, float angle_deg){
	//htim4
	uint32_t channel;
	uint32_t pulse;

	if(angle_deg < 0) angle_deg = 0;
	else if(angle_deg > 180) angle_deg = 180;

	switch(wheelNumber) {
	case 1:
		channel = TIM_CHANNEL_1;
		break;
	case 2:
		channel = TIM_CHANNEL_2;
		break;
	case 3:
		channel = TIM_CHANNEL_3;
		break;
	case 4:
		channel = TIM_CHANNEL_4;
		break;
	default:
		return; // 잘못된 휠 번호
	}

	pulse = 500 + (uint32_t)(10 * (angle_deg / 0.9));

	__HAL_TIM_SET_COMPARE(&htim2, channel, pulse);
}

//0 degree to pi/2 degree
void set_Ankle_Hori_Servo(uint8_t wheelNumber, float angle_deg){
	uint32_t channel;
	uint32_t pulse;

	if(angle_deg < 0) angle_deg = 0;
	else if(angle_deg > 180) angle_deg = 180;

	switch(wheelNumber) {
	case 1:
		channel = TIM_CHANNEL_1;
		break;
	case 2:
		channel = TIM_CHANNEL_2;
		break;
	case 3:
		channel = TIM_CHANNEL_3;
		break;
	case 4:
		channel = TIM_CHANNEL_4;
		break;
	default:
		return; // 잘못된 휠 번호
	}

	if(wheelNumber == 2) {
		pulse = 2500 - (uint32_t)(10 * (angle_deg / 0.9));
	}
	else {
		pulse = 500 + (uint32_t)(10 * (angle_deg / 0.9));
	}
	__HAL_TIM_SET_COMPARE(&htim4, channel, pulse);
}
void init_servo_set()
{
	set_Ankle_Hori_Servo(1, 90);
	set_Ankle_Hori_Servo(2, 90);
	set_Ankle_Hori_Servo(3, 90);
	set_Ankle_Hori_Servo(4, 90);
	set_Hip_Hori_Servo(1, 0);
	set_Hip_Hori_Servo(2, 0);
	set_Hip_Hori_Servo(3, 0);
	set_Hip_Hori_Servo(4, 0);
}
void set_Hip_Hori_Servo(uint8_t legNumber, float angle_deg){
	//htim3
	uint32_t channel;
	uint32_t pulse;

	if(angle_deg < 0) angle_deg = 0;
	else if(angle_deg > 180) angle_deg = 180;

	switch(legNumber) {
	case 1:
		channel = TIM_CHANNEL_1;
		break;
	case 2:
		channel = TIM_CHANNEL_2;
		break;
	case 3:
		channel = TIM_CHANNEL_3;
		break;
	case 4:
		channel = TIM_CHANNEL_4;
		break;
	default:
		return; // 잘못된 휠 번호
	}

	if(legNumber == 3 || legNumber == 1) {
		pulse = 2500 - (uint32_t)(10 * (angle_deg / 0.9));
	}
	else if(legNumber == 4 || legNumber == 2) {
		pulse = 500 + (uint32_t)(10 * (angle_deg / 0.9));
	}

	__HAL_TIM_SET_COMPARE(&htim3, channel, pulse);
}

void servo_init(){

	set_Hip_Hori_Servo(1, 0);
	set_Hip_Hori_Servo(2, 0);
	set_Hip_Hori_Servo(3, 0);
	set_Hip_Hori_Servo(4, 0);

}
