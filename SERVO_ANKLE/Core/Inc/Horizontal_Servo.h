/*
 * Vertical_Servo.h
 *
 *  Created on: Apr 14, 2025
 *      Author: jhfjf
 */

#ifndef INC_HORIZONTAL_SERVO_H_
#define INC_HORIZONTAL_SERVO_H_

#include "main.h"  // htim3, htim4 등의 정의를 위해 필요

// 서보 초기화 함수
void init_servo_set(void);

// 서보 제어 함수
void set_Ankle_Hori_Servo(uint8_t wheelNumber, float angle_deg);
void set_Hip_Hori_Servo(uint8_t legNumber, float angle_deg);
void set_Ankle_Vart_Servo(uint8_t legNumber, float angle_deg);

// 움직임 제어 함수
void turn_right(void);
void turn_left(void);
void crab_right(void);
void crab_left(void);
void align_Wheels(void);
void side_move(void);
void zero_turn(void);
void servo_init(void);
#endif /* INC_HORIZONTAL_SERVO_H_ */
