/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dxl_2_0.h"
#include "Horizontal_Servo.h"
#include <string.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern volatile uint8_t g_auto_mode;
static inline void reapply_current_goal_ticks();
#ifndef DXL_2_Present_Current
#define DXL_2_Present_Current 126
#endif



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TORQUE_ON 1
#define TORQUE_OFF 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DMA_BUF_SIZE 256         // DMA 버퍼 크기

/* UART 핸들 */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
void process_input(void);
int get_uart_index(UART_HandleTypeDef *huart);
void uart_rx_dma1_handler(void);

void UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/* 수신 버퍼 및 플래그 */
uint8_t uart1_tx_dma_buf[DMA_BUF_SIZE];
uint8_t uart2_tx_dma_buf[DMA_BUF_SIZE];
uint8_t uart3_tx_dma_buf[DMA_BUF_SIZE];
uint8_t uart4_tx_dma_buf[DMA_BUF_SIZE];
uint8_t uart5_tx_dma_buf[DMA_BUF_SIZE];
uint8_t pc_rx_buf[DMA_BUF_SIZE];
uint8_t uart2_rx_buf[DMA_BUF_SIZE];
uint8_t uart3_rx_buf[DMA_BUF_SIZE];
uint8_t uart4_rx_buf[DMA_BUF_SIZE];
uint8_t uart5_rx_buf[DMA_BUF_SIZE];
volatile uint8_t input_char;
volatile uint8_t new_data_received[5];
bool torque_status = false;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t angle_speed_Ankle[4] = {30, 30, 30, 30};
uint32_t angle_speed_Knee[4] = {40, 40, 40, 40};
uint32_t angle_speed_Hip[4] = {20, 20, 20, 20};
uint16_t torque_limit[4] = {1023, 1023, 1023, 1023};
uint16_t ss1[4];
bool leg1 = 0;
bool leg3 = 0;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

data_set_all data_set;
bool uart_tx_busy[5] = {false,false,false,false,false};

uint32_t tickstart;
uint32_t wait_time;

static inline void apply_impedance_profile(SendLegType leg, int stance);

enum { IDX_FL=0, IDX_FR=1, IDX_RR=2, IDX_RL=3 };  // 배열 인덱스
enum { HIP=0, KNEE=1, ANKLE=2 };                  // 관절 인덱스

static inline void settle_hold(uint32_t ms);
void add_and_apply_increment_smooth(const double dRad[4][3], uint32_t T_ms, uint32_t dt_ms);
static inline void reapply_current_goal_ticks(void);

static inline int leg_to_idx(SendLegType leg){
    switch (leg){
        case FL: return IDX_FL;
        case FR: return IDX_FR;
        case RR: return IDX_RR;
        case RL: return IDX_RL;
        default: return IDX_FL;
    }
}

static bool leg_contact_by_current(SendLegType leg, uint16_t thr_lsb){
    uint8_t err=0; uint16_t raw=0;
    uint8_t knee_id = legs[leg].knee;
    if (!dxl_read_2byte(knee_id, DXL_2_Present_Current, &raw, &err)) return false;
    int16_t s = (int16_t)raw;                 // signed
    uint16_t mag = (s<0) ? (uint16_t)(-s) : (uint16_t)s;
    return (mag >= thr_lsb);
}

static void wait_contact_or_timeout(SendLegType leg, uint16_t thr_lsb, uint32_t timeout_ms){
    uint32_t t0 = HAL_GetTick();
    for(;;){
        if (leg_contact_by_current(leg, thr_lsb)) break;
        if (HAL_GetTick()-t0 > timeout_ms) break;
        HAL_Delay(5);
    }
}

static inline void set_profile_for_phase(SendLegType leg, int swing){
    uint32_t vel[3], acc[3];
    if (swing){    // 부드럽게
        vel[0]=750;  vel[1]=800;  vel[2]=800;
        acc[0]=900;  acc[1]=900;  acc[2]=900;
    }else{         // 지지는 힘 있게
        vel[0]=1400; vel[1]=1500; vel[2]=1200;
        acc[0]=1800; acc[1]=1800; acc[2]=1500;
    }
    dxl_set_leg_profile(leg, vel, acc);
}

typedef struct {
    double   pose[4][3];
    uint16_t T_ms;
    uint16_t dt_ms;
    uint8_t  stance_mask;
    uint16_t hold_ms;
    uint8_t  flags;
} GaitPhase;

// 플래그
#define PH_ALIGN_WHEELS   (1u<<0)
#define PH_REAPPLY_GOAL   (1u<<1)
#define PH_WAIT_CONTACT_FL (1u<<2)
#define PH_WAIT_CONTACT_FR (1u<<3)
#define PH_WAIT_CONTACT_RR (1u<<4)
#define PH_WAIT_CONTACT_RL (1u<<5)

static inline void set_impedance_and_profile_all(int fl_stance,int fr_stance,int rr_stance,int rl_stance);

// stance_mask에서 비트 추출
static inline int bit(uint8_t m, int b){ return (m>>b)&1; }

static void run_phases(const GaitPhase* ph, int n)
{
    for(int k=0;k<n;k++){
        const GaitPhase* P = &ph[k];

        // 1) 임피던스/프로파일 세팅
        set_impedance_and_profile_all(
            bit(P->stance_mask,0), bit(P->stance_mask,1),
            bit(P->stance_mask,2), bit(P->stance_mask,3));

        // 2) 목표 자세 이동 (프레임-락 S-curve)
        apply_target_smooth_locked(P->pose, P->T_ms, P->dt_ms);

        // 3) 목표 재주입/휠정렬
        if (P->flags & PH_REAPPLY_GOAL) reapply_current_goal_ticks();
        if (P->flags & PH_ALIGN_WHEELS) align_Wheels();

        // 4) 착지 대기
        const uint16_t THR = 180;
        const uint32_t TMO = 250;     // ms
        if (P->flags & PH_WAIT_CONTACT_FL) wait_contact_or_timeout(FL, THR, TMO);
        if (P->flags & PH_WAIT_CONTACT_FR) wait_contact_or_timeout(FR, THR, TMO);
        if (P->flags & PH_WAIT_CONTACT_RR) wait_contact_or_timeout(RR, THR, TMO);
        if (P->flags & PH_WAIT_CONTACT_RL) wait_contact_or_timeout(RL, THR, TMO);
        if (P->hold_ms) HAL_Delay(P->hold_ms);
    }
}

#ifndef KEEP
#  define KEEP (NAN)
#endif
extern int32_t cur_tick[4][3];

#ifndef KEEP
#  define KEEP (NAN)
#endif

volatile uint8_t g_stop_gait = 0;

void gait_stop_request(void) {
    g_stop_gait = 1;
}

// 보행 러너
void run_gait_I_loopable(void)
{
    // ---- Phase 정의 (P0~P17) ----
    const GaitPhase seq[] = {
        // [0] ~ [17]
        { .pose={{ -1.0000, -1.5000, -0.5000 },{ -0.2500,-1.8000,-1.5500 },
                 { -0.2500,-1.8000,-1.5500 },{ -1.0000,-1.5000,-0.5000 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2),
          .hold_ms=200,.flags=PH_ALIGN_WHEELS|PH_REAPPLY_GOAL
        },
        { .pose={{ -1.0000,-1.0500,-0.0500},{ -0.2500,-1.8000,-1.5500 },
                 { -0.2500,-1.8000,-1.5500 },{ -1.0000,-1.0500,-0.0500 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -0.4500,-1.0500,-0.6000},{ -0.2500,-0.3000,-0.0500 },
                 { -0.4000,-1.6500,-1.2500 },{ -1.3000,-1.2000,0.1000 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -0.4000,-1.0500,-0.6500},{ -1.4000,-0.7000,0.7000 },
                 { -0.4500,-1.5500,-1.1000 },{ -1.3500,-1.2500,0.1000 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<0)|(1<<1)|(1<<3),
          .hold_ms=200,.flags=PH_ALIGN_WHEELS
        },
        { .pose={{ -0.3500,-1.0500,-0.7000},{ -1.4000,-1.7000,-0.3000 },
                 { -0.5000,-1.5000,-1.0000 },{ -1.4000,-1.3000,0.1000 }},
          .T_ms=2000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_ALIGN_WHEELS
        },
        { .pose={{ -0.5000,-1.5000,-1.0000},{ -1.4000,-1.3000,0.1000 },
                 { -0.3500,-1.0500,-0.7000 },{ -1.4000,-1.7000,-0.3000 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -0.3000,-1.7500,-1.4500},{ -1.1000,-1.0500,0.0500 },
                 { -0.9000,-1.0000,-0.1000 },{ -1.5000,-1.8500,-0.5500 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<0)|(1<<1)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -0.3000,-1.7500,-1.4500},{ -1.1000,-1.0500,0.0500 },
                 { -0.9000,-1.0000,-0.1000 },{ -1.0000,-0.3000,0.5000 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_ALIGN_WHEELS
        },
        { .pose={{ -0.2500,-1.7500,-1.5000},{ -1.0500,-1.0500,0.0000 },
                 { -0.9500,-1.0500,-0.1000 },{ -0.3000,-0.5000,-0.2000 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -0.2500,-1.8000,-1.5500},{ -1.0000,-1.0500,-0.0500 },
                 { -1.0000,-1.0500,-0.0500 },{ -0.2500,-1.8000,-1.5500 }},
          .T_ms=2000,.dt_ms=10,
          .stance_mask=(1<<0)|(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL|PH_ALIGN_WHEELS|PH_WAIT_CONTACT_FL
        },
        { .pose={{ -0.2500,-0.3000,-0.0500},{ -0.4500,-1.0500,-0.6000 },
                 { -1.3000,-1.2000,0.1000 },{ -0.4000,-1.6500,-1.2500 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -1.4000,-0.7000,0.7000},{ -0.4000,-1.0500,-0.6500 },
                 { -1.3500,-1.2500,0.1000 },{ -0.4500,-1.5500,-1.1000 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<0)|(1<<1)|(1<<3),
          .hold_ms=200,.flags=PH_ALIGN_WHEELS
        },
        { .pose={{ -1.4000,-1.7000,-0.3000},{ -0.3500,-1.0500,-0.7000 },
                 { -1.4000,-1.3000,0.1000 },{ -0.5000,-1.5000,-1.0000 }},
          .T_ms=2000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -1.4000,-1.3000,0.1000},{ -0.5000,-1.5000,-1.0000 },
                 { -1.4000,-1.7000,-0.3000 },{ -0.3500,-1.0500,-0.7000 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -1.1000,-1.0500,0.0500},{ -0.3000,-1.7500,-1.4500 },
                 { -1.5000,-1.8500,-0.5500 },{ -0.9000,-1.0000,-0.1000 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<0)|(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -1.1000,-1.0500,0.0500},{ -0.3000,-1.7500,-1.4500 },
                 { -0.3000,-0.5000,-0.2000 },{ -0.9000,-1.0000,-0.1000 }},
          .T_ms=3000,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -1.0500,-1.0500,0.0000},{ -0.2500,-1.7500,-1.5000 },
                 { -0.3000,-0.5000,-0.2000 },{ -0.9500,-1.0500,-0.1000 }},
          .T_ms=1500,.dt_ms=10,
          .stance_mask=(1<<0)|(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL
        },
        { .pose={{ -1.0000,-1.0500,-0.0500},{ -0.2500,-1.8000,-1.5500 },
                 { -0.2500,-1.8000,-1.5500 },{ -1.0000,-1.0500,-0.0500 }},
          .T_ms=1500,.dt_ms=10,
          .stance_mask=(1<<1)|(1<<2)|(1<<3),
          .hold_ms=200,.flags=PH_REAPPLY_GOAL|PH_ALIGN_WHEELS
        }
    };

    const int N = sizeof(seq)/sizeof(seq[0]);

    // 실행 루프
    g_stop_gait = 0;
    UART_Transmit_DMA(&huart1,(uint8_t*)"Gait loop start\r\n",17);

    // 첫 싸이클 0~17
    run_phases(seq, N);

    // 2~17 반복
    while (!g_stop_gait) {
        for (int i = 2; i < N && !g_stop_gait; i++) {
            run_phases(&seq[i], 1);  // 단일 phase 실행
        }
    }

    // 정지 루틴
    uint32_t hip[4], knee[4], ankle[4];
    for (int l=0;l<4;l++){
        hip[l]=(uint32_t)cur_tick[l][0];
        knee[l]=(uint32_t)cur_tick[l][1];
        ankle[l]=(uint32_t)cur_tick[l][2];
    }
    send_sync_write_2(ALL, DXL_2_Goal_Position, 4, hip, knee, ankle);
    align_Wheels();
    UART_Transmit_DMA(&huart1,(uint8_t*)"Gait stopped\r\n",14);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

enum { LEG_FL=0, LEG_FR=1, LEG_RR=2, LEG_RL=3 };

static inline void apply_impedance_profile(SendLegType leg, int stance)
{
    uint16_t P[3], I[3], D[3];
    uint16_t pwm_lim[3];

    if (stance) {
        // Stance: 한 단계 낮춘 값
        P[HIP]=520;  P[KNEE]=560;  P[ANKLE]=200;
        I[HIP]=0;    I[KNEE]=0;    I[ANKLE]=0;
        D[HIP]=50;   D[KNEE]=50;   D[ANKLE]=25;
        pwm_lim[HIP]=360; pwm_lim[KNEE]=400; pwm_lim[ANKLE]=320;
    } else {
        // Swing: 더 낮게
        P[HIP]=240;  P[KNEE]=280;  P[ANKLE]=200;
        I[HIP]=0;    I[KNEE]=0;    I[ANKLE]=0;
        D[HIP]=30;   D[KNEE]=30;   D[ANKLE]=20;
        pwm_lim[HIP]=300; pwm_lim[KNEE]=320; pwm_lim[ANKLE]=280;
    }

    dxl_set_leg_pid_2(leg, P, I, D);
    dxl_set_leg_pwm_limit_2(leg, pwm_lim);
}


// 단계적 램프 적용 버전
static inline void apply_impedance_profile_ramped(SendLegType leg, int stance){
    reapply_current_goal_ticks();  // 오차≈0

    if (stance){
        uint16_t I0[3]={0,0,0};

        // 단계1
        uint16_t P1[3]={480,520,200}, D1[3]={40,40,20}, L1[3]={460,500,340};
        dxl_set_leg_pid_2(leg,P1,I0,D1);
        dxl_set_leg_pwm_limit_2(leg,L1); HAL_Delay(25);

        // 단계2
        uint16_t P2[3]={560,620,210}, D2[3]={60,60,30}, L2[3]={490,530,360};
        dxl_set_leg_pid_2(leg,P2,I0,D2);
        dxl_set_leg_pwm_limit_2(leg,L2); HAL_Delay(25);

        // 단계3 (최종 목표)
        uint16_t P3[3]={620,680,220}, D3[3]={60,60,30}, L3[3]={520,560,380};
        dxl_set_leg_pid_2(leg,P3,I0,D3);
        dxl_set_leg_pwm_limit_2(leg,L3); HAL_Delay(25);
    }else{
        uint16_t P[3]={260,300,220}, I[3]={0,0,0}, D[3]={40,40,20};
        uint16_t L[3]={340,380,300};
        dxl_set_leg_pid_2(leg,P,I,D);
        dxl_set_leg_pwm_limit_2(leg,L);
        HAL_Delay(20);
    }
}



static inline void set_impedance_all_ramped(int fl_stance, int fr_stance, int rr_stance, int rl_stance)
{
    apply_impedance_profile_ramped(FL, fl_stance);
    apply_impedance_profile_ramped(FR, fr_stance);
    apply_impedance_profile_ramped(RR, rr_stance);
    apply_impedance_profile_ramped(RL, rl_stance);
}

static inline void settle_hold(uint32_t ms)
{
    reapply_current_goal_ticks();  // 지금 목표 틱 재전송
    HAL_Delay(ms);
}

static inline void set_impedance_and_profile_all(int fl_stance,int fr_stance,int rr_stance,int rl_stance){
    // PID/PWM
    apply_impedance_profile_ramped(FL, fl_stance);
    apply_impedance_profile_ramped(FR, fr_stance);
    apply_impedance_profile_ramped(RR, rr_stance);
    apply_impedance_profile_ramped(RL, rl_stance);

    // Profile V/A
    set_profile_for_phase(FL, !fl_stance);
    set_profile_for_phase(FR, !fr_stance);
    set_profile_for_phase(RR, !rr_stance);
    set_profile_for_phase(RL, !rl_stance);
}

// 각 관절(서보)의 안전 범위
static const int32_t TMIN = 0, TMAX = 4095;


// 스케일 k: ticks/rad
static float K_TICKS_PER_RAD[4][3] = {
     { 4096.0f/(2*M_PI), 4096.0f/(2*M_PI), 4096.0f/(2*M_PI) }, //FL
     { 4096.0f/(2*M_PI), 4096.0f/(2*M_PI), 4096.0f/(2*M_PI) }, //FR
     { 4096.0f/(2*M_PI), 4096.0f/(2*M_PI), 4096.0f/(2*M_PI) }, //RR
     { 4096.0f/(2*M_PI), 4096.0f/(2*M_PI), 4096.0f/(2*M_PI) } //RL
};


static int8_t SGN[4][3] = {
    { +1, -1, +1 },
    { -1, +1, -1 },   // HIP/ANKLE 반전 (거울대칭)
    { +1, -1, +1 },
    { -1, +1, -1 }
};


// 앉았을때 기준
int32_t cur_tick[4][3] = {
	    { 3365, 0, 2557 },
	    { 763, 4130, 1501 },
	    { 3361, 33, 2534 },
	    { 784, 4084, 1493 }
};


static uint32_t Hip_Position_tx[4], Knee_Position_tx[4], Ankle_Position_tx[4];


static inline void reapply_current_goal_ticks(void){
    uint32_t hip[4], knee[4], ankle[4];
    for(int l=0;l<4;l++){
        hip[l]   = (uint32_t)cur_tick[l][HIP];
        knee[l]  = (uint32_t)cur_tick[l][KNEE];
        ankle[l] = (uint32_t)cur_tick[l][ANKLE];
    }
    send_sync_write_2(ALL, DXL_2_Goal_Position, 4, hip, knee, ankle);
    HAL_Delay(20);
}

void apply_increment(const double dRad[4][3], uint32_t duration_ms)
{
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            // 라디안 → 틱(스케일·부호·제로 보정)
            float k   = K_TICKS_PER_RAD[leg][j];
            int8_t s  = SGN[leg][j];
            int32_t dticks = (int32_t)lroundf( s * k * (float)dRad[leg][j] );

            // int32 누적 및 클램핑
            int32_t next = cur_tick[leg][j] + dticks;
            cur_tick[leg][j] = CLAMP(next, TMIN, TMAX);
        }
    }

    // 전송 버퍼로 복사 (uint32_t로 캐스팅)
    for (int leg = 0; leg < 4; leg++) {
        Hip_Position_tx[leg]   = (uint32_t)cur_tick[leg][HIP];
        Knee_Position_tx[leg]  = (uint32_t)cur_tick[leg][KNEE];
        Ankle_Position_tx[leg] = (uint32_t)cur_tick[leg][ANKLE];
    }

    send_sync_write_2(ALL, DXL_2_Goal_Position, 4,
                      Hip_Position_tx, Knee_Position_tx, Ankle_Position_tx);

    HAL_Delay(duration_ms);
}

// 스탠스(1)=강하게, 스윙(0)=유연하게
static inline void set_impedance_all(int fl_stance, int fr_stance, int rr_stance, int rl_stance)
{
    apply_impedance_profile(FL, fl_stance);
    apply_impedance_profile(FR, fr_stance);
    apply_impedance_profile(RR, rr_stance);
    apply_impedance_profile(RL, rl_stance);
}

static double g_target_rad[4][3] = {
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0}
};

// cur_tick 기준으로 라디안 목표를 절대 틱으로 변환해 전송
static void apply_target(const double targetRad[4][3], uint32_t duration_ms)
{
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            float   k     = K_TICKS_PER_RAD[leg][j];
            int8_t  s     = SGN[leg][j];
            float   rad   = (float)targetRad[leg][j];
            int32_t tick  = (int32_t)lroundf( (float)cur_tick[leg][j] + s * k * rad );
            cur_tick[leg][j] = CLAMP(tick, TMIN, TMAX);
        }
    }

    // 전송 버퍼로 복사
    for (int leg = 0; leg < 4; leg++) {
        Hip_Position_tx[leg]   = (uint32_t)cur_tick[leg][HIP];
        Knee_Position_tx[leg]  = (uint32_t)cur_tick[leg][KNEE];
        Ankle_Position_tx[leg] = (uint32_t)cur_tick[leg][ANKLE];
    }

    send_sync_write_2(ALL, DXL_2_Goal_Position, 4,
                      Hip_Position_tx, Knee_Position_tx, Ankle_Position_tx);

    HAL_Delay(duration_ms);
}

static inline float s5(float s){ return s*s*s*(10.0f + s*(-15.0f + 6.0f*s)); }

void apply_target_smooth(const double targetRad[4][3], uint32_t T_ms, uint32_t dt_ms){
    double start[4][3];
    for(int l=0;l<4;l++)for(int j=0;j<3;j++) start[l][j]=g_target_rad[l][j];

    int steps = (int)(T_ms/dt_ms); if(steps<1) steps=1;
    for(int k=1;k<=steps;k++){
        float s = (float)k/(float)steps;
        float a = s5(s);
        for(int l=0;l<4;l++)for(int j=0;j<3;j++)
            g_target_rad[l][j] = start[l][j] + a*(targetRad[l][j]-start[l][j]);
        apply_target(g_target_rad, dt_ms);
    }
}


void apply_target_smooth_locked(const double targetRad[4][3], uint32_t T_ms, uint32_t dt_ms)
{
    double start[4][3];
    for(int l=0;l<4;l++) for(int j=0;j<3;j++) start[l][j]=g_target_rad[l][j];

    int steps = (int)(T_ms/dt_ms); if(steps<1) steps=1;
    uint32_t t_next = HAL_GetTick();
    uint32_t hip[4], knee[4], ankle[4];

    for(int k=1;k<=steps;k++){
        float s = (float)k/(float)steps;
        float a = s5(s);

        // 1) rad 갱신
        for(int l=0;l<4;l++){
            for(int j=0;j<3;j++){
                g_target_rad[l][j] = start[l][j] + a*(targetRad[l][j]-start[l][j]);
                float ktr  = K_TICKS_PER_RAD[l][j];
                int8_t sgn = SGN[l][j];
                int32_t tick = (int32_t)lroundf((float)cur_tick[l][j] + sgn * ktr * (float)g_target_rad[l][j]);
                tick = CLAMP(tick, TMIN, TMAX);
                cur_tick[l][j] = tick;
            }
        }

        // 2) 한 프레임 = 한 번의 SyncWrite
        for(int l=0;l<4;l++){
            hip[l]   = (uint32_t)cur_tick[l][0];
            knee[l]  = (uint32_t)cur_tick[l][1];
            ankle[l] = (uint32_t)cur_tick[l][2];
        }
        send_sync_write_2(ALL, DXL_2_Goal_Position, 4, hip, knee, ankle);

        // 3) 다음 프레임 목표 시각까지 정확히 대기
        t_next += dt_ms;
        while((int32_t)(HAL_GetTick() - t_next) < 0){
            __NOP();
        }
    }
}

static inline void reapply_current_goal_ticks(void);
static inline void goto_pose_abs_rad(const double pose[4][3], uint32_t T_ms, uint32_t dt_ms);

extern uint16_t ss1[4];

void move4(uint16_t incFL, uint16_t incFR, uint16_t incRR, uint16_t incRL)
{
    ss1[0] = ss1[1] = ss1[2] = ss1[3] = 0;

    for (int j = 0; j < 10; ++j) {
        ss1[0] += incFL;
        ss1[1] += incFR;
        ss1[2] += incRR;
        ss1[3] += incRL;

        send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, ss1);
        HAL_Delay(10);
    }
}

// 절대 라디안 포즈(4x3)를 그대로 목표로 하여 부드럽게 이동
static inline void goto_pose_abs_rad(const double pose[4][3], uint32_t T_ms, uint32_t dt_ms)
{
    double tgt[4][3];
    for (int l=0;l<4;l++) for (int j=0;j<3;j++) tgt[l][j] = pose[l][j];
    apply_target_smooth_locked(tgt, T_ms, dt_ms);   // ★ 변경 포인트
}

// 목표(rad)로 부드럽게 이동
void add_and_apply_increment_smooth(const double dRad[4][3],
                                    uint32_t T_ms, uint32_t dt_ms)
{
    // 1) 증분을 누적해서 절대 목표(rad) 만들기
    double targetRad[4][3];
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            g_target_rad[leg][j] += dRad[leg][j];
            targetRad[leg][j] = g_target_rad[leg][j];
        }
    }

    // 2) 절대 목표를 스무스하게 적용
    apply_target_smooth(targetRad, T_ms, dt_ms);
}

//void servo_init();

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(50);

	dxl_make_quiet_after_boot();
	HAL_Delay(20);    // EEPROM commit 여유

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	dxl_torque_set(ALL, TORQUE_ON, TORQUE_ON);

	ax12_force_wheel_mode_all();

	// 2.0/1.0 토크 ON
	dxl_torque_set(ALL, TORQUE_ON, TORQUE_ON);

	send_sync_write_2(ALL, DXL_2_Goal_Velocity, 4, angle_speed_Hip, angle_speed_Knee, angle_speed_Ankle);//정상동작

	HAL_UART_Receive_DMA(&huart1, pc_rx_buf, DMA_BUF_SIZE);

	tickstart = HAL_GetTick();
	wait_time = 15;
	servo_init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		uart_rx_dma1_handler();
		process_input();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void process_input() {

	if (new_data_received[0]) {

		switch(input_char) {
		case 'F': {   // AUTO 모드 토글
		    if (g_auto_mode == 1) {
		        g_auto_mode = 0;  // → AUTO OFF
		        UART_Transmit_DMA(&huart1, (uint8_t*)"AUTO MODE OFF\r\n", 15);
		    } else {
		        g_auto_mode = 1;  // → AUTO ON
		        UART_Transmit_DMA(&huart1, (uint8_t*)"AUTO MODE ON\r\n", 14);
		    }
		} break;

		case 'x':{
			align_Wheels();
			move4(-50,50,50,-50);
			UART_Transmit_DMA(&huart1, (uint8_t*)"Go backward\r\n", 13);
		} break;

		case 'w':{
			ax12_force_wheel_mode_all();
			align_Wheels();
			servo_init();
			move4(+50,-50,-50,+50);
			UART_Transmit_DMA(&huart1, (uint8_t*)"Go Straight\r\n", 13);
		} break;

		case 'q':{
			turn_left();
			move4(30,-33,-30,33);
			UART_Transmit_DMA(&huart1, (uint8_t*)"Go Left\r\n", 9);
		} break;

		case 'e':{
			turn_right();
			move4(30,-33,-30,33);
			UART_Transmit_DMA(&huart1, (uint8_t*)"Go Right\r\n", 10);
		} break;

		case 'd':{
			zero_turn();
			move4(45,45,45,45);
			UART_Transmit_DMA(&huart1, (uint8_t*)"ZERO_TURN_Right\r\n", 17);
		} break;

		case 'a':{
			zero_turn();
			move4(-45,-45,-45,-45);
			UART_Transmit_DMA(&huart1, (uint8_t*)"ZERO_TURN_LEFT\r\n", 16);
		} break;


		case 'z':{
			turn_left();
			move4(-30,33,33,-30);
			UART_Transmit_DMA(&huart1, (uint8_t*)"Back Left\r\n", 11);
		} break;

		case 'c':{
			turn_right();
			move4(-30,33,30,-33);
			UART_Transmit_DMA(&huart1, (uint8_t*)"Back Right\r\n", 12);
		} break;


		case 's':{
			UART_Transmit_DMA(&huart1, (uint8_t*)"Stop Move\r\n", 11);
			ss1[0] = ss1[1] = ss1[2] = 	ss1[3] = 0;
			send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, ss1);
			HAL_Delay(10);
		} break;

		case 'S':
		{
		    gait_stop_request(); // 러너 중단 요청
		    UART_Transmit_DMA(&huart1,(uint8_t*)"STOP GAIT\r\n",11);
		} break;



		case '8':{
			crab_left();
			move4(35,-35,-35,35);
			UART_Transmit_DMA(&huart1, (uint8_t*)"left crab Straight\r\n", 13);
			} break;

		case '9':{
			crab_right();
			move4(35,-35,-35,35);
			UART_Transmit_DMA(&huart1, (uint8_t*)"right crab Straight\r\n", 21);
		} break;

		case '5':{
			crab_right();
			move4(-35,35,35,-35);
			UART_Transmit_DMA(&huart1, (uint8_t*)"left crab backward\r\n", 35);
		} break;

		case '6':{
			crab_left();
			move4(-35,35,35,-35);
			UART_Transmit_DMA(&huart1, (uint8_t*)"right crab backward\r\n", 13);
		} break;

		case '<':{
			side_move();
			move4(-17,-17,17,17);
			UART_Transmit_DMA(&huart1, (uint8_t*)"side left\r\n", 11);
		} break;


		case '>':{
			side_move();
			move4(17,17,-17,-17);
			UART_Transmit_DMA(&huart1, (uint8_t*)"side right\r\n", 12);
		} break;


		case 'k': {
		    UART_Transmit_DMA(&huart1, (uint8_t*)"Sit Down\r\n", 9);
		    uint32_t Hip_Position[4], Knee_Position[4], Ankle_Position[4];
		    Hip_Position[0] = 3365;  Hip_Position[1] =  763;  Hip_Position[2] = 3361;  Hip_Position[3] =  784;
		    Knee_Position[0] =    0; Knee_Position[1] = 4130; Knee_Position[2] =   33; Knee_Position[3] = 4084;
		    Ankle_Position[0]= 2557; Ankle_Position[1]= 1501; Ankle_Position[2]= 2534; Ankle_Position[3]= 1493;
		    send_sync_write_2(ALL, DXL_2_Goal_Position, 4, Hip_Position, Knee_Position, Ankle_Position);
		    UART_Transmit_DMA(&huart1, (uint8_t*)"Done\r\n", 6);



		} break;

		case 'm':{
			servo_init();
			UART_Transmit_DMA(&huart1, (uint8_t*)"Stand Up\r\n", 10);
			uint32_t Hip_Position[4],	Knee_Position[4],	Ankle_Position[4];
			Hip_Position[0] = 3280;  	Hip_Position[1] = 890;	 	Hip_Position[2] = 3280;	 	Hip_Position[3] = 896;
			Knee_Position[0] = 680; Knee_Position[1] = 3450;	Knee_Position[2] = 680;	Knee_Position[3] = 3446;
			Ankle_Position[0] = 2048; 	Ankle_Position[1] = 2048;	Ankle_Position[2] = 2048;	Ankle_Position[3] = 2048;
			send_sync_write_2(ALL, DXL_2_Goal_Position, 4, Hip_Position, Knee_Position, Ankle_Position);
			leg1 = 1;

		} break;

		case 'N': {
		    servo_init();
		    ax12_force_joint_mode_all();
		    align_Wheels();

		    UART_Transmit_DMA(&huart1,(uint8_t*)"Gait I start\r\n",14);
		    run_gait_I_loopable();  // 자동 반복 실행
		    UART_Transmit_DMA(&huart1,(uint8_t*)"Gait I done\r\n",13);
		} break;


		case 'I': {    // 보행 2
		    servo_init();
			ax12_force_joint_mode_all();
		    uint16_t zero[4] = {0,0,0,0};
		    //send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, zero);
		    const uint32_t DT = 1;
		    //set_impedance_all(1,1,1,1);             // 모두 강(stance)으로 시작
		    HAL_Delay(2000);

		    double P0[4][3] = {
					{  -1.0000,  -1.5000,  -0.5000},  // FL
					{  -0.2500,  -1.8000,  -1.5500},  // FR
					{  -0.2500,  -1.8000,  -1.5500},  // RR
					{  -1.0000,  -1.5000,  -0.5000}   // RL
		    };
		    goto_pose_abs_rad(P0,3000, DT);

		    double P1[4][3] = {
					{  -1.0000,  -1.0500,  -0.0500},  // FL
					{  -0.2500,  -1.8000,  -1.5500},  // FR
					{  -0.2500,  -1.8000,  -1.5500},  // RR
					{  -1.0000,  -1.0500,  -0.0500}   // RL
		    };
		    goto_pose_abs_rad(P1,3000, DT);

		    double P2[4][3] = {
					{  -0.4500,  -1.0500,  -0.6000},  // FL
					{  -0.2500,  -0.3000,  -0.0500},  // FR
					{  -0.4000,  -1.6500,  -1.2500},  // RR
					{  -1.3000,  -1.2000,   0.1000}   // RL
		    };
		    goto_pose_abs_rad(P2,3000, DT);

		    double P3[4][3] = {
					{  -0.4000,  -1.0500,  -0.6500},  // FL
					{  -1.4000,  -0.7000,   0.7000},  // FR
					{  -0.4500,  -1.5500,  -1.1000},  // RR
					{  -1.3500,  -1.2500,   0.1000}   // RL
		    };
		    goto_pose_abs_rad(P3,3000, DT);

		    double P4[4][3] = {
					{  -0.3500,  -1.0500,  -0.7000},  // FL
					{  -1.4000,  -1.7000,  -0.3000},  // FR
					{  -0.5000,  -1.5000,  -1.0000},  // RR
					{  -1.4000,  -1.3000,   0.1000}   // RL
		    };
		    goto_pose_abs_rad(P4,1000, DT);

		    double P5[4][3] = {
					{  -0.5000,  -1.5000,  -1.0000},  // FL
					{  -1.4000,  -1.3000,   0.1000},  // FR
					{  -0.3500,  -1.0500,  -0.7000},  // RR
					{  -1.4000,  -1.7000,  -0.3000}   // RL
		    };
		    goto_pose_abs_rad(P5,3000, DT);

		    double P6[4][3] = {
					{  -0.3000,  -1.7500,  -1.4500},  // FL
					{  -1.1000,  -1.0500,   0.0500},  // FR
					{  -0.9000,  -1.0000,  -0.1000},  // RR
					{  -1.5000,  -1.8500,  -0.5500}   // RL
		    };
		    goto_pose_abs_rad(P6,3000, DT);

		    double P7[4][3] = {
					{  -0.3000,  -1.7500,  -1.4500},  // FL
					{  -1.1000,  -1.0500,   0.0500},  // FR
					{  -0.9000,  -1.0000,  -0.1000},  // RR
					{  -1.0000,  -0.3000,   0.5000}   // RL
		    };
		    goto_pose_abs_rad(P7,3000, DT);

		    double P8[4][3] = {
					{  -0.2500,  -1.7500,  -1.5000},  // FL
					{  -1.0500,  -1.0500,  -0.0000},  // FR
					{  -0.9500,  -1.0500,  -0.1000},  // RR
					{  -0.3000,  -0.5000,  -0.2000}   // RL
		    };
		    goto_pose_abs_rad(P8,3000, DT);

		    double P9[4][3] = {
					{  -0.2500,  -1.8000,  -1.5500},  // FL
					{  -1.0000,  -1.0500,  -0.0500},  // FR
					{  -1.0000,  -1.0500,  -0.0500},  // RR
					{  -0.2500,  -1.8000,  -1.5500}   // RL
		    };
		    goto_pose_abs_rad(P9,1000, DT);

		    HAL_Delay(2000);


		    double P10[4][3] = {
					{  -0.2500,  -0.3000,  -0.0500},  // FL
					{  -0.4500,  -1.0500,  -0.6000},  // FR
					{  -1.3000,  -1.2000,   0.1000},  // RR
					{  -0.4000,  -1.6500,  -1.2500}   // RL
		    };
		    goto_pose_abs_rad(P10,3000, DT);

		    double P11[4][3] = {
					{  -1.4000,  -0.7000,   0.7000},  // FL
					{  -0.4000,  -1.0500,  -0.6500},  // FR
					{  -1.3500,  -1.2500,   0.1000},  // RR
					{  -0.4500,  -1.5500,  -1.1000}   // RL
		    };
		    goto_pose_abs_rad(P11,3000, DT);

		    double P12[4][3] = {
					{  -1.4000,  -1.7000,  -0.3000},  // FL
					{  -0.3500,  -1.0500,  -0.7000},  // FR
					{  -1.4000,  -1.3000,   0.1000},  // RR
					{  -0.5000,  -1.5000,  -1.0000}   // RL
		    };
		    goto_pose_abs_rad(P12,1000, DT);

		    double P13[4][3] = {
					{  -1.4000,  -1.3000,   0.1000},  // FL
					{  -0.5000,  -1.5000,  -1.0000},  // FR
					{  -1.4000,  -1.7000,  -0.3000},  // RR
					{  -0.3500,  -1.0500,  -0.7000}   // RL
		    };
		    goto_pose_abs_rad(P13,3000, DT);

		    double P14[4][3] = {
					{  -1.1000,  -1.0500,   0.0500},  // FL
					{  -0.3000,  -1.7500,  -1.4500},  // FR
					{  -1.5000,  -1.8500,  -0.5500},  // RR
					{  -0.9000,  -1.0000,  -0.1000}   // RL
		    };
		    goto_pose_abs_rad(P14,3000, DT);

		    double P15[4][3] = {
					{  -1.1000,  -1.0500,   0.0500},  // FL
					{  -0.3000,  -1.7500,  -1.4500},  // FR
					{  -0.3000,  -0.5000,  -0.2000},  // RR
					{  -0.9000,  -1.0000,  -0.1000}   // RL
		    };
		    goto_pose_abs_rad(P15,3000, DT);

		    double P16[4][3] = {
					{  -1.0500,  -1.0500,   0.0000},  // FL
					{  -0.2500,  -1.7500,  -1.5000},  // FR
					{  -0.3000,  -0.5000,  -0.2000},  // RR
					{  -0.9500,  -1.0500,  -0.1000}   // RL
		    };
		    goto_pose_abs_rad(P16,500, DT);

		    double P17[4][3] = {
					{  -1.0000,  -1.0500,  -0.0500},  // FL
					{  -0.2500,  -1.8000,  -1.5500},  // FR
					{  -0.2500,  -1.8000,  -1.5500},  // RR
					{  -1.0000,  -1.0500,  -0.0500}   // RL
		    };
		    goto_pose_abs_rad(P17,1000, DT);


		}break;

		case 'Y': {    // 단차극복 1

		    servo_init();
			ax12_force_joint_mode_all();

		    uint16_t zero[4] = {0,0,0,0};
		    send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, zero);

		    const uint32_t DT = 10;
		    set_impedance_all(1,1,1,1);             // 모두 강(stance)으로 시작
		    double P0[4][3] = {
		                      {  -0.7000,  -1.4000,  -0.7000},  // FL
		                      {  -0.7000,  -1.4000,  -0.7000},  // FR
		                      {  -0.7000,  -1.4000,  -0.7000},  // RR
		                      {  -0.7000,  -1.4000,  -0.7000}   // RL
		       };
		    goto_pose_abs_rad(P0,3000, DT);

		    double P1[4][3] = {
		                      {  -0.7000,  -1.4000,  -0.7000},  // FL
		                      {  -0.7000,  -1.4000,  -0.7000},  // FR
		                      {  -0.7000,  -1.4000,  -0.7000},  // RR
		                      {  -0.3000,  -1.7000,  -1.4000}   // RL
		       };
		    goto_pose_abs_rad(P1,3000, DT);

		    double P2[4][3] = {
		                      {  -0.7000,  -1.4000,  -0.7000},  // FL
		                      {  -0.6000,  -1.0000,  -0.4000},  // FR
		                      {  -0.6000,  -1.0000,  -0.4000},  // RR
		                      {  -0.3000,  -1.7000,  -1.4000}   // RL
		       };
		    goto_pose_abs_rad(P2,3000, DT);

		    double P3[4][3] = {
		                      {  -0.6500,  -0.0000,   0.6500},  // FL
		                      {  -0.6000,  -1.0000,  -0.4000},  // FR
		                      {  -0.6000,  -1.0000,  -0.4000},  // RR
		                      {  -0.3000,  -1.7000,  -1.4000}   // RL
		       };
		    goto_pose_abs_rad(P3,3000, DT);

		    double P4[4][3] = {
		                      {  -1.7000,  -0.0000,   0.7000},  // FL
		                      {  -0.6000,  -1.0000,  -0.4000},  // FR
		                      {  -0.6000,  -1.0000,  -0.4000},  // RR
		                      {  -0.3000,  -1.7000,  -1.4000}   // RL
		       };
		    goto_pose_abs_rad(P4,3000, DT);

		    double P5[4][3] = {
		                      {  -1.7000,  -1.3000,   0.4000},  // FL
		                      {  -0.6000,  -1.0000,  -0.4000},  // FR
		                      {  -0.6000,  -1.0000,  -0.4000},  // RR
		                      {  -0.3000,  -1.7000,  -1.4000}   // RL
		       };
		    goto_pose_abs_rad(P5,3000, DT);

		    double P6[4][3] = {
		                      {  -1.7000,  -1.3000,   0.4000},  // FL
		                      {  -0.7000,  -1.4000,  -0.7000},  // FR
		                      {  -0.7000,  -1.4000,  -0.7000},  // RR
		                      {  -0.3000,  -1.7000,  -1.4000}   // RL
		       };
		    goto_pose_abs_rad(P6,3000, DT);

		    double P7[4][3] = {
		                      {  -1.7000,  -1.3000,   0.4000},  // FL
		                      {  -0.7000,  -1.4000,  -0.7000},  // FR
		                      {  -0.3000,  -1.7000,  -1.4000},  // RR
		                      {  -0.7000,  -1.4000,  -0.7000}   // RL
		       };
		    goto_pose_abs_rad(P7,3000, DT);

		    double P8[4][3] = {
		                      {  -1.8000,  -1.0000,   0.8000},  // FL
		                      {  -0.7000,  -1.4000,  -0.7000},  // FR
		                      {  -0.3000,  -1.7000,  -1.4000},  // RR
		                      {  -0.6000,  -1.1000,  -0.5000}   // RL
		       };
		    goto_pose_abs_rad(P8,3000, DT);

		    double P9[4][3] = {
		                      {  -1.8000,  -1.0000,   0.8000},  // FL
		                      {  -0.6500,  -0.0000,   0.6500},  // FR
		                      {  -0.3000,  -1.7000,  -1.4000},  // RR
		                      {  -0.6000,  -1.1000,  -0.5000}   // RL
		       };
		    goto_pose_abs_rad(P9,3000, DT);

		    double P10[4][3] = {
		                      {  -1.8000,  -1.0000,   0.8000},  // FL
		                      {  -1.7000,  -0.2000,   0.5000},  // FR
		                      {  -0.3000,  -1.7000,  -1.4000},  // RR
		                      {  -0.6000,  -1.1000,  -0.5000}   // RL
		       };
		    goto_pose_abs_rad(P10,3000, DT);

		    double P11[4][3] = {
		                      {  -1.8000,  -1.0000,   0.8000},  // FL
		                      {  -1.7000,  -1.3000,   0.4000},  // FR
		                      {  -0.3000,  -1.7000,  -1.4000},  // RR
		                      {  -0.6000,  -1.1000,  -0.5000}   // RL
		       };
		    goto_pose_abs_rad(P11,3000, DT);

		    double P12[4][3] = {
		                      {  -1.7000,  -1.3000,   0.4000},  // FL
		                      {  -1.7000,  -1.3000,   0.4000},  // FR
		                      {  -0.3000,  -1.7000,  -1.4000},  // RR
		                      {  -0.7000,  -1.4000,  -0.7000}   // RL
		       };
		    goto_pose_abs_rad(P12,3000, DT);

		    double P13[4][3] = {
		                      {  -1.2000,  -0.7000,   0.1770},  // FL
		                      {  -1.2000,  -0.7000,   0.1770},  // FR
		                      {  -0.7000,  -1.4000,  -0.7000},  // RR
		                      {  -0.7000,  -1.4000,  -0.7000}   // RL
		       };
		    goto_pose_abs_rad(P13,3000, DT);

		    double P14[4][3] = {
		                      {  -1.2000,  -0.7000,   0.1770},  // FL
		                      {  -1.2000,  -0.7000,   0.1770},  // FR
		                      {  -0.7000,  -1.4000,  -0.7000},  // RR
		                      {  -0.7000,  -1.4000,  -0.7000}   // RL
		       };
		    goto_pose_abs_rad(P14,3000, DT);

		    double P15[4][3] = {
		                      {  -0.0000,  -0.8000,  -0.8000},  // FL
		                      {  -0.5500,  -0.6000,  -0.0500},  // FR
		                      {  -1.3000,  -1.6500,  -0.3500},  // RR
		                      {  -0.7000,  -1.4000,  -0.7000}   // RL
		       };
		    goto_pose_abs_rad(P15,3000, DT);

		    double P16[4][3] = {
		                      {  -0.0000,  -0.8000,  -0.8000},  // FL
		                      {  -0.5500,  -0.3000,   0.2500},  // FR
		                      {  -1.1000,  -1.2000,  -0.1000},  // RR
		                      {  -0.7000,  -1.4000,  -0.7000}   // RL
		       };
		    goto_pose_abs_rad(P16,3000, DT);

		    double P17[4][3] = {
		                      {  -0.0000,  -0.8000,  -0.8000},  // FL
		                      {  -0.5500,  -0.3000,   0.2500},  // FR
		                      {  -1.1000,  -1.2000,  -0.1000},  // RR
		                      {  -0.6500,  -0.0000,   0.6500}   // RL
		       };
		    goto_pose_abs_rad(P17,3000, DT);

		    double P18[4][3] = {
		                      {  -0.0000,  -0.8000,  -0.8000},  // FL
		                      {  -0.5500,  -0.3000,   0.2500},  // FR
		                      {  -1.1000,  -1.2000,  -0.1000},  // RR
		                      {  -0.1000,  -0.0000,   0.6233}   // RL
		       };
		    goto_pose_abs_rad(P18,3000, DT);

		    double P19[4][3] = {
		                      {  -0.0000,  -0.8000,  -0.8000},  // FL
		                      {  -0.5500,  -0.3000,   0.2500},  // FR
		                      {  -1.1000,  -1.2000,  -0.1000},  // RR
		                      {  -0.0000,  -0.9500,  -0.5270}   // RL
		       };
		    goto_pose_abs_rad(P19,3000, DT);

		    double P20[4][3] = {
		                      {  -0.5500,  -0.6000,  -0.0500},  // FL
		                      {  -0.0000,  -0.8000,  -0.8000},  // FR
		                      {  -1.3000,  -1.6500,  -0.3500},  // RR
		                      {  -0.0000,  -0.8000,  -0.3770}   // RL
		       };
		    goto_pose_abs_rad(P20,3000, DT);

		    double P21[4][3] = {
		                      {  -0.7000,  -1.0000,  -0.3000},  // FL
		                      {  -0.0000,  -1.5000,  -1.5000},  // FR
		                      {  -1.4000,  -2.2000,  -0.8000},  // RR
		                      {  -0.4000,  -1.0500,  -0.6500}   // RL
		       };
		    goto_pose_abs_rad(P21,3000, DT);

		    double P22[4][3] = {
		                      {  -0.6000,  -0.7000,  -0.1000},  // FL
		                      {  -0.0000,  -1.5000,  -1.5000},  // FR
		                      {  -1.4000,  -2.2000,  -0.8000},  // RR
		                      {  -0.3000,  -0.8000,  -0.5000}   // RL
		       };
		    goto_pose_abs_rad(P22,3000, DT);

		    double P23[4][3] = {
		                      {  -0.6000,  -0.7000,  -0.1000},  // FL
		                      {  -0.0000,  -1.5000,  -1.5000},  // FR
		                      {  -1.4000,  -0.0000,   0.6150},  // RR
		                      {  -0.3000,  -0.8000,  -0.5000}   // RL
		       };
		    goto_pose_abs_rad(P23,3000, DT);

		    double P24[4][3] = {
		                      {  -0.6000,  -0.7000,  -0.1000},  // FL
		                      {  -0.0000,  -1.5000,  -1.5000},  // FR
		                      {  -0.5500,  -0.0000,   0.5500},  // RR
		                      {  -0.3000,  -0.8000,  -0.5000}   // RL
		       };
		    goto_pose_abs_rad(P24,3000, DT);

		    double P25[4][3] = {
		                      {  -0.6000,  -0.7000,  -0.1000},  // FL
		                      {  -0.0000,  -1.5000,  -1.5000},  // FR
		                      {  -0.4000,  -1.0500,  -0.6500},  // RR
		                      {  -0.3000,  -0.8000,  -0.5000}   // RL
		       };
		    goto_pose_abs_rad(P25,3000, DT);

		    double P26[4][3] = {
		                      {  -0.7000,  -1.0000,  -0.3000},  // FL
		                      {  -0.0000,  -1.5000,  -1.5000},  // FR
		                      {  -0.4000,  -1.0500,  -0.6500},  // RR
		                      {  -0.4000,  -1.0500,  -0.6500}   // RL
		       };
		    goto_pose_abs_rad(P26,3000, DT);




		}break;


		case 'O': {    // 보행 동작 (한 다리씩)

		    servo_init();
			ax12_force_joint_mode_all();

		    uint16_t zero[4] = {0,0,0,0};
		    send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, zero);

		    const uint32_t DT = 10;
		    const uint32_t HOLD_PRELOAD_MS     = 90;   // 프리로드(몸 기울임) 후 짧은 안정화
		    const uint32_t HOLD_APEX_MS        = 60;   // 스윙 발이 공중 최정점에서 잠깐 정지
		    const uint32_t HOLD_AFTER_BIGMOVE  = 80;   // 큰 포즈 변환 뒤 짧게(선택)
		    set_impedance_all(1,1,1,1);             // 모두 강(stance)으로 시작

		    double C1[4][3] = {
		        { -0.2500, -1.8000, -1.5500},   // FL
		        { -0.7000, -1.4000, -0.7000},   // FR
		        { -0.7000, -1.4000, -0.7000},   // RR
		        { -0.2500, -1.8000, -1.5500}    // RL
		    };
		    goto_pose_abs_rad(C1,1100, DT);


		    HAL_Delay(4000);


		    // --- [1] 초기 안정자세 ---
		    double C2[4][3] = {
			        { -0.2500, -1.8000, -1.5500},   // FL
			        { -0.6000, -1.0000, -0.4000},   // FR
			        { -0.6000, -1.0000, -0.4000},   // RR
			        { -0.2500, -1.8000, -1.5500}    // RL
		    };
		    goto_pose_abs_rad(C2, 1500, 20);
		    // 큰 자세 변환 직후: 목표 재주입 + 감쇠 hold
		    reapply_current_goal_ticks();
		    align_Wheels();
		    set_impedance_all(1,1,1,1);             // 모두 강(stance)으로 시작
		    settle_hold(HOLD_AFTER_BIGMOVE);
		    HAL_Delay(5000);

		    // --- [2] (FL 스윙을 위한) FR/RR 쪽으로 몸 기울이기: 프리로드 ---
		    double C3[4][3] = {
			        { -0.2500, -0.6000, -0.3500},   // FL
			        { -0.6000, -1.0000, -0.4000},   // FR
			        { -0.6000, -1.0000, -0.4000},   // RR
			        { -0.2500, -1.8000, -1.5500}    // RL
		    };
		    goto_pose_abs_rad(C3, 1100, DT);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    settle_hold(HOLD_PRELOAD_MS);           // 프리로드 안정화

		    // --- [3] FL 접기(리프트 시작): 임피던스 전환(FL=스윙, 나머지=스탠스) ---
		    set_impedance_all_ramped(0, 1, 1, 1);
		    double C4[4][3] = {
			        { -1.4000, -0.6000, 0.8000},   // FL
			        { -0.6000, -1.0000, -0.4000},   // FR
			        { -0.6000, -1.0000, -0.4000},   // RR
			        { -0.2500, -1.8000, -1.5500}    // RL
		    };
		    goto_pose_abs_rad(C4,1100, DT);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    settle_hold(HOLD_APEX_MS);             // 리프트 최정점에서 짧게 정지

		    // --- [4] FL 전진/펴기(스윙 중간 ~ 말기) ---
		    //  스윙 다리 임피던스는 그대로(FL=스윙), 지지 다리는 강
		    double C5[4][3] = {
			        { -1.4000, -1.7000, -0.3000},   // FL
			        { -0.6000, -1.0000, -0.4000},   // FR
			        { -0.6000, -1.0000, -0.4000},   // RR
			        { -0.2500, -1.8000, -1.5500}    // RL
		    };
		    goto_pose_abs_rad(C5, 3000, 20);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    HAL_Delay(1500);



		    double C6[4][3] = {
			        { -1.4000, -1.7000, -0.3000},   // FL
			        { -0.7000, -1.4000, -0.7000},   // FR
			        { -0.7000, -1.4000, -0.7000},   // RR
			        { -0.2500, -1.8000, -1.5500}    // RL
		    };
		    goto_pose_abs_rad(C6, 1600, DT);

		    double C7[4][3] = { //////////몸통이동
			        { -0.7000, -1.4000, -0.7000},   // FL
			        { -0.2500, -1.8000, -1.5500},   // FR
			        { -1.4000, -1.7000, -0.3000},   // RR
			        { -0.7000, -1.4000, -0.7000}    // RL
		    };
		    goto_pose_abs_rad(C7, 4500, 30);
		    HAL_Delay(2000);

		    double C8[4][3] = {
		    		{ -0.6000, -1.1000, -0.5000},   // FL
					{ -0.2500, -1.8000, -1.5500},   // FR
			        { -1.4000, -1.7000, -0.3000},   // RR
		    		{ -0.6000, -1.1000, -0.5000}    // RL
		    };
		    goto_pose_abs_rad(C8, 1600, DT);
		    HAL_Delay(2000);


		    double C9[4][3] = {
		    		{ -0.6000, -1.0000, -0.4000},   // FL
					{ -0.2500, -1.8000, -1.5500},   // FR
		    		{ -1.3000, -0.4000, 0.9000},   // RR
		    		{ -0.6000, -1.0000, -0.4000}    // RL
		    };
		    goto_pose_abs_rad(C9, 1600, DT);

		    double C10[4][3] = {
		    		{ -0.6000, -1.0000, -0.4000},   // FL
					{ -0.2500, -1.8000, -1.5500},   // FR
		    		{ -0.2500, -0.4000, -0.1500},   // RR
		    		{ -0.6000, -1.0000, -0.4000}    // RL
		    };
		    goto_pose_abs_rad(C10, 1600, DT);

		    double C11[4][3] = {
		    		{ -0.6000, -1.0000, -0.4000},   // FL
		    		{ -0.2500, -1.8000, -1.5500},   // FR
		    		{ -0.2500, -1.8000, -1.5500},   // RR
		    		{ -0.6000, -1.0000, -0.4000}    // RL
		    };
		    goto_pose_abs_rad(C11, 1600, DT);

		    double C12[4][3] = {
		    		{ -0.6000, -1.0000, -0.4000},   // FL
		    		{ -0.2500, -0.6000, -0.3500},   // FR
		    		{ -0.2500, -1.8000, -1.5500},   // RR
		    		{ -0.6000, -1.0000, -0.4000}    // RL
		    };
		    goto_pose_abs_rad(C12, 1600, DT);

		    double C13[4][3] = {
		    		{ -0.6000, -1.0000, -0.4000},   // FL
		    		{ -1.4000, -0.6000, 0.8000},   // FR
		    		{ -0.2500, -1.8000, -1.5500},   // RR
		    		{ -0.6000, -1.0000, -0.4000}    // RL
		    };
		    goto_pose_abs_rad(C13, 1600, DT);

		    double C14[4][3] = {
		    		{ -0.6000, -1.0000, -0.4000},   // FL
		    		{ -1.4000, -1.7000, -0.3000},   // FR
		    		{ -0.2800, -1.8000, -1.5200},   // RR
		    		{ -0.6000, -1.0000, -0.4000}    // RL
		    };
		    goto_pose_abs_rad(C14, 1600, DT);

		    double C15[4][3] = {
		    		{ -0.7000, -1.4000, -0.7000},   // FL
		    		{ -1.4000, -1.7000, -0.3000},   // FR
		    		{ -0.2500, -1.8000, -1.5500},   // RR
		    		{ -0.7000, -1.4000, -0.7000}    // RL
		    };
		    goto_pose_abs_rad(C15, 1600, DT);
		    HAL_Delay(1500);

		    double C16[4][3] = { //////////////// 천천히 이동
		    		{ -0.2500, -1.8000, -1.5500},   // FL
		    		{ -0.7000, -1.4000, -0.7000},   // FR
		    		{ -0.7000, -1.4000, -0.7000},   // RR
		    		{ -1.4000, -1.7000, -0.3000}    // RL
		    };
		    goto_pose_abs_rad(C16, 4500, 30);

		    HAL_Delay(1500);

		    double C17[4][3] = {
		    		{ -0.2500, -1.8000, -1.5500},   // FL
		    		{ -0.6000, -1.0000, -0.4000},   // FR
		    		{ -0.6000, -1.0000, -0.4000},   // RR
		    		{ -1.4000, -1.7000, -0.3000}    // RL
		    };
		    goto_pose_abs_rad(C17, 1600, DT);
		    HAL_Delay(800);
		    double C18[4][3] = {
		    		{ -0.2500, -1.8000, -1.5500},   // FL
		    		{ -0.6000, -1.0000, -0.4000},   // FR
		    		{ -0.6000, -1.0000, -0.4000},   // RR
		    		{ -1.2500, -0.4000, 0.8500}    // RL
		    };
		    goto_pose_abs_rad(C18, 1600, DT);

		    double C19[4][3] = {
		    		{ -0.2500, -1.8000, -1.5500},   // FL
		    		{ -0.6000, -1.0000, -0.4000},   // FR
		    		{ -0.6000, -1.0000, -0.4000},   // RR
		    		{ -0.2500, -0.4000, -0.1500}    // RL
		    };
		    goto_pose_abs_rad(C19, 1600, DT);

		    double C20[4][3] = {
		    		{ -0.2500, -1.8000, -1.5500},   // FL
		    		{ -0.6000, -1.0000, -0.4000},   // FR
		    		{ -0.6000, -1.0000, -0.4000},   // RR
		    		{ -0.2500, -1.8000, -1.5500}    // RL
		    };
		    goto_pose_abs_rad(C20, 1600, DT);
		    reapply_current_goal_ticks();
		    align_Wheels();

		}break;


		case '-': {    // 보행 동작 (한 다리씩: FL -> RR 순)

		    servo_init();
		    ax12_force_joint_mode_all;
		    uint16_t zero[4] = {0,0,0,0};
		    send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, zero);

		    const uint32_t DT = 10;

		    // --- 안정화/임피던스용 타이밍(현장 튜닝) ---
		    const uint32_t HOLD_PRELOAD_MS     = 90;   // 프리로드(몸 기울임) 후 짧은 안정화
		    const uint32_t HOLD_APEX_MS        = 60;   // 스윙 발이 공중 최정점에서 잠깐 정지
		    const uint32_t HOLD_AFTER_LAND_MS  = 140;  // 착지 직후 잔진동 감쇠
		    const uint32_t HOLD_AFTER_BIGMOVE  = 80;   // 큰 포즈 변환 뒤 짧게(선택)

		    double C1[4][3] = {
		        { -0.5000, -1.4000, -0.9000},   // FL
		        { -0.9500, -1.4000, -0.4500},   // FR
		        { -0.9500, -1.4000, -0.4500},   // RR
		        { -0.5000, -1.4000, -0.9000}    // RL
		    };
		    goto_pose_abs_rad(C1, 1100, DT);

		    HAL_Delay(1500);

		    // --- [1] 초기 안정자세 ---
		    double C2[4][3] = {
		        { -0.5000, -1.4000, -0.9000},   // FL
		        { -0.9500, -1.4000, -0.4500},   // FR
		        { -0.9500, -1.4000, -0.4500},   // RR
		        { -0.5000, -1.4000, -0.9000}    // RL
		    };
		    goto_pose_abs_rad(C2, 800, DT);
		    // 큰 자세 변환 직후: 목표 재주입 + 감쇠 hold
		    reapply_current_goal_ticks();
		    align_Wheels();                         // 바퀴(발목 수평) 90° 토크 홀드
		    set_impedance_all(1,1,1,1);             // 모두 강(stance)으로 시작
		    settle_hold(HOLD_AFTER_BIGMOVE);
		    // --- [2] (FL 스윙을 위한) FR/RR 쪽으로 몸 기울이기: 프리로드 ---
		    //  -> 스윙 시작 전에 하중을 지지측(FR/RR)에 미리 넘겨 출렁임 감소
		    double C3[4][3] = {
		        { -0.5000, -1.4000, -0.9000},   // FL(지지 유지)
		        { -0.8500, -1.0000, -0.1500},   // FR(지지 더 받게)
		        { -0.8500, -1.0000, -0.1500},   // RR(지지 더 받게)
		        { -0.5000, -1.4000, -0.9000}    // RL(지지 유지)
		    };
		    goto_pose_abs_rad(C3, 1100, DT);
		    HAL_Delay(1000);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    settle_hold(HOLD_PRELOAD_MS);           // 프리로드 안정화

		    // --- [3] FL 접기(리프트 시작): 임피던스 전환(FL=스윙, 나머지=스탠스) ---
		    set_impedance_all_ramped(0, 1, 1, 1);
		    double C4[4][3] = {
		        { -0.2000, -0.7000, -0.5000},   // FL  (스윙: 무릎 굽힘+발목 들기)
		        { -0.8500, -1.0000, -0.1500},   // FR  (지지)
		        { -0.8500, -1.0000,  0.1500},   // RR  (지지) ※ 너 포즈에 맞춰 유지
		        { -0.5000, -1.4000, -0.9000}    // RL  (지지)
		    };
		    goto_pose_abs_rad(C4,1100, DT);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    settle_hold(HOLD_APEX_MS);             // 리프트 최정점에서 짧게 정지

		    // --- [4] FL 전진/펴기(스윙 중간 ~ 말기) ---
		    //  스윙 다리 임피던스는 그대로(FL=스윙), 지지 다리는 강
		    double C5[4][3] = {
		        { -0.9500, -0.4000,  0.5500},   // FL(스윙 전진)
		        { -0.8500, -1.0000, -0.1500},   // FR(지지)
		        { -0.8500, -1.0000, -0.1500},   // RR(지지)
		        { -0.5000, -1.4000, -0.9000}    // RL(지지)
		    };
		    goto_pose_abs_rad(C5, 1600, DT);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    // (필요시) 공중 흔들림이 있으면 소량 hold 추가
		    // settle_hold(40);

		    // --- [5] FL 내리기(착지) + 착지 감쇠 ---
		    double C6[4][3] = {
		        { -0.9500, -1.4000, -0.4500},   // FL(착지)
		        { -0.8500, -1.0000, -0.1500},   // FR(지지)
		        { -0.8500, -1.0000, -0.1500},   // RR(지지)
		        { -0.5000, -1.4000, -0.9000}    // RL(지지)
		    };
		    goto_pose_abs_rad(C6, 1900, DT);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    set_impedance_all_ramped(1,1,1,1);     // 착지 직후 전 다리 강(stance)
		    settle_hold(HOLD_AFTER_LAND_MS);       // 잔진동 감쇠

		    // --- [6] 다음 스윙(RR)을 위한 몸 이동(프리로드) ---
		    double C7[4][3] = {
		        { -0.8000, -0.8500, -0.0500},   // FL (방금 착지: 지지)
		        { -0.9500, -1.4000, -0.4500},   // FR (지지)
		        { -0.9500, -1.4000, -0.4500},   // RR (곧 스윙할 다리: 아직 지지)
		        { -0.8500, -1.0000, -0.1500}    // RL (지지)
		    };
		    goto_pose_abs_rad(C7, 1100, DT);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    settle_hold(HOLD_PRELOAD_MS);          // 프리로드 안정화
		    HAL_Delay(1000);

		    // --- [7] RR 접기(리프트 시작): 임피던스 전환(RR=스윙, 나머지=지지) ---
		    set_impedance_all_ramped(1,1,0,1);
		    double C8[4][3] = {
		        { -0.8000, -0.8500, -0.0500},   // FL(지지)
		        { -0.9500, -1.4000, -0.4500},   // FR(지지)
		        { -0.7000, -0.6000,  0.1000},   // RR(스윙: 리프트)
		        { -0.8500, -1.0000, -0.1500}    // RL(지지)
		    };
		    goto_pose_abs_rad(C8, 1100, DT);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    settle_hold(HOLD_APEX_MS);             // RR 리프트 정점 hold

		    // --- [8] RR 스윙(전진/펴기) ---
		    double C9[4][3] = {
		        { -0.8000, -0.8500, -0.0500},   // FL(지지)
		        { -0.9500, -1.4000, -0.4500},   // FR(지지)
		        { -0.5000, -0.8000, -0.3000},   // RR(스윙 전진)
		        { -0.8500, -1.0000, -0.1500}    // RL(지지)
		    };
		    goto_pose_abs_rad(C9, 1300, DT);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    // --- [9] RR 내리기(착지) + 착지 감쇠 ---
		    double C10[4][3] = {
		        { -0.8500, -1.0000, -0.1500},   // FL(지지)
		        { -0.9500, -1.4000, -0.4500},   // FR(지지)
		        { -0.5000, -1.4000, -0.9000},   // RR(착지)
		        { -0.8500, -1.0000, -0.1500}    // RL(지지)
		    };
		    goto_pose_abs_rad(C10, 1500, DT);
		    reapply_current_goal_ticks();
		    align_Wheels();
		    set_impedance_all_ramped(1,1,1,1);     // 착지 직후 전 다리 강
		    settle_hold(HOLD_AFTER_LAND_MS);       // 잔진동 감쇠

		    // 다음 사이클로 이어가려면 여기서 다시 [2]~[9] 반복
		    // 끝낼 때는 프로파일/게인 복원 등 수행
		} break;




		case 't' :{
			uint32_t t_s_2[1];
			uint16_t t_s_1[1];
			send_sync_write_2(ALL, DXL_2_Goal_Velocity, 4, angle_speed_Hip, angle_speed_Knee, angle_speed_Ankle);
			HAL_Delay(10);
			send_sync_write_1(ALL, DXL_1_Goal_Torque_Limit, 2, torque_limit);
			HAL_Delay(10);
			if(torque_status == 0)
			{
				UART_Transmit_DMA(&huart1, (uint8_t*)"***Torque On***\r\n", 17);
				torque_status = 1;
				t_s_2[0] = 1;
				t_s_1[0] = 1;
				for(int i = 1; i <= 4; i++)
				{
					send_sync_write_1(i, DXL_1_Torque_Enable, 1, t_s_1);
					HAL_Delay(10);
					send_sync_write_2(i, DXL_2_Torque_Enable, 1, t_s_2, t_s_2, t_s_2);
					HAL_Delay(10);
				}
			}
			else{
				UART_Transmit_DMA(&huart1, (uint8_t*)"***Torque Off***\r\n", 18);
				torque_status = 0;
				t_s_2[0] = 0;
				t_s_1[0] = 0;

				for(int i = 1; i <= 4; i++)
				{
					send_sync_write_1(i, DXL_1_Torque_Enable, 1, t_s_1);
					HAL_Delay(10);
					send_sync_write_2(i, DXL_2_Torque_Enable, 1, t_s_2, t_s_2, t_s_2);
					HAL_Delay(10);
				}
			}
		} break;

		case ']':{
			UART_Transmit_DMA(&huart1, (uint8_t*)"***Angle_Speed_reset***\r\n", 25);
			send_sync_write_2(ALL, DXL_2_Goal_Velocity, 4, angle_speed_Hip, angle_speed_Knee, angle_speed_Ankle);
			HAL_Delay(10);

		} break;


		default:
			UART_Transmit_DMA(&huart1, (uint8_t*)"?", 1);
			HAL_Delay(10);
			break;
		}
		new_data_received[0] = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		//HAL_UART_Receive_DMA(huart, pc_rx_buf, 1);
		return;
	}
	if (huart->Instance == USART2) {
		new_data_received[1] = 1;	}
	if (huart->Instance == USART3) {
		new_data_received[2] = 1;
		HAL_UART_Receive_DMA(huart, uart3_rx_buf, 1);
	}
	if (huart->Instance == UART4) {
		new_data_received[3] = 1;
		HAL_UART_Receive_DMA(huart, uart4_rx_buf, 1);
	}
	if (huart->Instance == UART5) {
		new_data_received[4] = 1;
		HAL_UART_Receive_DMA(huart, uart5_rx_buf, 1);
	}
}

void UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	if (!uart_tx_busy[0] && huart->Instance == USART1) {
		uart_tx_busy[0] = 1;
		memcpy(uart1_tx_dma_buf, pData, Size);
		HAL_UART_Transmit_DMA(huart, uart1_tx_dma_buf, Size);
	}
	if (!uart_tx_busy[1] && huart->Instance == USART2) {
		uart_tx_busy[1] = 1;
		memcpy(uart2_tx_dma_buf, pData, Size);
		HAL_UART_Transmit_DMA(huart, uart2_tx_dma_buf, Size);
	}
	if (!uart_tx_busy[2] && huart->Instance == USART3) {
		uart_tx_busy[2] = 1;
		memcpy(uart3_tx_dma_buf, pData, Size);
		HAL_UART_Transmit_DMA(huart, uart3_tx_dma_buf, Size);
	}
	if (!uart_tx_busy[3] && huart->Instance == UART4) {
		uart_tx_busy[3] = 1;
		memcpy(uart4_tx_dma_buf, pData, Size);
		HAL_UART_Transmit_DMA(huart, uart4_tx_dma_buf, Size);
	}
	if (!uart_tx_busy[4] && huart->Instance == UART5) {
		uart_tx_busy[4] = 1;
		memcpy(uart5_tx_dma_buf, pData, Size);
		HAL_UART_Transmit_DMA(huart, uart5_tx_dma_buf, Size);
	}

}

int get_uart_index(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) return 0;
	if (huart->Instance == USART2) return 1;
	if (huart->Instance == USART3) return 2;
	if (huart->Instance == UART4) return 3;
	if (huart->Instance == UART5) return 4;
	return -1; // 에러
}

void uart_rx_dma1_handler() {
	static uint16_t old_pos = 0;
	uint16_t pos = DMA_BUF_SIZE - huart1.hdmarx->Instance->NDTR;

	if (pos != old_pos) {
		if (pos > old_pos) {
			for (uint16_t i = old_pos; i < pos; i++) {
				input_char = pc_rx_buf[i];
				new_data_received[0] = 1;
			}
		} else {
			for (uint16_t i = old_pos; i < DMA_BUF_SIZE; i++) {
				input_char = pc_rx_buf[i];
				new_data_received[0] = 1;
			}
			for (uint16_t i = 0; i < pos; i++) {
				input_char = pc_rx_buf[i];
				new_data_received[0] = 1;
			}
		}
		old_pos = pos;
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		uart_tx_busy[0] = false;
	}
	else if(huart->Instance == USART2)
	{
		if (huart->Instance == USART2) {
		        uart_tx_busy[1] = false;
		        return;
		    }
	}
	else if(huart->Instance == USART3)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
		uart_tx_busy[2] = false;
	}
	else if(huart->Instance == UART4)
	{
		uart_tx_busy[3] = false;
	}
	else if(huart->Instance == UART5)
	{
		uart_tx_busy[4] = false;
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
