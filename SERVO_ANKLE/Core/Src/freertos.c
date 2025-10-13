/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "rcl/error_handling.h"
#include <rmw/qos_profiles.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/time.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "usart.h"
#include "dma_transport.h"
#include "Horizontal_Servo.h"
#include "dxl_2_0.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;

extern void zero_turn(void);
extern void turn_left(void);
extern void turn_right(void);
extern void align_Wheels(void);
extern void ax12_force_wheel_mode_all(void);

/*  기본 파라미터  */
#define VX_SCALE_TO_MPS            1.0f
#define WZ_SCALE_TO_RADPS          1.0f
#define MODE_THRESH_VX_MPS         0.02f
#define MODE_THRESH_WZ_RADPS       0.50f
#define SPEED_STEP                 45u
#define SPEED_MAX                  900u
#define CMD_TIMEOUT_MS             800u
#define CTRL_PERIOD_MS             10u
#define RECONNECT_PERIOD_MS        1000u
#define AGENT_WAIT_TIMEOUT_MS      2000u
#define AGENT_RETRY_PERIOD_MS      200u

/*  조향/제로턴 판정  */
#define WZ_STEER_THRESH            0.05f   // 이 이상이면 조향(좌/우)
#define WZ_ZERO_TURN_THRESH        0.20f   // v≈0 & 이 이상이면 제로턴
#define V_MIN_FOR_STEER            0.08f   // 너무 느리면 조향 효과 약 → 바닥 보장
/* === 모드별 고정 속도 선언 === */
#define DXL_TICKS_STRAIGHT_MAX  SPEED_MAX   // 직진은 v 기반 + 이 상한으로 클램프
#define DXL_TICKS_STEER_LEFT    330u        // 좌회전 고정 속도
#define DXL_TICKS_STEER_RIGHT   330u        // 우회전 고정 속도
#define DXL_TICKS_ZERO_TURN     450u        // 제로턴 회전 속도
enum { IDX_FL = 0, IDX_FR = 1, IDX_RR = 2, IDX_RL = 3 };
static const int8_t FWD_SIGN[4] = { +1, -1, -1, +1 }; // FL, FR, RR, RL

typedef struct {
  float    v_mps;
  float    w_radps;
  uint32_t stamp_ms;
} cmdvel_setpoint_t;

static volatile cmdvel_setpoint_t g_sp = {0};
volatile uint8_t g_auto_mode = 0;
static uint8_t  s_ros_online = 0;
static uint32_t s_last_ping  = 0;

static inline uint16_t u16_min(uint16_t a, uint16_t b){return (a<b)?a:b;}
static inline uint16_t clamp_u16(uint16_t x,uint16_t lo,uint16_t hi){return(x<lo)?lo:((x>hi)?hi:x);}

extern void uart_rx_dma1_handler(void);
extern void process_input(void);

/* USER CODE END PTD */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes={
  .name="defaultTask",
  .stack_size=3000*4,
  .priority=(osPriority_t)osPriorityNormal,
};

/* USER CODE BEGIN PV */
static rcl_subscription_t cmd_sub;
static geometry_msgs__msg__Twist cmd_msg;
static rclc_executor_t executor;
static rcl_node_t node;
static rclc_support_t support;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static inline uint16_t sp2raw(int16_t sp) {
  uint16_t mag = (sp >= 0) ? (uint16_t)sp : (uint16_t)(-sp);
  if (sp < 0) mag |= 0x0400;
  return mag;
}

static inline void send_raw4_and_log(uint16_t w0, uint16_t w1, uint16_t w2, uint16_t w3, const char* tag){
  uint16_t a[4]={w0,w1,w2,w3};
  send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, a);
  char b[96];
  int n = snprintf(b, sizeof(b), "[RAW %s] FL=%04X FR=%04X RR=%04X RL=%04X\r\n", tag, a[0], a[1], a[2], a[3]);
  HAL_UART_Transmit(&huart1,(uint8_t*)b,n,10);
}


static void cmd_vel_callback(const void* msgin){
  const geometry_msgs__msg__Twist* m=(const geometry_msgs__msg__Twist*)msgin;
  g_sp.v_mps   =(float)m->linear.x  * VX_SCALE_TO_MPS;
  g_sp.w_radps =(float)m->angular.z * WZ_SCALE_TO_RADPS;
  g_sp.stamp_ms=osKernelGetTickCount();
  char buf[96];
  int n=snprintf(buf,sizeof(buf),"[cmd_vel] vx=%.2f, wz=%.2f\r\n",(double)m->linear.x,(double)m->angular.z);
  HAL_UART_Transmit(&huart1,(uint8_t*)buf,n,10);
}

void StartDefaultTask(void *argument);

static inline void send_wheels_like_manual(int16_t spd_fl,
                                           int16_t spd_fr,
                                           int16_t spd_rr,
                                           int16_t spd_rl)
{
    uint16_t word4[4];
    word4[IDX_FL] = dxl_speed_cmd_from_signed(spd_fl);
    word4[IDX_FR] = dxl_speed_cmd_from_signed(spd_fr);
    word4[IDX_RR] = dxl_speed_cmd_from_signed(spd_rr);
    word4[IDX_RL] = dxl_speed_cmd_from_signed(spd_rl);

    send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, word4);
}

/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void){defaultTaskHandle=osThreadNew(StartDefaultTask,NULL,&defaultTask_attributes);}

/* USER CODE BEGIN Header_StartDefaultTask */
void StartDefaultTask(void* argument)
{
  /* ── 초기화 ───────────────────────────────────────────── */
  init_servo_set();

  /* micro-ROS transport (UART4) */
  rmw_uros_set_custom_transport(
      true,(void*)&huart4,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);

  /* Agent 연결 대기 */
  osDelay(200);
  uint32_t t0=osKernelGetTickCount();
  while(rmw_uros_ping_agent(100,50)!=RMW_RET_OK){
    uart_rx_dma1_handler();   //DMA RX 링버퍼 펌핑
    process_input();
    if(!g_auto_mode)break;
    if((osKernelGetTickCount()-t0)>AGENT_WAIT_TIMEOUT_MS)break;
    osDelay(AGENT_RETRY_PERIOD_MS);
  }

  /* ── micro-ROS init ───────────────────────────────────── */
  if(rmw_uros_ping_agent(100,50)==RMW_RET_OK){
    rcl_allocator_t allocator=rcl_get_default_allocator();
    if(rclc_support_init(&support,0,NULL,&allocator)!=RCL_RET_OK) goto mr_fail;
    if(rclc_node_init_default(&node,"stm32_robot_node","",&support)!=RCL_RET_OK) goto mr_fail;

    rmw_qos_profile_t qos=rmw_qos_profile_default;
    qos.reliability=RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;  // 퍼블리셔와 매칭
    qos.history    =RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos.depth      =10;

    /* ★ 메시지 버퍼 초기화 */
    geometry_msgs__msg__Twist__init(&cmd_msg);

    if(rclc_subscription_init(&cmd_sub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),
        "/cmd_vel",&qos)!=RCL_RET_OK) goto mr_fail;

    if(rclc_executor_init(&executor,&support.context,1,&allocator)!=RCL_RET_OK) goto mr_fail;
    if(rclc_executor_add_subscription(&executor,&cmd_sub,&cmd_msg,&cmd_vel_callback,ON_NEW_DATA)!=RCL_RET_OK) goto mr_fail;

    s_ros_online=1;
    HAL_UART_Transmit(&huart1,(uint8_t*)"agent connected\r\n",17,50);
  }else{
    HAL_UART_Transmit(&huart1,(uint8_t*)"agent offline, manual mode\r\n",28,50);
  }

  /* ── 제어 루프 ───────────────────────────────────────── */
  uint32_t last_ctrl=osKernelGetTickCount();
  static uint8_t  mode=0;             // 0=STOP, 1=STRAIGHT, 2=ZERO_TURN, 3=STEER_LEFT, 4=STEER_RIGHT
  static uint16_t ss1[4]={0,0,0,0};   // 직진 램프 버퍼
  static uint8_t  last_auto=0;
  static uint8_t  straight_servo_inited=0;
  static uint32_t last_log=0;

  /* 명령 타임아웃/홀드 설정 */
  #undef  CMD_TIMEOUT_MS
  #define CMD_TIMEOUT_MS            1500u     // 콜백 잠깐 막혀도 바로 STOP 안나오게
  #define HOLD_WHEN_ACTIVE_MS       3000u     // 2/3/4 모드시 마지막 명령 유지 허용 시간

  for(;;){
    /* 1) 콘솔/수동 입력 + DMA RX 펌핑 */
    uart_rx_dma1_handler();  // micro-ROS RX 처리
    process_input();

    /* 2) micro-ROS spin */
    if(s_ros_online){
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }

    /* 3) AUTO에서만 Agent 재연결 시도 */
    if(g_auto_mode && !s_ros_online){
      uint32_t tick=osKernelGetTickCount();
      if((tick-s_last_ping)>RECONNECT_PERIOD_MS){
        if(rmw_uros_ping_agent(30,5)==RMW_RET_OK){
          rcl_allocator_t allocator=rcl_get_default_allocator();
          if(rclc_support_init(&support,0,NULL,&allocator)==RCL_RET_OK &&
             rclc_node_init_default(&node,"stm32_robot_node","",&support)==RCL_RET_OK){
            rmw_qos_profile_t qos=rmw_qos_profile_default;
            qos.reliability=RMW_QOS_POLICY_RELIABILITY_RELIABLE;
            qos.history    =RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            qos.depth      =10;
            geometry_msgs__msg__Twist__init(&cmd_msg);
            if(rclc_subscription_init(&cmd_sub,&node,
                 ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),
                 "/cmd_vel",&qos)==RCL_RET_OK &&
               rclc_executor_init(&executor,&support.context,1,&allocator)==RCL_RET_OK &&
               rclc_executor_add_subscription(&executor,&cmd_sub,&cmd_msg,&cmd_vel_callback,ON_NEW_DATA)==RCL_RET_OK){
              s_ros_online=1;
              HAL_UART_Transmit(&huart1,(uint8_t*)"reconnected!\r\n",14,20);
            }
          }
        }
        s_last_ping=tick;
      }
    }

    /* 4) AUTO 토글 엣지 */
    if(last_auto!=g_auto_mode){
      if(g_auto_mode){
        align_Wheels();
        ax12_force_wheel_mode_all();
        HAL_Delay(50);

      }else{
        uint16_t stop4[4]={0,0,0,0};
        send_sync_write_1(ALL,DXL_1_MOVING_SPEED,2,stop4);
        HAL_UART_Transmit(&huart1,(uint8_t*)"AUTO: OFF\r\n",11,20);
      }
      last_auto=g_auto_mode;
    }

    if(!g_auto_mode){ osDelay(CTRL_PERIOD_MS); continue; }

    /* 5) 100 Hz 제어 */
    uint32_t now=osKernelGetTickCount();
    if((now-last_ctrl)>=CTRL_PERIOD_MS){
      /* 유효성/홀드 계산 */
      uint32_t age = now - g_sp.stamp_ms;
      uint8_t  valid = (age <= CMD_TIMEOUT_MS);
      uint8_t  hold_ok = (!valid && (mode==2 || mode==3 || mode==4) && (age <= HOLD_WHEN_ACTIVE_MS));
      if(hold_ok) valid = 1;  // 활성 모드에선 잠시 콜백 끊겨도 유지

      float v = valid ? g_sp.v_mps   : 0.0f;
      float w = valid ? g_sp.w_radps : 0.0f;

      // 200ms마다 상태 로그
      if(now-last_log>=200){
        char b[128];
        int n=snprintf(b,sizeof(b),
                       "[cmd_vel] valid=%u age=%lu ms vx=%.3f wz=%.3f (mode=%u)\r\n",
                       (unsigned)(valid?1:0),(unsigned long)age,(double)v,(double)w,(unsigned)mode);
        HAL_UART_Transmit(&huart1,(uint8_t*)b,n,10);
        last_log=now;
      }

      /* 모드 판정 */
      uint8_t new_mode=0;
      float vx_abs=fabsf(v), wz_abs=fabsf(w);
      if(valid && (vx_abs<MODE_THRESH_VX_MPS && wz_abs>WZ_ZERO_TURN_THRESH)) new_mode=2;
      else if(valid && (vx_abs>=MODE_THRESH_VX_MPS)){
        if      (w> WZ_STEER_THRESH)  new_mode=3;
        else if (w<-WZ_STEER_THRESH)  new_mode=4;
        else                          new_mode=1;
      }else new_mode=0;

      if (new_mode != mode) {

          switch (new_mode) {

              case 2:  // ZERO_TURN
                  zero_turn();
                  straight_servo_inited = 0;   // 다음 조향 시 서보 재정렬 필요
                  HAL_UART_Transmit(&huart1, (uint8_t*)"MODE: ZERO_TURN\r\n", 18, 10);
                  break;

              case 3:  // TURN_LEFT
                  if (!straight_servo_inited) {
                      servo_init();  // 제로턴 후 방향 복원
                      straight_servo_inited = 1;
                      HAL_UART_Transmit(&huart1, (uint8_t*)"servo_init() before TURN_LEFT\r\n", 31, 10);
                  }
                  turn_left();
                  HAL_UART_Transmit(&huart1, (uint8_t*)"MODE: STEER_LEFT\r\n", 19, 10);
                  break;

              case 4:  // TURN_RIGHT
                  if (!straight_servo_inited) {
                      servo_init();  // 제로턴 후 방향 복원
                      straight_servo_inited = 1;
                      HAL_UART_Transmit(&huart1, (uint8_t*)"servo_init() before TURN_RIGHT\r\n", 32, 10);
                  }
                  turn_right();
                  HAL_UART_Transmit(&huart1, (uint8_t*)"MODE: STEER_RIGHT\r\n", 20, 10);
                  break;

              case 1:  // STRAIGHT
                  align_Wheels();
                  if (!straight_servo_inited) {
                      servo_init();  // 직진 진입 시 1회
                      straight_servo_inited = 1;
                      HAL_UART_Transmit(&huart1, (uint8_t*)"servo_init() at STRAIGHT\r\n", 26, 10);
                  }
                  HAL_UART_Transmit(&huart1, (uint8_t*)"MODE: STRAIGHT\r\n", 17, 10);
                  break;

              default:  // STOP
                  align_Wheels();
                  straight_servo_inited = 0;
                  HAL_UART_Transmit(&huart1, (uint8_t*)"MODE: STOP\r\n", 12, 10);
                  break;
          }

          mode = new_mode;
      }


      /* 바퀴 속도 산출 */
      int16_t sp_fl=0, sp_fr=0, sp_rr=0, sp_rl=0;
      if(mode==2){
        int sgnZ=(w>=0.0f)?-1:+1;
        int16_t z=(int16_t)DXL_TICKS_ZERO_TURN * sgnZ;
        sp_fl=z; sp_fr=z; sp_rr=z; sp_rl=z;
      }else if(mode==3){ // 좌회전: 고정속도, 진행방향은 v 부호
        int16_t base=(int16_t)DXL_TICKS_STEER_LEFT * ((v>=0.0f)?+1:-1);
        sp_fl= base * FWD_SIGN[IDX_FL];
        sp_fr= base * FWD_SIGN[IDX_FR];
        sp_rr= base * FWD_SIGN[IDX_RR];
        sp_rl= base * FWD_SIGN[IDX_RL];
      }else if(mode==4){ // 우회전
        int16_t base=(int16_t)DXL_TICKS_STEER_RIGHT * ((v>=0.0f)?+1:-1);
        sp_fl= base * FWD_SIGN[IDX_FL];
        sp_fr= base * FWD_SIGN[IDX_FR];
        sp_rr= base * FWD_SIGN[IDX_RR];
        sp_rl= base * FWD_SIGN[IDX_RL];
      }else if(mode==1){ // 직진: v 기반 + 상한
        float fwd=K_MPS_TO_DXL * v;
        float lim=(float)DXL_TICKS_STRAIGHT_MAX;
        if(fabsf(fwd)>lim) fwd=copysignf(lim,fwd);
        int16_t base=(int16_t)lroundf(fwd);
        sp_fl= base * FWD_SIGN[IDX_FL];
        sp_fr= base * FWD_SIGN[IDX_FR];
        sp_rr= base * FWD_SIGN[IDX_RR];
        sp_rl= base * FWD_SIGN[IDX_RL];
      } // STOP=0

      /* 200ms마다 휠속도 로그 */
      if(now-last_log>=200){
        char bs[128];
        int ns=snprintf(bs,sizeof(bs),
                        "[mode %u] sp FL=%d FR=%d RR=%d RL=%d\r\n",
                        (unsigned)mode,(int)sp_fl,(int)sp_fr,(int)sp_rr,(int)sp_rl);
        HAL_UART_Transmit(&huart1,(uint8_t*)bs,ns,10);
      }

      /* AX-12 포맷 변환 */
      uint16_t target[4];
      target[IDX_FL]=dxl_speed_cmd_from_signed(sp_fl);
      target[IDX_FR]=dxl_speed_cmd_from_signed(sp_fr);
      target[IDX_RR]=dxl_speed_cmd_from_signed(sp_rr);
      target[IDX_RL]=dxl_speed_cmd_from_signed(sp_rl);

      /* 전송: 직진만 램프, 나머지는 즉시 */
      if(mode==1){
        for(int i=0;i<4;i++){
          uint16_t cur_mag=ss1[i]&0x03FF, cur_dir=ss1[i]&0x0400;
          uint16_t tar_mag=target[i]&0x03FF, tar_dir=target[i]&0x0400;
          if(cur_dir!=tar_dir && cur_mag>SPEED_STEP){
            cur_mag-=u16_min(cur_mag,SPEED_STEP);
          }else{
            if      (cur_mag<tar_mag) cur_mag+=u16_min((uint16_t)(tar_mag-cur_mag),SPEED_STEP);
            else if (cur_mag>tar_mag) cur_mag-=u16_min((uint16_t)(cur_mag-tar_mag),SPEED_STEP);
            cur_dir=tar_dir;
          }
          ss1[i]=(cur_mag|cur_dir);
        }
        send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, ss1);
      }else{
        send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, target);
        /* 다음 직진 전환 시 튀지 않게 ss1 동기화 */
        ss1[0]=target[0]; ss1[1]=target[1]; ss1[2]=target[2]; ss1[3]=target[3];
      }

      last_ctrl=now;
    } /* 100Hz */

    osDelay(CTRL_PERIOD_MS);
  }

  /* 실패 처리 */
mr_fail:
  s_ros_online=0;
  HAL_UART_Transmit(&huart1,(uint8_t*)"micro-ROS init failed -> offline\r\n",35,50);
  { uint16_t zero[4]={0,0,0,0}; send_sync_write_1(ALL,DXL_1_MOVING_SPEED,2,zero); }
}

/* USER CODE END Header_StartDefaultTask */

/* USER CODE BEGIN MicroROS callback */
/* USER CODE END MicroROS callback */
