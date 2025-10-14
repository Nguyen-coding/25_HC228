/*
 * dlx_2_0.h
 *
 */

#ifndef INC_DXL_2_0_H_
#define INC_DXL_2_0_H_

#define leg_count 4
#define all_wheel_motor 4
#define all_hip_knee_ankle_motor 12

#define one_leg_wheel_motor 1
#define one_leg_hip_knee_ankle_motor 3


// 기본 최대 파라미터 크기 (일반 명령용)
#define DXL_STD_PARAMS 64

// Sync 명령용 파라미터 크기
#define DXL_SYNC_PARAMS 128  // 8모터 × (ID + 데이터(약 9바이트)) + 주소/길이

// Bulk 명령용 파라미터 크기
#define DXL_BULK_PARAMS 256  // 8모터 × (ID + 주소 + 길이 + 데이터)

#define DXL_DIR_GPIO_Port   GPIOD
#define DXL_DIR_Pin         GPIO_PIN_4

#define DXL_DIR_TX_ACTIVE_HIGH  1

enum DxlInst{
	DXL_INST_PING = 0x01,
	DXL_INST_READ = 0x02,
	DXL_INST_WRITE = 0x03,
	DXL_INST_REG_WRITE = 0x04,
	DXL_INST_ACTION = 0x05,
	DXL_INST_FACTORY_RESET = 0x06,
	DXL_INST_REBOOT = 0x08,
	DXL_INST_CLEAR = 0x10,
	DXL_INST_STATUS = 0x55,
	DXL_INST_SYNC_READ = 0x82,
	DXL_INST_SYNC_WRITE = 0x83,
	DXL_INST_BULK_READ = 0x92,
	DXL_INST_BULK_WRITE = 0x93,
};

enum Dxl_2_0_Addr{
	DXL_2_Torque_Enable = 64,
	DXL_2_LED = 65,
	DXL_2_Goal_PWM = 100,
	DXL_2_Goal_Velocity = 112,
	DXL_2_Goal_Position = 116,
	DXL_2_Present_Current = 126,
	DXL_2_Present_Position = 132,
	DXL_2_Present_Temperature = 146,
	DXL_2_PWM_Limit            = 36,
	DXL_2_Current_Limit        = 38,
	DXL_2_Position_D_Gain      = 80,
	DXL_2_Position_I_Gain      = 82,
	DXL_2_Position_P_Gain      = 84,
	DXL_2_Profile_Acceleration = 108,
	DXL_2_Profile_Velocity     = 112,
    DXL_2_Hardware_Error_Status = 70,
    DXL_2_Present_Input_Voltage = 144,
    DXL_2_Model_Number        = 0,
    DXL_2_Status_Return_Level  = 68,
    DXL_2_Return_Delay_Time    = 9
};

enum Dxl_1_0_Addr{
	DXL_1_Torque_Enable = 24,//1
	DXL_1_LED = 25,//1
	DXL_1_MOVING_SPEED = 32,
	DXL_1_Goal_Torque_Limit = 34,//1
	DXL_1_Goal_Position = 30,//2
	DXL_1_Present_Load = 40,//2
	DXL_1_Present_Position = 36,//2
	DXL_1_Present_Temperature = 43,//2
};

enum PacketType {
	PACKET_SINGLE = 1,
	PACKET_SYNC_WIRTE = 2,
	PACKET_SYNC_READ = 3,
	PACKET_BULK_WIRTE = 4,
	PACKET_BULK_READ = 5,
	PACKET_STATUS = 6
};

typedef enum {
	ALL = 0,
	FR = 2,
	FL = 1,
	RR = 3,
	RL = 4,
} SendLegType;

void dxl_set_leg_pid_2(SendLegType leg, uint16_t p[3], uint16_t i[3], uint16_t d[3]);
void dxl_set_leg_pwm_limit_2(SendLegType leg, uint16_t pwm_lim[3]);
void dxl_set_leg_profile(SendLegType leg, uint32_t vel[3], uint32_t acc[3]);

typedef struct {
	uint32_t hip_data[4];
	uint8_t hip_Hori_data[4];
	uint32_t knee_data[4];
	uint16_t ankle_data[4];
	uint8_t ankle_Hori_data[4];
	uint16_t wheel_data[4];
} data_set_all;

void dxl_set_leg_pid_2(SendLegType leg, uint16_t p[3], uint16_t i[3], uint16_t d[3]);
void dxl_set_leg_pwm_limit_2(SendLegType leg, uint16_t pwm_lim[3]);
void dxl_set_leg_profile(SendLegType leg, uint32_t vel[3], uint32_t acc[3]);


//-------------------single------------------------//
typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;
	uint16_t length;
	uint8_t inst;
	uint8_t params[DXL_STD_PARAMS];
	uint16_t crc;
} dxl_packet_2_0;

// 프로토콜 1.0 Instruction 패킷
typedef struct {
	uint8_t header[2];
	uint8_t id;
	uint8_t length;
	uint8_t inst;
	uint8_t params[DXL_STD_PARAMS];
	uint8_t checksum;
} dxl_packet_1_0;
//-------------------single------------------------//



//--------------------Sync-------------------------//
// Sync Write 패킷 구조체
typedef struct {
	uint8_t header[2];
	uint8_t id;
	uint8_t length;
	uint8_t inst;
	uint8_t start_addr;
	uint8_t data_len;
	uint8_t params[DXL_SYNC_PARAMS];
	uint8_t checksum;
} dxl_sync_write_1_0;

typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;
	uint16_t length;
	uint8_t inst;
	uint16_t start_addr;
	uint16_t data_len;
	uint8_t params[DXL_SYNC_PARAMS];
	uint16_t crc;
} dxl_sync_write_2_0;

// Sync Read 패킷 구조체 (프로토콜 2.0만 가능)
typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;
	uint16_t length;
	uint8_t inst;
	uint16_t start_addr;
	uint16_t data_len;
	uint8_t params[DXL_SYNC_PARAMS];
	uint16_t crc;
} dxl_sync_read_2_0;
//--------------------Sync-------------------------//



//--------------------Bulk-------------------------//
typedef struct {
	uint8_t header[2];
	uint8_t id;
	uint8_t length;
	uint8_t inst;
	uint8_t params[DXL_BULK_PARAMS];
	uint8_t checksum;
} dxl_bulk_read_1_0;

typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;
	uint16_t length;
	uint8_t inst;
	uint8_t params[DXL_BULK_PARAMS];
	uint16_t crc;
} dxl_bulk_write_2_0;

typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;
	uint16_t length;
	uint8_t inst;
	uint8_t params[DXL_BULK_PARAMS];
	uint16_t crc;
} dxl_bulk_read_2_0;
//--------------------Bulk-------------------------//



//-------------------Status------------------------//
// 프로토콜 2.0 Status 패킷
typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;
	uint16_t length;
	uint8_t inst;
	uint8_t error;
	uint8_t params[DXL_BULK_PARAMS];
	uint16_t crc;
} dxl_status_2_0;

// 프로토콜 1.0 Status 패킷
typedef struct {
	uint8_t header[2];
	uint8_t id;
	uint8_t length;
	uint8_t error;
	uint8_t params[DXL_BULK_PARAMS];
	uint8_t checksum;
} dxl_status_1_0;

typedef struct {
	uint8_t hip;
	uint8_t knee;
	uint8_t ankle;
	uint8_t wheel;
} LegMotors;

#define BASE_WIDTH_M              0.26f

// m/s → AX-12 moving speed (tick/s) 변환 계수
#define K_MPS_TO_DXL              2100.0f    // 1 m/s ≈ 700 ticks/s (대략값)

#define K_WZ_TO_DXL   (K_MPS_TO_DXL * (BASE_WIDTH_M * 0.5f))

static inline uint16_t dxl_speed_cmd_from_signed(float s)
{
  float mag = fabsf(s);
  uint16_t raw = (uint16_t)(mag + 0.5f);
  if (raw > 1023) raw = 1023;
  if (s < 0) raw |= 0x0400;   // 0x0400: CW 비트
  return raw;
}

void dxl_torque_set(uint8_t send_leg_type, uint8_t on1, uint8_t on2);
uint16_t clc_speed_1(int16_t wheel_speed);
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
uint8_t calculate_checksum_1_0(uint8_t *data, uint16_t length);
dxl_sync_write_1_0 creat_sync_packet_1_0(void);
dxl_sync_write_2_0 creat_sync_packet_2_0(void);
dxl_bulk_read_1_0 creat_bulk_packet_1_0(void);
dxl_bulk_read_2_0 creat_bulk_packet_2_0(void);
uint16_t serialize_sync_write_2_0(dxl_sync_write_2_0 *packet, uint8_t *id_array, uint8_t id_count, uint32_t *data_array, uint8_t *buffer, uint16_t buffer_size);
void send_sync_write_1(uint8_t send_leg_type, uint16_t addr, uint16_t data_len, const uint16_t *wheel_data);

void send_sync_write_2(uint8_t send_leg_type, uint16_t addr, uint16_t data_len, const uint32_t *hip_data, const uint32_t *knee_data, const uint32_t *ankle_data);
void uart_transmit_packet(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
extern LegMotors legs[5];
bool dxl_read(uint8_t id, uint16_t addr, uint16_t len, uint8_t *out, uint8_t *err);
bool dxl_read_4byte(uint8_t id, uint16_t addr, int32_t *outVal, uint8_t *err);
bool dxl_read_1byte(uint8_t id, uint16_t addr, uint8_t *outVal, uint8_t *err);
void dxl_write_1byte(uint8_t id, uint16_t addr, uint8_t data, uint8_t *err);
void dxl_tx_enable(void);
void dxl_rx_enable(void);
void dxl_wait_tx_complete(UART_HandleTypeDef *huart);
void dxl_make_quiet_after_boot(void);
void ax12_force_wheel_mode_all(void);
void ax12_set_punch_all(uint16_t punch);
void ax12_set_torque_limit_all(uint16_t tl);
void ax12_force_joint_mode_all(void);
bool dxl_read_2byte(uint8_t id, uint16_t addr, uint16_t *outVal, uint8_t *err);
void ax1_tx_enable(void);
void ax1_rx_enable(void);
uint8_t ax1_checksum(const uint8_t *from_id, int len);
void ax12_write_n(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t n);
bool ax12_read_n(uint8_t id, uint8_t addr, uint8_t n, uint8_t* out);
bool ax12_read_u16(uint8_t id, uint16_t addr, uint16_t* out);
void ax12_write_u16(uint8_t id, uint8_t addr, uint16_t v);
void ax12_write_u8(uint8_t id, uint8_t addr, uint8_t v);


#endif /* INC_DXL_2_0_H_ */
