/*
 * dxl_2_0.c
 *
 */
#include "main.h"
#include "dxl_2_0.h"
#include "stdint.h"
#include "usart.h"
#include <stdbool.h>
#include <string.h>  // memcpy
#include <stdio.h>   // sprintf

#define DXL_PACKET_LEN   512


static uint8_t dxl_rx_buf[DXL_PACKET_LEN];

#ifndef DXL_TX_DMA_BUF_SIZE
#define DXL_TX_DMA_BUF_SIZE 256   // 필요시 256~512로 여유 있게
#endif
static uint8_t dxl_tx_dma_buf[DXL_TX_DMA_BUF_SIZE];


extern bool uart_tx_busy[5];        // main.c에 있는 배열
extern UART_HandleTypeDef huart1;   // 로그용 (UART1)
static inline void print1(const char* s){
    HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 50);
}


void dxl_tx_enable(void) {
#if DXL_DIR_TX_ACTIVE_HIGH
    HAL_GPIO_WritePin(DXL_DIR_GPIO_Port, DXL_DIR_Pin, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(DXL_DIR_GPIO_Port, DXL_DIR_Pin, GPIO_PIN_RESET);
#endif
}

void dxl_rx_enable(void) {
#if DXL_DIR_TX_ACTIVE_HIGH
    HAL_GPIO_WritePin(DXL_DIR_GPIO_Port, DXL_DIR_Pin, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(DXL_DIR_GPIO_Port, DXL_DIR_Pin, GPIO_PIN_SET);
#endif
}



volatile uint8_t dxl_bus_lock = 0;

static inline int dxl_bus_acquire(uint32_t tout_ms){
    uint32_t t0 = HAL_GetTick();
    for(;;){
        __disable_irq();
        if (!dxl_bus_lock){ dxl_bus_lock = 1; __enable_irq(); return 1; }
        __enable_irq();
        if (HAL_GetTick()-t0 > tout_ms) return 0;
    }
}
static inline void dxl_bus_release(void){
    __disable_irq(); dxl_bus_lock = 0; __enable_irq();
}


static bool dxl_wait_tx_complete_tmo(UART_HandleTypeDef *huart, uint32_t timeout_ms)
 {
     uint32_t t0 = HAL_GetTick();
     if (huart->Instance == USART2) {
         while (uart_tx_busy[1]) {
             if (HAL_GetTick()-t0 > timeout_ms) return false;
         }
     }
     while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {
         if (HAL_GetTick()-t0 > timeout_ms) return false;
     }
     for (volatile int i=0;i<1200;++i) __NOP(); // 작은 가드타임
     return true;
 }


void dxl_wait_tx_complete(UART_HandleTypeDef *huart)
{
    (void)dxl_wait_tx_complete_tmo(huart, 20); // 기본 타임아웃 예: 20ms
}




LegMotors legs[5] = {
		{31, 32, 33, 34}, // 코드 제작용 시험 모터ID 21~22: 2.0, 27~28: 1.0
		{1, 51, 21, 61},  // 1 다리
		{2, 52, 22, 62},  // 2 다리
		{3, 53, 23, 63},  // 3 다리
		{4, 54, 24, 64}   // 4 다리
};


//-------------------CRC & CheckSum-----------------------
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
	unsigned short i, j;
	unsigned short crc_table[256] = {
			0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
			0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
			0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
			0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
			0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
			0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
			0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
			0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
			0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
			0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
			0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
			0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
			0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
			0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
			0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
			0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
			0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
			0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
			0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
			0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
			0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
			0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
			0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
			0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
			0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
			0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
			0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
			0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
			0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
			0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
			0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
			0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	for(j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}


void dxl_set_leg_pid_2(SendLegType leg, uint16_t p[3], uint16_t i[3], uint16_t d[3])
{
    uint32_t hip[1], knee[1], ankle[1];

    // P
    hip[0] = p[0]; knee[0] = p[1]; ankle[0] = p[2];
    send_sync_write_2(leg, DXL_2_Position_P_Gain, 2, hip, knee, ankle);

    // I
    hip[0] = i[0]; knee[0] = i[1]; ankle[0] = i[2];
    send_sync_write_2(leg, DXL_2_Position_I_Gain, 2, hip, knee, ankle);

    // D
    hip[0] = d[0]; knee[0] = d[1]; ankle[0] = d[2];
    send_sync_write_2(leg, DXL_2_Position_D_Gain, 2, hip, knee, ankle);
}

// PWM Limit 쓰기 (2바이트) — 모델에 따라 “Current Limit”만 있는 경우가 있으므로 주의
void dxl_set_leg_pwm_limit_2(SendLegType leg, uint16_t pwm_lim[3])
{
    uint32_t hip[1]   = { (uint32_t)pwm_lim[0] };
    uint32_t knee[1]  = { (uint32_t)pwm_lim[1] };
    uint32_t ankle[1] = { (uint32_t)pwm_lim[2] };
    send_sync_write_2(leg, DXL_2_PWM_Limit, 2, hip, knee, ankle);
}

// 프로파일 속도/가속도 (4바이트)
void dxl_set_leg_profile(SendLegType leg, uint32_t vel[3], uint32_t acc[3])
{
    send_sync_write_2(leg, DXL_2_Profile_Velocity,     4, &vel[0], &vel[1], &vel[2]);
    send_sync_write_2(leg, DXL_2_Profile_Acceleration, 4, &acc[0], &acc[1], &acc[2]);
}

void dxl_make_quiet_after_boot(void){
    uint32_t lvl4[4] = {1,1,1,1};   // READ에만 응답(=Status Return Level = 1)
    send_sync_write_2(ALL, DXL_2_Status_Return_Level, 1, lvl4, lvl4, lvl4);
    HAL_Delay(2);

    // ★ RDT=16(2us 단위 → 32us). 24(48us)도 좋고, 12(24us)도 가능.
    uint32_t rdt4[4] = {16,16,16,16};
    send_sync_write_2(ALL, DXL_2_Return_Delay_Time, 1, rdt4, rdt4, rdt4);
    HAL_Delay(2);
}



// === Core/Src/dxl_2_0.c : 기존 dxl_read() 전체 교체 ===
bool dxl_read(uint8_t id, uint16_t addr, uint16_t len, uint8_t *out, uint8_t *err)
{
    // 0) 짧은 버스락
    if (!dxl_bus_acquire(10)) { print1("[dxl_read] bus lock fail\r\n"); return false; }

    // 1) READ 패킷 구성  (Length=7: INST(1)+ADDR(2)+LEN(2)+CRC(2))
    uint8_t  tx[16];
    uint16_t ti = 0;
    tx[ti++] = 0xFF; tx[ti++] = 0xFF; tx[ti++] = 0xFD; tx[ti++] = 0x00;
    tx[ti++] = id;
    uint16_t plen = 7;
    tx[ti++] = (uint8_t)(plen & 0xFF);
    tx[ti++] = (uint8_t)(plen >> 8);
    tx[ti++] = 0x02; // READ
    tx[ti++] = (uint8_t)(addr & 0xFF);
    tx[ti++] = (uint8_t)(addr >> 8);
    tx[ti++] = (uint8_t)(len  & 0xFF);
    tx[ti++] = (uint8_t)(len  >> 8);
    unsigned short crc = update_crc(0, tx, ti);
    tx[ti++] = (uint8_t)(crc & 0xFF);
    tx[ti++] = (uint8_t)(crc >> 8);

    // 2) 직전 TX가 있다면 잠깐만 대기
    uint32_t t0 = HAL_GetTick();
    while (uart_tx_busy[1]) {
        if (HAL_GetTick() - t0 > 10) { print1("[dxl_read] prev TX busy\r\n"); goto fail; }
    }

    // 3) *** DMA 말고 블로킹 송신으로 단순화 ***
    dxl_tx_enable();
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
    if (HAL_UART_Transmit(&huart2, tx, ti, 10) != HAL_OK) {
        print1("[dxl_read] TX fail\r\n");
        goto fail;
    }
    // 송신 시프트레지스터까지 비울 때까지 TC 대기
    t0 = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {
        if (HAL_GetTick() - t0 > 5) { print1("[dxl_read] TC timeout\r\n"); goto fail; }
    }

    // 4) RX 모드 전환 + 에러플래그 정리 (+ SR/DR 더미 리드로 ORE 확실히 클리어)
    dxl_rx_enable();
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    volatile uint32_t sr = huart2.Instance->SR; (void)sr;
    volatile uint32_t dr = huart2.Instance->DR; (void)dr;

    // 5) 헤더 스캔 (FF FF FD 00), 바이트 단위로 25ms 안에 찾아냄
    uint8_t h0=0,h1=0,h2=0,h3=0,b=0;
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 25) {
        if (HAL_UART_Receive(&huart2, &b, 1, 2) == HAL_OK) {
            h0=h1; h1=h2; h2=h3; h3=b;
            if (h0==0xFF && h1==0xFF && h2==0xFD && h3==0x00) break;
        }
    }
    if (!(h0==0xFF && h1==0xFF && h2==0xFD && h3==0x00)) {
        print1("[dxl_read] RX timeout (no header)\r\n");
        goto fail;
    }

    // 6) ID + LEN(2)
    uint8_t id_len[3];
    if (HAL_UART_Receive(&huart2, id_len, 3, 3) != HAL_OK) {
        print1("[dxl_read] LEN/ID timeout\r\n"); goto fail;
    }
    uint8_t  rx_id  = id_len[0];
    uint16_t rx_len = (uint16_t)(id_len[1] | (id_len[2] << 8));
    if (rx_id != id) {
        print1("[dxl_read] Wrong ID\r\n"); goto fail;
    }
    if (rx_len < 4 || rx_len > DXL_PACKET_LEN) {
        print1("[dxl_read] LEN out\r\n"); goto fail;
    }

    // 7) 0x55 + ERR
    uint8_t inst_err[2];
    if (HAL_UART_Receive(&huart2, inst_err, 2, 3) != HAL_OK) {
        print1("[dxl_read] Core timeout\r\n"); goto fail;
    }
    if (inst_err[0] != 0x55) { print1("[dxl_read] Not status\r\n"); goto fail; }
    uint8_t rx_err = inst_err[1];

    // 8) DATA (rx_len-4) + CRC(2)
    uint16_t data_len = (uint16_t)(rx_len - 4);
    if (data_len) {
        if (HAL_UART_Receive(&huart2, dxl_rx_buf, data_len, 5) != HAL_OK) {
            print1("[dxl_read] DATA timeout\r\n"); goto fail;
        }
    }
    uint8_t crc2[2];
    if (HAL_UART_Receive(&huart2, crc2, 2, 3) != HAL_OK) {
        print1("[dxl_read] CRC timeout\r\n"); goto fail;
    }
    unsigned short rx_crc = (unsigned short)(crc2[0] | (crc2[1] << 8));

    // 9) CRC 검증
    uint8_t head[7]; uint16_t ci=0;
    head[ci++]=0xFF; head[ci++]=0xFF; head[ci++]=0xFD; head[ci++]=0x00;
    head[ci++]=rx_id; head[ci++]=(uint8_t)(rx_len & 0xFF); head[ci++]=(uint8_t)(rx_len >> 8);
    unsigned short calc = update_crc(0, head, ci);
    uint8_t pay2[2] = {0x55, rx_err};
    calc = update_crc(calc, pay2, 2);
    if (data_len) calc = update_crc(calc, dxl_rx_buf, data_len);
    if (calc != rx_crc) { print1("[dxl_read] CRC mismatch\r\n"); goto fail; }

    // 10) 결과 반환
    if (err) *err = rx_err;
    if (out && len) {
        uint16_t ncopy = (data_len < len) ? data_len : len;
        if (ncopy) memcpy(out, dxl_rx_buf, ncopy);
        if (ncopy < len) memset(out + ncopy, 0, (size_t)(len - ncopy));
    }
    dxl_bus_release();
    return true;

fail:
    dxl_bus_release();
    return false;
}

bool dxl_read_4byte(uint8_t id, uint16_t addr, int32_t *outVal, uint8_t *err)
{
    uint8_t data[4];
    if (!dxl_read(id, addr, 4, data, err)) return false;
    *outVal = (int32_t)(data[0] | (data[1]<<8) | (data[2]<<16) | (data[3]<<24));
    return true;
}

bool dxl_read_1byte(uint8_t id, uint16_t addr, uint8_t *outVal, uint8_t *err)
{
    uint8_t data[1];
    if (!dxl_read(id, addr, 1, data, err)) return false;
    *outVal = data[0];
    return true;
}

bool dxl_read_2byte(uint8_t id, uint16_t addr, uint16_t *outVal, uint8_t *err){
    uint8_t data[2];
    if (!dxl_read(id, addr, 2, data, err)) return false;
    *outVal = (uint16_t)(data[0] | (data[1]<<8));
    return true;
}

uint8_t calculate_checksum_1_0(uint8_t *data, uint16_t length) {
	uint8_t checksum = 0;

	for (uint16_t i = 2; i < length + 3; i++) {  // 2는 헤더 이후부터 시작
		checksum += data[i];
	}
	return ~checksum;
}
//-------------------CRC & CheckSum-----------------------

dxl_sync_write_1_0 creat_sync_packet_1_0(void)
{
	dxl_sync_write_1_0 sw1;

	sw1.header[0] = 0xFF;
	sw1.header[1] = 0XFF;
	sw1.id = 0xFE;
	sw1.inst = 0x83;

	return sw1;
}


dxl_sync_write_2_0 creat_sync_packet_2_0(void)
{
	dxl_sync_write_2_0 sw2;

	sw2.header[0] = 0xFF;
	sw2.header[1] = 0xFF;
	sw2.header[2] = 0xFD;
	sw2.reserved = 0;
	sw2.id = 0xFE;
	sw2.inst = 0x83;

	return sw2;
}


dxl_bulk_read_1_0 creat_bulk_packet_1_0(void)
{
	dxl_bulk_read_1_0 br1;

	br1.header[0] = 0xFF;
	br1.header[1] = 0XFF;
	br1.id = 0xFE;
	br1.inst = 0x92;

	return br1;
}


dxl_bulk_read_2_0 creat_bulk_packet_2_0(void)
{
	dxl_bulk_read_2_0 br2;

	br2.header[0] = 0xFF;
	br2.header[1] = 0xFF;
	br2.header[2] = 0xFD;
	br2.reserved = 0;
	br2.id = 0xFE;
	br2.inst = 0x92;

	return br2;
}


void uart_transmit_packet(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    if (huart->Instance == USART2)
    {
        dxl_tx_enable();
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);

        // 다른 TX가 끝날 때까지 대기
        while (uart_tx_busy[1]) { /* spin */ }

        if (size > DXL_TX_DMA_BUF_SIZE) {
            print1("[dxl] TX packet too big\r\n");
            return;
        }

        memcpy(dxl_tx_dma_buf, data, size);

        if (HAL_UART_Transmit_DMA(&huart2, dxl_tx_dma_buf, size) == HAL_OK) {
            uart_tx_busy[1] = true;
        } else {
            print1("[dxl] HAL_UART_Transmit_DMA fail\r\n");
        }
        return;
    }

    if(huart->Instance == USART1 && uart_tx_busy[0] == 0) {
        HAL_UART_Transmit_DMA(huart, data, size);
        uart_tx_busy[0] = true;
    } else if(huart->Instance == USART3 && uart_tx_busy[2] == 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_UART_Transmit_DMA(huart, data, size);
        uart_tx_busy[2] = true;
    } else if(huart->Instance == UART4 && uart_tx_busy[3] == 0) {
        HAL_UART_Transmit_DMA(huart, data, size);
        uart_tx_busy[3] = true;
    } else if(huart->Instance == UART5 && uart_tx_busy[4] == 0) {
        HAL_UART_Transmit_DMA(huart, data, size);
        uart_tx_busy[4] = true;
    }
}


void dxl_torque_set(uint8_t send_leg_type, uint8_t on1, uint8_t on2)
{
	uint16_t on_1[1];
	uint32_t on_2[4];
	on_1[0] = on1;
	on_2[0] = on2;
	on_2[1] = on2;
	on_2[2] = on2;
	on_2[3] = on2;

	if(send_leg_type == 0)
	{
		send_sync_write_2(send_leg_type, DXL_2_Torque_Enable, 1, on_2, on_2, on_2);
		HAL_Delay(10);
		send_sync_write_1(send_leg_type, DXL_1_Torque_Enable, 1, on_1);
		HAL_Delay(10);
	}
	else if(send_leg_type >= 1 && send_leg_type <= 4)
	{
		send_sync_write_1(send_leg_type, DXL_1_Torque_Enable, 1, on_1);
		HAL_Delay(10);
	}
}


uint16_t clc_speed_1(int16_t wheel_speed){
	uint16_t speed_value;

	if (wheel_speed < -1023 || wheel_speed > 1023) {
		return 0;  // Invalid speed
	}

	if (wheel_speed < 0) {
		speed_value = (uint16_t)(-wheel_speed) + 1024;  // CCW direction
	} else {
		speed_value = (uint16_t)wheel_speed;  // CW direction
	}
	return speed_value;
}


uint16_t serialize_sync_write_1_0(dxl_sync_write_1_0 *packet, uint8_t *id_array, uint8_t id_count,
		uint16_t *data_array, uint8_t *buffer, uint16_t buffer_size)
{
	uint16_t index = 0;

	// 버퍼 크기 체크
	uint16_t required_size = 8 + id_count * (packet->data_len + 1);
	if (buffer_size < required_size) {
		return 0; // 버퍼 크기 부족
	}

	buffer[index++] = packet->header[0];
	buffer[index++] = packet->header[1];
	buffer[index++] = packet->id;
	buffer[index++] = packet->length;
	buffer[index++] = packet->inst;
	buffer[index++] = packet->start_addr;
	buffer[index++] = packet->data_len;

	for (int i = 0; i < id_count; i++) {
		buffer[index++] = id_array[i]; // ID

		// 데이터 추가 (리틀 엔디안)
		for (int j = 0; j < packet->data_len; j++) {
			buffer[index++] = (data_array[i] >> (8 * j)) & 0xFF;
		}
	}
	packet->checksum = calculate_checksum_1_0(buffer, index);
	//packet->checksum = calculate_checksum_1_0(buffer, packet->length);
	buffer[index++] = packet->checksum;

	return index;

}

uint16_t serialize_sync_write_2_0(dxl_sync_write_2_0 *packet, uint8_t *id_array, uint8_t id_count,
		uint32_t *data_array, uint8_t *buffer, uint16_t buffer_size)
{
	uint16_t index = 0;

	// 버퍼 크기 체크
	uint16_t required_size = 12 + id_count * (packet->data_len + 1) + 2; // 헤더+ID+파라미터+CRC
	if (buffer_size < required_size) {
		return 0; // 버퍼 크기 부족
	}

	buffer[index++] = packet->header[0];
	buffer[index++] = packet->header[1];
	buffer[index++] = packet->header[2];
	buffer[index++] = packet->reserved;
	buffer[index++] = packet->id;
	buffer[index++] = packet->length & 0xFF;
	buffer[index++] = (packet->length >> 8) & 0xFF;
	buffer[index++] = packet->inst;
	buffer[index++] = packet->start_addr & 0xFF;
	buffer[index++] = (packet->start_addr >> 8) & 0xFF;
	buffer[index++] = packet->data_len & 0xFF;
	buffer[index++] = (packet->data_len >> 8) & 0xFF;

	for (int i = 0; i < id_count; i++) {
		buffer[index++] = id_array[i]; // ID

		// 데이터 추가 (리틀 엔디안)
		for (int j = 0; j < packet->data_len; j++) {
			buffer[index++] = (data_array[i] >> (8 * j)) & 0xFF;
		}
	}

	unsigned short crc = update_crc(0, buffer, index);
	buffer[index++] = crc & 0x00FF;
	buffer[index++] = (crc >> 8) & 0x00FF;

	return index; // 실제 사용된 버퍼 크기 반환

}

// 단일 ID 1바이트 WRITE (Protocol 2.0, StatusReturnLevel=1이면 응답 안 옴)
void dxl_write_1byte(uint8_t id, uint16_t addr, uint8_t data, uint8_t *err)
{
    if (err) *err = 0xFF;         // 기본값: 실패로 시작

    if (!dxl_bus_acquire(10)) {
        print1("[w1] bus lock fail\r\n");
        return;
    }

    uint8_t  tx[16];
    uint16_t idx = 0;
    tx[idx++] = 0xFF; tx[idx++] = 0xFF; tx[idx++] = 0xFD; tx[idx++] = 0x00;
    tx[idx++] = id;

    uint16_t plen = 4; // INST(1) + ADDR(2) + DATA(1)
    tx[idx++] = (uint8_t)(plen & 0xFF);
    tx[idx++] = (uint8_t)(plen >> 8);
    tx[idx++] = 0x03;  // WRITE
    tx[idx++] = (uint8_t)(addr & 0xFF);
    tx[idx++] = (uint8_t)(addr >> 8);
    tx[idx++] = data;

    unsigned short crc = update_crc(0, tx, idx);
    tx[idx++] = (uint8_t)(crc & 0xFF);
    tx[idx++] = (uint8_t)(crc >> 8);

    // TX 버스가 비기를 짧게 대기
    uint32_t t0 = HAL_GetTick();
    while (uart_tx_busy[1]) {
        if (HAL_GetTick() - t0 > 20) {
            print1("[w1] TX busy timeout\r\n");
            dxl_bus_release();
            return;
        }
    }

    // 송신 → TC 대기 → 곧바로 RX로
    uart_transmit_packet(&huart2, tx, idx);
    dxl_wait_tx_complete(&huart2);
    dxl_rx_enable();

    if (err) *err = 0;   // 여기까지 오면 OK 처리
    dxl_bus_release();
}



void send_sync_write_1(uint8_t send_leg_type, uint16_t addr, uint16_t data_len, const uint16_t *wheel_data)
{
    uint8_t id_count = 0;

    if (send_leg_type == 0) id_count = all_wheel_motor;           // 4
    else if (send_leg_type >= 1 && send_leg_type <= 4) id_count = one_leg_wheel_motor; // 1

    uint8_t  id[id_count];
    uint16_t data_array[id_count];
    dxl_sync_write_1_0 sw1 = creat_sync_packet_1_0();

    // ID 빌드 (FL, FR, RR, RL 고정)
    if (send_leg_type == 0) {
        id[0] = legs[1].wheel; id[1] = legs[2].wheel;
        id[2] = legs[3].wheel; id[3] = legs[4].wheel;
    } else {
        id[0] = legs[send_leg_type].wheel;
    }

    // --- 핵심: AX-12 속도 레지스터일 때만 관대한 변환 적용 ---
    if (addr == DXL_1_MOVING_SPEED && data_len == 2) {
        for (int i = 0; i < id_count; i++) {
            uint16_t w = wheel_data[i];

            if (w <= 2047) {
                data_array[i] = (uint16_t)(w & 0x07FF); // 상위 쓰레기 비트 제거
            } else {
                // wrap-around로 부호를 숨긴 값: int16_t로 해석 후 변환
                int16_t s = (int16_t)w;
                if (s >  1023) s =  1023;    // 과도값 안전 클램프
                if (s < -1023) s = -1023;
                data_array[i] = clc_speed_1(s);  // 0~1023(+0x400) 원시워드로
            }
        }
    } else {
        // 일반 레지스터는 손대지 않고 그대로 보냄
        for (int i = 0; i < id_count; i++) {
            data_array[i] = wheel_data[i];
        }
    }

    // 패킷 작성/송신
    sw1.start_addr = addr;
    sw1.data_len   = data_len;
    sw1.length     = 4 + (id_count * (sw1.data_len + 1));

    uint16_t required_size = 8 + id_count * (sw1.data_len + 1);
    uint8_t  packet_buffer[256] = {0};

    if (required_size <= sizeof(packet_buffer)) {
        uint16_t packet_size = serialize_sync_write_1_0(
            &sw1, id, id_count, data_array, packet_buffer, sizeof(packet_buffer));

        while (uart_tx_busy[2] != 0) { /* wait */ }
        if (packet_size > 0) {
            uart_transmit_packet(&huart3, packet_buffer, packet_size);
        }
    }
}


void send_sync_write_2(uint8_t send_leg_type, uint16_t addr, uint16_t data_len,
                       const uint32_t *hip_data, const uint32_t *knee_data, const uint32_t *ankle_data)
{
    uint8_t  id_count = 0;
    uint8_t  id[12];            // all_hip_knee_ankle_motor = 12 기준
    uint32_t data_array[12];
    dxl_sync_write_2_0 sw2 = creat_sync_packet_2_0();

    if (send_leg_type == 0) {
        id_count = all_hip_knee_ankle_motor; // 12
        for (int i = 1; i <= leg_count; ++i) {
            id[3*(i-1) + 0] = legs[i].hip;
            id[3*(i-1) + 1] = legs[i].knee;
            id[3*(i-1) + 2] = legs[i].ankle;

            data_array[3*(i-1) + 0] = hip_data[i-1];
            data_array[3*(i-1) + 1] = knee_data[i-1];
            data_array[3*(i-1) + 2] = ankle_data[i-1];
        }
    } else if (send_leg_type >= 1 && send_leg_type <= 4) {
        id_count = one_leg_hip_knee_ankle_motor; // 3
        id[0] = legs[send_leg_type].hip;
        id[1] = legs[send_leg_type].knee;
        id[2] = legs[send_leg_type].ankle;
        data_array[0] = hip_data[0];
        data_array[1] = knee_data[0];
        data_array[2] = ankle_data[0];
    } else if (send_leg_type == 5) {
        id_count = 4;  // Hip 4개
        id[0] = legs[1].hip; id[1] = legs[2].hip; id[2] = legs[3].hip; id[3] = legs[4].hip;
        data_array[0] = hip_data[0]; data_array[1] = hip_data[1];
        data_array[2] = hip_data[2]; data_array[3] = hip_data[3];
    } else if (send_leg_type == 6) {
        id_count = 4;  // Knee 4개
        id[0] = legs[1].knee; id[1] = legs[2].knee; id[2] = legs[3].knee; id[3] = legs[4].knee;
        data_array[0] = knee_data[0]; data_array[1] = knee_data[1];
        data_array[2] = knee_data[2]; data_array[3] = knee_data[3];
    } else if (send_leg_type == 7) {
        id_count = 4;  // Ankle 4개
        id[0] = legs[1].ankle; id[1] = legs[2].ankle; id[2] = legs[3].ankle; id[3] = legs[4].ankle;
        data_array[0] = ankle_data[0]; data_array[1] = ankle_data[1];
        data_array[2] = ankle_data[2]; data_array[3] = ankle_data[3];
    } else {
        print1("[syncw2] invalid send_leg_type\r\n");
        return;
    }

    sw2.start_addr = addr;
    sw2.data_len   = data_len;
    sw2.length     = 7 + (id_count * (sw2.data_len + 1));

    uint16_t required_size = 12 + id_count * (sw2.data_len + 1) + 2;
    uint8_t  packet_buffer[256] = {0};

    if (required_size > sizeof(packet_buffer)) {
        print1("[syncw2] packet too big\r\n");
        return;
    }

    uint16_t packet_size = serialize_sync_write_2_0(&sw2, id, id_count,
                                                    data_array, packet_buffer, sizeof(packet_buffer));
    if (packet_size == 0) {
        print1("[syncw2] serialize fail\r\n");
        return;
    }

    // ---- 단일 경로: 버스락 → TX → TC 대기 → RX로 되돌림 → 락 해제 ----
    if (!dxl_bus_acquire(10)) { print1("[syncw2] bus lock fail\r\n"); return; }

    uint32_t t0 = HAL_GetTick();
    while (uart_tx_busy[1]) {
        if (HAL_GetTick() - t0 > 20) { print1("[syncw2] TX busy timeout\r\n"); dxl_bus_release(); return; }
    }

    uart_transmit_packet(&huart2, packet_buffer, packet_size);
    dxl_wait_tx_complete(&huart2);   // DMA 완료 + TC + 짧은 가드
    dxl_rx_enable();                 // 항상 RX idle로 돌려놓기
    dxl_bus_release();
}

///////////////////////////// 10월2일 추가//////////////////////////

// ★ AX-12: Wheel 모드 = CW/CCW Angle Limit(6/8) 모두 0
void ax12_force_wheel_mode_all(void){
    // 1) 토크 OFF
    uint16_t off4[4] = {0,0,0,0};
    send_sync_write_1(ALL, DXL_1_Torque_Enable, 1, off4);
    HAL_Delay(4);

    // 2) EEPROM: CW/CCW 각도리밋을 0으로 → Wheel 모드
    uint16_t z4[4] = {0,0,0,0};
    send_sync_write_1(ALL, 6, 2, z4);    // CW Angle Limit (2B)
    HAL_Delay(6);
    send_sync_write_1(ALL, 8, 2, z4);    // CCW Angle Limit (2B)
    HAL_Delay(8);

    // 3) 토크 ON
    uint16_t on4[4] = {1,1,1,1};
    send_sync_write_1(ALL, DXL_1_Torque_Enable, 1, on4);
    HAL_Delay(4);
}

// 펀치(48, 2B) — 정지마찰 탈출 도움
void ax12_set_punch_all(uint16_t punch){
    uint16_t p4[4] = {punch, punch, punch, punch};
    send_sync_write_1(ALL, 48, 2, p4);
    HAL_Delay(2);
}

// 토크 리밋(34, 2B)
void ax12_set_torque_limit_all(uint16_t tl){
    uint16_t v4[4] = {tl, tl, tl, tl};
    send_sync_write_1(ALL, DXL_1_Goal_Torque_Limit, 2, v4);
    HAL_Delay(2);
}


void ax12_force_joint_mode_all(void){
    uint16_t off4[4]={0,0,0,0}; send_sync_write_1(ALL, DXL_1_Torque_Enable, 1, off4); HAL_Delay(4);
    uint16_t cw0[4]={0,0,0,0}, ccw1023[4]={1023,1023,1023,1023};
    send_sync_write_1(ALL, 6, 2, cw0);       HAL_Delay(6);   // CW limit = 0
    send_sync_write_1(ALL, 8, 2, ccw1023);   HAL_Delay(8);   // CCW limit = 1023 → Joint 모드
    uint16_t on4[4]={1,1,1,1}; send_sync_write_1(ALL, DXL_1_Torque_Enable, 1, on4); HAL_Delay(4);
}

void ax12_write_u8(uint8_t id, uint8_t addr, uint8_t v){
    uint8_t d[1] = { v };
    send_sync_write_1(id, addr, 1, (uint16_t*)d); // 네 기존 send_sync_write_1 활용
}

void ax12_write_u16(uint8_t id, uint8_t addr, uint16_t v){
    uint16_t d[1] = { v };
    send_sync_write_1(id, addr, 2, d);
}

bool ax12_read_u16(uint8_t id, uint16_t addr, uint16_t* out){
    uint8_t buf[2]; uint8_t err=0;
    if (!dxl_read(id, addr, 2, buf, &err)) return false;
    *out = (uint16_t)(buf[0] | (buf[1]<<8));
    return true;
}


////////////////////////////////10월2일 종료//////////////////


