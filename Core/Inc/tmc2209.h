/*
 * tmc2209.h
 *
 *  Created on: Sep 9, 2025
 *      Author: KESHAVA HEGDE
 */

#ifndef INC_TMC2209_H_
#define INC_TMC2209_H_

//#include "main.h"
#include <stdint.h>


//TMC Datagram structure

extern uint8_t hardware_uart2_read();
extern void software_UART_TX(uint8_t data);
extern uint8_t software_UART_RX(uint32_t timeout_us);
extern void UART_software_config_outputmode(void);

typedef struct
{
	uint8_t sync_byte;
	uint8_t slave_tmc;
	uint8_t reg_r_w;
	uint8_t datagram[4];
	uint8_t crc;

}TMCdatagram_t;


uint8_t tmc_current_to_cs(float Irms);

uint8_t microsteps_to_mres(uint16_t microsteps);


void tmc_init();//things to be done to configure the TMC

//Builder for datagram packets

void tmc_writeregister(TMCdatagram_t *tmcpkt, uint8_t slave_addr, uint8_t reg, uint32_t *data);

void tmc_readregister(TMCdatagram_t *tmcpkt,uint8_t slave_addr, uint8_t reg);

//TMC low level functions
uint8_t tmc_calc_crc(uint8_t *data, uint8_t len);

void tmc_send(TMCdatagram_t *tmcpkt);

void tmc_receive(TMCdatagram_t *tmcpkt);




#endif /* INC_TMC2209_H_ */
