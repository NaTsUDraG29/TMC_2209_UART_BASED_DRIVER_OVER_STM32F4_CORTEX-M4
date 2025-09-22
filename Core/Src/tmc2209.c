/*
 * tmc2209.c
 *
 *  Created on: Sep 9, 2025
 *      Author: KESHAVA HEGDE
 */
#include <stdint.h>
#include <stdio.h>
#include "tmc2209.h"
#include "stepper_config.h"

int tmc_detect(uint8_t slave_addr)
{
    TMCdatagram_t tmcpkt;

    // Ask IOIN register (0x6C)
    tmc_readregister(&tmcpkt, slave_addr, CHOP_CONF);
    tmc_send(&tmcpkt);

    // Wait for reply into pkt
//    if (!tmc_receive(&tmcpkt)) { // 5000 us timeout
////lets make printf to work as UART output in com port
//        return -1;
// }
    tmc_receive(&tmcpkt);

    // Check CRC
    uint8_t crc = tmc_calc_crc((uint8_t *)&tmcpkt, 7);
    if (crc != tmcpkt.crc) {

        return -2;
    }

    // Decode IOIN reply: upper 8 bits = version
    uint32_t val = (tmcpkt.datagram[0] << 24) | (tmcpkt.datagram[1] << 16) | \
    		(tmcpkt.datagram[2] << 8)  | (tmcpkt.datagram[3]);

    uint8_t version = (val >> 24) & 0xFF;
    if (version == 0x21) {
        printf("TMC configured successfully!\n");
        return 0;
    } else {
        printf("Error: Unexpected device ID 0x%02X\n", version);
        return -3;
    }
}


void tmc_init()
{
	//Set the PC4 pin as TX
	UART_software_config_outputmode();


	//Then send the configuration setup for the TMC
	HAL_Delay(10);

	if(tmc_detect(SLAVE_ADDR) == -1)
	{
        printf("Error: TMC not connected\n");
        return;
	}
	else if(tmc_detect(SLAVE_ADDR) == -2)
	{
		printf("Error: TMC communication failed (CRC)\n");
		return;
	}
	else if(tmc_detect(SLAVE_ADDR) == -3)
	{
		printf("Error: Unexpected device ID 0x%02X\n");
		return;
	}

	TMCdatagram_t tmcpkt;

	uint8_t ihold = tmc_current_to_cs(IHOLD);

	uint8_t irun = tmc_current_to_cs(IRUN);

	uint32_t ihold_irun_val = (6 << 16) | (irun << 8) | ihold;


	//already in stealth chop by default
	//setting run and hold current
	tmc_writeregister(&tmcpkt, SLAVE_ADDR, IHOLD_IRUN, &ihold_irun_val);
	tmc_send(&tmcpkt);

	//Setting microsteps

	uint32_t microsteps_val = (microsteps_to_mres(Microsteps) << 24);
	tmc_writeregister(&tmcpkt, SLAVE_ADDR, CHOP_CONF, &microsteps_val);
	tmc_send(&tmcpkt);



}

uint8_t microsteps_to_mres(uint16_t microsteps)
{
    if (microsteps == 0 || (256 % microsteps) != 0)
    {
        return 0; // fallback invalid input
    }

    uint16_t ratio = 256 / microsteps;
    uint8_t mres = 0;

    while (ratio > 1) {
        ratio >>= 1;   // divide by 2
        mres++;
    }

    return mres;  // gives the MRES code
}


uint8_t tmc_current_to_cs(float Irms)
{
    float cs = ((32.0f * Irms * 1.414f * Rsense) / Vref) - 1.0f;
    if (cs < 0) cs = 0;
    if (cs > 31) cs = 31;
    return (uint8_t)cs;
}


// calculate  CRC for data
uint8_t tmc_calc_crc(uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;
	uint8_t i,j;

	for(i = 0; i < len; i++)
	{
		crc ^= data[i];

		for(j = 0; j < 8; j++)
		{
			crc = (crc & 0x01)?(crc >> 1) & 0x8C : (crc >> 1);
		}

	}
	return crc;
}


void tmc_writeregister(TMCdatagram_t *tmcpkt, uint8_t slave_addr, uint8_t reg, uint32_t *data)
{
	tmcpkt->sync_byte = SYNC_BYTE;
	tmcpkt->slave_tmc = SLAVE_ADDR;
	tmcpkt->reg_r_w = 0x80 | reg; // write is 10000000 setting msb to 1
	tmcpkt->datagram[0] = (*data >> 24) & 0xFF; // get msb 8 bits as tmc needs msb to be sent first
	tmcpkt->datagram[1] = (*data >> 16) & 0xFF;
	tmcpkt->datagram[2] = (*data >> 8) & 0xFF;
	tmcpkt->datagram[3] =  (*data & 0xFF);

	uint8_t buf[7];

	buf[0] = tmcpkt->sync_byte;
	buf[1] = tmcpkt->slave_tmc;
	buf[2] = tmcpkt->reg_r_w;
	buf[3] = tmcpkt->datagram[0];
	buf[4] = tmcpkt->datagram[1];
	buf[5] = tmcpkt->datagram[2];
	buf[6] = tmcpkt->datagram[3];


	tmcpkt->crc = tmc_calc_crc(buf, 7);
   //The datagram for the TMC is read to be transmitted for writing into a tmc register
}

void tmc_readregister(TMCdatagram_t *tmcpkt,uint8_t slave_addr, uint8_t reg)
{
	tmcpkt->sync_byte = SYNC_BYTE;
	tmcpkt->slave_tmc = SLAVE_ADDR;
	tmcpkt->reg_r_w = reg;

	uint8_t buf[3];

	buf[0] = tmcpkt->sync_byte;
	buf[1] = tmcpkt->slave_tmc;
	buf[2] = tmcpkt->reg_r_w;

	for(int i = 0; i < 4; i++)
		tmcpkt->datagram[i] = 0x00;

	tmcpkt->crc = tmc_calc_crc(buf, 3);

	//ready to send the datagram for requesting data bytes from the TMC
}


//TMC send and receive datapackets

void tmc_send(TMCdatagram_t *tmcpkt)
{
	software_UART_TX(tmcpkt->sync_byte);
	software_UART_TX(tmcpkt->slave_tmc);
	software_UART_TX(tmcpkt->reg_r_w);
	for(int i = 0; i < 4; i++)
		software_UART_TX(tmcpkt->datagram[i]);
	software_UART_TX(tmcpkt->crc);

}


void tmc_receive(TMCdatagram_t *tmcpkt)
{
	tmcpkt->sync_byte = software_UART_RX(SOFTWARE_RX_TO);
	tmcpkt->slave_tmc = software_UART_RX(SOFTWARE_RX_TO);
	tmcpkt->reg_r_w = software_UART_RX(SOFTWARE_RX_TO);

	for(int i = 0; i < 4; i++ )
		tmcpkt->datagram[i] = software_UART_RX(SOFTWARE_RX_TO);

	tmcpkt->crc = software_UART_RX(SOFTWARE_RX_TO);
}



