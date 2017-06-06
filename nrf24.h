/*
 * nrf24.h
 *
 *  Created on: 23.05.2017
 *      Author: merhart
 */

#ifndef NRF24_H_
#define NRF24_H_
#include <SPI.h>

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define W_TX_PL_NOACK 0xB0
#define NOP           0XFF

/* Memory Map */
#define NRF_CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/* Pwr level */
#define MAX_PA      0x06
#define HIGH_PA     0x04
#define MID_PA      0x02
#define LOW_PA      0x00

/* Transmit Speed */
#define oneMbps     0x00
#define twoMbps     0x08
#define quarterMbps 0x10

using namespace std;

class nrf24 {
	uint8_t ce_pin, csn_pin;


private:
  void csn(bool state);
  void ce(bool state);
	uint8_t readReg(int reg);
	void readMReg(int reg, uint8_t *buf, int c);
	void writeReg(int reg, int value);
	void writeMReg(int reg, uint8_t *buf, int c);
	void flushTX();
	void flushRX();
  void writeMTXPayload(uint8_t*, int);
	void readMRXPayload(uint8_t *buf, int c);
public:
	nrf24(const int ce, const int csn);
	void begin();
	void setChannel(uint8_t);
  void setPower(uint8_t);
	void setRXaddress(uint8_t*);
  void setTXaddress(uint8_t*);
//  int send(uint8_t *packet, int len);
	void send(uint8_t *buf, int c);
	void sendACK(uint8_t *buf, int c);
	virtual ~nrf24();

};
inline void nrf24::csn(bool state) {digitalWrite(csn_pin, state);}
inline void nrf24::ce(bool state) {digitalWrite(ce_pin, state);}


#endif /* NRF24_H_ */
