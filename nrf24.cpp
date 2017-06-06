/*
 * nrf24.cpp
 *
 *  Created on: 23.05.2017
 *      Author: merhart
 */

#include "nrf24.h"

using namespace std;

nrf24::nrf24(const int _csn, const int _ce): csn_pin(_csn), ce_pin(_ce) {
	SPISettings spiSet(10000000, MSBFIRST, SPI_MODE0);
	// TODO Auto-generated constructor stub

}
/*
void nrf24::ce(bool state){
	digitalWrite(ce_pin, state);
}

void nrf24::csn(bool state){
	digitalWrite(csn_pin,state);
}
*/
void nrf24::begin(){
	SPI.begin();
	byte a1[] = {0x52, 0x01, 0x00, 0x00, 0x00}; // receive address, network number
	byte a2[] = {0xFF, 0x01, 0x00, 0x00, 0x00}; // broadcast address, network number
	byte a3[] = {0x51, 0x01, 0x00, 0x00, 0x00}; // destination address, network number

	writeReg(0, 0b01111110);  // 0(1 bit), Mask ints (3 bits), 8-bit CRC (2 bits), PowerUp (1 bit), PTX (1 bit)
	delay(5);                 // allow 5 ms. for power up.
	writeReg(1, 3);           // enable AA on pipes 0,1
	writeReg(2, 3);           // enable pipes 0, 1
	writeReg(3, 3);           // 5 byte addresses
	writeReg(5, 80);          // RF_CH frequency channel 80 (out of the range: 0-127)
	writeReg(6, 0x06);        // -6dBm
	writeMReg(0x0A, a1, 5);   // my receive (RX) address
	writeMReg(0x0B, a2, 5);   // the broadcast address
	writeMReg(0x10, a3, 5);   // my transmit (TX) address
	writeReg(0x1D, 5);        // enable TX NO ACK, DYN PAYLOAD
	writeReg(0x1C, 3);        // enable DYN pipes 0, 1

}
void nrf24::setPower(uint8_t pwr){

}

void nrf24::setChannel(uint8_t ch){
	nrf24::writeReg(5, ch);
}
void nrf24::setRXaddress(uint8_t *a){
	writeMReg(0x0A, a, 5);
}
void nrf24::setTXaddress(uint8_t *a){
  writeMReg(0x10, a, 5);
}


void writeReg(int reg, int value){
  csn(LOW);
  SPI.transfer(reg | 0x20);
  SPI.transfer(value);
  csn(HIGH); 
}

uint8_t nrf24::readReg(int reg){
  uint8_t rv;

  csn(LOW);
  SPI.transfer(reg);
  rv = SPI.transfer(NOP);
  csn(HIGH);
  return (rv);
}
void nrf24::readMReg(int reg, uint8_t *buf, int c){
  int j;

  //digitalWrite(CSN, LOW);
  csn(LOW);
  SPI.transfer(reg | R_REGISTER);
  for (j=0; j<c; j++)
    {
    buf[j] = SPI.transfer(NOP);
    }
  //digitalWrite(CSN, HIGH);
  csn(HIGH);
}

void nrf24::writeMReg(int reg, uint8_t *buf, int c){
  int j;

  //digitalWrite(CSN, LOW);
  csn(LOW);
  SPI.transfer(reg | W_REGISTER);
  for (j=0; j<c; j++){
    SPI.transfer(buf[j]);
  }
  //digitalWrite(CSN, HIGH);
  csn(HIGH);
}

void nrf24::readMRXPayload(uint8_t *buf, int c){
  int j;

  //digitalWrite(CSN, LOW);
  csn(LOW);
  SPI.transfer(NOP);		// command to read RX Payload
  for (j=0; j<c; j++){
    buf[j] = SPI.transfer(0xFF); // NOP output, read data byte
  }
  //digitalWrite(CSN, HIGH);
  csn(HIGH);
}

void nrf24::writeMTXPayload(uint8_t *buf, int c){
  //int j;

  //digitalWrite(CSN, LOW);
  csn(LOW);
  SPI.transfer(W_TX_PAYLOAD);		// command to write TX Payload
  for (int j=0; j<c; j++)
    {
    SPI.transfer(buf[j]); // put the output bytes in the TX FIFO
    }

  for (int j=c; j<32; j++) SPI.transfer(0);  // pad with 0 bytes to fill FIFO
  //digitalWrite(CSN, HIGH);
  csn(HIGH);
}
void nrf24::send(uint8_t *buf, int c){
  csn(LOW);
  SPI.transfer(W_TX_PL_NOACK);    // command to write TX Payload NO ACK
  for (int j=0; j<c; j++)
    {
    SPI.transfer(buf[j]); // put the output bytes in the TX FIFO
    }
  csn(LOW);
}

void nrf24::flushTX(){   // flush TX FIFO
	  //digitalWrite(CSN, LOW);
	  csn(LOW);
	  SPI.transfer(FLUSH_TX);
	  //digitalWrite(CSN, HIGH);
	  csn(HIGH);
}

void nrf24::flushRX(){   // flush RX FIFO
	//digitalWrite(CSN, LOW);
	csn(LOW);
	SPI.transfer(FLUSH_RX);
	csn(HIGH);
	//digitalWrite(CSN, HIGH);
}




nrf24::~nrf24() {
	// TODO Auto-generated destructor stub
}


