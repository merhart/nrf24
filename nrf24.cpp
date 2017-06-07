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

void nrf24::begin(){
	SPI.begin();
	byte a1[] = {0x52, 0x01, 0x00, 0x00, 0x00}; // receive address, network number
	byte a2[] = {0xFF, 0x01, 0x00, 0x00, 0x00}; // broadcast address, network number
	byte a3[] = {0x51, 0x01, 0x00, 0x00, 0x00}; // destination address, network number
  writeReg(NRF_CONFIG, RX_INIT);
	delay(5);                 // allow 5 ms. for power up.
	writeReg(EN_AA, 3);           // enable AA on pipes 0,1
	writeReg(EN_RXADDR, 3);           // enable pipes 0, 1
	writeReg(SETUP_AW, 3);           // 5 byte addresses
	writeReg(RF_CH, 80);          // RF_CH frequency channel 80 (out of the range: 0-127)
	writeReg(RF_SETUP, MAX_PA);        // -6dBm
	writeMReg(RX_ADDR_P0, a1, 5);   // my receive (RX) address
	writeMReg(RX_ADDR_P0, a2, 5);   // the broadcast address
	writeMReg(TX_ADDR, a3, 5);   // my transmit (TX) address
	writeReg(FEATURE, 5);        // enable TX NO ACK, DYN PAYLOAD
	writeReg(DYNPD, 3);        // enable DYN pipes 0, 1

}
void nrf24::setPower(uint8_t pwr){
	writeReg(RF_SETUP, pwr);
}

void nrf24::setChannel(uint8_t ch){
	nrf24::writeReg(RF_CH, ch);
}
void nrf24::setRXaddress(uint8_t *a){
	writeMReg(0x0A, a, 5);
}
void nrf24::setTXaddress(uint8_t *a){
  writeMReg(0x10, a, 5);
}


void nrf24::writeReg(int reg, int value){
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
  csn(LOW);
  SPI.transfer(reg | R_REGISTER);
  for (j=0; j<c; j++)
    {
    buf[j] = SPI.transfer(NOP);
    }
  csn(HIGH);
}

void nrf24::writeMReg(int reg, uint8_t *buf, int c){
  int j;
  csn(LOW);
  SPI.transfer(reg | W_REGISTER);
  for (j=0; j<c; j++){
    SPI.transfer(buf[j]);
  }
  csn(HIGH);
}

void nrf24::readMRXPayload(uint8_t *buf, int c){
  int j;

  csn(LOW);
  SPI.transfer(NOP);		// command to read RX Payload
  for (j=0; j<c; j++){
    buf[j] = SPI.transfer(NOP); // NOP output, read data byte
  }
  csn(HIGH);
}

void nrf24::writeMTXPayload(uint8_t *buf, int c){
  csn(LOW);
  SPI.transfer(W_TX_PAYLOAD);		// command to write TX Payload
  for (int j=0; j<c; j++)
    {
    SPI.transfer(buf[j]); // put the output bytes in the TX FIFO
    }
  csn(HIGH);
}
void nrf24::send(intt *buf, unsigned int c){
	writeReg(NRF_CONFIG, TX_INIT); 
  csn(LOW);
  delay(5);
  SPI.transfer(W_TX_PL_NACK);    // command to write TX Payload NO ACK
  for (int j=0; j<c; j++)
    {
    SPI.transfer(buf[j]); // put the output bytes in the TX FIFO
    }
  csn(LOW);
	writeReg(NRF_CONFIG, RX_INIT);
  delay(5); 
}

void nrf24::receive(uint8_t *buf, int size){
    
  csn(LOW);
  SPI.transfer(R_RX_PAYLOAD);// command to read RX Payload
  for (int j=0; j<size; j++){
     buf[j] = SPI.transfer(NOP); // NOP output, read data byte  
  }
  csn(HIGH);
}

void nrf24::receive(uint8_t *buf){
  uint8_t size;
  size = nrf24::readPLWidth();
    
  csn(LOW);
  SPI.transfer(R_RX_PL_WID);// command to read RX Payload
  for (int j=0; j<size; j++){
     buf[j] = SPI.transfer(NOP); // NOP output, read data byte  
  }
  csn(HIGH);

}

uint8_t nrf24::readPLWidth(){
  uint8_t rv;
             
  csn(LOW); 
  SPI.transfer(W_TX_PL_NACK);     // read payload width via R_RX_PL_WID
  rv = SPI.transfer(NOP);
  csn(HIGH);  
  return (rv);   
}

bool nrf24::available(){
  char status;

  status = readReg(NRF_STATUS);
  writeReg(NRF_STATUS, 0x40);
  return (status & 0x40);
}

void nrf24::flushTX(){   // flush TX FIFO
	  csn(LOW);
	  SPI.transfer(FLUSH_TX);
	  csn(HIGH);
}

void nrf24::flushRX(){   // flush RX FIFO
	csn(LOW);
	SPI.transfer(FLUSH_RX);
	csn(HIGH);
}

nrf24::~nrf24() {
	// TODO Auto-generated destructor stub
}


