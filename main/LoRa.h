#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <esp_log.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifndef LoRa_h
#define LoRa_h

#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LR_OCP				 0X0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PaDac				 0x4d
#define REG_PA_DAC               0x4d
#define PA_BOOST                 0x80

#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

#define MAX_PKT_LENGTH	255

#define PA_OUTPUT_PA_BOOST_PIN  1
#define PA_OUTPUT_RFO_PIN       0
#define RF_PACONFIG_PASELECT_MASK                   0x7F
#define RF_PACONFIG_PASELECT_PABOOST                0x80
#define RF_PACONFIG_PASELECT_RFO                    0x00
#define RF_PACONFIG_MAX_POWER_MASK                  0x8F
#define RF_PACONFIG_OUTPUTPOWER_MASK                0xF0
#define RF_PADAC_20DBM_MASK                         0xF8
#define RF_PADAC_20DBM_ON                           0x07
#define RF_PADAC_20DBM_OFF                          0x04

#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40


class LoRa
{
 public:
	LoRa( int mosi, int miso, int clk, int cs, int reset, int dio, int power );
    
	int parsePacket(int size);
	int read();
	void receive(int size);
	void handleDataReceived();
	int available();

	void dumpRegisters();

	int beginPacket(int implicitHeader);
	size_t write(const uint8_t *buffer, size_t size);
	int endPacket(bool async);

	void explicitHeaderMode();
	void implicitHeaderMode();

	void setOCP(uint8_t mA);
	void setTxPower(int8_t power, int8_t outputPin);
	void setCRC( bool crc );
	void setSyncWord(int sw);

	void sleep();
	void idle();

	void setFrequency(long frequency);
	void setSpreadingFactor(int sf);
	void setSignalBandwidth(long sbw);

	void initializeSPI( int mosi, int miso, int clk, int cs );
	void initializeReset( int reset );
	void initializeDIO( int dio );
	void initialize( int power );

	void setDataReceived( bool r )	{ _dataReceived = r; }
	bool getDataReceived()	{ return _dataReceived; }

 protected:
	void writeRegister( uint8_t reg, uint8_t data );
	uint8_t readRegister( uint8_t reg );

 private:
	void delay( int delay );

	spi_device_handle_t 	_spi;
	int 					_packetIndex = 0;
	int 					_implicitHeaderMode = 0;
	bool					_dataReceived = false;



};

#endif
