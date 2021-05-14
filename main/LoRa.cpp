/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "LoRa.h"

static const char *TAG = "LoRa";
#define ESP_INTR_FLAG_DEFAULT 0

LoRa::LoRa( int mosi, int miso, int clk, int cs, int reset, int dio )
{
	printf( "In LoRa cstor\n");

	initializeSPI( mosi, miso, clk, cs );
	initializeReset( reset );
	initializeDIO( dio );
	initialize();

}

void LoRa::initializeSPI( int mosi, int miso, int clk, int cs )
{
    esp_err_t ret;

    spi_bus_config_t buscfg;
	memset( &buscfg, 0, sizeof(spi_bus_config_t) );

    buscfg.mosi_io_num = mosi;
    buscfg.miso_io_num = miso;
	buscfg.sclk_io_num = clk;
	buscfg.quadwp_io_num = -1;
	buscfg.quadhd_io_num = -1;
	buscfg.max_transfer_sz = 0;
	buscfg.flags = 0;
	buscfg.intr_flags = 0;


    // Started working after reduce clock speed to 8MHz but then when I changed
    // back to 10 Mhz it continued working. Not sure whats going on

    spi_device_interface_config_t devcfg;
	memset( &devcfg, 0, sizeof(spi_device_interface_config_t) );

   	devcfg.address_bits = 8;
    devcfg.mode=0;
	devcfg.clock_speed_hz=200000;
	devcfg.spics_io_num=cs;
	devcfg.flags = SPI_DEVICE_HALFDUPLEX;
	devcfg.queue_size = 1;


    ret=spi_bus_initialize(SPI2_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    printf("Bus Init: %d\n", ret);

    if ( ret > 0 )
    	return;

    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);
    printf("Add device: %d\n", ret);


}

void LoRa::initializeReset( int reset )
{
	gpio_num_t r = (gpio_num_t) reset;

    gpio_pad_select_gpio( r );
    gpio_set_direction( r, GPIO_MODE_OUTPUT);

    gpio_set_level(r, 0);
    delay(20);
    gpio_set_level(r, 1);
    delay(50);

}

extern "C" {
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    LoRa *s = (LoRa*) arg;
    s->setDataReceived(true);
}
}

void LoRa::initializeDIO( int dio )
{
    gpio_config_t io_conf;
    gpio_num_t pin = (gpio_num_t) dio;

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << pin );
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add( pin, gpio_isr_handler, (void*) this );

}

void LoRa::initialize()
{
	sleep();

    uint8_t version = readRegister(REG_VERSION);
    printf( "Version: [%d]\n", version );
    setFrequency( 915E6 );

    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);
    writeRegister(REG_MODEM_CONFIG_3, 0x04);

    setTxPower(14, RF_PACONFIG_PASELECT_PABOOST);

    setSpreadingFactor(11);
	setSignalBandwidth(125E3);
	setSyncWord(0x34);

	setCRC( false );
	setCRC( true );

	idle();

}


void LoRa::sleep()
{
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRa::idle()
{
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}


void LoRa::setFrequency(long frequency)
{
	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
	writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
	writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
	writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRa::setSpreadingFactor(int sf)
{
	if (sf < 6)
		sf = 6;
	else if (sf > 12)
		sf = 12;

	if (sf == 6)
	{
		writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
	}
	else
	{
		writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
	}

	int val = (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0);
	writeRegister(REG_MODEM_CONFIG_2, val );
}

void LoRa::setSignalBandwidth(long sbw)
{
	int bw;

	if (sbw <= 7.8E3) { bw = 0; }
	else if (sbw <= 10.4E3) { bw = 1; }
	else if (sbw <= 15.6E3) { bw = 2; }
	else if (sbw <= 20.8E3) { bw = 3; }
	else if (sbw <= 31.25E3) { bw = 4; }
	else if (sbw <= 41.7E3) { bw = 5; }
	else if (sbw <= 62.5E3) { bw = 6; }
	else if (sbw <= 125E3) { bw = 7; }
	else if (sbw <= 250E3) { bw = 8; }
	else /*if (sbw <= 250E3)*/ { bw = 9; }
	writeRegister(REG_MODEM_CONFIG_1,(readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void LoRa::setSyncWord(int sw)
{
	writeRegister(REG_SYNC_WORD, sw);
}

void LoRa::setCRC( bool crc )
{
	if ( crc )
		writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
	else
		writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRa::setTxPower(int8_t power, int8_t outputPin)
{
	  uint8_t paConfig = 0;
	  uint8_t paDac = 0;

	  paConfig = readRegister( REG_PA_CONFIG );
	  paDac = readRegister( REG_PaDac );

	  paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | outputPin;
	  paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

	  if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
	  {
	    if( power > 17 )
	    {
	      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
	    }
	    else
	    {
	      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
	    }
	    if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
	    {
	      if( power < 5 )
	      {
	        power = 5;
	      }
	      if( power > 20 )
	      {
	        power = 20;
	      }
	      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
	    }
	    else
	    {
	      if( power < 2 )
	      {
	        power = 2;
	      }
	      if( power > 17 )
	      {
	        power = 17;
	      }
	      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
	    }
	  }
	  else
	  {
	    if( power < -1 )
	    {
	      power = -1;
	    }
	    if( power > 14 )
	    {
	      power = 14;
	    }
	    paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
	  }
	  writeRegister( REG_PA_CONFIG, paConfig );
	  writeRegister( REG_PaDac, paDac );
}

void LoRa::explicitHeaderMode()
{
  _implicitHeaderMode = 0;
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRa::implicitHeaderMode()
{
  _implicitHeaderMode = 1;
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

int LoRa::beginPacket(int implicitHeader)
{
  // put in standby mode
  idle();
  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }
  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);
  return 1;
}

int LoRa::endPacket(bool async)
{
  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  if (async) {
    // grace time is required for the radio
    delay(1);
  } else {
	// wait for TX done
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	//printf( "TX Done\n");
  }

  return 1;
}


size_t LoRa::write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);
  //printf( "Current: %d Size: %d\n" , currentLength, size );

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  //printf( "New Size: %d\n" , size );

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }
  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
  return size;
}

void LoRa::dumpRegisters()
{
  for (int i = 0; i < 128; i++)
	  printf( "%02x: %02x\n", i, readRegister(i) );
}

int LoRa::available()
{
	//printf( "RX Nbr Bytes: [%d]\n", readRegister(REG_RX_NB_BYTES) );
	return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRa::read()
{
	if ( !available() )
		return -1;

	_packetIndex++;
	return readRegister(REG_FIFO);
}

void LoRa::receive(int size)
{
	if (size > 0)
	{
		implicitHeaderMode();
		writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
	}
	else
		explicitHeaderMode();

	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRa::handleDataReceived()
{
	int irqFlags = readRegister(REG_IRQ_FLAGS);
	writeRegister(REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
	{
		int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);
		printf( "In handleDataReceived: len: %d fifo_addr: %d rx_addr: %d\n",
				packetLength,
				readRegister(REG_FIFO_ADDR_PTR),
				readRegister(REG_FIFO_RX_CURRENT_ADDR) );

		_packetIndex = 0;

		writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
//		if (_onReceive) { _onReceive(packetLength); }

		for (int i = 0; i < packetLength; i++)
			printf( "%c", read() );
		printf( "\n");

		writeRegister(REG_FIFO_ADDR_PTR, 0);
  }
}

int LoRa::parsePacket(int size)
{
	int packetLength = 0;

	if (size > 0)
	{
		implicitHeaderMode();
		writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
	}
	else
		explicitHeaderMode();

	// Check the IRQ_RX_DONE interrupt. If we have one that means there
	// is data ready to read. Clear the IRQ in any case.

	int irqFlags = readRegister(REG_IRQ_FLAGS);
	writeRegister(REG_IRQ_FLAGS, irqFlags);

  	if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
  	{
  		_packetIndex = 0;

  		if (_implicitHeaderMode)
  			packetLength = readRegister(REG_PAYLOAD_LENGTH);
  		else
  			packetLength = readRegister(REG_RX_NB_BYTES);

  		// set FIFO address to current RX address
  		writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
  		// put in standby mode
  		idle();
  	}
  	else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
  	{
    // not currently in RX mode
    // reset FIFO address
  		writeRegister(REG_FIFO_ADDR_PTR, 0);
  		writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  	}

  	return packetLength;
}



void LoRa::delay( int msec )
{
    vTaskDelay( msec / portTICK_PERIOD_MS);
}


void LoRa::writeRegister( uint8_t reg, uint8_t data )
{
//    printf("Writing Register [%02x]=[%02x]\n", reg, data);
	reg = reg | 0x80;

	spi_transaction_t transaction;
	memset( &transaction, 0, sizeof(spi_transaction_t) );

	transaction.length = 8;
	transaction.rxlength = 0;
	transaction.addr = reg;
	transaction.flags = SPI_TRANS_USE_TXDATA;

	memcpy(transaction.tx_data, &data, 1);

	esp_err_t err = spi_device_polling_transmit(_spi, &transaction);

	if (err != ESP_OK)
	    ESP_LOGE(TAG, "Error adding SPI device: %s", esp_err_to_name(err));
}

uint8_t LoRa::readRegister( uint8_t reg )
{
	uint8_t result;

	spi_transaction_t transaction;
	memset( &transaction, 0, sizeof(spi_transaction_t) );

	transaction.length = 0;
	transaction.rxlength = 8;
	transaction.addr = reg & 0x7f;
	transaction.flags = SPI_TRANS_USE_RXDATA;

	esp_err_t err = spi_device_polling_transmit( _spi, &transaction);

	if (err != ESP_OK)
	    ESP_LOGE(TAG, "Error adding SPI device: %s", esp_err_to_name(err));

	memcpy(&result, transaction.rx_data, 1);
//    printf("Reading Register [%02x]=[%02x]\n", reg, result);

	return result;
}

