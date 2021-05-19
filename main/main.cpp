#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <esp_log.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "LoRa.h"
#include "OLED.h"

#define PIN_NUM_MISO 	19
#define PIN_NUM_MOSI 	27
#define PIN_NUM_CLK  	5
#define PIN_NUM_CS   	18
#define PIN_NUM_DIO		26
#define RESET_PIN  		14

#define SENDER_RECEIVER_PIN	12
#define	FLASH_PIN			25

int _counter = 0;
OLED *_oled;
OLEDDISPLAY_COLOR _color = WHITE;


void writeMessage( LoRa* lora )
{
	char buf[100];
	lora->beginPacket(false);

	//sprintf( buf, "123456789012345678901234567890123456789012345678901234567890: [%d]", _counter++);
	sprintf( buf, "Count: [%d]", _counter++);

	//lora->setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
	lora->write( (uint8_t*) buf, (size_t) strlen(buf) );
	lora->endPacket(false);

	char msg[200];
	sprintf( msg, "Sender\n%s", buf );
	_oled->clear();
	_oled->drawString(0, 12, msg, _color );
	_oled->sendDataBack();

}

void readMessage( LoRa* lora )
{
	int packetSize = lora->parsePacket(0);

	if (packetSize)
	{
		char msg[200];
		sprintf( msg, "Receiver: %d", packetSize );
		_oled->clear();
		_oled->drawString(0, 12, msg, _color );
		_oled->sendDataBack();

		for (int i = 0; i < packetSize; i++)
			printf( "%c", lora->read() );

		printf( "\n");
	}
}

void delay( int msec )
{
    vTaskDelay( msec / portTICK_PERIOD_MS);
}

extern "C" void app_main();
extern "C" void lora_task( void *);

bool getConfigPin()
{
    gpio_config_t io_conf;
    gpio_num_t pin = (gpio_num_t) SENDER_RECEIVER_PIN;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << pin );
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    gpio_config(&io_conf);

    return gpio_get_level( pin ) == 0;
}

//void app_main()
void lora_task( void* param )
{

	_oled = new OLED( 128, 64, 4, 15, 16 );
	_oled->setFont(ArialMT_Plain_16 );
	_oled->clear();
	_oled->sendData();

	_oled->clear();
	_oled->drawString(0, 12, "Starting...", _color );
	_oled->sendDataBack();

	LoRa lora( PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, PIN_NUM_CS, RESET_PIN, PIN_NUM_DIO, 10 );

	bool _write = getConfigPin();

	gpio_num_t fp = (gpio_num_t) FLASH_PIN;
	gpio_pad_select_gpio( fp );
	gpio_set_direction( fp , GPIO_MODE_OUTPUT);


	if ( !_write )
		lora.receive(0);

	for ( ;; )
	{

		if ( _write )
		{
			writeMessage( &lora );

			gpio_set_level( fp, 1);
			delay(100);

			gpio_set_level( fp, 0);
			delay(1000);
		}

		if ( !_write && lora.getDataReceived() )
		{
			for ( int i = 0 ; i < 3 ; i++)
			{
				gpio_set_level( fp, 1);
				delay(50);
				gpio_set_level( fp, 0);
				delay(50);
			}

			char buf[200];
			char msg[100];

			int packetSize = lora.handleDataReceived( msg );
			lora.setDataReceived( false );

			sprintf( buf, "<%10s>\n(%d) RSSI: %d", msg, packetSize, lora.getPacketRssi() );

			_oled->clear();
			_oled->drawString(0, 12, buf, _color );
			_oled->sendDataBack();
		}

		vTaskDelay(10);

	}

}


void app_main()
{
	// Had to set the task size to 10k otherwise I would get various instabilities
	// Around 2k or less I would get the stack overflow warning but at 2048 it would
	// just crash in various random ways

	xTaskCreate(lora_task, "lora_task", 10000, NULL, 1, NULL);
}

