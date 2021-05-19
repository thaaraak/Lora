
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "font.h"

#ifndef OLED_h
#define OLED_h

#define OLED_ADDRESS 0x3C

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_NUM 		1
#define I2C_MASTER_FREQ_HZ 	200000						      /*!< I2C master clock frequency */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

// Header Values
#define JUMPTABLE_BYTES 4

#define JUMPTABLE_LSB   1
#define JUMPTABLE_SIZE  2
#define JUMPTABLE_WIDTH 3
#define JUMPTABLE_START 4

#define WIDTH_POS 0
#define HEIGHT_POS 1
#define FIRST_CHAR_POS 2
#define CHAR_NUM_POS 3


// Display commands
#define CHARGEPUMP 0x8D
#define COLUMNADDR 0x21
#define COMSCANDEC 0xC8
#define COMSCANINC 0xC0
#define DISPLAYALLON 0xA5
#define DISPLAYALLON_RESUME 0xA4
#define DISPLAYOFF 0xAE
#define DISPLAYON 0xAF
#define EXTERNALVCC 0x1
#define INVERTDISPLAY 0xA7
#define MEMORYMODE 0x20
#define NORMALDISPLAY 0xA6
#define PAGEADDR 0x22
#define SEGREMAP 0xA0
#define SETCOMPINS 0xDA
#define SETCONTRAST 0x81
#define SETDISPLAYCLOCKDIV 0xD5
#define SETDISPLAYOFFSET 0xD3
#define SETHIGHCOLUMN 0x10
#define SETLOWCOLUMN 0x00
#define SETMULTIPLEX 0xA8
#define SETPRECHARGE 0xD9
#define SETSEGMENTREMAP 0xA1
#define SETSTARTLINE 0x40
#define SETVCOMDETECT 0xDB
#define SWITCHCAPVCC 0x2

enum OLEDDISPLAY_COLOR
{
	BLACK = 0,
	WHITE = 1,
	INVERSE = 2
};

class OLED
{
 public:
	OLED( int width, int height, int sda, int scl, int reset );
    
	void drawString(int16_t xMove, int16_t yMove, const char *stringUser, OLEDDISPLAY_COLOR color );
	uint16_t getStringWidth(const char* text, uint16_t length);
	void setFont( const uint8_t* f ) { fontData = f; }
	void setPixelColor(int16_t x, int16_t y, OLEDDISPLAY_COLOR color);

	void clear();
	void sendDataBack();
	void sendData();


 protected:
	void drawStringInternal(int16_t xMove, int16_t yMove, char* text, uint16_t textLength, uint16_t textWidth, OLEDDISPLAY_COLOR color);
	void drawInternal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *data, uint16_t offset, uint16_t bytesInData, OLEDDISPLAY_COLOR color);

 private:
	void initializeReset( int reset );
	esp_err_t initializeI2C( int sda, int scl );
	void initialize();

	int writeRegister( uint8_t addr, uint8_t reg, uint8_t value);
	int writeData( uint8_t addr, uint8_t* buf, int len );

	void sendCommand( uint8_t command );

	uint8_t *_buffer;
	uint8_t *_buffer_back;

	const uint8_t* fontData = ArialMT_Plain_16;

	int _displayWidth;
	int _displayHeight;
	int _displayBufferSize;


};

#endif
