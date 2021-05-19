/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "OLED.h"

static const char *TAG = "OLED";
#define ESP_INTR_FLAG_DEFAULT 0

OLED::OLED( int width, int height, int sda, int scl, int reset )
{
	_displayWidth = width;
	_displayHeight = height;
    _displayBufferSize = _displayWidth * _displayHeight/8;

    _buffer = (uint8_t*) malloc((sizeof(uint8_t) * _displayBufferSize) );
  	_buffer_back = (uint8_t*) malloc((sizeof(uint8_t) * _displayBufferSize) );
    memset(_buffer, 0, _displayBufferSize);
    memset(_buffer_back, 0, _displayBufferSize);

	initializeReset( reset );
	initializeI2C( sda, scl );
	initialize();

}

esp_err_t OLED::initializeI2C( int sda, int scl )
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void OLED::initializeReset( int reset )
{
	gpio_num_t r = (gpio_num_t) reset;

    gpio_pad_select_gpio( r );
    gpio_set_direction( r, GPIO_MODE_OUTPUT);

    gpio_set_level(r, 0);
    vTaskDelay( 50 / portTICK_PERIOD_MS);
    gpio_set_level(r, 1);
    vTaskDelay( 50 / portTICK_PERIOD_MS);
}


void OLED::initialize()
{
	sendCommand(DISPLAYOFF);
	sendCommand(SETDISPLAYCLOCKDIV);
	sendCommand(0xF0); // Increase speed of the display max ~96Hz
	sendCommand(SETMULTIPLEX);
	sendCommand(_displayHeight - 1);
	sendCommand(SETDISPLAYOFFSET);
	sendCommand(0x00);
	sendCommand(SETSTARTLINE);
	sendCommand(CHARGEPUMP);
	sendCommand(0x14);
	sendCommand(MEMORYMODE);
	sendCommand(0x00);
	sendCommand(SEGREMAP|0x01);
	sendCommand(COMSCANDEC);
	sendCommand(SETCOMPINS);

	if ( _displayHeight == 64 )
	    sendCommand(0x12);
	else
	    sendCommand(0x02);

	sendCommand(SETCONTRAST);
    sendCommand(0xCF);

    sendCommand(SETPRECHARGE);
    sendCommand(0xF1);
    sendCommand(SETVCOMDETECT); //0xDB, (additionally needed to lower the contrast)
    sendCommand(0x40);	        //0x40 default, to lower the contrast, put 0
    sendCommand(DISPLAYALLON_RESUME);
    sendCommand(NORMALDISPLAY);
    sendCommand(0x2e);            // stop scroll
    sendCommand(DISPLAYON);

}

int OLED::writeData( uint8_t addr, uint8_t* buf, int len )
{
    uint8_t buffer = 0x40;

    //printf( "Writing Data: [%d]\n", len );

    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, buffer, ACK_CHECK_EN);

    i2c_master_write(cmd, buf, len - 1, ACK_VAL);
    i2c_master_write_byte(cmd, buf[len - 1], NACK_VAL);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    //printf( " Return: %d\n", ret );

    if (ret != ESP_OK) {
        return ret;
    }

    return 0;
}

int OLED::writeRegister( uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buffer[3];

    //printf( "Writing [%d]=[%d]\n ", reg, value );

    buffer[0] = reg;
    buffer[1] = value;

    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, &buffer[0], 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK( (ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS) ) );
    i2c_cmd_link_delete(cmd);

    //printf( " Return: %d\n", ret );

    if (ret != ESP_OK) {
        return ret;
    }

    return 0;
}


void OLED::sendCommand( uint8_t command )
{
	writeRegister( OLED_ADDRESS, 0x00, command );
}

void OLED::sendDataBack()
{

	uint8_t minBoundY = UINT8_MAX;
	uint8_t maxBoundY = 0;

	uint8_t minBoundX = UINT8_MAX;
	uint8_t maxBoundX = 0;
	uint8_t x, y;


	// In double buffered mode we first determine the area of difference
	// between the current write buffer and the previously saved back buffer
	// These bounds become the start column and page later on

	for (y = 0; y < _displayHeight / 8 ; y++)
	{
		for (x = 0; x < _displayWidth; x++)
		{
			uint16_t pos = x + y * _displayWidth;
			if (_buffer[pos] != _buffer_back[pos])
			{
				minBoundY = fmin(minBoundY, y);
				maxBoundY = fmax(maxBoundY, y);
				minBoundX = fmin(minBoundX, x);
				maxBoundX = fmax(maxBoundX, x);
			}
			_buffer_back[pos] = _buffer[pos];
		}
		//yield();
	}

	if (minBoundY == UINT8_MAX) return;

	// Set the start column and page to the previously calulated bounds

	sendCommand(COLUMNADDR);
	sendCommand( minBoundX);
	sendCommand( maxBoundX);

	sendCommand(PAGEADDR);
	sendCommand(minBoundY);
	sendCommand(maxBoundY);

	// Now write out the changed portion of the buffer in 16 byte blocks

	uint8_t sbuf[16];
	int idx = 0;

	for (y = minBoundY; y <= maxBoundY; y++)
	{
		for (x = minBoundX; x <= maxBoundX; x++)
		{
			sbuf[idx++] = _buffer[x + y * _displayWidth];
			if ( idx == 16 )
			{
				writeData( OLED_ADDRESS, &sbuf[0], 16 );
				idx = 0;
			}
		}
	}

	// Finally write out the remaining left over block

	if ( idx > 0 )
		writeData( OLED_ADDRESS, &sbuf[0], idx );


}



void OLED::sendData()
{
	sendCommand(COLUMNADDR);
	sendCommand(0);
	sendCommand((_displayWidth - 1));

	sendCommand(PAGEADDR);
	sendCommand(0x0);

	if ( _displayHeight == 64 )
		sendCommand(0x7);
	else
		sendCommand(0x3);

	for ( uint16_t i = 0; i < _displayBufferSize/16; i++)
	{
		writeData( OLED_ADDRESS, &_buffer[i*16], 16 );
	}

}

void OLED::clear()
{
	memset(_buffer, 0, _displayBufferSize);
}

void OLED::setPixelColor(int16_t x, int16_t y, OLEDDISPLAY_COLOR color)
{
	switch (color)
	{
      case WHITE:   _buffer[x + (y >> 3) * _displayWidth ] |=  (1 << (y & 7)); break;
      case BLACK:   _buffer[x + (y >> 3) * _displayWidth ] &= ~(1 << (y & 7)); break;
      case INVERSE: _buffer[x + (y >> 3) * _displayWidth ] ^=  (1 << (y & 7)); break;
    }
}

void OLED::drawInternal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *data, uint16_t offset, uint16_t bytesInData, OLEDDISPLAY_COLOR color)
{
  if (width < 0 || height < 0) return;
  if (yMove + height < 0 || yMove > _displayHeight)  return;
  if (xMove + width  < 0 || xMove > _displayWidth )   return;

  uint8_t  rasterHeight = 1 + ((height - 1) >> 3); // fast ceil(height / 8.0)
  int8_t   yOffset      = yMove & 7;

  bytesInData = bytesInData == 0 ? width * rasterHeight : bytesInData;

  int16_t initYMove   = yMove;
  int8_t  initYOffset = yOffset;


  for (uint16_t i = 0; i < bytesInData; i++) {

    // Reset if next horizontal drawing phase is started.
    if ( i % rasterHeight == 0) {
      yMove   = initYMove;
      yOffset = initYOffset;
    }

    uint8_t currentByte = *(data + offset + i);

    int16_t xPos = xMove + (i / rasterHeight);
    int16_t yPos = ((yMove >> 3) + (i % rasterHeight)) * _displayWidth;

    int16_t dataPos    = xPos  + yPos;

    if (dataPos >=  0  && dataPos < _displayBufferSize &&
        xPos    >=  0  && xPos    < _displayWidth ) {

      if (yOffset >= 0) {
        switch (color) {
          case WHITE:   _buffer[dataPos] |= currentByte << yOffset; break;
          case BLACK:   _buffer[dataPos] &= ~(currentByte << yOffset); break;
          case INVERSE: _buffer[dataPos] ^= currentByte << yOffset; break;
        }

        if (dataPos < (_displayBufferSize - _displayWidth)) {
          switch (color) {
            case WHITE:   _buffer[dataPos + _displayWidth] |= currentByte >> (8 - yOffset); break;
            case BLACK:   _buffer[dataPos + _displayWidth] &= ~(currentByte >> (8 - yOffset)); break;
            case INVERSE: _buffer[dataPos + _displayWidth] ^= currentByte >> (8 - yOffset); break;
          }
        }
      } else {
        // Make new offset position
        yOffset = -yOffset;

        switch (color) {
          case WHITE:   _buffer[dataPos] |= currentByte >> yOffset; break;
          case BLACK:   _buffer[dataPos] &= ~(currentByte >> yOffset); break;
          case INVERSE: _buffer[dataPos] ^= currentByte >> yOffset; break;
        }

        // Prepare for next iteration by moving one block up
        yMove -= 8;

        // and setting the new yOffset
        yOffset = 8 - yOffset;
      }
    }
  }
}


void OLED::drawStringInternal(int16_t xMove, int16_t yMove, char* text, uint16_t textLength, uint16_t textWidth, OLEDDISPLAY_COLOR color)
{
  uint8_t textHeight       = *(fontData + HEIGHT_POS);
  uint8_t firstChar        = *(fontData + FIRST_CHAR_POS);
  uint16_t sizeOfJumpTable = *(fontData + CHAR_NUM_POS)  * JUMPTABLE_BYTES;

  uint16_t cursorX         = 0;
  uint16_t cursorY         = 0;

  // Don't draw anything if it is not on the screen.
  if (xMove + textWidth  < 0 || xMove > _displayWidth ) {return;}
  if (yMove + textHeight < 0 || yMove > _displayHeight ) {return;}

  for (uint16_t j = 0; j < textLength; j++) {
    int16_t xPos = xMove + cursorX;
    int16_t yPos = yMove + cursorY;

    uint8_t code = text[j];
    if (code >= firstChar) {
      uint8_t charCode = code - firstChar;

      // 4 Bytes per char code
      uint8_t msbJumpToChar    = *( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES );                  // MSB  \ JumpAddress
      uint8_t lsbJumpToChar    = *( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_LSB);   // LSB /
      uint8_t charByteSize     = *( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_SIZE);  // Size
      uint8_t currentCharWidth = *( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_WIDTH); // Width

      // Test if the char is drawable
      if (!(msbJumpToChar == 255 && lsbJumpToChar == 255)) {
        // Get the position of the char data
        uint16_t charDataPosition = JUMPTABLE_START + sizeOfJumpTable + ((msbJumpToChar << 8) + lsbJumpToChar);
        drawInternal(xPos, yPos, currentCharWidth, textHeight, fontData, charDataPosition, charByteSize, color);
      }

      cursorX += currentCharWidth;
    }
  }
}

uint16_t OLED::getStringWidth(const char* text, uint16_t length)
{
  uint16_t firstChar        = *(fontData + FIRST_CHAR_POS);

  uint16_t stringWidth = 0;
  uint16_t maxWidth = 0;

  while (length--) {
    stringWidth += *(fontData + JUMPTABLE_START + (text[length] - firstChar) * JUMPTABLE_BYTES + JUMPTABLE_WIDTH);
    if (text[length] == 10) {
      maxWidth = fmax(maxWidth, stringWidth);
      stringWidth = 0;
    }
  }

  return fmax(maxWidth, stringWidth);
}

void OLED::drawString(int16_t xMove, int16_t yMove, const char *stringUser, OLEDDISPLAY_COLOR color )
{
	uint16_t lineHeight = *(fontData + HEIGHT_POS);

	uint16_t yOffset = 0;
	uint16_t line = 0;

	char *t = (char *) malloc( (size_t)strlen(stringUser)+1 );
	strcpy( t, stringUser );
	char* rest = t;
	char* textPart;

	while ( ( textPart = strtok_r( rest, "\n", &rest)) != NULL)
	{
		uint16_t length = strlen(textPart);
		drawStringInternal(xMove, yMove - yOffset + (line++) * lineHeight, textPart, length, getStringWidth(textPart, length), color);
	}

	free(t);

}
