#include "oled_small.h"

// OLED initialisation sequence
const uint8_t OLED_INIT_CMD[] = {
  OLED_MULTIPLEX,   0x3F,                 // set multiplex ratio  
  OLED_CHARGEPUMP,  0x14,                 // set DC-DC enable  
  OLED_MEMORYMODE,  0x02,                 // set page addressing mode
  OLED_COMPINS,     0x12,                 // set com pins
  OLED_XFLIP, OLED_YFLIP,                 // flip screen
  OLED_DISPLAY_ON                         // display on
};

const uint8_t ssd1306_init_sequence [] = {	// Initialization Sequence
	0xAE,			// Set Display ON/OFF - AE=OFF, AF=ON
	0xD5, 0xF0,		// Set display clock divide ratio/oscillator frequency, set divide ratio
	0xA8, 0x3F,		// Set multiplex ratio (1 to 64) ... (height - 1)
	0xD3, 0x00,		// Set display offset. 00 = no offset
	0x40 | 0x00,	// Set start line address, at 0.
	0x8D, 0x14,		// Charge Pump Setting, 14h = Enable Charge Pump
	0x20, 0x00,		// Set Memory Addressing Mode - 00=Horizontal, 01=Vertical, 10=Page, 11=Invalid
	0xA0 | 0x01,	// Set Segment Re-map
	0xC8,			// Set COM Output Scan Direction
	0xDA, 0x12,		// Set COM Pins Hardware Configuration - 128x32:0x02, 128x64:0x12
	0x81, 0x3F,		// Set contrast control register - 0x01 to 0xFF - Default: 0x3F
	0xD9, 0x22,		// Set pre-charge period (0x22 or 0xF1)
	0xDB, 0x20,		// Set Vcomh Deselect Level - 0x00: 0.65 x VCC, 0x20: 0.77 x VCC (RESET), 0x30: 0.83 x VCC
	0xA4,			// Entire Display ON (resume) - output RAM to display
	0xA6,			// Set Normal/Inverse Display mode. A6=Normal; A7=Inverse
	0x2E,			// Deactivate Scroll command
	0xAF,			// Set Display ON/OFF - AE=OFF, AF=ON
	0x22, 0x00, 0x3f,	// Set Page Address (start,end) 0 - 63
	0x21, 0x00,	0x7f,	// Set Column Address (start,end) 0 - 127
};

// OLED global variables
uint8_t line, column, scroll;

typedef struct {
    uint16_t unicode;
    uint8_t  cp437;
} unicode_to_cp437_entry_t;

static const unicode_to_cp437_entry_t unicode_to_cp437_table[] = {
    // Заглавные
    {0x0410, 0x80}, // А
    {0x0411, 0x81}, // Б
    {0x0412, 0x82}, // В
    {0x0413, 0x83}, // Г
    {0x0414, 0x84}, // Д
    {0x0415, 0x85}, // Е
    {0x0416, 0x86}, // Ж
    {0x0417, 0x87}, // З
    {0x0418, 0x88}, // И
    {0x0419, 0x89}, // Й
    {0x041A, 0x8A}, // К
    {0x041B, 0x8B}, // Л
    {0x041C, 0x8C}, // М
    {0x041D, 0x8D}, // Н
    {0x041E, 0x8E}, // О
    {0x041F, 0x8F}, // П
    {0x0420, 0x90}, // Р
    {0x0421, 0x91}, // С
    {0x0422, 0x92}, // Т
    {0x0423, 0x93}, // У
    {0x0424, 0x94}, // Ф
    {0x0425, 0x95}, // Х
    {0x0426, 0x96}, // Ц
    {0x0427, 0x97}, // Ч
    {0x0428, 0x98}, // Ш
    {0x0429, 0x99}, // Щ
    {0x042A, 0x9A}, // Ъ
    {0x042B, 0x9B}, // Ы
    {0x042C, 0x9C}, // Ь
    {0x042D, 0x9D}, // Э
    {0x042E, 0x9E}, // Ю
    {0x042F, 0x9F}, // Я
    {0x0401, 0xF0}, // Ё
    // Строчные
    {0x0430, 0xA0}, // а
    {0x0431, 0xA1}, // б
    {0x0432, 0xA2}, // в
    {0x0433, 0xA3}, // г
    {0x0434, 0xA4}, // д
    {0x0435, 0xA5}, // е
    {0x0436, 0xA6}, // ж
    {0x0437, 0xA7}, // з
    {0x0438, 0xA8}, // и
    {0x0439, 0xA9}, // й
    {0x043A, 0xAA}, // к
    {0x043B, 0xAB}, // л
    {0x043C, 0xAC}, // м
    {0x043D, 0xAD}, // н
    {0x043E, 0xAE}, // о
    {0x043F, 0xAF}, // п
    {0x0440, 0xE0}, // р
    {0x0441, 0xE1}, // с
    {0x0442, 0xE2}, // т
    {0x0443, 0xE3}, // у
    {0x0444, 0xE4}, // ф
    {0x0445, 0xE5}, // х
    {0x0446, 0xE6}, // ц
    {0x0447, 0xE7}, // ч
    {0x0448, 0xE8}, // ш
    {0x0449, 0xE9}, // щ
    {0x044A, 0xEA}, // ъ
    {0x044B, 0xEB}, // ы
    {0x044C, 0xEC}, // ь
    {0x044D, 0xED}, // э
    {0x044E, 0xEE}, // ю
    {0x044F, 0xEF}, // я
    {0x0451, 0xF1}, // ё
};

// OLED set cursor to line start
void OLED_setline(uint8_t line) 
{
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  I2C_write(OLED_PAGE + line);            // set line
  I2C_write(0x00); I2C_write(0x10);       // set column to "0"
  I2C_stop();                             // stop transmission
}

// OLED clear line
void OLED_clearline(uint8_t line) 
{
  uint8_t i;
  OLED_setline(line);                     // set cursor to line start
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  for(i=128; i; i--) {                    // clear the line
    I2C_write(0x00);
  }     
  I2C_stop();                             // stop transmission
}

// OLED clear screen and buffer
void OLED_clear(void) 
{
  uint16_t i;
		OLED_setpos(0, 0);                            // set cursor to first digit
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  for(i=128*8; i; i--) {                          // clear screen and buffer
    I2C_write(0x00);
    }           
  I2C_stop();                                     // stop transmission
}

// OLED clear the top line, then scroll the display up by one line
void OLED_scrollDisplay(void) 
{
  OLED_clearline(scroll);                 // clear line
  scroll = (scroll + 1) & 0x07;           // set next line
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  I2C_write(OLED_OFFSET);                 // set display offset:
  I2C_write(scroll << 3);                 // scroll up
  I2C_stop();                             // stop transmission
}

// OLED init function
void OLED_init(void) 
{
  uint8_t i;
  I2C_init();                             // initialize I2C first
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  for(i = 0; i < sizeof(ssd1306_init_sequence); i++)
    I2C_write(ssd1306_init_sequence[i]);          // send the command bytes
  I2C_stop();                             // stop transmission
  scroll = 0;                             // start with zero scroll
  column = 0;
  line = 0; 
  OLED_clear();                           // clear screen
  OLED_setpos(0,0);
}

uint8_t utf8_to_cp437(const char **p) {
    uint8_t c = (uint8_t)**p;
    if (c < 0x80) {
        (*p)++; 
        return c;
    }
  
    if ((c & 0xE0) == 0xC0) {
        uint8_t c2 = (uint8_t)*(*p + 1);
        uint16_t unicode = ((c & 0x1F) << 6) | (c2 & 0x3F);
  
        for (size_t i = 0; i < sizeof(unicode_to_cp437_table)/sizeof(unicode_to_cp437_table[0]); i++) {
            if (unicode_to_cp437_table[i].unicode == unicode) {
                *p += 2;
                return unicode_to_cp437_table[i].cp437;
            }
        }
        (*p)++; 
        return '?';
    }

    if ((c & 0xF0) == 0xE0) { *p += 3; }
    else if ((c & 0xF8) == 0xF0) { *p += 4; }
    else { (*p)++; }
    return '?';
}
// OLED plot a single character
void OLED_plotChar(uint8_t c, bool inverted) 
{
  const uint8_t *glyph = ibm8x8_font_cp437[c];
  I2C_start(OLED_ADDR);
  I2C_write(OLED_DAT_MODE);
  for (short i = 0; i < 8; i++) {
    I2C_write(inverted ? ~glyph[i] : glyph[i]);
  }
  I2C_write(inverted ? ~0x00 : 0x00);
  I2C_stop();
}

// OLED write a character or handle control characters
void OLED_write(uint8_t c, bool inverted) 
{
  if(c == '\n') {
    column = 0;
    if(line != 7) line++;
    OLED_setline((line + scroll) & 0x07);
  }
  else if(c == '\r') {
    column = 0;
    OLED_setline((line + scroll) & 0x07);
  }
  OLED_plotChar(c, inverted);
}

// OLED print string
void OLED_print(char* str) 
{
  while(*str) {
    OLED_write(*str++, false);
  }
}

// OLED print string
void OLED_printS(const char* str, bool inverted) 
{
  while (*str) {
    uint8_t cp437 = utf8_to_cp437(&str);
    OLED_write(cp437, inverted);
  }
}

// OLED print string with newline
void OLED_println(char* str, bool inverted) 
{
  OLED_printS(str, inverted);
  OLED_write('\n', inverted);
}

// For BCD conversion
const uint32_t DIVIDER[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

// Print decimal value (BCD conversion by substraction method)
void OLED_printD(uint32_t value, bool inverted) 
{
  uint8_t digits   = 10;                          // print 10 digits
  uint8_t leadflag = 0;                           // flag for leading spaces
  while(digits--) {                               // for all digits
    uint8_t digitval = 0;                         // start with digit value 0
    uint32_t divider = DIVIDER[digits];           // read current divider
    while(value >= divider) {                     // if current divider fits into the value
      leadflag = 1;                               // end of leading spaces
      digitval++;                                 // increase digit value
      value -= divider;                           // decrease value by divider
    }
    if(!digits)  leadflag++;                      // least digit has to be printed
    if(leadflag) OLED_write(digitval + '0', inverted);      // print the digit
  }
}

void OLED_printF(float value, uint8_t precision, bool inverted) 
{
  static const uint32_t pow10[7] = {1, 10, 100, 1000, 10000, 100000, 1000000};
  if (precision > 6) 
    precision = 6;
  if (value < 0) {
    OLED_write('-', inverted);
    value = -value;
  }
    // Округление
  value += 0.5f / pow10[precision];
    // Целая часть
  uint32_t int_part = (uint32_t)value;
  OLED_printD(int_part, inverted);

    // Дробная часть
  if (precision > 0) {
    OLED_write('.', inverted);
      // Получаем дробную часть как целое число
    float frac_part = value - int_part;
    uint32_t frac_int = (uint32_t)(frac_part * pow10[precision]);
        // Выводим с ведущими нулями
    uint8_t digits = precision;
    while (digits--) {
      uint32_t divider = pow10[digits];
      uint8_t digitval = 0;
      while (frac_int >= divider) {
        digitval++;
        frac_int -= divider;
      }
      OLED_write(digitval + '0', inverted);
    }
  }
}

// Convert byte nibble into hex character and print it
void OLED_printN(uint8_t nibble, bool inverted) 
{
  OLED_write((nibble <= 9) ? ('0' + nibble) : ('A' - 10 + nibble), inverted);
}

// Convert byte into hex characters and print it
void OLED_printB(uint8_t value, bool inverted) 
{
  OLED_printN(value >> 4, inverted);
  OLED_printN(value & 0x0f, inverted);
}

// Convert word into hex characters and print it
void OLED_printW(uint16_t value, bool inverted) 
{
  OLED_printB(value >> 8, inverted);
  OLED_printB(value, inverted);
}

// Convert long into hex characters and print it
void OLED_printL(uint32_t value, bool inverted) 
{
  OLED_printW(value >> 16, inverted);
  OLED_printW(value, inverted);
}

// OLED set cursor position 
void OLED_setpos(uint8_t x, uint8_t y) 
{
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  I2C_write(OLED_PAGE | y);	              // set page start address
  I2C_write(x & 0x0F);			              // set lower nibble of start column
  I2C_write(OLED_COLUMN_HIGH | (x >> 4)); // set higher nibble of start column
  I2C_stop();                             // stop transmission
}

void ssd1306_start_data(void) 
{
  I2C_start(OLED_ADDR);   
	I2C_write(0x40);			// Control byte: D/C=1 - write data
}

// OLED fill screen
void OLED_fill(uint8_t p) {
  OLED_setpos(0, 0);                      // set cursor to display start
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  for(uint16_t i=128*8; i; i--){          // send pattern
    I2C_write(p);
  } 
  I2C_stop();                             // stop transmission
}

// OLED draw bitmap
void OLED_DrawBitmap(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, const uint8_t* bmp, bool inverted) 
{
	int z=0;
  for(uint8_t y = y0; y < y0+(h/8); y++) {
    OLED_setpos(x0, y);
    I2C_start(OLED_ADDR);
    I2C_write(OLED_DAT_MODE);
    for(uint8_t x = x0; x < x0+w; x++)
		{
      I2C_write(inverted? ~(bmp[z]) : bmp[z] );
			z++;
		}
    I2C_stop();
  }
}

// float value symbol length for positioning
size_t float_num_len(float value, uint8_t decimals)
{
    uint8_t buffer[16] = {};
    uint8_t i = 0;
    if (value < 0) {
        buffer[i] = '-';
        value = -value;
    }

    uint32_t factor = 1;
    for (uint8_t i = 0; i < decimals; i++) factor *= 10;
    uint32_t scaled = (uint32_t)(value * factor + 0.5f);

    uint32_t int_part = scaled / factor;
    uint32_t frac_part = scaled % factor;

    uint8_t temp[16];
    uint8_t j = 0;
    if (int_part == 0) {
        temp[j++] = '0';
    } else {
        uint32_t num = int_part;
        while (num > 0) {
            temp[j++] = '0' + (num % 10);
            num /= 10;
        }
    }
    while (j > 0) {
        buffer[i++] = temp[--j];
    }

    buffer[i++] = '.';

    uint32_t divisor = factor / 10;
    uint32_t frac = frac_part;
    for (uint8_t d = 0; d < decimals; d++) {
        buffer[i++] = '0' + (frac / divisor);
        frac %= divisor;
        divisor /= 10;
    }

    buffer[i++] = '\0';

    return strlen(buffer);
}