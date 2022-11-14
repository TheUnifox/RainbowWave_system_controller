#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <string>

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "RA8875.h"
#include "pico/time.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))

inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c){
  return gfxFont->glyph+c;
}

inline uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont) {
  return gfxFont->bitmap;
}

raspiRA8875::raspiRA8875(uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso) {
  cs_pin = cs;
  sck_pin = sck;
  mosi_pin = mosi;
  miso_pin = miso;
  spi_init(spi0, 400 * 1000);
  gpio_set_function(cs_pin, GPIO_FUNC_SPI);
  gpio_set_function(sck_pin, GPIO_FUNC_SPI);
  gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
  gpio_set_function(miso_pin, GPIO_FUNC_SPI);
}

uint8_t raspiRA8875::readData(void) {
  gpio_put(cs_pin, 0);
  uint8_t temp = RA8875_DATAREAD;
  spi_write_blocking(spi0, &temp, 1);
  uint8_t c;
  spi_read_blocking(spi0, 0x0, &c, 1);
  gpio_put(cs_pin, 1);
  return c;
}

void raspiRA8875::writeData(uint8_t c) {
  gpio_put(cs_pin, 0);
  uint8_t d = 0x00;
  spi_write_blocking(spi0, &d, 1);
  spi_write_blocking(spi0, &c, 1);
  gpio_put(cs_pin, 1);
}

void raspiRA8875::writeCommand(uint8_t c) {
  gpio_put(cs_pin, 0);
  uint8_t d = 0x80;
  spi_write_blocking(spi0, &d, 1);
  spi_write_blocking(spi0, &c, 1);
  gpio_put(cs_pin, 1);
}

uint8_t raspiRA8875::reg_read( spi_inst_t *spi, const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t mb = 0;

    // Determine if multiple byte (MB) bit should be set
    if (nbytes < 1) {
        return -1;
    } else if (nbytes == 1) {
        mb = 0;
    } else {
        mb = 1;
    }

    // Construct message (set ~W bit high)
    uint8_t msg = 0x80 | (mb << 6) | reg;

    // Read from register
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, &msg, 1);
    num_bytes_read = spi_read_blocking(spi, 0, buf, nbytes);
    gpio_put(cs_pin, 1);

    return num_bytes_read;
}

void raspiRA8875::reg_write(spi_inst_t *spi, const uint8_t reg, uint8_t data) {
  uint8_t msg[2];
                
  // Construct message (set ~W bit low, MB bit low)
  msg[0] = 0x00 | reg;
  msg[1] = data;

  // Write to register
  gpio_put(cs_pin, 0);
  spi_write_blocking(spi, msg, 2);
  gpio_put(cs_pin, 1);
}

void raspiRA8875::waitBusy(uint8_t res) {
  uint8_t temp;
  unsigned long start = us_to_ms(time_us_64());
  do {
  	if (res == 0x01) writeCommand(RA8875_DMACR);
  	gpio_put(cs_pin, 0);
    uint8_t temp1 = RA8875_CMDREAD;
  	spi_write_blocking(spi0, &temp1, 1);
  	spi_read_blocking(spi0, 0x0, &temp, 1);
  	gpio_put(cs_pin, 1);
  	if ((us_to_ms(time_us_64()) - start) > 10) return;
  } while ((temp & res) == res);
}

bool raspiRA8875::waitPoll(uint8_t regname, uint8_t waitflag) {
  while (1) {
    writeCommand(regname);
    uint8_t temp = readData();
    if (!(temp & waitflag))
      return true;
  }
  return false;
}

int16_t raspiRA8875::applyRotationX(int16_t x) {
  switch (_rotation) {
  case 2:
    x = _width - 1 - x;
    break;
  }
  return x;
}

int16_t raspiRA8875::applyRotationY(int16_t y) {
  switch (_rotation) {
  case 2:
    y = _height - 1 - y;
    break;
  }
  return y + _voffset;
}

bool raspiRA8875::displayBegin(enum RA8875sizes size) {
  _size = size;
  uint8_t pixclk;
  uint8_t hsync_start;
  uint8_t hsync_pw;
  uint8_t hsync_finetune;
  uint8_t hsync_nondispl;
  uint8_t vsync_pw;
  uint16_t vsync_nondisp;
  uint16_t vsync_start;
  if (_size == RA8875_480x80) {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondispl = 10;
    hsync_start = 8;
    hsync_pw = 48;
    hsync_finetune = 0;
    vsync_nondisp = 3;
    vsync_start = 8;
    vsync_pw = 10;
    _voffset = 192;
    _width = 480;
    _height = 80;
  } else if (_size == RA8875_480x128) {
    _width = 480;
    _height = 128;
  } else if (_size == RA8875_480x272) {
    _width = 480;
    _height = 272;
  } else if (_size == RA8875_800x480) {
    _width = 800;
    _height = 480;
  } else return false;
  if(spi_init(spi0, 10000000) == -1) {
    printf("BCM2835 init error!\n");
    return false;
  }
  gpio_put(cs_pin, 0);
  gpio_put(cs_pin, 1);
  sleep_ms(100);;
  reg_write(spi0,RA8875_P1CR,(RA8875_P1CR_ENABLE | (RA8875_PWM_CLK_DIV1024 & 0xF)));
  reg_write(spi0,RA8875_P1DCR,255);
  reg_write(spi0,RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
  sleep_ms(1);
  reg_write(spi0,RA8875_PLLC2, RA8875_PLLC2_DIV4);
  sleep_ms(1);
  reg_write(spi0,RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);
  reg_write(spi0,RA8875_PCSR, RA8875_PCSR_PDATL | RA8875_PCSR_4CLK);
  sleep_ms(1);
  reg_write(spi0,RA8875_HDWR, (800 / 8) - 1);
  reg_write(spi0,RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + 0);
  reg_write(spi0,RA8875_HNDR, (26 - 0 - 2)/8);
  reg_write(spi0,RA8875_HSTR, 23 / 8 -1);
  reg_write(spi0,RA8875_HPWR, RA8875_HPWR_LOW + (2 / 8 - 1));
  reg_write(spi0,RA8875_VDHR0, (uint16_t)(480 -1 + 0) & 0xFF);
  reg_write(spi0,RA8875_VDHR1, (uint16_t)(480 -1 + 0) >> 8);
  reg_write(spi0,RA8875_VNDR0, 26 - 1);
  reg_write(spi0,RA8875_VNDR1, 26 >> 8);
  reg_write(spi0,RA8875_VSTR0, 32 - 1);
  reg_write(spi0,RA8875_VSTR1, 32 >> 8);
  reg_write(spi0,RA8875_VPWR, RA8875_VPWR_LOW + 2 - 1);
  reg_write(spi0,RA8875_HSAW0, 0);
  reg_write(spi0,RA8875_HSAW1, 0);
  reg_write(spi0,RA8875_HEAW0, (uint16_t)(800 - 1) & 0xFF);
  reg_write(spi0,RA8875_HEAW1, (uint16_t)(800 - 1) >> 8);
  reg_write(spi0,RA8875_VSAW0, 0 + 0);
  reg_write(spi0,RA8875_VSAW1, 0 + 0);
  reg_write(spi0,RA8875_VEAW0, (uint16_t)(480 - 1 + 0) & 0xFF);
  reg_write(spi0,RA8875_VEAW1, (uint16_t)(480 - 1 + 0) >> 8);
  // Clear the entire window
  reg_write(spi0,RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
  sleep_ms(500);
  return true;
}

void raspiRA8875::displayOn(bool on) {
  if (on) reg_write(spi0,RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
  else reg_write(spi0,RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}

void raspiRA8875::textMode(void) {
  _textMode = true;
  writeCommand(RA8875_MWCR0);
  uint8_t temp = readData();
  temp |= RA8875_MWCR0_TXTMODE;
  writeData(temp);
  writeCommand(0x21);
  temp = readData();
  temp &= ~((1 << 7) | (1 << 5));
  writeData(temp);
}

void raspiRA8875::graphicsMode(void) {
  writeCommand(RA8875_MWCR0);
  uint8_t temp = readData();
  temp &= ~RA8875_MWCR0_TXTMODE;
  writeData(temp);
}

void raspiRA8875::textWrite(const char *buffer) {
  uint16_t len = strlen(buffer);
  for (uint16_t i = 0; i < len; i++) {
  	textEnlarge(_scale);
  	writeCommand(RA8875_MRWC);
  	writeData(buffer[i]);
  	waitBusy(0x80);
  }
}

void raspiRA8875::setTextCursor(uint16_t x, uint16_t y) {
  writeCommand(0x2A);
  writeData(x & 0xFF);
  writeCommand(0x2B);
  writeData(x >> 8);
  writeCommand(0x2C);
  writeData(y & 0xFF);
  writeCommand(0x2D);
  writeData(y >> 8);
}

void raspiRA8875::textEnlarge(uint8_t scale) {
if (scale > 3)
  scale = 3;
writeCommand(0x22);
uint8_t temp = readData();
temp &= ~(0xF);
temp |= scale << 2;
temp |= scale;
writeCommand(0x22);
writeData(temp);
_scale = scale;
}

void raspiRA8875::setTextColor(uint16_t foreColor, uint16_t bgColor)
{
// set Color Text
writeCommand(0x63);
writeData((foreColor & 0xF800) >> 11);
writeCommand(0x64);
writeData((foreColor & 0x07E0) >> 5);
writeCommand(0x65);
writeData((foreColor & 0x001F));

// set Color Background
writeCommand(0x60);
writeData((bgColor & 0xF800) >> 11);
writeCommand(0x61);
writeData((bgColor & 0x07E0) >> 5);
writeCommand(0x62);
writeData((bgColor & 0x001F));

// clear Transparency flag
writeCommand(0x22);
uint8_t temp = readData();
temp &= ~(1 << 6);
writeData(temp);
}

void raspiRA8875::drawPixel(int16_t x, int16_t y, uint16_t color) {
reg_write(spi0,RA8875_CURH0, x);
reg_write(spi0,RA8875_CURH1, x >> 8);
reg_write(spi0,RA8875_CURV0, y);
reg_write(spi0,RA8875_CURV1, y >> 8);
writeCommand(RA8875_MRWC);
gpio_put(cs_pin, 0);
spi_write_blocking(spi0, RA8875_DATAWRITE, 1); 
uint8_t temp = (color & 0xFF00) >> 8;
uint8_t temp2 = color & 0x00FF ;
spi_write_blocking(spi0, &temp, 1);
spi_write_blocking(spi0, &temp2, 1);
gpio_put(cs_pin, 1);
}

void raspiRA8875::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint16_t color) {
//  x0 = applyRotationX(x0);
//  y0 = applyRotationY(y0);
//  x1 = applyRotationX(x1);
//  y1 = applyRotationY(y1);

  writeCommand(0x91);
  writeData(x0);
  writeCommand(0x92);
  writeData(x0 >> 8);

  writeCommand(0x93);
  writeData(y0);
  writeCommand(0x94);
  writeData(y0 >> 8);

  writeCommand(0x95);
  writeData(x1);
  writeCommand(0x96);
  writeData((x1) >> 8);

  writeCommand(0x97);
  writeData(y1);
  writeCommand(0x98);
  writeData((y1) >> 8);

  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  writeCommand(RA8875_DCR);
  writeData(0x80);

  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

void raspiRA8875::drawRect(int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color) {
  rectHelper(x, y, x + w - 1, y + h - 1, color, false);
}

void raspiRA8875::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color) {
  rectHelper(x, y, x + w - 1, y + h - 1, color, true);
}

void raspiRA8875::fillScreen(uint16_t color) {
  rectHelper(0, 0, _width - 1, _height - 1, color, true);
}

void raspiRA8875::drawCircle(int16_t x, int16_t y, int16_t r,
                                 uint16_t color) {
  circleHelper(x, y, r, color, false);
}

void raspiRA8875::fillCircle(int16_t x, int16_t y, int16_t r,
                                 uint16_t color) {
  circleHelper(x, y, r, color, true);
}

void raspiRA8875::drawTriangle(int16_t x0, int16_t y0, int16_t x1,
                                   int16_t y1, int16_t x2, int16_t y2,
                                   uint16_t color) {
  triangleHelper(x0, y0, x1, y1, x2, y2, color, false);
}

void raspiRA8875::fillTriangle(int16_t x0, int16_t y0, int16_t x1,
                                   int16_t y1, int16_t x2, int16_t y2,
                                   uint16_t color) {
  triangleHelper(x0, y0, x1, y1, x2, y2, color, true);
}

void raspiRA8875::drawEllipse(int16_t xCenter, int16_t yCenter,
                                  int16_t longAxis, int16_t shortAxis,
                                  uint16_t color) {
  ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, false);
}

void raspiRA8875::fillEllipse(int16_t xCenter, int16_t yCenter,
                                  int16_t longAxis, int16_t shortAxis,
                                  uint16_t color) {
  ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, true);
}

void raspiRA8875::drawCurve(int16_t xCenter, int16_t yCenter,
                                int16_t longAxis, int16_t shortAxis,
                                uint8_t curvePart, uint16_t color) {
  curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, false);
}

void raspiRA8875::fillCurve(int16_t xCenter, int16_t yCenter,
                                int16_t longAxis, int16_t shortAxis,
                                uint8_t curvePart, uint16_t color) {
  curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, true);
}

void raspiRA8875::drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                    int16_t r, uint16_t color) {
  roundRectHelper(x, y, x + w, y + h, r, color, false);
}

void raspiRA8875::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                    int16_t r, uint16_t color) {
  roundRectHelper(x, y, x + w, y + h, r, color, true);
}

void raspiRA8875::circleHelper(int16_t x, int16_t y, int16_t r,
                                   uint16_t color, bool filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);

  /* Set X */
  writeCommand(0x99);
  writeData(x);
  writeCommand(0x9a);
  writeData(x >> 8);

  /* Set Y */
  writeCommand(0x9b);
  writeData(y);
  writeCommand(0x9c);
  writeData(y >> 8);

  /* Set Radius */
  writeCommand(0x9d);
  writeData(r);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled) {
    writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
  } else {
    writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
}

void raspiRA8875::rectHelper(int16_t x, int16_t y, int16_t w, int16_t h,
                                 uint16_t color, bool filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);
  w = applyRotationX(w);
  h = applyRotationY(h);

  /* Set X */
  writeCommand(0x91);
  writeData(x);
  writeCommand(0x92);
  writeData(x >> 8);

  /* Set Y */
  writeCommand(0x93);
  writeData(y);
  writeCommand(0x94);
  writeData(y >> 8);

  /* Set X1 */
  writeCommand(0x95);
  writeData(w);
  writeCommand(0x96);
  writeData((w) >> 8);

  /* Set Y1 */
  writeCommand(0x97);
  writeData(h);
  writeCommand(0x98);
  writeData((h) >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled) {
    writeData(0xB0);
  } else {
    writeData(0x90);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}


void raspiRA8875::triangleHelper(int16_t x0, int16_t y0, int16_t x1,
                                     int16_t y1, int16_t x2, int16_t y2,
                                     uint16_t color, bool filled) {
  x0 = applyRotationX(x0);
  y0 = applyRotationY(y0);
  x1 = applyRotationX(x1);
  y1 = applyRotationY(y1);
  x2 = applyRotationX(x2);
  y2 = applyRotationY(y2);

  /* Set Point 0 */
  writeCommand(0x91);
  writeData(x0);
  writeCommand(0x92);
  writeData(x0 >> 8);
  writeCommand(0x93);
  writeData(y0);
  writeCommand(0x94);
  writeData(y0 >> 8);

  /* Set Point 1 */
  writeCommand(0x95);
  writeData(x1);
  writeCommand(0x96);
  writeData(x1 >> 8);
  writeCommand(0x97);
  writeData(y1);
  writeCommand(0x98);
  writeData(y1 >> 8);

  /* Set Point 2 */
  writeCommand(0xA9);
  writeData(x2);
  writeCommand(0xAA);
  writeData(x2 >> 8);
  writeCommand(0xAB);
  writeData(y2);
  writeCommand(0xAC);
  writeData(y2 >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled) {
    writeData(0xA1);
  } else {
    writeData(0x81);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

void raspiRA8875::ellipseHelper(int16_t xCenter, int16_t yCenter,
                                    int16_t longAxis, int16_t shortAxis,
                                    uint16_t color, bool filled) {
  xCenter = applyRotationX(xCenter);
  yCenter = applyRotationY(yCenter);

  /* Set Center Point */
  writeCommand(0xA5);
  writeData(xCenter);
  writeCommand(0xA6);
  writeData(xCenter >> 8);
  writeCommand(0xA7);
  writeData(yCenter);
  writeCommand(0xA8);
  writeData(yCenter >> 8);

  /* Set Long and Short Axis */
  writeCommand(0xA1);
  writeData(longAxis);
  writeCommand(0xA2);
  writeData(longAxis >> 8);
  writeCommand(0xA3);
  writeData(shortAxis);
  writeCommand(0xA4);
  writeData(shortAxis >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(0xA0);
  if (filled) {
    writeData(0xC0);
  } else {
    writeData(0x80);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

void raspiRA8875::curveHelper(int16_t xCenter, int16_t yCenter,
                                  int16_t longAxis, int16_t shortAxis,
                                  uint8_t curvePart, uint16_t color,
                                  bool filled) {
  xCenter = applyRotationX(xCenter);
  yCenter = applyRotationY(yCenter);
  curvePart = (curvePart + _rotation) % 4;

  /* Set Center Point */
  writeCommand(0xA5);
  writeData(xCenter);
  writeCommand(0xA6);
  writeData(xCenter >> 8);
  writeCommand(0xA7);
  writeData(yCenter);
  writeCommand(0xA8);
  writeData(yCenter >> 8);

  /* Set Long and Short Axis */
  writeCommand(0xA1);
  writeData(longAxis);
  writeCommand(0xA2);
  writeData(longAxis >> 8);
  writeCommand(0xA3);
  writeData(shortAxis);
  writeCommand(0xA4);
  writeData(shortAxis >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(0xA0);
  if (filled) {
    writeData(0xD0 | (curvePart & 0x03));
  } else {
    writeData(0x90 | (curvePart & 0x03));
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

void raspiRA8875::roundRectHelper(int16_t x, int16_t y, int16_t w,
                                      int16_t h, int16_t r, uint16_t color,
                                      bool filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);
  w = applyRotationX(w);
  h = applyRotationY(h);
  if (x > w)
    swap(x, w);
  if (y > h)
    swap(y, h);

  /* Set X */
  writeCommand(0x91);
  writeData(x);
  writeCommand(0x92);
  writeData(x >> 8);

  /* Set Y */
  writeCommand(0x93);
  writeData(y);
  writeCommand(0x94);
  writeData(y >> 8);

  /* Set X1 */
  writeCommand(0x95);
  writeData(w);
  writeCommand(0x96);
  writeData((w) >> 8);

  /* Set Y1 */
  writeCommand(0x97);
  writeData(h);
  writeCommand(0x98);
  writeData((h) >> 8);

  writeCommand(0xA1);
  writeData(r);
  writeCommand(0xA2);
  writeData((r) >> 8);

  writeCommand(0xA3);
  writeData(r);
  writeCommand(0xA4);
  writeData((r) >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_ELLIPSE);
  if (filled) {
    writeData(0xE0);
  } else {
    writeData(0xA0);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_ELLIPSE, RA8875_DCR_LINESQUTRI_STATUS);
}

void raspiRA8875::setScrollWindow(int16_t x, int16_t y, int16_t w,
                                      int16_t h, uint8_t mode) {
  // Horizontal Start point of Scroll Window
  writeCommand(0x38);
  writeData(x);
  writeCommand(0x39);
  writeData(x >> 8);

  // Vertical Start Point of Scroll Window
  writeCommand(0x3a);
  writeData(y);
  writeCommand(0x3b);
  writeData(y >> 8);

  // Horizontal End Point of Scroll Window
  writeCommand(0x3c);
  writeData(x + w);
  writeCommand(0x3d);
  writeData((x + w) >> 8);

  // Vertical End Point of Scroll Window
  writeCommand(0x3e);
  writeData(y + h);
  writeCommand(0x3f);
  writeData((y + h) >> 8);

  // Scroll function setting
  writeCommand(0x52);
  writeData(mode);
}

/**************************************************************************/
/*!
    Scroll in the X direction

    @param dist The distance to scroll

 */
/**************************************************************************/
void raspiRA8875::scrollX(int16_t dist) {
  writeCommand(0x24);
  writeData(dist);
  writeCommand(0x25);
  writeData(dist >> 8);
}

/**************************************************************************/
/*!
     Scroll in the Y direction

     @param dist The distance to scroll

 */
/**************************************************************************/
void raspiRA8875::scrollY(int16_t dist) {
  writeCommand(0x26);
  writeData(dist);
  writeCommand(0x27);
  writeData(dist >> 8);
}

/************************* Mid Level ***********************************/

/**************************************************************************/
/*!
    Set the Extra General Purpose IO Register

    @param on Whether to turn Extra General Purpose IO on or not

 */
/**************************************************************************/
void raspiRA8875::GPIOX(bool on) {
  if (on)
    reg_write(spi0,RA8875_GPIOX, 1);
  else
    reg_write(spi0,RA8875_GPIOX, 0);
}

/**************************************************************************/
/*!
    Set the duty cycle of the PWM 1 Clock

    @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void raspiRA8875::PWM1out(uint8_t p) { reg_write(spi0,RA8875_P1DCR, p); }

/**************************************************************************/
/*!
     Set the duty cycle of the PWM 2 Clock

     @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void raspiRA8875::PWM2out(uint8_t p) { reg_write(spi0,RA8875_P2DCR, p); }

/**************************************************************************/
/*!
    Configure the PWM 1 Clock

    @param on Whether to enable the clock
    @param clock The Clock Divider
*/
/**************************************************************************/
void raspiRA8875::PWM1config(bool on, uint8_t clock) {
  if (on) {
    reg_write(spi0,RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
  } else {
    reg_write(spi0,RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
  }
}

void raspiRA8875::PWM2config(bool on, uint8_t clock) {
  if (on) {
    reg_write(spi0,RA8875_P2CR, RA8875_P2CR_ENABLE | (clock & 0xF));
  } else {
    reg_write(spi0,RA8875_P2CR, RA8875_P2CR_DISABLE | (clock & 0xF));
  }
}

void raspiRA8875::clearMemory(bool stop) {
uint8_t temp;
writeCommand(RA8875_MCLR);
if (stop) temp &= ~(1 << 7);
else temp |= (1 << 7);
writeData(temp);
waitBusy(0x80);
}

void raspiRA8875::clearActiveWindow(bool full) {
uint8_t temp;
writeCommand(RA8875_MCLR);
temp = readData();
if (full == true) temp &= ~(1 << 6);
else temp |= (1 << 6);
writeData(temp);
}

void raspiRA8875::setFontGFX(const GFXfont *f) {
gfxFont = (GFXfont *)f;
}

void raspiRA8875::drawCharGFX(int16_t x, int16_t y, const char *c, uint16_t color, uint8_t size) {
for(uint16_t t=0;t<= strlen(c)-1; t++) {
  uint8_t z = c[t] - (uint8_t)pgm_read_byte(&gfxFont->first);
  GFXglyph *glyph =  pgm_read_glyph_ptr(gfxFont,z);
  uint8_t *bitmap = pgm_read_bitmap_ptr(gfxFont);
  uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
  uint8_t  w  = pgm_read_byte(&glyph->width);
  uint8_t  h  = pgm_read_byte(&glyph->height);
  int8_t   xo = pgm_read_byte(&glyph->xOffset);
  int8_t   yo = pgm_read_byte(&glyph->yOffset);
  uint8_t  av = pgm_read_byte(&glyph->xAdvance);
  uint16_t xx, yy, bits = 0,bit = 0;
  int16_t  xo16 = 0, yo16 = 0;
	for(yy=0; yy<h; yy++) {
	for(xx=0; xx<w; xx++) {
	  if(!(bit++ & 7)) {
	    bits = pgm_read_byte(&bitmap[bo++]);
	    }
	  if(bits & 0x80) {
	    if(size==1) drawPixel(x+xo+xx, y+yo+yy, color);
	    }
          bits <<= 1;
       }
     }
     x=x+(av*size);
}
}
