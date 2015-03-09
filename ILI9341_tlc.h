// https://github.com/PaulStoffregen/ILI9341_TLC
// http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-(320x240-TFT-color-display)-library

/***************************************************
  This is our library for the Adafruit  ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _ILI9341_TLCH_
#define _ILI9341_TLCH_

#include "Arduino.h"
#include <SPI.h>

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
/*
#define ILI9341_PWCTR6  0xFC

*/

// Color definitions
#define	ILI9341_BLACK   0x0000
#define	ILI9341_BLUE    0x001F
#define	ILI9341_RED     0xF800
#define	ILI9341_GREEN   0x07E0
#define ILI9341_CYAN    0x07FF
#define ILI9341_MAGENTA 0xF81F
#define ILI9341_YELLOW  0xFFE0
#define ILI9341_WHITE   0xFFFF

#ifdef KINETISK_SPI0
#define KSPI0 KINETISK_SPI0
#else
#define KSPI0 KINETISL_SPI0
#endif

// Teensy 3.1 can only generate 30 MHz SPI when running at 120 MHz (overclock)
// At all other speeds, SPI.beginTransaction() will use the fastest available clock
#define ILI9341_SPICLOCK 30000000

class ILI9341_TLC : public Print
{
  public:
	ILI9341_TLC(uint8_t _CS, uint8_t _DC, uint8_t _RST = 255, uint8_t _MOSI=11, uint8_t _SCLK=13, uint8_t _MISO=12);
	void begin(void);
	void pushColor(uint16_t color);
	void fillScreen(uint16_t color);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
	void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	void setRotation(uint8_t r);
	void invertDisplay(boolean i);
	void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
	// Pass 8-bit (each) R,G,B, get back 16-bit packed color
	static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
		return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
	}

	//uint8_t readdata(void);
	uint8_t readcommand8(uint8_t reg, uint8_t index = 0);

	// Added functions to read pixel data...
	uint16_t readPixel(int16_t x, int16_t y);
    void readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors);
    void writeRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors);

	// from Adafruit_GFX.h
	void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
	void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
	void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
	void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
	void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
	void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
	void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
	void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
	void setCursor(int16_t x, int16_t y);
	void setTextColor(uint16_t c);
	void setTextColor(uint16_t c, uint16_t bg);
	void setTextSize(uint8_t s);
	void setTextWrap(boolean w);
	virtual size_t write(uint8_t);
	int16_t width(void)  { return _width; }
	int16_t height(void) { return _height; }
	uint8_t getRotation(void);
	void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
	void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);


 protected:
  int16_t
    _width, _height, // Display w/h as modified by current rotation
    cursor_x, cursor_y;
  uint16_t
    textcolor, textbgcolor;
  uint8_t
    textsize,
    rotation;
  boolean
    wrap; // If set, 'wrap' text at right edge of display

  private:
  	uint8_t  _rst;
  	uint8_t _cs, _dc;
    uint8_t _miso, _mosi, _sclk;
    uint8_t _fSPI1;
	uint8_t pcs_data, pcs_command;
    KINETISL_SPI_t *_pKSPI;
    
    volatile uint8_t *dcportSet, *dcportClear, *csportSet, *csportClear;
    uint8_t  cspinmask, dcpinmask;
    uint8_t  fDCHigh, fCSHigh, fByteOutput;

    void spiBegin(void)  __attribute__((always_inline)) {
        if (_fSPI1)
            SPI1.beginTransaction(SPISettings(ILI9341_SPICLOCK, MSBFIRST, SPI_MODE0));
        else
            SPI.beginTransaction(SPISettings(ILI9341_SPICLOCK, MSBFIRST, SPI_MODE0));
    }
    void spiEnd(void)  __attribute__((always_inline)) {
        if (_fSPI1)
            SPI1.endTransaction();
        else
            SPI.endTransaction();
    }
    
	void setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
	  __attribute__((always_inline)) {
		writecommand_cont(ILI9341_CASET); // Column addr set
		writedata16_cont(x0);   // XSTART
		writedata16_cont(x1);   // XEND
		writecommand_cont(ILI9341_PASET); // Row addr set
		writedata16_cont(y0);   // YSTART
		writedata16_cont(y1);   // YEND
	}
    //void writeSPIByte(uint8_t c)  __attribute__((always_inline)) {
    void writeSPIByte(uint8_t val) {
        uint32_t sr;
        do {
            sr = _pKSPI->S;
    		if ((_pKSPI->S & SPI_S_SPRF)) 
                uint32_t tmp __attribute__((unused)) = _pKSPI->DL;
		} while (!(sr & SPI_S_SPTEF)) ; // room for byte to output.
		_pKSPI->DL = val;
        fByteOutput = 1;
    }
    
	void waitTransmitComplete(void) {
        if (fByteOutput) {
            fByteOutput = 0;    // Don't do twice...
            uint32_t sr;
            if (!(_pKSPI->S & SPI_S_SPTEF))  {    // still something to output
                do {
                    sr = _pKSPI->S;
                    if ((_pKSPI->S & SPI_S_SPRF)) 
                        uint32_t tmp __attribute__((unused)) = _pKSPI->DL;
                } while (!(sr & SPI_S_SPTEF)) ; // room for byte to output.
            } else { 
                uint16_t wDontHang = 20;     // loop through a wait for byte to be ready...
                while (!(_pKSPI->S & SPI_S_SPRF) && wDontHang--) ;   // wait for read data to come back...
                uint32_t tmp __attribute__((unused)) = _pKSPI->DL;
            }    
        }
	}

    // For Teensy lets try using set and clear register.
    void dcHigh()  __attribute__((always_inline)) {
        if (!fDCHigh) {
            waitTransmitComplete();
            *dcportSet = dcpinmask;
            fDCHigh = 1;
        }
    }

    void dcLow()  __attribute__((always_inline)) {
        if (fDCHigh) {
            waitTransmitComplete();
            *dcportClear = dcpinmask;
            fDCHigh = 0;
        }    
    }

    void csHigh()  __attribute__((always_inline)) {
        if (!fCSHigh) {
            waitTransmitComplete();
            *csportSet = cspinmask;
            fCSHigh = 1;
        }    
    }
    void csLow()  __attribute__((always_inline)) {
        if (fCSHigh) {
            waitTransmitComplete();
            *csportClear = cspinmask;
            fCSHigh = 0;
        }
    }

	void writecommand_cont(uint8_t c) __attribute__((always_inline)) {
        dcLow();
        csLow();
        writeSPIByte(c);
	}
	void writedata8_cont(uint8_t c) __attribute__((always_inline)) {
        dcHigh();
        csLow();
        writeSPIByte(c);
	}
	void writedata16_cont(uint16_t d) __attribute__((always_inline)) {
        dcHigh();
        csLow();
        writeSPIByte(d>>8);
        writeSPIByte(d & 0xff);
	}
	void writecommand_last(uint8_t c) __attribute__((always_inline)) {
        dcLow();
        csLow();
        writeSPIByte(c);
        csHigh();
	}
	void writedata8_last(uint8_t c) __attribute__((always_inline)) {
        dcHigh();
        csLow();
        writeSPIByte(c);
        csHigh();
	}
	void writedata16_last(uint16_t d) __attribute__((always_inline)) {
        dcHigh();
        csLow();
        writeSPIByte(d>>8);
        writeSPIByte(d & 0xff);
        csHigh();
	}
	void HLine(int16_t x, int16_t y, int16_t w, uint16_t color)
	  __attribute__((always_inline)) {
		setAddr(x, y, x+w-1, y);
		writecommand_cont(ILI9341_RAMWR);
		do { writedata16_cont(color); } while (--w > 0);
	}
	void VLine(int16_t x, int16_t y, int16_t h, uint16_t color)
	  __attribute__((always_inline)) {
		setAddr(x, y, x, y+h-1);
		writecommand_cont(ILI9341_RAMWR);
		do { writedata16_cont(color); } while (--h > 0);
	}
	void Pixel(int16_t x, int16_t y, uint16_t color)
	  __attribute__((always_inline)) {
		setAddr(x, y, x, y);
		writecommand_cont(ILI9341_RAMWR);
		writedata16_cont(color);
	}
};

#ifndef swap
#define swap(a, b) { typeof(a) t = a; a = b; b = t; }
#endif

#endif
