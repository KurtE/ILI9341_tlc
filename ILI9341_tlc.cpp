// https://github.com/PaulStoffregen/ILI9341_TLC
// http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-(320x240-TFT-color-display)-library

/***************************************************
  This is our library for the Adafruit ILI9341 Breakout and Shield
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

#include "ILI9341_tlc.h"
#include <SPI.h>


#define WIDTH  ILI9341_TFTWIDTH
#define HEIGHT ILI9341_TFTHEIGHT

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ILI9341_TLC::ILI9341_TLC(uint8_t cs, uint8_t dc, uint8_t rst, uint8_t mosi, uint8_t sclk, uint8_t miso)

{
	_cs   = cs;
	_dc   = dc;
	_rst  = rst;
    _mosi = mosi;
    _sclk = sclk;
    _miso = miso;
	_width    = WIDTH;
	_height   = HEIGHT;
	rotation  = 0;
	cursor_y  = cursor_x    = 0;
	textsize  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	_cWritesPending = 0;
}

void ILI9341_TLC::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	spiBegin();
	setAddr(x0, y0, x1, y1);
	writecommand_last(ILI9341_RAMWR); // write to RAM
	spiEnd();
}

void ILI9341_TLC::pushColor(uint16_t color)
{
	spiBegin();

	dcHigh();
	csLow();
	set16BitWrite();
	writedata16(color);
	set8BitWrite();
	csHigh();
	spiEnd();
}

void ILI9341_TLC::drawPixel(int16_t x, int16_t y, uint16_t color) {

	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

	spiBegin();
	Pixel(x, y, color);
	csHigh();
	spiEnd();
}

void ILI9341_TLC::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	fillRect(x, y, 1, h, color);
}

void ILI9341_TLC::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	fillRect(x, y, w, 1, color);
}

void ILI9341_TLC::fillScreen(uint16_t color)
{
	fillRect(0, 0, _width, _height, color);
}


// fill a rectangle
void ILI9341_TLC::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= _width) || (y >= _height)) return;
	if((x + w - 1) >= _width)  w = _width  - x;
	if((y + h - 1) >= _height) h = _height - y;

	// TODO: this can result in a very long transaction time
	// should break this into multiple transactions, even though
	// it'll cost more overhead, so we don't stall other SPI libs
	spiBegin();
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	set16BitWrite();	// this will setup for 16 bit writes...
	uint32_t c = w * h;
	while (c--) {
		writedata16(color);
	}
	set8BitWrite();
	csHigh();
	spiEnd();
}



#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ILI9341_TLC::setRotation(uint8_t m)
{
	writecommand_cont(ILI9341_MADCTL);
	rotation = m % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
		writedata8_last(MADCTL_MX | MADCTL_BGR);
		_width  = ILI9341_TFTWIDTH;
		_height = ILI9341_TFTHEIGHT;
		break;
	case 1:
		writedata8_last(MADCTL_MV | MADCTL_BGR);
		_width  = ILI9341_TFTHEIGHT;
		_height = ILI9341_TFTWIDTH;
		break;
	case 2:
		writedata8_last(MADCTL_MY | MADCTL_BGR);
		_width  = ILI9341_TFTWIDTH;
		_height = ILI9341_TFTHEIGHT;
		break;
	case 3:
		writedata8_last(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		_width  = ILI9341_TFTHEIGHT;
		_height = ILI9341_TFTWIDTH;
		break;
	}
}


void ILI9341_TLC::invertDisplay(boolean i)
{
	writecommand_last(i ? ILI9341_INVON : ILI9341_INVOFF);
}










/*
uint8_t ILI9341_TLC::readdata(void)
{
  uint8_t r;
       // Try to work directly with SPI registers...
       // First wait until output queue is empty
        uint16_t wTimeout = 0xffff;
        while (((SPI0.SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty
        
//       	SPI0_MCR |= SPI_MCR_CLR_RXF; // discard any received data
//		SPI0_SR = SPI_SR_TCF;
        
        // Transfer a 0 out... 
        writedata8_cont(0);   
        
        // Now wait until completed. 
        wTimeout = 0xffff;
        while (((SPI0.SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty
        r = SPI0.POPR;  // get the received byte... should check for it first...
    return r;
}
 */
 

uint8_t ILI9341_TLC::readcommand8(uint8_t c, uint8_t index)
{
    uint8_t r=0;

    spiBegin();
    
    writecommand_cont(0xD9); // sekret command
    writedata8_cont(0x10 + index);
    writecommand_cont(c);
    r = transferdata8_last(0);
    spiEnd();
    return r;  // get the received byte... should check for it first...
}


// Read Pixel at x,y and get back 16-bit packed color
uint16_t ILI9341_TLC::readPixel(int16_t x, int16_t y)
{
	uint8_t r,g,b;

	spiBegin();

	setAddr(x, y, x, y);
	writecommand_cont(ILI9341_RAMRD); // read from RAM

	// Read Pixel Data
	r = transferdata8_cont(0);	// Read a DUMMY byte of GRAM
	r = transferdata8_cont(0);		// Read a RED byte of GRAM
	g = transferdata8_cont(0);		// Read a GREEN byte of GRAM
	b = transferdata8_last(0);		// Read a BLUE byte of GRAM

	spiEnd();
	return color565(r,g,b);
}

// Now lets see if we can read in multiple pixels
void ILI9341_TLC::readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors) 
{
    // First quick and dirty version. 
	uint8_t ab[4];      // buffer to read in colors for one pixel.
    int8_t ib;         // index into the buffer; 
    uint32_t c = w * h;
	spiBegin();

	setAddr(x, y, x+w-1, y+h-1);
   	writecommand_cont(ILI9341_RAMRD); // read from RAM
    dcHigh();
    // We will need to transfer c*3+1 bytes
    uint32_t cRead = c*3+1;   // see how many bytes we need to write
    ib = 0;    // we will ignore first N bytes returned. 
#ifdef USE_SPI1
	uint32_t cWrite = cRead;
	while (cRead) {
		// Wait for input queue to have room
		if (cWrite) {
			if (!(SPI1_S & SPI_S_TXFULLF)) {
				SPI1_DL = 0;     // push a 0 to start next transfer
				cWrite--;
			}
		}

		if (!(SPI1_S & SPI_S_RFIFOEF)) {
			ab[ib++] = SPI0_DL; // read in the byte;
			cRead--;
			if (ib == 4) {
				// we have a pixel so lets build it 
				// Sort of hack, try to build pixel while next byte is being output as to 
				// better maximize the use of the SPI buss
				*pcolors++ = color565(ab[1], ab[2], ab[3]);
				ib = 1;
			}
		}
		// Now wait until byte has been output
	}
	// But that implies that we then need to generate the last pixel outside of the loop.
	*pcolors = color565(ab[1], ab[2], ab[3]);
#else
	while (cRead) {
		// Wait for input queue to have room
		while (!(SPI0_S & SPI_S_SPTEF));
		SPI0_DL = 0;     // push a 0 to start next transfer
		if (ib == 4) {
			// we have a pixel so lets build it 
			// Sort of hack, try to build pixel while next byte is being output as to 
			// better maximize the use of the SPI buss
			*pcolors++ = color565(ab[1], ab[2], ab[3]);
			ib = 1;
		}

		// Now wait until byte has been output
		while (!(SPI0_S & SPI_S_SPRF));
		ab[ib++] = SPI0_DL; // read in the byte;
		cRead--;
	}
#endif
	// But that implies that we then need to generate the last pixel outside of the loop.
	*pcolors = color565(ab[1], ab[2], ab[3]);
    csHigh();

	spiEnd();
}

// Now lets see if we can writemultiple pixels
void ILI9341_TLC::writeRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors)
{
	spiBegin();
	setAddr(x, y, x + w - 1, y + h - 1);
	writecommand_cont(ILI9341_RAMWR);
	set16BitWrite();	// this will setup for 16 bit writes...
	uint32_t c = w * h;
	while (c--) {
		writedata16(*pcolors++);
	}
	set8BitWrite();
	csHigh();
	spiEnd();
}





static const uint8_t init_commands[] = {
	4, 0xEF, 0x03, 0x80, 0x02,
	4, 0xCF, 0x00, 0XC1, 0X30,
	5, 0xED, 0x64, 0x03, 0X12, 0X81,
	4, 0xE8, 0x85, 0x00, 0x78,
	6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
	2, 0xF7, 0x20,
	3, 0xEA, 0x00, 0x00,
	2, ILI9341_PWCTR1, 0x23, // Power control
	2, ILI9341_PWCTR2, 0x10, // Power control
	3, ILI9341_VMCTR1, 0x3e, 0x28, // VCM control
	2, ILI9341_VMCTR2, 0x86, // VCM control2
	2, ILI9341_MADCTL, 0x48, // Memory Access Control
	2, ILI9341_PIXFMT, 0x55,
	3, ILI9341_FRMCTR1, 0x00, 0x18,
	4, ILI9341_DFUNCTR, 0x08, 0x82, 0x27, // Display Function Control
	2, 0xF2, 0x00, // Gamma Function Disable
	2, ILI9341_GAMMASET, 0x01, // Gamma curve selected
	16, ILI9341_GMCTRP1, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
		0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
	16, ILI9341_GMCTRN1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
		0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
	0
};

void ILI9341_TLC::begin(void)
{
	spiInit();	// put in header file to allow compile option for which SPI buss
	/*
	uint8_t x = readcommand8(ILI9341_RDMODE);
	Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDMADCTL);
	Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDPIXFMT);
	Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDIMGFMT);
	Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDSELFDIAG);
	Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
	*/
	spiBegin();
	const uint8_t *addr = init_commands;
	while (1) {
		uint8_t count = *addr++;
		if (count-- == 0) break;
		writecommand_cont(*addr++);
		while (count-- > 0) {
			writedata8_cont(*addr++);
		}
	}
	writecommand_last(ILI9341_SLPOUT);    // Exit Sleep
	spiEnd();

	delay(120); 		
	spiBegin();
	writecommand_last(ILI9341_DISPON);    // Display on
	spiEnd();
}




/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!
 
Copyright (c) 2013 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "glcdfont.c"

// Draw a circle outline
void ILI9341_TLC::drawCircle(int16_t x0, int16_t y0, int16_t r,
    uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;
  spiBegin();
  Pixel(x0, y0 + r, color);
  Pixel(x0  , y0-r, color);
  Pixel(x0+r, y0  , color);
  Pixel(x0-r, y0  , color);
  csHigh();
  spiEnd();

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
	spiBegin();
	Pixel(x0 + x, y0 + y, color);
    Pixel(x0 - x, y0 + y, color);
    Pixel(x0 + x, y0 - y, color);
    Pixel(x0 - x, y0 - y, color);
    Pixel(x0 + y, y0 + x, color);
    Pixel(x0 - y, y0 + x, color);
    Pixel(x0 + y, y0 - x, color);
    Pixel(x0 - y, y0 - x, color);
	csHigh();
	spiEnd();
  }
}

void ILI9341_TLC::drawCircleHelper( int16_t x0, int16_t y0,
               int16_t r, uint8_t cornername, uint16_t color) {
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  spiBegin();
  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      Pixel(x0 + x, y0 + y, color);
      Pixel(x0 + y, y0 + x, color);
    } 
    if (cornername & 0x2) {
      Pixel(x0 + x, y0 - y, color);
      Pixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      Pixel(x0 - y, y0 + x, color);
      Pixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      Pixel(x0 - y, y0 - x, color);
      Pixel(x0 - x, y0 - y, color);
    }
  }
  csHigh();
  spiEnd();
}

void ILI9341_TLC::fillCircle(int16_t x0, int16_t y0, int16_t r,
			      uint16_t color) {
  drawFastVLine(x0, y0-r, 2*r+1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}

// Used to do circles and roundrects
void ILI9341_TLC::fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
    uint8_t cornername, int16_t delta, uint16_t color) {

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
	spiBegin();
    if (cornername & 0x1) {
      VLine(x0+x, y0-y, 2*y+1+delta, color);
      VLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      VLine(x0-x, y0-y, 2*y+1+delta, color);
      VLine(x0-y, y0-x, 2*x+1+delta, color);
    }
	csHigh();
	spiEnd();
  }
}


// Bresenham's algorithm - thx wikpedia
void ILI9341_TLC::drawLine(int16_t x0, int16_t y0,
	int16_t x1, int16_t y1, uint16_t color)
{
	if (y0 == y1) {
		if (x1 > x0) {
			drawFastHLine(x0, y0, x1 - x0 + 1, color);
		} else if (x1 < x0) {
			drawFastHLine(x1, y0, x0 - x1 + 1, color);
		} else {
			drawPixel(x0, y0, color);
		}
		return;
	} else if (x0 == x1) {
		if (y1 > y0) {
			drawFastVLine(x0, y0, y1 - y0 + 1, color);
		} else {
			drawFastVLine(x0, y1, y0 - y1 + 1, color);
		}
		return;
	}

	bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	spiBegin();
	int16_t xbegin = x0;
	if (steep) {
		for (; x0<=x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					VLine(y0, xbegin, len + 1, color);
				} else {
					Pixel(y0, x0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			VLine(y0, xbegin, x0 - xbegin, color);
		}

	} else {
		for (; x0<=x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					HLine(xbegin, y0, len + 1, color);
				} else {
					Pixel(x0, y0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			HLine(xbegin, y0, x0 - xbegin, color);
		}
	}
	csHigh();
	spiEnd();
}

// Draw a rectangle
void ILI9341_TLC::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	spiBegin();
	HLine(x, y, w, color);
	HLine(x, y+h-1, w, color);
	VLine(x, y, h, color);
	VLine(x+w-1, y, h, color);
	csHigh();
	spiEnd();
}

// Draw a rounded rectangle
void ILI9341_TLC::drawRoundRect(int16_t x, int16_t y, int16_t w,
  int16_t h, int16_t r, uint16_t color) {
  // smarter version
  spiBegin();
  HLine(x + r, y, w - 2 * r, color); // Top
  HLine(x+r  , y+h-1, w-2*r, color); // Bottom
  VLine(x    , y+r  , h-2*r, color); // Left
  VLine(x+w-1, y+r  , h-2*r, color); // Right
  spiEnd();
  // draw four corners
  drawCircleHelper(x+r    , y+r    , r, 1, color);
  drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

// Fill a rounded rectangle
void ILI9341_TLC::fillRoundRect(int16_t x, int16_t y, int16_t w,
				 int16_t h, int16_t r, uint16_t color) {
  // smarter version
  fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

// Draw a triangle
void ILI9341_TLC::drawTriangle(int16_t x0, int16_t y0,
				int16_t x1, int16_t y1,
				int16_t x2, int16_t y2, uint16_t color) {
  drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color);
}

// Fill a triangle
void ILI9341_TLC::fillTriangle ( int16_t x0, int16_t y0,
				  int16_t x1, int16_t y1,
				  int16_t x2, int16_t y2, uint16_t color) {

  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    drawFastHLine(a, y0, b-a+1, color);
    return;
  }

  int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1,
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it
  spiBegin();
  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    HLine(a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    HLine(a, y, b-a+1, color);
  }
  csHigh();
  spiEnd();
}

void ILI9341_TLC::drawBitmap(int16_t x, int16_t y,
			      const uint8_t *bitmap, int16_t w, int16_t h,
			      uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
		drawPixel(x+i, y+j, color);
      }
    }
  }
}

size_t ILI9341_TLC::write(uint8_t c) {
  if (c == '\n') {
    cursor_y += textsize*8;
    cursor_x  = 0;
  } else if (c == '\r') {
    // skip em
  } else {
    drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
    cursor_x += textsize*6;
    if (wrap && (cursor_x > (_width - textsize*6))) {
      cursor_y += textsize*8;
      cursor_x = 0;
    }
  }
  return 1;
}

// Draw a character
void ILI9341_TLC::drawChar(int16_t x, int16_t y, unsigned char c,
			    uint16_t fgcolor, uint16_t bgcolor, uint8_t size)
{
	if((x >= _width)            || // Clip right
	   (y >= _height)           || // Clip bottom
	   ((x + 6 * size - 1) < 0) || // Clip left  TODO: is this correct?
	   ((y + 8 * size - 1) < 0))   // Clip top   TODO: is this correct?
		return;

	if (fgcolor == bgcolor) {
		// This transparent approach is only about 20% faster
		if (size == 1) {
			uint8_t mask = 0x01;
			int16_t xoff, yoff;
			for (yoff=0; yoff < 8; yoff++) {
				spiBegin();
				uint8_t line = 0;
				for (xoff=0; xoff < 5; xoff++) {
					if (font[c * 5 + xoff] & mask) line |= 1;
					line <<= 1;
				}
				line >>= 1;
				xoff = 0;
				while (line) {
					if (line == 0x1F) {
						HLine(x + xoff, y + yoff, 5, fgcolor);
						break;
					} else if (line == 0x1E) {
						HLine(x + xoff, y + yoff, 4, fgcolor);
						break;
					} else if ((line & 0x1C) == 0x1C) {
						HLine(x + xoff, y + yoff, 3, fgcolor);
						line <<= 4;
						xoff += 4;
					} else if ((line & 0x18) == 0x18) {
						HLine(x + xoff, y + yoff, 2, fgcolor);
						line <<= 3;
						xoff += 3;
					} else if ((line & 0x10) == 0x10) {
						Pixel(x + xoff, y + yoff, fgcolor);
						line <<= 2;
						xoff += 2;
					} else {
						line <<= 1;
						xoff += 1;
					}
				}
				mask = mask << 1;
				csHigh();
				spiEnd();
			}
		} else {
			uint8_t mask = 0x01;
			int16_t xoff, yoff;
			for (yoff=0; yoff < 8; yoff++) {
				uint8_t line = 0;
				for (xoff=0; xoff < 5; xoff++) {
					if (font[c * 5 + xoff] & mask) line |= 1;
					line <<= 1;
				}
				line >>= 1;
				xoff = 0;
				while (line) {
					if (line == 0x1F) {
						fillRect(x + xoff * size, y + yoff * size,
							5 * size, size, fgcolor);
						break;
					} else if (line == 0x1E) {
						fillRect(x + xoff * size, y + yoff * size,
							4 * size, size, fgcolor);
						break;
					} else if ((line & 0x1C) == 0x1C) {
						fillRect(x + xoff * size, y + yoff * size,
							3 * size, size, fgcolor);
						line <<= 4;
						xoff += 4;
					} else if ((line & 0x18) == 0x18) {
						fillRect(x + xoff * size, y + yoff * size,
							2 * size, size, fgcolor);
						line <<= 3;
						xoff += 3;
					} else if ((line & 0x10) == 0x10) {
						fillRect(x + xoff * size, y + yoff * size,
							size, size, fgcolor);
						line <<= 2;
						xoff += 2;
					} else {
						line <<= 1;
						xoff += 1;
					}
				}
				mask = mask << 1;
			}
		}
	} else {
		// This solid background approach is about 5 time faster
		spiBegin();
		setAddr(x, y, x + 6 * size - 1, y + 8 * size - 1);
		writecommand_cont(ILI9341_RAMWR);
		set16BitWrite();	// this will setup for 16 bit writes...
		uint8_t xr, yr;
		uint8_t mask = 0x01;
		uint16_t color;
		for (y=0; y < 8; y++) {
			for (yr=0; yr < size; yr++) {
				for (x=0; x < 5; x++) {
					if (font[c * 5 + x] & mask) {
						color = fgcolor;
					} else {
						color = bgcolor;
					}
					for (xr=0; xr < size; xr++) {
						writedata16(color);
					}
				}
				for (xr=0; xr < size; xr++) {
					writedata16(bgcolor);
				}
			}
			mask = mask << 1;
		}
		set8BitWrite();	// this will setup for 16 bit writes...
		csHigh();
		spiEnd();
	}
}

void ILI9341_TLC::setCursor(int16_t x, int16_t y) {
  cursor_x = x;
  cursor_y = y;
}

void ILI9341_TLC::getCursor(int16_t *x, int16_t *y) {
  *x = cursor_x;
  *y = cursor_y;
}

void ILI9341_TLC::setTextSize(uint8_t s) {
  textsize = (s > 0) ? s : 1;
}

void ILI9341_TLC::setTextColor(uint16_t c) {
  // For 'transparent' background, we'll set the bg 
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}

void ILI9341_TLC::setTextColor(uint16_t c, uint16_t b) {
  textcolor   = c;
  textbgcolor = b; 
}

void ILI9341_TLC::setTextWrap(boolean w) {
  wrap = w;
}

uint8_t ILI9341_TLC::getRotation(void) {
  return rotation;
}


