#include "common.h"
#include "config.h"
#include "hspi.h"
#include "lcd7565.h"
#include "font_6x8.h"
#include "font_bignum.h"

#define TAG "lcd7565"

uint8_t FrameBuf[FRAME_WIDTH*8];   // FRAME_WIDTHx64 lcd
int FramePage;
int FrameCol;

static void lcd_setPageCol(int page, int col);



void lcd_init(uint8_t brightness) {
    FramePage = 0;
    FrameCol = 0;
	LCD_CS_LO();
	
	LCD_RST_LO();
	delayMs(100);
	LCD_RST_HI();
    // LCD bias select
    lcd_sendCmd(CMD_SET_BIAS_7);

    // adc reverse and com reverse for connector ribbon on top
    // ADC select
    lcd_sendCmd(CMD_SET_ADC_REVERSE);
    //lcd_sendCmd(CMD_SET_ADC_REVERSE);
    // SHL select
    lcd_sendCmd(CMD_SET_COM_NORMAL);
    //lcd_sendCmd(CMD_SET_COM_REVERSE);

    // Initial display line
    lcd_sendCmd(CMD_SET_DISP_START_LINE);

    // turn on voltage converter (VC=1, VR=0, VF=0)
    lcd_sendCmd(CMD_SET_POWER_CONTROL | 0x4);
    // wait for 50% rising
    delayMs(50);

    // turn on voltage regulator (VC=1, VR=1, VF=0)
    lcd_sendCmd(CMD_SET_POWER_CONTROL | 0x6);
    // wait >=50ms
    delayMs(50);

    // turn on voltage follower (VC=1, VR=1, VF=1)
    lcd_sendCmd(CMD_SET_POWER_CONTROL | 0x7);
    // wait
    delayMs(10);

    // set lcd operating voltage (regulator resistor, ref voltage resistor)
    lcd_sendCmd(CMD_SET_RESISTOR_RATIO | 0x6);
	lcd_sendCmd(CMD_DISPLAY_ON);

	lcd_sendCmd(CMD_SET_ALLPTS_NORMAL);

    lcd_sendCmd(CMD_SET_VOLUME_FIRST);
    lcd_sendCmd(CMD_SET_VOLUME_SECOND | (brightness & 0x3f));

    delayMs(50);
    lcd_clear();
    }
	
   
void lcd_sendData(uint8_t data)	{
	hspi_beginTransaction();
	LCD_A0_HI();
	LCD_CS_LO();
	hspi_write(data);
	LCD_CS_HI();
	hspi_endTransaction();
	}


void lcd_sendCmd(uint8_t cmd)	{
	hspi_beginTransaction();
	LCD_A0_LO();
	LCD_CS_LO();
	hspi_write(cmd);
	LCD_CS_HI();
	hspi_endTransaction();
	}

static void lcd_setPageCol(int page,int col){
	hspi_beginTransaction();
	LCD_A0_LO();
	LCD_CS_LO();
	hspi_write((CMD_SET_PAGE|((uint8_t)page & 0x0F)));
	hspi_write((CMD_SET_COLUMN_UPPER|(((uint8_t)col >> 4))));
	hspi_write((CMD_SET_COLUMN_LOWER|(3+((uint8_t)col & 0x0F))));
	LCD_CS_HI();
	hspi_endTransaction();
	}


////////////////////// these work on the frame buffer ////////////////	


// 6x8 font
void lcd_putChar(uint8_t ch){
	int inx = 6*(ch - 32);
	for (int i = 0; i < 6; i++ ){
		FrameBuf[FRAME_WIDTH*FramePage + FrameCol] = Font6x8[inx+i];
      FrameCol++;
		}
	}


// 6x8 font, doubled
void lcd_putCharX2 (uint8_t ch )   {
	if ((ch < 32) || (ch > 127)) return;	
	int inx = 6*(ch - 32);
   int cnt = 6;
   int b1,b2;
   while (cnt--) {
      uint8_t col = Font6x8[inx];
      b1 =  (col & 0x01) * 3;
      b1 |= (col & 0x02) * 6;
      b1 |= (col & 0x04) * 12;
      b1 |= (col & 0x08) * 24;
      col >>= 4;
      b2 =  (col & 0x01) * 3;
      b2 |= (col & 0x02) * 6;
      b2 |= (col & 0x04) * 12;
      b2 |= (col & 0x08) * 24;
      FrameBuf[FRAME_WIDTH*FramePage+FrameCol] = b1;
      FrameBuf[FRAME_WIDTH*FramePage+FrameCol+1] = b1;
      lcd_setFramePos(FramePage+1,FrameCol);
      FrameBuf[FRAME_WIDTH*FramePage+FrameCol] = b2;
      FrameBuf[FRAME_WIDTH*FramePage+FrameCol+1] = b2;
      lcd_setFramePos(FramePage-1,FrameCol+2);
      inx++;
      }
   }
   
// 6x8
void lcd_printSz(int page, int col, char* sz)	{
	lcd_setFramePos(page,col);
	while(*sz){
		lcd_putChar(*sz++);
		}
	}

// 6x8
void lcd_printf(bool immed, int page, int col, char* format, ...)    {
	char szbuf[22];
   va_list args;
   va_start(args,format);
   vsprintf(szbuf,format,args);
   va_end(args);
	lcd_printSz(page, col, szbuf);
   if (immed) lcd_sendFrame();
   }	

void lcd_printlnf(bool immed, int page, const char* format, ...)    {
	char szbuf[22];
   va_list args;
   va_start(args,format);
   vsprintf(szbuf,format,args);
   va_end(args);
   lcd_clearPage(page);
	lcd_printSz(page, 0, szbuf);
   if (immed) lcd_sendFrame();
   }	


// 6x8, doubled
void lcd_printSzX2(int page, int col, char* sz)	{
	lcd_setFramePos(page,col);
	while(*sz){
		lcd_putCharX2(*sz++);
		}
	}


//-------------------- draw bitmap -----------------------

void lcd_putImage(int page, int col, const uint8_t *pIm){
	int width = (int) *pIm;
	int height = (int) *(pIm+1);
	uint8_t * Image = (uint8_t*)(pIm+2);
	int i,j; 
   	
	for(j = 0;j <= (height/8);j++){
		lcd_setFramePos(page+j,col-width/2);
		for(i = 0;i < width;i++){
         FrameBuf[FRAME_WIDTH*FramePage + FrameCol + i] = Image[(width*j) + i];
		   }
	   }
   }


// ---------------- 10x16  ' +-0123456789' ----------

void lcd_putLNum(char ch) {
	int inx;
	if ((ch > 0x2F) && (ch < 0x3A)) {
		inx = 60 + 20*(ch - 0x30);
		}
	else {
	   switch (ch) {
         case '+' : inx = 20; 
	      break;
         case '-' : inx = 40;
	      break;
         case ' ' : 
         default : inx = 0; 
	      break;
         }
      }
   int cnt;
   for (cnt = 0; cnt < 10; cnt++) {
      FrameBuf[FRAME_WIDTH*FramePage + FrameCol + cnt] = FontBigNum[inx];
	   inx++;
      }
   lcd_setFramePos(FramePage+1, FrameCol); 
   for (cnt = 0; cnt < 10; cnt++) {
      FrameBuf[FRAME_WIDTH*FramePage + FrameCol + cnt] = FontBigNum[inx];
	   inx++;
      }
   lcd_setFramePos(FramePage-1,FrameCol+11);
   }  


void lcd_printSzLNum(int page, int col, char* sz) {
 	lcd_setFramePos(page,col);
	while (*sz) {
		lcd_putLNum(*sz++);
      }
	}

//--------------------------------------------------------------------------

inline void lcd_setFramePos(int page, int col) {
   FramePage = page;
   FrameCol = col;
   }


void lcd_clear(void){
   memset(FrameBuf, 0, FRAME_WIDTH*NUM_PAGES);
   lcd_sendFrame();
	}
	

void lcd_clearFrame(void) {
   memset(FrameBuf, 0, FRAME_WIDTH*NUM_PAGES);
	}


void lcd_invertFrame() {
   for (int inx = 0; inx < FRAME_WIDTH*NUM_PAGES; inx++) {
      FrameBuf[inx] = ~FrameBuf[inx];
      }
   }

void lcd_clearPage(int page) {
   memset(&FrameBuf[FRAME_WIDTH*page], 0, FRAME_WIDTH);
   }


void lcd_clearSubFrame(int page, int col, int numPages, int numCols) {
   for (int p = page; p < (page+numPages); p++) {
      memset(&FrameBuf[FRAME_WIDTH*p + col], 0, numCols);
	   }
   }


void lcd_invertSubFrame(int page, int col, int numPages, int numCols) {
   for (int p = page; p < (page+numPages); p++) 
      for (int c = col; c < (col+numCols); c++){
         int inx = FRAME_WIDTH*p + col;  
         FrameBuf[inx] = ~FrameBuf[inx]; 
	      }
   }


void lcd_sendFrame(void) {
	for(int page = 0; page < 8; page++) {
		lcd_setPageCol(page,0); 
    	hspi_beginTransaction();
	    LCD_A0_HI();
	    LCD_CS_LO();
	    hspi_writeBytes(&FrameBuf[FRAME_WIDTH*page], FRAME_WIDTH);
	    LCD_CS_HI();
	    hspi_endTransaction();
		}
	}


void lcd_setPixel( int x, int y) {
   FrameBuf[((y/8)*FRAME_WIDTH)+x] |=  0x01<<(y%8);
	}


void lcd_clearPixel( int x, int y) {
	FrameBuf[x+ (y/8)*FRAME_WIDTH] &= ~(0x01<<(y%8));
	}


void lcd_drawLine( int x0, int y0, int x1, int y1) {
	int temp;
	int dx, dy;
	int steep = (y1 - y0) > (x1 - x0);

	if (steep) {
		temp=x0;
		x0 = y0;
		y0 = temp;

		temp=x1;
		x1 = y1;
		y1 = temp;
		}

	if (x0 > x1) {
		temp=x0;
		x0 = x1;
		x1 = temp;

		temp=y0;
		y0 = y1;
		y1 = temp;
		}

	dx = x1 - x0;
	dy = (y1 - y0);

	int err = dx / 2;
	int ystep;

	if (y0 < y1) {
		ystep = 1;
		} 
	else {
		ystep = -1;
		}

	for (; x0<x1; x0++) {
		if (steep) {
			lcd_setPixel( y0, x0);
			} 
		else{
			lcd_setPixel( x0, y0);
			}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
			}
		}
	}


void lcd_fillRect( int x, int y, int w, int h) {
	for (int i = x; i < x+w; i++) {
	    for (int j = y; j < y+h; j++) {
			lcd_setPixel(i, j);
			}
		}
	}


void lcd_drawRect( int x, int y, int w, int h) {
	for (int i = x; i < x+w; i++) {
	    lcd_setPixel( i, y);
	    lcd_setPixel( i, y+h-1);
		}

	for (int i = y; i < y+h; i++) {
	    lcd_setPixel( x, i);
	    lcd_setPixel( x+w-1, i);
		}
	}


void lcd_drawCircle(int x0, int y0, int r ) {
	int f = 1 - r;
	int ddF_x = 1;
	int ddF_y = -2 * r;
	int x = 0;
	int y = r;

	lcd_setPixel( x0, y0+r);
	lcd_setPixel( x0, y0-r);
	lcd_setPixel( x0+r, y0);
	lcd_setPixel( x0-r, y0);

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
			}
		x++;
		ddF_x += 2;
		f += ddF_x;

		lcd_setPixel( x0 + x, y0 + y);
		lcd_setPixel( x0 - x, y0 + y);
		lcd_setPixel( x0 + x, y0 - y);
		lcd_setPixel( x0 - x, y0 - y);

		lcd_setPixel( x0 + y, y0 + x);
		lcd_setPixel( x0 - y, y0 + x);
		lcd_setPixel( x0 + y, y0 - x);
		lcd_setPixel( x0 - y, y0 - x);
		}
	}

	
