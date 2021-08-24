#ifndef LCD7565_H_
#define LCD7565_H_

// driver for GMG12864-06D 128x64 lcd  using ST7565 controller
// 1/65 duty 1/9 bias
// may need to be tweaked for other lcds using the same controller

#define FRAME_WIDTH     128
#define NUM_PAGES         8


#define LEFT         0
#define RIGHT     9999
#define CENTER    9998
#define LCD_DATA 	0x00

#define CMD_DISPLAY_OFF   			0xAE
#define CMD_DISPLAY_ON    			0xAF

#define CMD_SET_DISP_START_LINE  0x40
#define CMD_SET_PAGE  				0xB0

#define CMD_SET_COLUMN_UPPER  	0x10
#define CMD_SET_COLUMN_LOWER  	0x00

#define CMD_SET_ADC_NORMAL  		0xA0
#define CMD_SET_ADC_REVERSE 		0xA1

#define CMD_SET_DISP_NORMAL 		0xA6
#define CMD_SET_DISP_REVERSE 		0xA7

#define CMD_SET_ALLPTS_NORMAL 	0xA4
#define CMD_SET_ALLPTS_ON  		0xA5
#define CMD_SET_BIAS_9 				0xA2
#define CMD_SET_BIAS_7 				0xA3

#define CMD_RMW  					   0xE0
#define CMD_RMW_CLEAR 				0xEE
#define CMD_INTERNAL_RESET  		0xE2
#define CMD_SET_COM_NORMAL  		0xC0
#define CMD_SET_COM_REVERSE  		0xC8
#define CMD_SET_POWER_CONTROL  	0x28
#define CMD_SET_RESISTOR_RATIO  	0x21

#define CMD_SET_VOLUME_FIRST  	0x81
#define CMD_SET_VOLUME_SECOND  		0
#define CMD_SET_STATIC_OFF  		0xAC
#define CMD_SET_STATIC_ON  		0xAD
#define CMD_SET_STATIC_REG  		0x00
#define CMD_SET_BOOSTER_FIRST  	0xF8
#define CMD_SET_BOOSTER_234  		   0
#define CMD_SET_BOOSTER_5  			1
#define CMD_SET_BOOSTER_6  			3
#define CMD_NOP  					   0xE3
#define CMD_TEST  					0xF0


extern uint8_t FrameBuf[FRAME_WIDTH*NUM_PAGES];   // 128x64 lcd
extern int FramePage;
extern int FrameCol;

void lcd_init(uint8_t brightness);
void lcd_clear();
void lcd_put_char(uint8_t ch);
void lcd_put_charX2(uint8_t ch );
void lcd_send_data(uint8_t data);
void lcd_send_cmd(uint8_t cmd);
void lcd_print_sz(int page, int c,  const char* sz);
void lcd_print_szX2(int page, int c,  const char* sz);
void lcd_printf(bool immed, int page, int c, const char* format, ...);
void lcd_printlnf(bool immed, int page, const char* format, ...);


// 11x16 numeric font +,-,space

void lcd_put_lnum(char ch);
void lcd_print_sz_lnum(int page, int col, const char* sz);
void lcd_put_image(int page, int col, const uint8_t *pIm);
void lcd_draw_circle(int x0, int y0, int r );
void lcd_draw_rect(int x, int y, int w, int h);
void lcd_fill_rect(int x, int y, int w, int h);
void lcd_clear_pixel(int x, int y);
void lcd_set_pixel(int x, int y);
void lcd_draw_line(int x0, int y0, int x1, int y1);

inline void lcd_set_frame_pos(int page, int col) {   
    FramePage = page;
    FrameCol = col;
    }
void lcd_clear_frame(void);
void lcd_invert_frame(void);
void lcd_clear_subframe(int page, int col, int numPages, int numCols);
void lcd_invert_subframe(int page, int col, int numPages, int numCols);
void lcd_send_frame(void);
void lcd_clear_page(int page);

#endif
