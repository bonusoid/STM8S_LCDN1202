//2020-11-27
//Created by : bonusoid
//LCD Nokia 1202 Demo

#include"delay.h"
#include"delay.c"
#include"periph_stm8s.h"
#include"periph_stm8s.c"
#include"lcd_n1202.h"
#include"lcd_n1202.c"

unsigned char dsine[10] = {0x18,0x06,0x01,0x01,0x06,0x18,0x60,0x80,0x80,0x60}; //Sinewave pattern
unsigned char dtri[14] = {0x08,0x04,0x02,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x40,0x20,0x10}; //Trianglewave pattern

void gpio_init();
void loop();

void drawInt();	//Draw Integer demo
void drawAlphanum(); //Draw Letter & Number demo
void drawPunct(); //Draw Punctuation demo
void drawFrame(); //Draw Frame demo
void drawArrow(); //Draw Arrow demo
void drawBytes(); //Draw Pattern demo

void drawLoadingBar(); //Draw Loading Bar animation

//^^^^^^^^^^ INIT ^^^^^^^^^^//
int main()
{
  clock_init();
  delay_init();
  gpio_init();
  lcdn1202_init();
  LCD_clear();
  
  drawLoadingBar();
  
  loop();
  return 0;
}
//__________ INIT __________//


//^^^^^^^^^^ LOOP ^^^^^^^^^^//
void loop()
{
	while(1)
	{
		drawBytes();
		delay_ms(1000);
                LCD_clearblock(3,5,84); //Finish column = 5 + 8*10 - 1
   		delay_ms(500);
		LCD_clearblock(5,3,86); //Finish column = 3 + 6*14 - 1
   		delay_ms(500);

		drawInt();
		delay_ms(1000); 
		LCD_clear();

		drawAlphanum();
		delay_ms(1000); 
		LCD_reverse();
		delay_ms(1000);
		LCD_clear();
		LCD_normal();
		drawPunct();
		delay_ms(1000); 
		LCD_reverse();
		delay_ms(1000);
		LCD_clear();
		LCD_normal();

		drawFrame();
		delay_ms(700); 
		LCD_clearblock(3,36,43); //Finish column = 36 + 8 - 1
		delay_ms(700);
		LCD_clear();
		drawArrow();
		delay_ms(700); 
		LCD_clearblock(3,36,43); //Finish column = 36 + 8 - 1
		delay_ms(700);
		LCD_clear();
	} 	
}
//__________ LOOP __________//

void gpio_init()
{
	
}

void drawInt()
{
	LCD_drawint(64, 1, 8);
	LCD_drawint(-64, 1, 48); //Negative number is not supported
				 //Its 2's complement will be displayed

	LCD_drawint(100, 3, 8);
	LCD_drawchar(SYM_DEGREE, 3, 32);
	LCD_drawchar('C', 3, 40);

	LCD_drawint(65535, 5, 8); //Max. is 65535

	LCD_drawint(064, 3, 70); //Octal displayed as Decimal
	LCD_drawint(0x64, 5, 70); //Hexadecimal displayed as Decimal
}

void drawAlphanum()
{
	LCD_drawtext("ABCDEFGHIJKL",0,0);
	LCD_drawtext("MNOPQRSTUVWX",1,0);
	LCD_drawtext("YZ",2,0);	
	LCD_drawtext("abcdefghijkl",3,0);
	LCD_drawtext("mnopqrstuvwxyz",4,0);
	LCD_drawtext("yz",5,0);	
	LCD_drawtext("0123456789",6,0);
}

void drawPunct()
{
	LCD_drawtext("<{([+_-=])}>",0,0);
	LCD_drawtext("!@#$%^&*`|~?",2,0);
	LCD_drawtext(".\,\"\'\\/ :;",4,0);
}

void drawFrame()
{
	unsigned char startcol=20;

	LCD_drawchar(FRAME_TOP_LEFT,1,startcol);
	LCD_drawchar(FRAME_LINE_HOR,1,startcol+8);
	LCD_drawchar(FRAME_TOP,1,startcol+16);
	LCD_drawchar(FRAME_LINE_HOR,1,startcol+24);
	LCD_drawchar(FRAME_TOP_RIGHT,1,startcol+32);
	
	LCD_drawchar(FRAME_LINE_VER,2,startcol);
	LCD_drawchar(FRAME_LINE_VER,2,startcol+16);
	LCD_drawchar(FRAME_LINE_VER,2,startcol+32);

	LCD_drawchar(FRAME_MID_LEFT,3,startcol);
	LCD_drawchar(FRAME_LINE_HOR,3,startcol+8);
	LCD_drawchar(FRAME_CENTER,3,startcol+16);
	LCD_drawchar(FRAME_LINE_HOR,3,startcol+24);
	LCD_drawchar(FRAME_MID_RIGHT,3,startcol+32);

	LCD_drawchar(FRAME_LINE_VER,4,startcol);
	LCD_drawchar(FRAME_LINE_VER,4,startcol+16);
	LCD_drawchar(FRAME_LINE_VER,4,startcol+32);

	LCD_drawchar(FRAME_BOT_LEFT,5,startcol);
	LCD_drawchar(FRAME_LINE_HOR,5,startcol+8);
	LCD_drawchar(FRAME_BOT,5,startcol+16);
	LCD_drawchar(FRAME_LINE_HOR,5,startcol+24);
	LCD_drawchar(FRAME_BOT_RIGHT,5,startcol+32);
}
void drawArrow()
{
	unsigned char startcol=20;

	LCD_drawchar(ARROW_UP_LEFT,1,startcol);
	LCD_drawchar(ARROW_UP,1,startcol+16);
	LCD_drawchar(ARROW_UP_RIGHT,1,startcol+32);

	LCD_drawchar(ARROW_LEFT,3,startcol);
	LCD_drawchar(ARROW_POINT,3,startcol+16);
	LCD_drawchar(ARROW_RIGHT,3,startcol+32);

	LCD_drawchar(ARROW_DOWN_LEFT,5,startcol);
	LCD_drawchar(ARROW_DOWN,5,startcol+16);
	LCD_drawchar(ARROW_DOWN_RIGHT,5,startcol+32);
}

void drawBytes()
{
	unsigned char Ts,ds;

	LCD_setpos(3,5);
	for(Ts=0;Ts<8;Ts++) //Draw pattern 8 times
	{
		for(ds=0;ds<10;ds++)
		{
			LCD_drawbyte(dsine[ds]);
		}
	}

	LCD_setpos(5,3);
	for(Ts=0;Ts<6;Ts++) //Draw pattern 6 times
	{
		for(ds=0;ds<14;ds++)
		{
			LCD_drawbyte(dtri[ds]);
		}
	}
}

void drawLoadingBar()
{
	unsigned char lb;

	LCD_setpos(4,5);

	for(lb=5;lb<91;lb++)
	{
		LCD_drawbyte(0xFF);
		delay_ms(10);
	}
	delay_ms(1000);
	LCD_clearblock(4,5,90); //Start & finish column = start & finish lb
}
