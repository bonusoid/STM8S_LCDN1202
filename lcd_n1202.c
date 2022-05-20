//2020-11-27
//bonusoid

#include "lcd_n1202.h"
#include "lcd_n1202_stm8s.h"
#include "lcd_n1202_stm8s.c"
#include "font.h"

void lcdn1202_init()
{
	lcdn1202_gpio_init();

	//Hard Reset -> By HW using R-C

	delay_ms(10);

	lcdn1202_sendcom(0xE2);	//Soft Reset
	delay_ms(1);
	lcdn1202_sendcom(0xA4); //Normal Display Mode
	lcdn1202_sendcom(0x2F);	//Power Control = Max (Booster On, VReg On, VFol On)

	lcdn1202_sendcom(0xA0); //Segment Driver Direction = Normal (lines start at left)
	lcdn1202_sendcom(0xC0); //Common Driver Direction = Normal
	lcdn1202_sendcom(0x80|16); //Set Contrast to default

	lcdn1202_sendcom(0xAF);	//Display On

	LCD_BL_OFF(); //Backlight off
	LCD_clear();  //Clear pixel memory
	LCD_BL_ON();  //Backlight on
}

void lcdn1202_sendcom(unsigned char ssd1306com)
{
	lcdn1202_9bsend(0,ssd1306com); //Send Command
}

void lcdn1202_senddat(unsigned char ssd1306dat)
{
	lcdn1202_9bsend(1,ssd1306dat); //Send Data
}

void lcdn1202_setpos(unsigned char row, unsigned char col)
{
	lcdn1202_sendcom(0xB0|(row&0x0F)); //Set page of row
	lcdn1202_sendcom(0x00|(col&0x0F)); //Set lower nibble of Column
	lcdn1202_sendcom(0x10|((col>>4)&0x0F)); //Set upper nibble of Column
}

void lcdn1202_clear()
{
	unsigned char col,row;
	lcdn1202_setpos(0,0);
  	for(row=0;row<LCDN1202_ROW;row++)	//Scan rows (pages)
  	   {
      		for(col=0;col<LCDN1202_COL;col++)	//Scan columns
      		   {
        		lcdn1202_senddat(0);	//Send 0 to every pixel
      		   }
  	   }
}

void LCD_setpos(unsigned char row, unsigned char col)
{
	lcdn1202_setpos(row,col); //Set coordinate (for LCD_drawbyte)
}

void LCD_drawbyte(unsigned char dbyte)
{
	lcdn1202_senddat(dbyte); //Send 1 byte data only
}

void LCD_drawchar(unsigned char chr, unsigned char chrrow, unsigned char chrcol)
{
	unsigned char ci,fchar;
	unsigned int chridx;

	lcdn1202_setpos(chrrow,chrcol);

	if((chr>31)&&(chr<128))	//Alphanumeric & Punctuation Area
	  {
	    lcdn1202_senddat(0x00);
            chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
            for(ci=0;ci<5;ci++)
		{
		   fchar = font_arr[chridx+ci]; //Get character pattern from Font Array
		   lcdn1202_senddat(fchar); //Send pattern 1 byte at a time
		}
          }
 	else if((chr>127)&&(chr<148))	//Frame & Arrow Area
	  {
	    chridx=(chr-128)*8; //Start at index 128. 5 columns for each symbol
            for(ci=0;ci<8;ci++)
		{
 		   fchar = font_arr[chridx+480+ci]; //Get symbol pattern from Font Array
		   lcdn1202_senddat(fchar); //Send pattern 1 byte at a time
		}
	  }
	else{}
}

void LCD_drawtext(char *text, unsigned char txtrow, unsigned char txtcol)
{
	unsigned int stridx = 0;

	while(text[stridx] != 0) //Scan characters in string
	  {
		LCD_drawchar(text[stridx],txtrow,txtcol+(8*stridx)); //Display each character
		stridx++;
	  }
}

void LCD_drawint(unsigned int num, unsigned char numrow, unsigned char numcol)
{
	char ibuff[6]; //MAX : 5 DIGIT -> 65535

	unsigned char ndigit=0,nd;
	unsigned int numb; //Must be unsigned, so max. number can be 65535
			   //If set to signed, max. number only 32767

	numb = num;
	while(numb!=0) //Counting digit
	  {
	  	ndigit++;
		numb /= 10; 
	  }
	for(nd=0;nd<ndigit;nd++) //Converting each digit
	  {
		numb = num%10;
		num = num/10;
		ibuff[ndigit-(nd+1)] = numb + '0'; //Start from last_index-1
	  }
	ibuff[ndigit] = '\0'; //Last character is null

	LCD_drawtext(ibuff,numrow,numcol); //Display number as text
}

void LCD_clear()
{
	lcdn1202_sendcom(0xAE);  //Set Display off
	lcdn1202_clear(); //Clear display
	lcdn1202_sendcom(0xAF); //Set Display on
}

void LCD_clearblock(unsigned char row, unsigned char col_start, unsigned char col_fin)
{
	unsigned char col;

	lcdn1202_setpos(row,col_start); //Set start position
	for(col=col_start;col<=col_fin;col++) //Scan columns
	   {
		lcdn1202_senddat(0);	//Send 0 to every pixel in a column
	   }
}

void LCD_normal()
{
	lcdn1202_sendcom(0xA6);	//Black Pixel in White Background
}

void LCD_reverse()
{
	lcdn1202_sendcom(0xA7);	//White Pixel in Black Background
}

void LCD_BL_ON()
{
	lcdn1202_blon(); //Backlight on
}

void LCD_BL_OFF()
{
	lcdn1202_bloff(); //Backlight off
}
