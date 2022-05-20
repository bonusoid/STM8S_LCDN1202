//2020-11-27
//bonusoid

#include"lcd_n1202_stm8s.h"
#include"REG/stm8s_gpio.h"

void lcdn1202_gpio_init()
{
	LCDDDR |= (OUTPUT<<LCDDAT)|(OUTPUT<<LCDCLK)|(OUTPUT<<LCDBL);	//Configure GPIO as Output
	LCDCR1 |= (pushpull<<LCDDAT)|(pushpull<<LCDCLK)|(pushpull<<LCDBL); //Configure Output Type
	LCDCR2 |= (speed_10MHz<<LCDDAT)|(speed_10MHz<<LCDCLK)|(speed_10MHz<<LCDBL); //Configure GPIO speed
	LCDODR = 0x00; //Starting value
}

void lcdn1202_9bsend(unsigned char cdsign, unsigned char comdat)
{
	unsigned char cdi;

	if(cdsign==0) LCDODR &= LCDDAT_MASKL; //1st bit is 0 for Command
	else LCDODR |= LCDDAT_MASKH; //1st bit is 1 for Data
	lcdn1202_clock1();

	for(cdi=0;cdi<8;cdi++) //Send 2nd-9th bit
	   {
		if(comdat & 0x80) LCDODR |= LCDDAT_MASKH; //LCDDAT = '1'
		else LCDODR &= LCDDAT_MASKL;		  //LCDDAT = '0'
		lcdn1202_clock1();
		comdat <<= 1; //Shift to next bit
	   }
	LCDODR &= LCDDAT_MASKL;
}

void lcdn1202_clock1()
{
	LCDODR |= LCDCLK_MASKH; //Send 1 pulse to LCDCLK
	delay_us(1); //Short delay
	LCDODR &= LCDCLK_MASKL;
}

void lcdn1202_blon()
{
	LCDODR |= LCDBL_MASKH; //LCDBL = '1'
}

void lcdn1202_bloff()
{
	LCDODR &= LCDBL_MASKL; //LCDBL = '0'
}
