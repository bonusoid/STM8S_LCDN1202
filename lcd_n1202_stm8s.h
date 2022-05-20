//2020-11-27
//bonusoid

#ifndef __LCD_N1202_STM8S_H
#define __LCD_N1202_STM8S_H

#define LCDN1202_COL	96	//96 columns
#define LCDN1202_ROW	9	//8,5 rows (8 byte and 1 nibble)

#define LCDDAT 	P1	//LCD DATA
#define LCDCLK 	P2	//LCD CLOCK
#define LCDBL	P3	//LCD BACKLIGHT CONTROL
#define LCDODR 	PA_ODR	//LCD Output Port
#define LCDDDR 	PA_DDR	//LCD Data Direction
#define LCDCR1 	PA_CR1	//LCD Pin Config 1
#define LCDCR2 	PA_CR2	//LCD Pin Config 2
#define LCDDAT_MASKL 	P1_MASKL  //LCDDAT LOW MASK
#define LCDDAT_MASKH 	P1_MASKH  //LCDDAT HIGH MASK
#define LCDCLK_MASKL 	P2_MASKL  //LCDCLK LOW MASK
#define LCDCLK_MASKH 	P2_MASKH  //LCDCLK HIGH MASK
#define LCDBL_MASKL 	P3_MASKL  //LCDBL LOW MASK
#define LCDBL_MASKH 	P3_MASKH  //LCDBL HIGH MASK

//LCD Hardware Access Functions
void lcdn1202_gpio_init(); //LCD Interface Initialization
void lcdn1202_9bsend(unsigned char cdsign, unsigned char comdat); //Send 9 bit Command/Data to LCDDAT
void lcdn1202_clock1(); //Send clock pulse to LCDCLK
void lcdn1202_blon();	//LCDBL = '1'
void lcdn1202_bloff();	//LCDBL = '0'

#endif
