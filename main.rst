                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.5.0 #9253 (Apr  3 2018) (Linux)
                                      4 ; This file was generated Fri May 20 20:03:55 2022
                                      5 ;--------------------------------------------------------
                                      6 	.module main
                                      7 	.optsdcc -mstm8
                                      8 	
                                      9 ;--------------------------------------------------------
                                     10 ; Public variables in this module
                                     11 ;--------------------------------------------------------
                                     12 	.globl _font_arr
                                     13 	.globl _main
                                     14 	.globl _dtri
                                     15 	.globl _dsine
                                     16 	.globl _readreg
                                     17 	.globl _delay_init
                                     18 	.globl _delay_us
                                     19 	.globl _delay_ms
                                     20 	.globl _delay_timer
                                     21 	.globl _clock_init
                                     22 	.globl _i2c_init
                                     23 	.globl _i2c_set_start
                                     24 	.globl _i2c_set_address
                                     25 	.globl _i2c_set_stop
                                     26 	.globl _i2c_clear_ack
                                     27 	.globl _i2c_set_ack
                                     28 	.globl _i2c_ack_pos_current
                                     29 	.globl _i2c_ack_pos_next
                                     30 	.globl _i2c_poll_SB
                                     31 	.globl _i2c_poll_ADDR
                                     32 	.globl _i2c_poll_BTF
                                     33 	.globl _i2c_poll_TXE
                                     34 	.globl _i2c_poll_RXNE
                                     35 	.globl _i2c_clear_bits
                                     36 	.globl _i2c_clear_ADDR
                                     37 	.globl _i2c_enable_interrupts
                                     38 	.globl _i2c_disable_interrupts
                                     39 	.globl _adc_init
                                     40 	.globl _read_adc
                                     41 	.globl _uart1_init
                                     42 	.globl _uart1_send
                                     43 	.globl _uart1_recv
                                     44 	.globl _uart1_recv_i
                                     45 	.globl _pwm1_init
                                     46 	.globl _pwm2_init
                                     47 	.globl _pwm1ch1_enable
                                     48 	.globl _pwm1ch1_disable
                                     49 	.globl _pwm2ch1_enable
                                     50 	.globl _pwm2ch1_disable
                                     51 	.globl _pwm1_update
                                     52 	.globl _pwm2_update
                                     53 	.globl _lcdn1202_gpio_init
                                     54 	.globl _lcdn1202_9bsend
                                     55 	.globl _lcdn1202_clock1
                                     56 	.globl _lcdn1202_blon
                                     57 	.globl _lcdn1202_bloff
                                     58 	.globl _lcdn1202_init
                                     59 	.globl _lcdn1202_sendcom
                                     60 	.globl _lcdn1202_senddat
                                     61 	.globl _lcdn1202_setpos
                                     62 	.globl _lcdn1202_clear
                                     63 	.globl _LCD_setpos
                                     64 	.globl _LCD_drawbyte
                                     65 	.globl _LCD_drawchar
                                     66 	.globl _LCD_drawtext
                                     67 	.globl _LCD_drawint
                                     68 	.globl _LCD_clear
                                     69 	.globl _LCD_clearblock
                                     70 	.globl _LCD_normal
                                     71 	.globl _LCD_reverse
                                     72 	.globl _LCD_BL_ON
                                     73 	.globl _LCD_BL_OFF
                                     74 	.globl _loop
                                     75 	.globl _gpio_init
                                     76 	.globl _drawInt
                                     77 	.globl _drawAlphanum
                                     78 	.globl _drawPunct
                                     79 	.globl _drawFrame
                                     80 	.globl _drawArrow
                                     81 	.globl _drawBytes
                                     82 	.globl _drawLoadingBar
                                     83 ;--------------------------------------------------------
                                     84 ; ram data
                                     85 ;--------------------------------------------------------
                                     86 	.area DATA
      000001                         87 _readreg::
      000001                         88 	.ds 1
                                     89 ;--------------------------------------------------------
                                     90 ; ram data
                                     91 ;--------------------------------------------------------
                                     92 	.area INITIALIZED
      000002                         93 _dsine::
      000002                         94 	.ds 10
      00000C                         95 _dtri::
      00000C                         96 	.ds 14
                                     97 ;--------------------------------------------------------
                                     98 ; Stack segment in internal ram 
                                     99 ;--------------------------------------------------------
                                    100 	.area	SSEG
      00001A                        101 __start__stack:
      00001A                        102 	.ds	1
                                    103 
                                    104 ;--------------------------------------------------------
                                    105 ; absolute external ram data
                                    106 ;--------------------------------------------------------
                                    107 	.area DABS (ABS)
                                    108 ;--------------------------------------------------------
                                    109 ; interrupt vector 
                                    110 ;--------------------------------------------------------
                                    111 	.area HOME
      008000                        112 __interrupt_vect:
      008000 82 00 80 83            113 	int s_GSINIT ;reset
      008004 82 00 00 00            114 	int 0x0000 ;trap
      008008 82 00 00 00            115 	int 0x0000 ;int0
      00800C 82 00 00 00            116 	int 0x0000 ;int1
      008010 82 00 00 00            117 	int 0x0000 ;int2
      008014 82 00 00 00            118 	int 0x0000 ;int3
      008018 82 00 00 00            119 	int 0x0000 ;int4
      00801C 82 00 00 00            120 	int 0x0000 ;int5
      008020 82 00 00 00            121 	int 0x0000 ;int6
      008024 82 00 00 00            122 	int 0x0000 ;int7
      008028 82 00 00 00            123 	int 0x0000 ;int8
      00802C 82 00 00 00            124 	int 0x0000 ;int9
      008030 82 00 00 00            125 	int 0x0000 ;int10
      008034 82 00 00 00            126 	int 0x0000 ;int11
      008038 82 00 00 00            127 	int 0x0000 ;int12
      00803C 82 00 00 00            128 	int 0x0000 ;int13
      008040 82 00 00 00            129 	int 0x0000 ;int14
      008044 82 00 00 00            130 	int 0x0000 ;int15
      008048 82 00 00 00            131 	int 0x0000 ;int16
      00804C 82 00 00 00            132 	int 0x0000 ;int17
      008050 82 00 00 00            133 	int 0x0000 ;int18
      008054 82 00 00 00            134 	int 0x0000 ;int19
      008058 82 00 00 00            135 	int 0x0000 ;int20
      00805C 82 00 00 00            136 	int 0x0000 ;int21
      008060 82 00 00 00            137 	int 0x0000 ;int22
      008064 82 00 00 00            138 	int 0x0000 ;int23
      008068 82 00 00 00            139 	int 0x0000 ;int24
      00806C 82 00 00 00            140 	int 0x0000 ;int25
      008070 82 00 00 00            141 	int 0x0000 ;int26
      008074 82 00 00 00            142 	int 0x0000 ;int27
      008078 82 00 00 00            143 	int 0x0000 ;int28
      00807C 82 00 00 00            144 	int 0x0000 ;int29
                                    145 ;--------------------------------------------------------
                                    146 ; global & static initialisations
                                    147 ;--------------------------------------------------------
                                    148 	.area HOME
                                    149 	.area GSINIT
                                    150 	.area GSFINAL
                                    151 	.area GSINIT
      008083                        152 __sdcc_gs_init_startup:
      008083                        153 __sdcc_init_data:
                                    154 ; stm8_genXINIT() start
      008083 AE 00 01         [ 2]  155 	ldw x, #l_DATA
      008086 27 07            [ 1]  156 	jreq	00002$
      008088                        157 00001$:
      008088 72 4F 00 00      [ 1]  158 	clr (s_DATA - 1, x)
      00808C 5A               [ 2]  159 	decw x
      00808D 26 F9            [ 1]  160 	jrne	00001$
      00808F                        161 00002$:
      00808F AE 00 18         [ 2]  162 	ldw	x, #l_INITIALIZER
      008092 27 09            [ 1]  163 	jreq	00004$
      008094                        164 00003$:
      008094 D6 8E 38         [ 1]  165 	ld	a, (s_INITIALIZER - 1, x)
      008097 D7 00 01         [ 1]  166 	ld	(s_INITIALIZED - 1, x), a
      00809A 5A               [ 2]  167 	decw	x
      00809B 26 F7            [ 1]  168 	jrne	00003$
      00809D                        169 00004$:
                                    170 ; stm8_genXINIT() end
                                    171 	.area GSFINAL
      00809D CC 80 80         [ 2]  172 	jp	__sdcc_program_startup
                                    173 ;--------------------------------------------------------
                                    174 ; Home
                                    175 ;--------------------------------------------------------
                                    176 	.area HOME
                                    177 	.area HOME
      008080                        178 __sdcc_program_startup:
      008080 CC 86 1A         [ 2]  179 	jp	_main
                                    180 ;	return from main will return to caller
                                    181 ;--------------------------------------------------------
                                    182 ; code
                                    183 ;--------------------------------------------------------
                                    184 	.area CODE
                                    185 ;	delay.c: 7: void delay_init()
                                    186 ;	-----------------------------------------
                                    187 ;	 function delay_init
                                    188 ;	-----------------------------------------
      0080A0                        189 _delay_init:
                                    190 ;	delay.c: 9: TIM4_PSCR = 4; // CLK/16
      0080A0 35 04 53 47      [ 1]  191 	mov	0x5347+0, #0x04
      0080A4 81               [ 4]  192 	ret
                                    193 ;	delay.c: 12: void delay_us(unsigned long delus)
                                    194 ;	-----------------------------------------
                                    195 ;	 function delay_us
                                    196 ;	-----------------------------------------
      0080A5                        197 _delay_us:
      0080A5 52 06            [ 2]  198 	sub	sp, #6
                                    199 ;	delay.c: 16: for(du=0;du<(delus/10);du++)
      0080A7 4B 0A            [ 1]  200 	push	#0x0a
      0080A9 5F               [ 1]  201 	clrw	x
      0080AA 89               [ 2]  202 	pushw	x
      0080AB 4B 00            [ 1]  203 	push	#0x00
      0080AD 1E 0F            [ 2]  204 	ldw	x, (0x0f, sp)
      0080AF 89               [ 2]  205 	pushw	x
      0080B0 1E 0F            [ 2]  206 	ldw	x, (0x0f, sp)
      0080B2 89               [ 2]  207 	pushw	x
      0080B3 CD 8D 63         [ 4]  208 	call	__divulong
      0080B6 5B 08            [ 2]  209 	addw	sp, #8
      0080B8 1F 05            [ 2]  210 	ldw	(0x05, sp), x
      0080BA 17 03            [ 2]  211 	ldw	(0x03, sp), y
      0080BC 5F               [ 1]  212 	clrw	x
      0080BD 1F 01            [ 2]  213 	ldw	(0x01, sp), x
      0080BF                        214 00103$:
      0080BF 1E 01            [ 2]  215 	ldw	x, (0x01, sp)
      0080C1 90 5F            [ 1]  216 	clrw	y
      0080C3 13 05            [ 2]  217 	cpw	x, (0x05, sp)
      0080C5 90 9F            [ 1]  218 	ld	a, yl
      0080C7 12 04            [ 1]  219 	sbc	a, (0x04, sp)
      0080C9 90 9E            [ 1]  220 	ld	a, yh
      0080CB 12 03            [ 1]  221 	sbc	a, (0x03, sp)
      0080CD 24 0D            [ 1]  222 	jrnc	00101$
                                    223 ;	delay.c: 18: delay_timer(100);
      0080CF 4B 64            [ 1]  224 	push	#0x64
      0080D1 CD 81 3A         [ 4]  225 	call	_delay_timer
      0080D4 84               [ 1]  226 	pop	a
                                    227 ;	delay.c: 16: for(du=0;du<(delus/10);du++)
      0080D5 1E 01            [ 2]  228 	ldw	x, (0x01, sp)
      0080D7 5C               [ 2]  229 	incw	x
      0080D8 1F 01            [ 2]  230 	ldw	(0x01, sp), x
      0080DA 20 E3            [ 2]  231 	jra	00103$
      0080DC                        232 00101$:
                                    233 ;	delay.c: 20: delay_timer(delus%10);
      0080DC 4B 0A            [ 1]  234 	push	#0x0a
      0080DE 5F               [ 1]  235 	clrw	x
      0080DF 89               [ 2]  236 	pushw	x
      0080E0 4B 00            [ 1]  237 	push	#0x00
      0080E2 1E 0F            [ 2]  238 	ldw	x, (0x0f, sp)
      0080E4 89               [ 2]  239 	pushw	x
      0080E5 1E 0F            [ 2]  240 	ldw	x, (0x0f, sp)
      0080E7 89               [ 2]  241 	pushw	x
      0080E8 CD 8C F3         [ 4]  242 	call	__modulong
      0080EB 5B 08            [ 2]  243 	addw	sp, #8
      0080ED 9F               [ 1]  244 	ld	a, xl
      0080EE 88               [ 1]  245 	push	a
      0080EF CD 81 3A         [ 4]  246 	call	_delay_timer
      0080F2 5B 07            [ 2]  247 	addw	sp, #7
      0080F4 81               [ 4]  248 	ret
                                    249 ;	delay.c: 23: void delay_ms(unsigned long delms)
                                    250 ;	-----------------------------------------
                                    251 ;	 function delay_ms
                                    252 ;	-----------------------------------------
      0080F5                        253 _delay_ms:
      0080F5 52 08            [ 2]  254 	sub	sp, #8
                                    255 ;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
      0080F7 1E 0D            [ 2]  256 	ldw	x, (0x0d, sp)
      0080F9 89               [ 2]  257 	pushw	x
      0080FA 1E 0D            [ 2]  258 	ldw	x, (0x0d, sp)
      0080FC 89               [ 2]  259 	pushw	x
      0080FD 4B 64            [ 1]  260 	push	#0x64
      0080FF 5F               [ 1]  261 	clrw	x
      008100 89               [ 2]  262 	pushw	x
      008101 4B 00            [ 1]  263 	push	#0x00
      008103 CD 8D BD         [ 4]  264 	call	__mullong
      008106 5B 08            [ 2]  265 	addw	sp, #8
      008108 1F 07            [ 2]  266 	ldw	(0x07, sp), x
      00810A 17 05            [ 2]  267 	ldw	(0x05, sp), y
      00810C 5F               [ 1]  268 	clrw	x
      00810D 4F               [ 1]  269 	clr	a
      00810E 0F 01            [ 1]  270 	clr	(0x01, sp)
      008110                        271 00103$:
      008110 88               [ 1]  272 	push	a
      008111 13 08            [ 2]  273 	cpw	x, (0x08, sp)
      008113 7B 01            [ 1]  274 	ld	a, (1, sp)
      008115 12 07            [ 1]  275 	sbc	a, (0x07, sp)
      008117 7B 02            [ 1]  276 	ld	a, (0x02, sp)
      008119 12 06            [ 1]  277 	sbc	a, (0x06, sp)
      00811B 84               [ 1]  278 	pop	a
      00811C 24 19            [ 1]  279 	jrnc	00105$
                                    280 ;	delay.c: 29: delay_timer(100);
      00811E 88               [ 1]  281 	push	a
      00811F 89               [ 2]  282 	pushw	x
      008120 4B 64            [ 1]  283 	push	#0x64
      008122 CD 81 3A         [ 4]  284 	call	_delay_timer
      008125 84               [ 1]  285 	pop	a
      008126 85               [ 2]  286 	popw	x
      008127 84               [ 1]  287 	pop	a
                                    288 ;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
      008128 1C 00 01         [ 2]  289 	addw	x, #0x0001
      00812B A9 00            [ 1]  290 	adc	a, #0x00
      00812D 88               [ 1]  291 	push	a
      00812E 7B 02            [ 1]  292 	ld	a, (0x02, sp)
      008130 A9 00            [ 1]  293 	adc	a, #0x00
      008132 6B 02            [ 1]  294 	ld	(0x02, sp), a
      008134 84               [ 1]  295 	pop	a
      008135 20 D9            [ 2]  296 	jra	00103$
      008137                        297 00105$:
      008137 5B 08            [ 2]  298 	addw	sp, #8
      008139 81               [ 4]  299 	ret
                                    300 ;	delay.c: 33: void delay_timer(unsigned char deltim)
                                    301 ;	-----------------------------------------
                                    302 ;	 function delay_timer
                                    303 ;	-----------------------------------------
      00813A                        304 _delay_timer:
                                    305 ;	delay.c: 35: TIM4_CR1 = (1<<TIM4_CR1_CEN);
      00813A 35 01 53 40      [ 1]  306 	mov	0x5340+0, #0x01
                                    307 ;	delay.c: 36: while(TIM4_CNTR<deltim);
      00813E                        308 00101$:
      00813E AE 53 46         [ 2]  309 	ldw	x, #0x5346
      008141 F6               [ 1]  310 	ld	a, (x)
      008142 11 03            [ 1]  311 	cp	a, (0x03, sp)
      008144 25 F8            [ 1]  312 	jrc	00101$
                                    313 ;	delay.c: 37: TIM4_CR1 = (0<<TIM4_CR1_CEN);
      008146 35 00 53 40      [ 1]  314 	mov	0x5340+0, #0x00
                                    315 ;	delay.c: 38: TIM4_CNTR = 0; //reset timer	
      00814A 35 00 53 46      [ 1]  316 	mov	0x5346+0, #0x00
      00814E 81               [ 4]  317 	ret
                                    318 ;	periph_stm8s.c: 16: void clock_init()
                                    319 ;	-----------------------------------------
                                    320 ;	 function clock_init
                                    321 ;	-----------------------------------------
      00814F                        322 _clock_init:
                                    323 ;	periph_stm8s.c: 18: CLK_CKDIVR = 0x00; //fMASTER = fCPU = fHSI = 16MHz
      00814F 35 00 50 C6      [ 1]  324 	mov	0x50c6+0, #0x00
                                    325 ;	periph_stm8s.c: 19: CLK_ICKR = (1<<CLK_ICKR_HSIEN);	//High Speed Internal RC Oscillator enabled
      008153 35 01 50 C0      [ 1]  326 	mov	0x50c0+0, #0x01
      008157 81               [ 4]  327 	ret
                                    328 ;	periph_stm8s.c: 24: void i2c_init()
                                    329 ;	-----------------------------------------
                                    330 ;	 function i2c_init
                                    331 ;	-----------------------------------------
      008158                        332 _i2c_init:
                                    333 ;	periph_stm8s.c: 26: I2C_CR1 = (0<<I2C_CR1_PE); //Disable I2C before configuration starts
      008158 35 00 52 10      [ 1]  334 	mov	0x5210+0, #0x00
                                    335 ;	periph_stm8s.c: 27: I2C_FREQR = 16;	//fCLK = 16 MHz
      00815C 35 10 52 12      [ 1]  336 	mov	0x5212+0, #0x10
                                    337 ;	periph_stm8s.c: 28: I2C_CCRH = (0<<I2C_CCRH_FS)|(0<<I2C_CCRH_CCR11)|(0<<I2C_CCRH_CCR10)|(0<<I2C_CCRH_CCR9)|(0<<I2C_CCRH_CCR8); //Standard Mode
      008160 35 00 52 1C      [ 1]  338 	mov	0x521c+0, #0x00
                                    339 ;	periph_stm8s.c: 29: I2C_CCRL = 0x80;  //Clock Speed = 100 kHz
      008164 35 80 52 1B      [ 1]  340 	mov	0x521b+0, #0x80
                                    341 ;	periph_stm8s.c: 31: I2C_OARH = (0<<I2C_OARH_ADDMODE)|(1<<I2C_OARH_ADDCONF); //7-bit address mode, ADDCONF always must be 1
      008168 35 40 52 14      [ 1]  342 	mov	0x5214+0, #0x40
                                    343 ;	periph_stm8s.c: 32: I2C_TRISER = 17;  //Setup Bus Characteristic
      00816C 35 11 52 1D      [ 1]  344 	mov	0x521d+0, #0x11
                                    345 ;	periph_stm8s.c: 37: I2C_CR1 = (1<<I2C_CR1_PE);  //Enable I2C after configuration complete
      008170 35 01 52 10      [ 1]  346 	mov	0x5210+0, #0x01
      008174 81               [ 4]  347 	ret
                                    348 ;	periph_stm8s.c: 40: void i2c_set_start()
                                    349 ;	-----------------------------------------
                                    350 ;	 function i2c_set_start
                                    351 ;	-----------------------------------------
      008175                        352 _i2c_set_start:
                                    353 ;	periph_stm8s.c: 42: I2C_CR2 |= (1<<I2C_CR2_START);
      008175 72 10 52 11      [ 1]  354 	bset	0x5211, #0
      008179 81               [ 4]  355 	ret
                                    356 ;	periph_stm8s.c: 45: void i2c_set_address(unsigned char addr, unsigned char dir)
                                    357 ;	-----------------------------------------
                                    358 ;	 function i2c_set_address
                                    359 ;	-----------------------------------------
      00817A                        360 _i2c_set_address:
                                    361 ;	periph_stm8s.c: 47: if(dir==I2C_READ) I2C_DR = (addr<<1)|dir;
      00817A 7B 03            [ 1]  362 	ld	a, (0x03, sp)
      00817C 97               [ 1]  363 	ld	xl, a
      00817D 58               [ 2]  364 	sllw	x
      00817E 7B 04            [ 1]  365 	ld	a, (0x04, sp)
      008180 A1 01            [ 1]  366 	cp	a, #0x01
      008182 26 09            [ 1]  367 	jrne	00104$
      008184 9F               [ 1]  368 	ld	a, xl
      008185 1A 04            [ 1]  369 	or	a, (0x04, sp)
      008187 AE 52 16         [ 2]  370 	ldw	x, #0x5216
      00818A F7               [ 1]  371 	ld	(x), a
      00818B 20 0D            [ 2]  372 	jra	00106$
      00818D                        373 00104$:
                                    374 ;	periph_stm8s.c: 48: else if(dir==I2C_WRITE) I2C_DR = (addr<<1)&dir;
      00818D 7B 04            [ 1]  375 	ld	a, (0x04, sp)
      00818F A1 FE            [ 1]  376 	cp	a, #0xfe
      008191 26 07            [ 1]  377 	jrne	00106$
      008193 9F               [ 1]  378 	ld	a, xl
      008194 14 04            [ 1]  379 	and	a, (0x04, sp)
      008196 AE 52 16         [ 2]  380 	ldw	x, #0x5216
      008199 F7               [ 1]  381 	ld	(x), a
      00819A                        382 00106$:
      00819A 81               [ 4]  383 	ret
                                    384 ;	periph_stm8s.c: 52: void i2c_set_stop()
                                    385 ;	-----------------------------------------
                                    386 ;	 function i2c_set_stop
                                    387 ;	-----------------------------------------
      00819B                        388 _i2c_set_stop:
                                    389 ;	periph_stm8s.c: 54: I2C_CR2 |= (1<<I2C_CR2_STOP);
      00819B AE 52 11         [ 2]  390 	ldw	x, #0x5211
      00819E F6               [ 1]  391 	ld	a, (x)
      00819F AA 02            [ 1]  392 	or	a, #0x02
      0081A1 F7               [ 1]  393 	ld	(x), a
      0081A2 81               [ 4]  394 	ret
                                    395 ;	periph_stm8s.c: 57: void i2c_clear_ack()
                                    396 ;	-----------------------------------------
                                    397 ;	 function i2c_clear_ack
                                    398 ;	-----------------------------------------
      0081A3                        399 _i2c_clear_ack:
                                    400 ;	periph_stm8s.c: 59: I2C_CR2 &= ~(1<<I2C_CR2_ACK); //Disable Acknowledge
      0081A3 AE 52 11         [ 2]  401 	ldw	x, #0x5211
      0081A6 F6               [ 1]  402 	ld	a, (x)
      0081A7 A4 FB            [ 1]  403 	and	a, #0xfb
      0081A9 F7               [ 1]  404 	ld	(x), a
      0081AA 81               [ 4]  405 	ret
                                    406 ;	periph_stm8s.c: 62: void i2c_set_ack()
                                    407 ;	-----------------------------------------
                                    408 ;	 function i2c_set_ack
                                    409 ;	-----------------------------------------
      0081AB                        410 _i2c_set_ack:
                                    411 ;	periph_stm8s.c: 64: I2C_CR2 |= (1<<I2C_CR2_ACK); //Enable Acknowledge
      0081AB AE 52 11         [ 2]  412 	ldw	x, #0x5211
      0081AE F6               [ 1]  413 	ld	a, (x)
      0081AF AA 04            [ 1]  414 	or	a, #0x04
      0081B1 F7               [ 1]  415 	ld	(x), a
      0081B2 81               [ 4]  416 	ret
                                    417 ;	periph_stm8s.c: 67: void i2c_ack_pos_current()
                                    418 ;	-----------------------------------------
                                    419 ;	 function i2c_ack_pos_current
                                    420 ;	-----------------------------------------
      0081B3                        421 _i2c_ack_pos_current:
                                    422 ;	periph_stm8s.c: 69: I2C_CR2 &= ~(1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
      0081B3 AE 52 11         [ 2]  423 	ldw	x, #0x5211
      0081B6 F6               [ 1]  424 	ld	a, (x)
      0081B7 A4 F7            [ 1]  425 	and	a, #0xf7
      0081B9 F7               [ 1]  426 	ld	(x), a
      0081BA 81               [ 4]  427 	ret
                                    428 ;	periph_stm8s.c: 72: void i2c_ack_pos_next()
                                    429 ;	-----------------------------------------
                                    430 ;	 function i2c_ack_pos_next
                                    431 ;	-----------------------------------------
      0081BB                        432 _i2c_ack_pos_next:
                                    433 ;	periph_stm8s.c: 74: I2C_CR2 |= (1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register
      0081BB AE 52 11         [ 2]  434 	ldw	x, #0x5211
      0081BE F6               [ 1]  435 	ld	a, (x)
      0081BF AA 08            [ 1]  436 	or	a, #0x08
      0081C1 F7               [ 1]  437 	ld	(x), a
      0081C2 81               [ 4]  438 	ret
                                    439 ;	periph_stm8s.c: 77: void i2c_poll_SB()
                                    440 ;	-----------------------------------------
                                    441 ;	 function i2c_poll_SB
                                    442 ;	-----------------------------------------
      0081C3                        443 _i2c_poll_SB:
                                    444 ;	periph_stm8s.c: 79: while((I2C_SR1&(1<<I2C_SR1_SB))!=(1<<I2C_SR1_SB)){}
      0081C3                        445 00101$:
      0081C3 AE 52 17         [ 2]  446 	ldw	x, #0x5217
      0081C6 F6               [ 1]  447 	ld	a, (x)
      0081C7 A4 01            [ 1]  448 	and	a, #0x01
      0081C9 A1 01            [ 1]  449 	cp	a, #0x01
      0081CB 26 F6            [ 1]  450 	jrne	00101$
      0081CD 81               [ 4]  451 	ret
                                    452 ;	periph_stm8s.c: 82: void i2c_poll_ADDR()
                                    453 ;	-----------------------------------------
                                    454 ;	 function i2c_poll_ADDR
                                    455 ;	-----------------------------------------
      0081CE                        456 _i2c_poll_ADDR:
                                    457 ;	periph_stm8s.c: 84: while((I2C_SR1&(1<<I2C_SR1_ADDR))!=(1<<I2C_SR1_ADDR)){}
      0081CE                        458 00101$:
      0081CE AE 52 17         [ 2]  459 	ldw	x, #0x5217
      0081D1 F6               [ 1]  460 	ld	a, (x)
      0081D2 A4 02            [ 1]  461 	and	a, #0x02
      0081D4 A1 02            [ 1]  462 	cp	a, #0x02
      0081D6 26 F6            [ 1]  463 	jrne	00101$
      0081D8 81               [ 4]  464 	ret
                                    465 ;	periph_stm8s.c: 87: void i2c_poll_BTF()
                                    466 ;	-----------------------------------------
                                    467 ;	 function i2c_poll_BTF
                                    468 ;	-----------------------------------------
      0081D9                        469 _i2c_poll_BTF:
                                    470 ;	periph_stm8s.c: 89: while((I2C_SR1&(1<<I2C_SR1_BTF))!=(1<<I2C_SR1_BTF)){}
      0081D9                        471 00101$:
      0081D9 AE 52 17         [ 2]  472 	ldw	x, #0x5217
      0081DC F6               [ 1]  473 	ld	a, (x)
      0081DD A4 04            [ 1]  474 	and	a, #0x04
      0081DF A1 04            [ 1]  475 	cp	a, #0x04
      0081E1 26 F6            [ 1]  476 	jrne	00101$
      0081E3 81               [ 4]  477 	ret
                                    478 ;	periph_stm8s.c: 92: void i2c_poll_TXE()
                                    479 ;	-----------------------------------------
                                    480 ;	 function i2c_poll_TXE
                                    481 ;	-----------------------------------------
      0081E4                        482 _i2c_poll_TXE:
                                    483 ;	periph_stm8s.c: 94: while((I2C_SR1&(1<<I2C_SR1_TXE))!=(1<<I2C_SR1_TXE)){}
      0081E4                        484 00101$:
      0081E4 AE 52 17         [ 2]  485 	ldw	x, #0x5217
      0081E7 F6               [ 1]  486 	ld	a, (x)
      0081E8 A4 80            [ 1]  487 	and	a, #0x80
      0081EA A1 80            [ 1]  488 	cp	a, #0x80
      0081EC 26 F6            [ 1]  489 	jrne	00101$
      0081EE 81               [ 4]  490 	ret
                                    491 ;	periph_stm8s.c: 97: void i2c_poll_RXNE()
                                    492 ;	-----------------------------------------
                                    493 ;	 function i2c_poll_RXNE
                                    494 ;	-----------------------------------------
      0081EF                        495 _i2c_poll_RXNE:
                                    496 ;	periph_stm8s.c: 99: while((I2C_SR1&(1<<I2C_SR1_RXNE))!=(1<<I2C_SR1_RXNE)){}
      0081EF                        497 00101$:
      0081EF AE 52 17         [ 2]  498 	ldw	x, #0x5217
      0081F2 F6               [ 1]  499 	ld	a, (x)
      0081F3 A4 40            [ 1]  500 	and	a, #0x40
      0081F5 A1 40            [ 1]  501 	cp	a, #0x40
      0081F7 26 F6            [ 1]  502 	jrne	00101$
      0081F9 81               [ 4]  503 	ret
                                    504 ;	periph_stm8s.c: 102: void i2c_clear_bits()
                                    505 ;	-----------------------------------------
                                    506 ;	 function i2c_clear_bits
                                    507 ;	-----------------------------------------
      0081FA                        508 _i2c_clear_bits:
                                    509 ;	periph_stm8s.c: 104: readreg = I2C_SR1;
      0081FA AE 52 17         [ 2]  510 	ldw	x, #0x5217
      0081FD F6               [ 1]  511 	ld	a, (x)
      0081FE C7 00 01         [ 1]  512 	ld	_readreg+0, a
      008201 81               [ 4]  513 	ret
                                    514 ;	periph_stm8s.c: 107: void i2c_clear_ADDR()
                                    515 ;	-----------------------------------------
                                    516 ;	 function i2c_clear_ADDR
                                    517 ;	-----------------------------------------
      008202                        518 _i2c_clear_ADDR:
                                    519 ;	periph_stm8s.c: 109: readreg = I2C_SR1;
      008202 AE 52 17         [ 2]  520 	ldw	x, #0x5217
      008205 F6               [ 1]  521 	ld	a, (x)
                                    522 ;	periph_stm8s.c: 110: readreg = I2C_SR3;
      008206 AE 52 19         [ 2]  523 	ldw	x, #0x5219
      008209 F6               [ 1]  524 	ld	a, (x)
      00820A C7 00 01         [ 1]  525 	ld	_readreg+0, a
      00820D 81               [ 4]  526 	ret
                                    527 ;	periph_stm8s.c: 113: void i2c_enable_interrupts()
                                    528 ;	-----------------------------------------
                                    529 ;	 function i2c_enable_interrupts
                                    530 ;	-----------------------------------------
      00820E                        531 _i2c_enable_interrupts:
                                    532 ;	periph_stm8s.c: 115: I2C_ITR = 0x07;
      00820E 35 07 52 1A      [ 1]  533 	mov	0x521a+0, #0x07
      008212 81               [ 4]  534 	ret
                                    535 ;	periph_stm8s.c: 117: void i2c_disable_interrupts()
                                    536 ;	-----------------------------------------
                                    537 ;	 function i2c_disable_interrupts
                                    538 ;	-----------------------------------------
      008213                        539 _i2c_disable_interrupts:
                                    540 ;	periph_stm8s.c: 119: I2C_ITR = 0x00;
      008213 35 00 52 1A      [ 1]  541 	mov	0x521a+0, #0x00
      008217 81               [ 4]  542 	ret
                                    543 ;	periph_stm8s.c: 124: void adc_init()
                                    544 ;	-----------------------------------------
                                    545 ;	 function adc_init
                                    546 ;	-----------------------------------------
      008218                        547 _adc_init:
                                    548 ;	periph_stm8s.c: 126: ADC1_CR1 = fADC_fMASTER_8<<ADC1_CR1_SPSEL; // ADCCLK = MCLK/8
      008218 35 40 54 01      [ 1]  549 	mov	0x5401+0, #0x40
                                    550 ;	periph_stm8s.c: 127: ADC1_CR2 = (1<<ADC1_CR2_ALIGN);  // right alignment adc data
      00821C 35 08 54 02      [ 1]  551 	mov	0x5402+0, #0x08
                                    552 ;	periph_stm8s.c: 129: ADC1_CR1 |= (1<<ADC1_CR1_ADON);  // turn on ADC
      008220 72 10 54 01      [ 1]  553 	bset	0x5401, #0
      008224 81               [ 4]  554 	ret
                                    555 ;	periph_stm8s.c: 133: unsigned int read_adc(unsigned char adcch)
                                    556 ;	-----------------------------------------
                                    557 ;	 function read_adc
                                    558 ;	-----------------------------------------
      008225                        559 _read_adc:
      008225 52 04            [ 2]  560 	sub	sp, #4
                                    561 ;	periph_stm8s.c: 137: ADC1_CSR &= 0xF0;  // select
      008227 AE 54 00         [ 2]  562 	ldw	x, #0x5400
      00822A F6               [ 1]  563 	ld	a, (x)
      00822B A4 F0            [ 1]  564 	and	a, #0xf0
      00822D F7               [ 1]  565 	ld	(x), a
                                    566 ;	periph_stm8s.c: 138: ADC1_CSR |= adcch; // channel
      00822E AE 54 00         [ 2]  567 	ldw	x, #0x5400
      008231 F6               [ 1]  568 	ld	a, (x)
      008232 1A 07            [ 1]  569 	or	a, (0x07, sp)
      008234 AE 54 00         [ 2]  570 	ldw	x, #0x5400
      008237 F7               [ 1]  571 	ld	(x), a
                                    572 ;	periph_stm8s.c: 141: ADC1_CR1 |= (1<<ADC1_CR1_ADON); // start conversion
      008238 72 10 54 01      [ 1]  573 	bset	0x5401, #0
                                    574 ;	periph_stm8s.c: 142: while(!((ADC1_CSR)&(1<<ADC1_CSR_EOC)));; // conversion is in progress
      00823C                        575 00101$:
      00823C AE 54 00         [ 2]  576 	ldw	x, #0x5400
      00823F F6               [ 1]  577 	ld	a, (x)
      008240 4D               [ 1]  578 	tnz	a
      008241 2A F9            [ 1]  579 	jrpl	00101$
                                    580 ;	periph_stm8s.c: 143: adcval = (ADC1_DRH<<8) + ADC1_DRL;
      008243 AE 54 04         [ 2]  581 	ldw	x, #0x5404
      008246 F6               [ 1]  582 	ld	a, (x)
      008247 0F 03            [ 1]  583 	clr	(0x03, sp)
      008249 6B 01            [ 1]  584 	ld	(0x01, sp), a
      00824B 0F 02            [ 1]  585 	clr	(0x02, sp)
      00824D AE 54 05         [ 2]  586 	ldw	x, #0x5405
      008250 F6               [ 1]  587 	ld	a, (x)
      008251 5F               [ 1]  588 	clrw	x
      008252 97               [ 1]  589 	ld	xl, a
      008253 72 FB 01         [ 2]  590 	addw	x, (0x01, sp)
                                    591 ;	periph_stm8s.c: 144: ADC1_CSR |= (0<<ADC1_CSR_EOC); // reset EOC
      008256 90 AE 54 00      [ 2]  592 	ldw	y, #0x5400
      00825A 90 F6            [ 1]  593 	ld	a, (y)
      00825C 90 AE 54 00      [ 2]  594 	ldw	y, #0x5400
      008260 90 F7            [ 1]  595 	ld	(y), a
                                    596 ;	periph_stm8s.c: 146: return adcval;
      008262 5B 04            [ 2]  597 	addw	sp, #4
      008264 81               [ 4]  598 	ret
                                    599 ;	periph_stm8s.c: 151: void uart1_init(unsigned char rxien) //UART Initialization
                                    600 ;	-----------------------------------------
                                    601 ;	 function uart1_init
                                    602 ;	-----------------------------------------
      008265                        603 _uart1_init:
                                    604 ;	periph_stm8s.c: 155: UART1_BRR1 = 0x68;
      008265 35 68 52 32      [ 1]  605 	mov	0x5232+0, #0x68
                                    606 ;	periph_stm8s.c: 156: UART1_BRR2 = 0x03;
      008269 35 03 52 33      [ 1]  607 	mov	0x5233+0, #0x03
                                    608 ;	periph_stm8s.c: 158: UART1_CR1 |= (0<<UART1_CR1_M)|(0<<UART1_CR1_PCEN); //8 bit Data; No Parity
      00826D AE 52 34         [ 2]  609 	ldw	x, #0x5234
      008270 F6               [ 1]  610 	ld	a, (x)
      008271 AE 52 34         [ 2]  611 	ldw	x, #0x5234
      008274 F7               [ 1]  612 	ld	(x), a
                                    613 ;	periph_stm8s.c: 159: UART1_CR3 |= (0<<UART1_CR3_STOP); //Stop Bit = 1
      008275 AE 52 36         [ 2]  614 	ldw	x, #0x5236
      008278 F6               [ 1]  615 	ld	a, (x)
      008279 AE 52 36         [ 2]  616 	ldw	x, #0x5236
      00827C F7               [ 1]  617 	ld	(x), a
                                    618 ;	periph_stm8s.c: 161: if(rxien==1) 
      00827D 7B 03            [ 1]  619 	ld	a, (0x03, sp)
      00827F A1 01            [ 1]  620 	cp	a, #0x01
      008281 26 0B            [ 1]  621 	jrne	00102$
                                    622 ;	periph_stm8s.c: 163: UART1_CR2 |= (1<<UART1_CR2_RIEN); //Enable Interrupt on Receiver Mode
      008283 AE 52 35         [ 2]  623 	ldw	x, #0x5235
      008286 F6               [ 1]  624 	ld	a, (x)
      008287 AA 20            [ 1]  625 	or	a, #0x20
      008289 F7               [ 1]  626 	ld	(x), a
                                    627 ;	periph_stm8s.c: 164: ITC_SPR5 = (level_2<<ITC_SPR5_VECT18); //UART Interrupt Setting
      00828A 35 00 7F 74      [ 1]  628 	mov	0x7f74+0, #0x00
      00828E                        629 00102$:
                                    630 ;	periph_stm8s.c: 167: UART1_CR2 |= (1<<UART1_CR2_TEN); //Enable Transmitter Mode
      00828E AE 52 35         [ 2]  631 	ldw	x, #0x5235
      008291 F6               [ 1]  632 	ld	a, (x)
      008292 AA 08            [ 1]  633 	or	a, #0x08
      008294 F7               [ 1]  634 	ld	(x), a
                                    635 ;	periph_stm8s.c: 168: UART1_CR2 |= (1<<UART1_CR2_REN); //Enable Receiver Mode
      008295 AE 52 35         [ 2]  636 	ldw	x, #0x5235
      008298 F6               [ 1]  637 	ld	a, (x)
      008299 AA 04            [ 1]  638 	or	a, #0x04
      00829B F7               [ 1]  639 	ld	(x), a
      00829C 81               [ 4]  640 	ret
                                    641 ;	periph_stm8s.c: 171: void uart1_send(unsigned char usend) //UART Transmit a Byte
                                    642 ;	-----------------------------------------
                                    643 ;	 function uart1_send
                                    644 ;	-----------------------------------------
      00829D                        645 _uart1_send:
                                    646 ;	periph_stm8s.c: 173: UART1_DR = usend; //Write to UART Data Register
      00829D AE 52 31         [ 2]  647 	ldw	x, #0x5231
      0082A0 7B 03            [ 1]  648 	ld	a, (0x03, sp)
      0082A2 F7               [ 1]  649 	ld	(x), a
                                    650 ;	periph_stm8s.c: 174: while((UART1_SR&(1<<UART1_SR_TXE))!=(1<<UART1_SR_TXE)); //Wait until Transmission complete
      0082A3                        651 00101$:
      0082A3 AE 52 30         [ 2]  652 	ldw	x, #0x5230
      0082A6 F6               [ 1]  653 	ld	a, (x)
      0082A7 A4 80            [ 1]  654 	and	a, #0x80
      0082A9 A1 80            [ 1]  655 	cp	a, #0x80
      0082AB 26 F6            [ 1]  656 	jrne	00101$
      0082AD 81               [ 4]  657 	ret
                                    658 ;	periph_stm8s.c: 177: unsigned char uart1_recv() //UART Receive a Byte (using Polling)
                                    659 ;	-----------------------------------------
                                    660 ;	 function uart1_recv
                                    661 ;	-----------------------------------------
      0082AE                        662 _uart1_recv:
                                    663 ;	periph_stm8s.c: 180: if((UART1_SR&(1<<UART1_SR_RXNE))==(1<<UART1_SR_RXNE)) //Check if any data in Data Register
      0082AE AE 52 30         [ 2]  664 	ldw	x, #0x5230
      0082B1 F6               [ 1]  665 	ld	a, (x)
      0082B2 A4 20            [ 1]  666 	and	a, #0x20
      0082B4 A1 20            [ 1]  667 	cp	a, #0x20
      0082B6 26 05            [ 1]  668 	jrne	00102$
                                    669 ;	periph_stm8s.c: 182: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
      0082B8 AE 52 31         [ 2]  670 	ldw	x, #0x5231
      0082BB F6               [ 1]  671 	ld	a, (x)
                                    672 ;	periph_stm8s.c: 184: else urecv=0;
      0082BC 21                     673 	.byte 0x21
      0082BD                        674 00102$:
      0082BD 4F               [ 1]  675 	clr	a
      0082BE                        676 00103$:
                                    677 ;	periph_stm8s.c: 185: return urecv;
      0082BE 81               [ 4]  678 	ret
                                    679 ;	periph_stm8s.c: 188: unsigned char uart1_recv_i() //UART Receive a Byte (using Interrupt)
                                    680 ;	-----------------------------------------
                                    681 ;	 function uart1_recv_i
                                    682 ;	-----------------------------------------
      0082BF                        683 _uart1_recv_i:
                                    684 ;	periph_stm8s.c: 191: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
      0082BF AE 52 31         [ 2]  685 	ldw	x, #0x5231
      0082C2 F6               [ 1]  686 	ld	a, (x)
                                    687 ;	periph_stm8s.c: 192: return urecv;
      0082C3 81               [ 4]  688 	ret
                                    689 ;	periph_stm8s.c: 198: void pwm1_init(unsigned int timval)
                                    690 ;	-----------------------------------------
                                    691 ;	 function pwm1_init
                                    692 ;	-----------------------------------------
      0082C4                        693 _pwm1_init:
      0082C4 52 02            [ 2]  694 	sub	sp, #2
                                    695 ;	periph_stm8s.c: 200: TIM1_PSCRH = 0x00; //TIM_CLK = CLK
      0082C6 35 00 52 60      [ 1]  696 	mov	0x5260+0, #0x00
                                    697 ;	periph_stm8s.c: 201: TIM1_PSCRL = 0x00; //TIM_CLK = CLK
      0082CA 35 00 52 61      [ 1]  698 	mov	0x5261+0, #0x00
                                    699 ;	periph_stm8s.c: 202: TIM1_ARRH = (timval >> 8); //TIM RELOAD
      0082CE 7B 05            [ 1]  700 	ld	a, (0x05, sp)
      0082D0 0F 01            [ 1]  701 	clr	(0x01, sp)
      0082D2 AE 52 62         [ 2]  702 	ldw	x, #0x5262
      0082D5 F7               [ 1]  703 	ld	(x), a
                                    704 ;	periph_stm8s.c: 203: TIM1_ARRL = (timval & 0x00FF); //TIM RELOAD
      0082D6 7B 06            [ 1]  705 	ld	a, (0x06, sp)
      0082D8 95               [ 1]  706 	ld	xh, a
      0082D9 4F               [ 1]  707 	clr	a
      0082DA 9E               [ 1]  708 	ld	a, xh
      0082DB AE 52 63         [ 2]  709 	ldw	x, #0x5263
      0082DE F7               [ 1]  710 	ld	(x), a
                                    711 ;	periph_stm8s.c: 204: pwm1ch1_enable();
      0082DF CD 83 3A         [ 4]  712 	call	_pwm1ch1_enable
                                    713 ;	periph_stm8s.c: 205: TIM1_CCER1 |= (0<<TIM1_CCER1_CC1P); //Output active high
      0082E2 AE 52 5C         [ 2]  714 	ldw	x, #0x525c
      0082E5 F6               [ 1]  715 	ld	a, (x)
      0082E6 AE 52 5C         [ 2]  716 	ldw	x, #0x525c
      0082E9 F7               [ 1]  717 	ld	(x), a
                                    718 ;	periph_stm8s.c: 206: TIM1_CCMR1 = (TIM1_OCxREF_PWM_mode1<<TIM1_CCMR1_OC1M); //PWM MODE 1 for Channel 1
      0082EA 35 60 52 58      [ 1]  719 	mov	0x5258+0, #0x60
                                    720 ;	periph_stm8s.c: 207: pwm1_update(0x0000); //Start Value
      0082EE 5F               [ 1]  721 	clrw	x
      0082EF 89               [ 2]  722 	pushw	x
      0082F0 CD 83 4E         [ 4]  723 	call	_pwm1_update
      0082F3 5B 02            [ 2]  724 	addw	sp, #2
                                    725 ;	periph_stm8s.c: 208: TIM1_BKR = (1<<TIM1_BKR_MOE); //ENABLE MAIN OUTPUT 
      0082F5 35 80 52 6D      [ 1]  726 	mov	0x526d+0, #0x80
                                    727 ;	periph_stm8s.c: 209: TIM1_CR1 |= (1<<TIM1_CR1_CEN); //ENABLE TIM
      0082F9 AE 52 50         [ 2]  728 	ldw	x, #0x5250
      0082FC F6               [ 1]  729 	ld	a, (x)
      0082FD AA 01            [ 1]  730 	or	a, #0x01
      0082FF F7               [ 1]  731 	ld	(x), a
      008300 5B 02            [ 2]  732 	addw	sp, #2
      008302 81               [ 4]  733 	ret
                                    734 ;	periph_stm8s.c: 212: void pwm2_init(unsigned int timval)
                                    735 ;	-----------------------------------------
                                    736 ;	 function pwm2_init
                                    737 ;	-----------------------------------------
      008303                        738 _pwm2_init:
      008303 52 02            [ 2]  739 	sub	sp, #2
                                    740 ;	periph_stm8s.c: 214: TIM2_PSCR = 0x00; //TIM_CLK = CLK
      008305 35 00 53 0E      [ 1]  741 	mov	0x530e+0, #0x00
                                    742 ;	periph_stm8s.c: 215: TIM2_ARRH = (timval >> 8); //TIM RELOAD
      008309 7B 05            [ 1]  743 	ld	a, (0x05, sp)
      00830B 0F 01            [ 1]  744 	clr	(0x01, sp)
      00830D AE 53 0F         [ 2]  745 	ldw	x, #0x530f
      008310 F7               [ 1]  746 	ld	(x), a
                                    747 ;	periph_stm8s.c: 216: TIM2_ARRL = (timval & 0x00FF); //TIM RELOAD
      008311 7B 06            [ 1]  748 	ld	a, (0x06, sp)
      008313 95               [ 1]  749 	ld	xh, a
      008314 4F               [ 1]  750 	clr	a
      008315 9E               [ 1]  751 	ld	a, xh
      008316 AE 53 10         [ 2]  752 	ldw	x, #0x5310
      008319 F7               [ 1]  753 	ld	(x), a
                                    754 ;	periph_stm8s.c: 217: pwm2ch1_enable();
      00831A CD 83 44         [ 4]  755 	call	_pwm2ch1_enable
                                    756 ;	periph_stm8s.c: 218: TIM2_CCER1 |= (0<<TIM2_CCER1_CC1P); //Output active high
      00831D AE 53 0A         [ 2]  757 	ldw	x, #0x530a
      008320 F6               [ 1]  758 	ld	a, (x)
      008321 AE 53 0A         [ 2]  759 	ldw	x, #0x530a
      008324 F7               [ 1]  760 	ld	(x), a
                                    761 ;	periph_stm8s.c: 219: TIM2_CCMR1 = (TIM2_OCxREF_PWM_mode1<<TIM2_CCMR1_OC1M); //PWM MODE 1 for Channel 1 
      008325 35 60 53 07      [ 1]  762 	mov	0x5307+0, #0x60
                                    763 ;	periph_stm8s.c: 220: pwm2_update(0x0000); //Start Value
      008329 5F               [ 1]  764 	clrw	x
      00832A 89               [ 2]  765 	pushw	x
      00832B CD 83 64         [ 4]  766 	call	_pwm2_update
      00832E 5B 02            [ 2]  767 	addw	sp, #2
                                    768 ;	periph_stm8s.c: 221: TIM2_CR1 |= (1<<TIM2_CR1_CEN); //ENABLE TIM
      008330 AE 53 00         [ 2]  769 	ldw	x, #0x5300
      008333 F6               [ 1]  770 	ld	a, (x)
      008334 AA 01            [ 1]  771 	or	a, #0x01
      008336 F7               [ 1]  772 	ld	(x), a
      008337 5B 02            [ 2]  773 	addw	sp, #2
      008339 81               [ 4]  774 	ret
                                    775 ;	periph_stm8s.c: 224: void pwm1ch1_enable()
                                    776 ;	-----------------------------------------
                                    777 ;	 function pwm1ch1_enable
                                    778 ;	-----------------------------------------
      00833A                        779 _pwm1ch1_enable:
                                    780 ;	periph_stm8s.c: 226: TIM1_CCER1 |= (1<<TIM1_CCER1_CC1E);
      00833A 72 10 52 5C      [ 1]  781 	bset	0x525c, #0
      00833E 81               [ 4]  782 	ret
                                    783 ;	periph_stm8s.c: 229: void pwm1ch1_disable()
                                    784 ;	-----------------------------------------
                                    785 ;	 function pwm1ch1_disable
                                    786 ;	-----------------------------------------
      00833F                        787 _pwm1ch1_disable:
                                    788 ;	periph_stm8s.c: 231: TIM1_CCER1 &= ~(1<<TIM1_CCER1_CC1E);
      00833F 72 11 52 5C      [ 1]  789 	bres	0x525c, #0
      008343 81               [ 4]  790 	ret
                                    791 ;	periph_stm8s.c: 234: void pwm2ch1_enable()
                                    792 ;	-----------------------------------------
                                    793 ;	 function pwm2ch1_enable
                                    794 ;	-----------------------------------------
      008344                        795 _pwm2ch1_enable:
                                    796 ;	periph_stm8s.c: 236: TIM2_CCER1 |= (1<<TIM2_CCER1_CC1E);
      008344 72 10 53 0A      [ 1]  797 	bset	0x530a, #0
      008348 81               [ 4]  798 	ret
                                    799 ;	periph_stm8s.c: 239: void pwm2ch1_disable()
                                    800 ;	-----------------------------------------
                                    801 ;	 function pwm2ch1_disable
                                    802 ;	-----------------------------------------
      008349                        803 _pwm2ch1_disable:
                                    804 ;	periph_stm8s.c: 241: TIM2_CCER1 &= ~(1<<TIM2_CCER1_CC1E);
      008349 72 11 53 0A      [ 1]  805 	bres	0x530a, #0
      00834D 81               [ 4]  806 	ret
                                    807 ;	periph_stm8s.c: 244: void pwm1_update(unsigned int pwmval)
                                    808 ;	-----------------------------------------
                                    809 ;	 function pwm1_update
                                    810 ;	-----------------------------------------
      00834E                        811 _pwm1_update:
      00834E 52 02            [ 2]  812 	sub	sp, #2
                                    813 ;	periph_stm8s.c: 246: TIM1_CCR1L = (pwmval & 0x00FF);
      008350 7B 06            [ 1]  814 	ld	a, (0x06, sp)
      008352 95               [ 1]  815 	ld	xh, a
      008353 4F               [ 1]  816 	clr	a
      008354 9E               [ 1]  817 	ld	a, xh
      008355 AE 52 66         [ 2]  818 	ldw	x, #0x5266
      008358 F7               [ 1]  819 	ld	(x), a
                                    820 ;	periph_stm8s.c: 247: TIM1_CCR1H = (pwmval >> 8);
      008359 7B 05            [ 1]  821 	ld	a, (0x05, sp)
      00835B 0F 01            [ 1]  822 	clr	(0x01, sp)
      00835D AE 52 65         [ 2]  823 	ldw	x, #0x5265
      008360 F7               [ 1]  824 	ld	(x), a
      008361 5B 02            [ 2]  825 	addw	sp, #2
      008363 81               [ 4]  826 	ret
                                    827 ;	periph_stm8s.c: 250: void pwm2_update(unsigned int pwmval)
                                    828 ;	-----------------------------------------
                                    829 ;	 function pwm2_update
                                    830 ;	-----------------------------------------
      008364                        831 _pwm2_update:
      008364 52 02            [ 2]  832 	sub	sp, #2
                                    833 ;	periph_stm8s.c: 252: TIM2_CCR1L = (pwmval & 0x00FF);
      008366 7B 06            [ 1]  834 	ld	a, (0x06, sp)
      008368 95               [ 1]  835 	ld	xh, a
      008369 4F               [ 1]  836 	clr	a
      00836A 9E               [ 1]  837 	ld	a, xh
      00836B AE 53 12         [ 2]  838 	ldw	x, #0x5312
      00836E F7               [ 1]  839 	ld	(x), a
                                    840 ;	periph_stm8s.c: 253: TIM2_CCR1H = (pwmval >> 8);
      00836F 7B 05            [ 1]  841 	ld	a, (0x05, sp)
      008371 0F 01            [ 1]  842 	clr	(0x01, sp)
      008373 AE 53 11         [ 2]  843 	ldw	x, #0x5311
      008376 F7               [ 1]  844 	ld	(x), a
      008377 5B 02            [ 2]  845 	addw	sp, #2
      008379 81               [ 4]  846 	ret
                                    847 ;	lcd_n1202_stm8s.c: 7: void lcdn1202_gpio_init()
                                    848 ;	-----------------------------------------
                                    849 ;	 function lcdn1202_gpio_init
                                    850 ;	-----------------------------------------
      00837A                        851 _lcdn1202_gpio_init:
                                    852 ;	lcd_n1202_stm8s.c: 9: LCDDDR |= (OUTPUT<<LCDDAT)|(OUTPUT<<LCDCLK)|(OUTPUT<<LCDBL);	//Configure GPIO as Output
      00837A AE 50 02         [ 2]  853 	ldw	x, #0x5002
      00837D F6               [ 1]  854 	ld	a, (x)
      00837E AA 0E            [ 1]  855 	or	a, #0x0e
      008380 F7               [ 1]  856 	ld	(x), a
                                    857 ;	lcd_n1202_stm8s.c: 10: LCDCR1 |= (pushpull<<LCDDAT)|(pushpull<<LCDCLK)|(pushpull<<LCDBL); //Configure Output Type
      008381 AE 50 03         [ 2]  858 	ldw	x, #0x5003
      008384 F6               [ 1]  859 	ld	a, (x)
      008385 AA 0E            [ 1]  860 	or	a, #0x0e
      008387 F7               [ 1]  861 	ld	(x), a
                                    862 ;	lcd_n1202_stm8s.c: 11: LCDCR2 |= (speed_10MHz<<LCDDAT)|(speed_10MHz<<LCDCLK)|(speed_10MHz<<LCDBL); //Configure GPIO speed
      008388 AE 50 04         [ 2]  863 	ldw	x, #0x5004
      00838B F6               [ 1]  864 	ld	a, (x)
      00838C AA 0E            [ 1]  865 	or	a, #0x0e
      00838E F7               [ 1]  866 	ld	(x), a
                                    867 ;	lcd_n1202_stm8s.c: 12: LCDODR = 0x00; //Starting value
      00838F 35 00 50 00      [ 1]  868 	mov	0x5000+0, #0x00
      008393 81               [ 4]  869 	ret
                                    870 ;	lcd_n1202_stm8s.c: 15: void lcdn1202_9bsend(unsigned char cdsign, unsigned char comdat)
                                    871 ;	-----------------------------------------
                                    872 ;	 function lcdn1202_9bsend
                                    873 ;	-----------------------------------------
      008394                        874 _lcdn1202_9bsend:
      008394 88               [ 1]  875 	push	a
                                    876 ;	lcd_n1202_stm8s.c: 19: if(cdsign==0) LCDODR &= LCDDAT_MASKL; //1st bit is 0 for Command
      008395 0D 04            [ 1]  877 	tnz	(0x04, sp)
      008397 26 09            [ 1]  878 	jrne	00102$
      008399 AE 50 00         [ 2]  879 	ldw	x, #0x5000
      00839C F6               [ 1]  880 	ld	a, (x)
      00839D A4 FD            [ 1]  881 	and	a, #0xfd
      00839F F7               [ 1]  882 	ld	(x), a
      0083A0 20 07            [ 2]  883 	jra	00103$
      0083A2                        884 00102$:
                                    885 ;	lcd_n1202_stm8s.c: 20: else LCDODR |= LCDDAT_MASKH; //1st bit is 1 for Data
      0083A2 AE 50 00         [ 2]  886 	ldw	x, #0x5000
      0083A5 F6               [ 1]  887 	ld	a, (x)
      0083A6 AA 02            [ 1]  888 	or	a, #0x02
      0083A8 F7               [ 1]  889 	ld	(x), a
      0083A9                        890 00103$:
                                    891 ;	lcd_n1202_stm8s.c: 21: lcdn1202_clock1();
      0083A9 CD 83 D8         [ 4]  892 	call	_lcdn1202_clock1
                                    893 ;	lcd_n1202_stm8s.c: 23: for(cdi=0;cdi<8;cdi++) //Send 2nd-9th bit
      0083AC 0F 01            [ 1]  894 	clr	(0x01, sp)
      0083AE                        895 00108$:
                                    896 ;	lcd_n1202_stm8s.c: 25: if(comdat & 0x80) LCDODR |= LCDDAT_MASKH; //LCDDAT = '1'
      0083AE 0D 05            [ 1]  897 	tnz	(0x05, sp)
      0083B0 2A 09            [ 1]  898 	jrpl	00105$
      0083B2 AE 50 00         [ 2]  899 	ldw	x, #0x5000
      0083B5 F6               [ 1]  900 	ld	a, (x)
      0083B6 AA 02            [ 1]  901 	or	a, #0x02
      0083B8 F7               [ 1]  902 	ld	(x), a
      0083B9 20 07            [ 2]  903 	jra	00106$
      0083BB                        904 00105$:
                                    905 ;	lcd_n1202_stm8s.c: 26: else LCDODR &= LCDDAT_MASKL;		  //LCDDAT = '0'
      0083BB AE 50 00         [ 2]  906 	ldw	x, #0x5000
      0083BE F6               [ 1]  907 	ld	a, (x)
      0083BF A4 FD            [ 1]  908 	and	a, #0xfd
      0083C1 F7               [ 1]  909 	ld	(x), a
      0083C2                        910 00106$:
                                    911 ;	lcd_n1202_stm8s.c: 27: lcdn1202_clock1();
      0083C2 CD 83 D8         [ 4]  912 	call	_lcdn1202_clock1
                                    913 ;	lcd_n1202_stm8s.c: 28: comdat <<= 1; //Shift to next bit
      0083C5 08 05            [ 1]  914 	sll	(0x05, sp)
                                    915 ;	lcd_n1202_stm8s.c: 23: for(cdi=0;cdi<8;cdi++) //Send 2nd-9th bit
      0083C7 0C 01            [ 1]  916 	inc	(0x01, sp)
      0083C9 7B 01            [ 1]  917 	ld	a, (0x01, sp)
      0083CB A1 08            [ 1]  918 	cp	a, #0x08
      0083CD 25 DF            [ 1]  919 	jrc	00108$
                                    920 ;	lcd_n1202_stm8s.c: 30: LCDODR &= LCDDAT_MASKL;
      0083CF AE 50 00         [ 2]  921 	ldw	x, #0x5000
      0083D2 F6               [ 1]  922 	ld	a, (x)
      0083D3 A4 FD            [ 1]  923 	and	a, #0xfd
      0083D5 F7               [ 1]  924 	ld	(x), a
      0083D6 84               [ 1]  925 	pop	a
      0083D7 81               [ 4]  926 	ret
                                    927 ;	lcd_n1202_stm8s.c: 33: void lcdn1202_clock1()
                                    928 ;	-----------------------------------------
                                    929 ;	 function lcdn1202_clock1
                                    930 ;	-----------------------------------------
      0083D8                        931 _lcdn1202_clock1:
                                    932 ;	lcd_n1202_stm8s.c: 35: LCDODR |= LCDCLK_MASKH; //Send 1 pulse to LCDCLK
      0083D8 AE 50 00         [ 2]  933 	ldw	x, #0x5000
      0083DB F6               [ 1]  934 	ld	a, (x)
      0083DC AA 04            [ 1]  935 	or	a, #0x04
      0083DE F7               [ 1]  936 	ld	(x), a
                                    937 ;	lcd_n1202_stm8s.c: 36: delay_us(1); //Short delay
      0083DF 4B 01            [ 1]  938 	push	#0x01
      0083E1 5F               [ 1]  939 	clrw	x
      0083E2 89               [ 2]  940 	pushw	x
      0083E3 4B 00            [ 1]  941 	push	#0x00
      0083E5 CD 80 A5         [ 4]  942 	call	_delay_us
      0083E8 5B 04            [ 2]  943 	addw	sp, #4
                                    944 ;	lcd_n1202_stm8s.c: 37: LCDODR &= LCDCLK_MASKL;
      0083EA AE 50 00         [ 2]  945 	ldw	x, #0x5000
      0083ED F6               [ 1]  946 	ld	a, (x)
      0083EE A4 FB            [ 1]  947 	and	a, #0xfb
      0083F0 F7               [ 1]  948 	ld	(x), a
      0083F1 81               [ 4]  949 	ret
                                    950 ;	lcd_n1202_stm8s.c: 40: void lcdn1202_blon()
                                    951 ;	-----------------------------------------
                                    952 ;	 function lcdn1202_blon
                                    953 ;	-----------------------------------------
      0083F2                        954 _lcdn1202_blon:
                                    955 ;	lcd_n1202_stm8s.c: 42: LCDODR |= LCDBL_MASKH; //LCDBL = '1'
      0083F2 AE 50 00         [ 2]  956 	ldw	x, #0x5000
      0083F5 F6               [ 1]  957 	ld	a, (x)
      0083F6 AA 08            [ 1]  958 	or	a, #0x08
      0083F8 F7               [ 1]  959 	ld	(x), a
      0083F9 81               [ 4]  960 	ret
                                    961 ;	lcd_n1202_stm8s.c: 45: void lcdn1202_bloff()
                                    962 ;	-----------------------------------------
                                    963 ;	 function lcdn1202_bloff
                                    964 ;	-----------------------------------------
      0083FA                        965 _lcdn1202_bloff:
                                    966 ;	lcd_n1202_stm8s.c: 47: LCDODR &= LCDBL_MASKL; //LCDBL = '0'
      0083FA AE 50 00         [ 2]  967 	ldw	x, #0x5000
      0083FD F6               [ 1]  968 	ld	a, (x)
      0083FE A4 F7            [ 1]  969 	and	a, #0xf7
      008400 F7               [ 1]  970 	ld	(x), a
      008401 81               [ 4]  971 	ret
                                    972 ;	lcd_n1202.c: 9: void lcdn1202_init()
                                    973 ;	-----------------------------------------
                                    974 ;	 function lcdn1202_init
                                    975 ;	-----------------------------------------
      008402                        976 _lcdn1202_init:
                                    977 ;	lcd_n1202.c: 11: lcdn1202_gpio_init();
      008402 CD 83 7A         [ 4]  978 	call	_lcdn1202_gpio_init
                                    979 ;	lcd_n1202.c: 15: delay_ms(10);
      008405 4B 0A            [ 1]  980 	push	#0x0a
      008407 5F               [ 1]  981 	clrw	x
      008408 89               [ 2]  982 	pushw	x
      008409 4B 00            [ 1]  983 	push	#0x00
      00840B CD 80 F5         [ 4]  984 	call	_delay_ms
      00840E 5B 04            [ 2]  985 	addw	sp, #4
                                    986 ;	lcd_n1202.c: 17: lcdn1202_sendcom(0xE2);	//Soft Reset
      008410 4B E2            [ 1]  987 	push	#0xe2
      008412 CD 84 4E         [ 4]  988 	call	_lcdn1202_sendcom
      008415 84               [ 1]  989 	pop	a
                                    990 ;	lcd_n1202.c: 18: delay_ms(1);
      008416 4B 01            [ 1]  991 	push	#0x01
      008418 5F               [ 1]  992 	clrw	x
      008419 89               [ 2]  993 	pushw	x
      00841A 4B 00            [ 1]  994 	push	#0x00
      00841C CD 80 F5         [ 4]  995 	call	_delay_ms
      00841F 5B 04            [ 2]  996 	addw	sp, #4
                                    997 ;	lcd_n1202.c: 19: lcdn1202_sendcom(0xA4); //Normal Display Mode
      008421 4B A4            [ 1]  998 	push	#0xa4
      008423 CD 84 4E         [ 4]  999 	call	_lcdn1202_sendcom
      008426 84               [ 1] 1000 	pop	a
                                   1001 ;	lcd_n1202.c: 20: lcdn1202_sendcom(0x2F);	//Power Control = Max (Booster On, VReg On, VFol On)
      008427 4B 2F            [ 1] 1002 	push	#0x2f
      008429 CD 84 4E         [ 4] 1003 	call	_lcdn1202_sendcom
      00842C 84               [ 1] 1004 	pop	a
                                   1005 ;	lcd_n1202.c: 22: lcdn1202_sendcom(0xA0); //Segment Driver Direction = Normal (lines start at left)
      00842D 4B A0            [ 1] 1006 	push	#0xa0
      00842F CD 84 4E         [ 4] 1007 	call	_lcdn1202_sendcom
      008432 84               [ 1] 1008 	pop	a
                                   1009 ;	lcd_n1202.c: 23: lcdn1202_sendcom(0xC0); //Common Driver Direction = Normal
      008433 4B C0            [ 1] 1010 	push	#0xc0
      008435 CD 84 4E         [ 4] 1011 	call	_lcdn1202_sendcom
      008438 84               [ 1] 1012 	pop	a
                                   1013 ;	lcd_n1202.c: 24: lcdn1202_sendcom(0x80|16); //Set Contrast to default
      008439 4B 90            [ 1] 1014 	push	#0x90
      00843B CD 84 4E         [ 4] 1015 	call	_lcdn1202_sendcom
      00843E 84               [ 1] 1016 	pop	a
                                   1017 ;	lcd_n1202.c: 26: lcdn1202_sendcom(0xAF);	//Display On
      00843F 4B AF            [ 1] 1018 	push	#0xaf
      008441 CD 84 4E         [ 4] 1019 	call	_lcdn1202_sendcom
      008444 84               [ 1] 1020 	pop	a
                                   1021 ;	lcd_n1202.c: 28: LCD_BL_OFF(); //Backlight off
      008445 CD 86 17         [ 4] 1022 	call	_LCD_BL_OFF
                                   1023 ;	lcd_n1202.c: 29: LCD_clear();  //Clear pixel memory
      008448 CD 85 D9         [ 4] 1024 	call	_LCD_clear
                                   1025 ;	lcd_n1202.c: 30: LCD_BL_ON();  //Backlight on
      00844B CC 86 14         [ 2] 1026 	jp	_LCD_BL_ON
                                   1027 ;	lcd_n1202.c: 33: void lcdn1202_sendcom(unsigned char ssd1306com)
                                   1028 ;	-----------------------------------------
                                   1029 ;	 function lcdn1202_sendcom
                                   1030 ;	-----------------------------------------
      00844E                       1031 _lcdn1202_sendcom:
                                   1032 ;	lcd_n1202.c: 35: lcdn1202_9bsend(0,ssd1306com); //Send Command
      00844E 7B 03            [ 1] 1033 	ld	a, (0x03, sp)
      008450 88               [ 1] 1034 	push	a
      008451 4B 00            [ 1] 1035 	push	#0x00
      008453 CD 83 94         [ 4] 1036 	call	_lcdn1202_9bsend
      008456 5B 02            [ 2] 1037 	addw	sp, #2
      008458 81               [ 4] 1038 	ret
                                   1039 ;	lcd_n1202.c: 38: void lcdn1202_senddat(unsigned char ssd1306dat)
                                   1040 ;	-----------------------------------------
                                   1041 ;	 function lcdn1202_senddat
                                   1042 ;	-----------------------------------------
      008459                       1043 _lcdn1202_senddat:
                                   1044 ;	lcd_n1202.c: 40: lcdn1202_9bsend(1,ssd1306dat); //Send Data
      008459 7B 03            [ 1] 1045 	ld	a, (0x03, sp)
      00845B 88               [ 1] 1046 	push	a
      00845C 4B 01            [ 1] 1047 	push	#0x01
      00845E CD 83 94         [ 4] 1048 	call	_lcdn1202_9bsend
      008461 5B 02            [ 2] 1049 	addw	sp, #2
      008463 81               [ 4] 1050 	ret
                                   1051 ;	lcd_n1202.c: 43: void lcdn1202_setpos(unsigned char row, unsigned char col)
                                   1052 ;	-----------------------------------------
                                   1053 ;	 function lcdn1202_setpos
                                   1054 ;	-----------------------------------------
      008464                       1055 _lcdn1202_setpos:
                                   1056 ;	lcd_n1202.c: 45: lcdn1202_sendcom(0xB0|(row&0x0F)); //Set page of row
      008464 7B 03            [ 1] 1057 	ld	a, (0x03, sp)
      008466 A4 0F            [ 1] 1058 	and	a, #0x0f
      008468 AA B0            [ 1] 1059 	or	a, #0xb0
      00846A 88               [ 1] 1060 	push	a
      00846B CD 84 4E         [ 4] 1061 	call	_lcdn1202_sendcom
      00846E 84               [ 1] 1062 	pop	a
                                   1063 ;	lcd_n1202.c: 46: lcdn1202_sendcom(0x00|(col&0x0F)); //Set lower nibble of Column
      00846F 7B 04            [ 1] 1064 	ld	a, (0x04, sp)
      008471 A4 0F            [ 1] 1065 	and	a, #0x0f
      008473 88               [ 1] 1066 	push	a
      008474 CD 84 4E         [ 4] 1067 	call	_lcdn1202_sendcom
      008477 84               [ 1] 1068 	pop	a
                                   1069 ;	lcd_n1202.c: 47: lcdn1202_sendcom(0x10|((col>>4)&0x0F)); //Set upper nibble of Column
      008478 7B 04            [ 1] 1070 	ld	a, (0x04, sp)
      00847A 4E               [ 1] 1071 	swap	a
      00847B A4 0F            [ 1] 1072 	and	a, #0x0f
      00847D A4 0F            [ 1] 1073 	and	a, #0x0f
      00847F AA 10            [ 1] 1074 	or	a, #0x10
      008481 88               [ 1] 1075 	push	a
      008482 CD 84 4E         [ 4] 1076 	call	_lcdn1202_sendcom
      008485 84               [ 1] 1077 	pop	a
      008486 81               [ 4] 1078 	ret
                                   1079 ;	lcd_n1202.c: 50: void lcdn1202_clear()
                                   1080 ;	-----------------------------------------
                                   1081 ;	 function lcdn1202_clear
                                   1082 ;	-----------------------------------------
      008487                       1083 _lcdn1202_clear:
      008487 88               [ 1] 1084 	push	a
                                   1085 ;	lcd_n1202.c: 53: lcdn1202_setpos(0,0);
      008488 4B 00            [ 1] 1086 	push	#0x00
      00848A 4B 00            [ 1] 1087 	push	#0x00
      00848C CD 84 64         [ 4] 1088 	call	_lcdn1202_setpos
      00848F 5B 02            [ 2] 1089 	addw	sp, #2
                                   1090 ;	lcd_n1202.c: 54: for(row=0;row<LCDN1202_ROW;row++)	//Scan rows (pages)
      008491 0F 01            [ 1] 1091 	clr	(0x01, sp)
                                   1092 ;	lcd_n1202.c: 56: for(col=0;col<LCDN1202_COL;col++)	//Scan columns
      008493                       1093 00109$:
      008493 4F               [ 1] 1094 	clr	a
      008494                       1095 00103$:
                                   1096 ;	lcd_n1202.c: 58: lcdn1202_senddat(0);	//Send 0 to every pixel
      008494 88               [ 1] 1097 	push	a
      008495 4B 00            [ 1] 1098 	push	#0x00
      008497 CD 84 59         [ 4] 1099 	call	_lcdn1202_senddat
      00849A 84               [ 1] 1100 	pop	a
      00849B 84               [ 1] 1101 	pop	a
                                   1102 ;	lcd_n1202.c: 56: for(col=0;col<LCDN1202_COL;col++)	//Scan columns
      00849C 4C               [ 1] 1103 	inc	a
      00849D A1 60            [ 1] 1104 	cp	a, #0x60
      00849F 25 F3            [ 1] 1105 	jrc	00103$
                                   1106 ;	lcd_n1202.c: 54: for(row=0;row<LCDN1202_ROW;row++)	//Scan rows (pages)
      0084A1 0C 01            [ 1] 1107 	inc	(0x01, sp)
      0084A3 7B 01            [ 1] 1108 	ld	a, (0x01, sp)
      0084A5 A1 09            [ 1] 1109 	cp	a, #0x09
      0084A7 25 EA            [ 1] 1110 	jrc	00109$
      0084A9 84               [ 1] 1111 	pop	a
      0084AA 81               [ 4] 1112 	ret
                                   1113 ;	lcd_n1202.c: 63: void LCD_setpos(unsigned char row, unsigned char col)
                                   1114 ;	-----------------------------------------
                                   1115 ;	 function LCD_setpos
                                   1116 ;	-----------------------------------------
      0084AB                       1117 _LCD_setpos:
                                   1118 ;	lcd_n1202.c: 65: lcdn1202_setpos(row,col); //Set coordinate (for LCD_drawbyte)
      0084AB 7B 04            [ 1] 1119 	ld	a, (0x04, sp)
      0084AD 88               [ 1] 1120 	push	a
      0084AE 7B 04            [ 1] 1121 	ld	a, (0x04, sp)
      0084B0 88               [ 1] 1122 	push	a
      0084B1 CD 84 64         [ 4] 1123 	call	_lcdn1202_setpos
      0084B4 5B 02            [ 2] 1124 	addw	sp, #2
      0084B6 81               [ 4] 1125 	ret
                                   1126 ;	lcd_n1202.c: 68: void LCD_drawbyte(unsigned char dbyte)
                                   1127 ;	-----------------------------------------
                                   1128 ;	 function LCD_drawbyte
                                   1129 ;	-----------------------------------------
      0084B7                       1130 _LCD_drawbyte:
                                   1131 ;	lcd_n1202.c: 70: lcdn1202_senddat(dbyte); //Send 1 byte data only
      0084B7 7B 03            [ 1] 1132 	ld	a, (0x03, sp)
      0084B9 88               [ 1] 1133 	push	a
      0084BA CD 84 59         [ 4] 1134 	call	_lcdn1202_senddat
      0084BD 84               [ 1] 1135 	pop	a
      0084BE 81               [ 4] 1136 	ret
                                   1137 ;	lcd_n1202.c: 73: void LCD_drawchar(unsigned char chr, unsigned char chrrow, unsigned char chrcol)
                                   1138 ;	-----------------------------------------
                                   1139 ;	 function LCD_drawchar
                                   1140 ;	-----------------------------------------
      0084BF                       1141 _LCD_drawchar:
      0084BF 52 0B            [ 2] 1142 	sub	sp, #11
                                   1143 ;	lcd_n1202.c: 78: lcdn1202_setpos(chrrow,chrcol);
      0084C1 7B 10            [ 1] 1144 	ld	a, (0x10, sp)
      0084C3 88               [ 1] 1145 	push	a
      0084C4 7B 10            [ 1] 1146 	ld	a, (0x10, sp)
      0084C6 88               [ 1] 1147 	push	a
      0084C7 CD 84 64         [ 4] 1148 	call	_lcdn1202_setpos
      0084CA 5B 02            [ 2] 1149 	addw	sp, #2
                                   1150 ;	lcd_n1202.c: 83: chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
      0084CC 7B 0E            [ 1] 1151 	ld	a, (0x0e, sp)
      0084CE 6B 0B            [ 1] 1152 	ld	(0x0b, sp), a
      0084D0 0F 0A            [ 1] 1153 	clr	(0x0a, sp)
                                   1154 ;	lcd_n1202.c: 80: if((chr>31)&&(chr<128))	//Alphanumeric & Punctuation Area
      0084D2 7B 0E            [ 1] 1155 	ld	a, (0x0e, sp)
      0084D4 A1 1F            [ 1] 1156 	cp	a, #0x1f
      0084D6 23 3F            [ 2] 1157 	jrule	00107$
      0084D8 7B 0E            [ 1] 1158 	ld	a, (0x0e, sp)
      0084DA A1 80            [ 1] 1159 	cp	a, #0x80
      0084DC 24 39            [ 1] 1160 	jrnc	00107$
                                   1161 ;	lcd_n1202.c: 82: lcdn1202_senddat(0x00);
      0084DE 4B 00            [ 1] 1162 	push	#0x00
      0084E0 CD 84 59         [ 4] 1163 	call	_lcdn1202_senddat
      0084E3 84               [ 1] 1164 	pop	a
                                   1165 ;	lcd_n1202.c: 83: chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
      0084E4 1E 0A            [ 2] 1166 	ldw	x, (0x0a, sp)
      0084E6 1D 00 20         [ 2] 1167 	subw	x, #0x0020
      0084E9 89               [ 2] 1168 	pushw	x
      0084EA 4B 05            [ 1] 1169 	push	#0x05
      0084EC 4B 00            [ 1] 1170 	push	#0x00
      0084EE CD 8C D2         [ 4] 1171 	call	__mulint
      0084F1 5B 04            [ 2] 1172 	addw	sp, #4
      0084F3 1F 08            [ 2] 1173 	ldw	(0x08, sp), x
                                   1174 ;	lcd_n1202.c: 84: for(ci=0;ci<5;ci++)
      0084F5 AE 89 E7         [ 2] 1175 	ldw	x, #_font_arr+0
      0084F8 1F 06            [ 2] 1176 	ldw	(0x06, sp), x
      0084FA 4F               [ 1] 1177 	clr	a
      0084FB                       1178 00110$:
                                   1179 ;	lcd_n1202.c: 86: fchar = font_arr[chridx+ci]; //Get character pattern from Font Array
      0084FB 5F               [ 1] 1180 	clrw	x
      0084FC 97               [ 1] 1181 	ld	xl, a
      0084FD 72 FB 08         [ 2] 1182 	addw	x, (0x08, sp)
      008500 72 FB 06         [ 2] 1183 	addw	x, (0x06, sp)
      008503 88               [ 1] 1184 	push	a
      008504 F6               [ 1] 1185 	ld	a, (x)
      008505 97               [ 1] 1186 	ld	xl, a
      008506 84               [ 1] 1187 	pop	a
                                   1188 ;	lcd_n1202.c: 87: lcdn1202_senddat(fchar); //Send pattern 1 byte at a time
      008507 88               [ 1] 1189 	push	a
      008508 89               [ 2] 1190 	pushw	x
      008509 5B 01            [ 2] 1191 	addw	sp, #1
      00850B CD 84 59         [ 4] 1192 	call	_lcdn1202_senddat
      00850E 84               [ 1] 1193 	pop	a
      00850F 84               [ 1] 1194 	pop	a
                                   1195 ;	lcd_n1202.c: 84: for(ci=0;ci<5;ci++)
      008510 4C               [ 1] 1196 	inc	a
      008511 A1 05            [ 1] 1197 	cp	a, #0x05
      008513 25 E6            [ 1] 1198 	jrc	00110$
      008515 20 39            [ 2] 1199 	jra	00114$
      008517                       1200 00107$:
                                   1201 ;	lcd_n1202.c: 90: else if((chr>127)&&(chr<148))	//Frame & Arrow Area
      008517 7B 0E            [ 1] 1202 	ld	a, (0x0e, sp)
      008519 A1 7F            [ 1] 1203 	cp	a, #0x7f
      00851B 23 33            [ 2] 1204 	jrule	00114$
      00851D 7B 0E            [ 1] 1205 	ld	a, (0x0e, sp)
      00851F A1 94            [ 1] 1206 	cp	a, #0x94
      008521 24 2D            [ 1] 1207 	jrnc	00114$
                                   1208 ;	lcd_n1202.c: 92: chridx=(chr-128)*8; //Start at index 128. 5 columns for each symbol
      008523 1E 0A            [ 2] 1209 	ldw	x, (0x0a, sp)
      008525 1D 00 80         [ 2] 1210 	subw	x, #0x0080
      008528 58               [ 2] 1211 	sllw	x
      008529 58               [ 2] 1212 	sllw	x
      00852A 58               [ 2] 1213 	sllw	x
                                   1214 ;	lcd_n1202.c: 93: for(ci=0;ci<8;ci++)
      00852B 90 AE 89 E7      [ 2] 1215 	ldw	y, #_font_arr+0
      00852F 17 04            [ 2] 1216 	ldw	(0x04, sp), y
      008531 1C 01 E0         [ 2] 1217 	addw	x, #0x01e0
      008534 1F 02            [ 2] 1218 	ldw	(0x02, sp), x
      008536 0F 01            [ 1] 1219 	clr	(0x01, sp)
      008538                       1220 00112$:
                                   1221 ;	lcd_n1202.c: 95: fchar = font_arr[chridx+480+ci]; //Get symbol pattern from Font Array
      008538 5F               [ 1] 1222 	clrw	x
      008539 7B 01            [ 1] 1223 	ld	a, (0x01, sp)
      00853B 97               [ 1] 1224 	ld	xl, a
      00853C 72 FB 02         [ 2] 1225 	addw	x, (0x02, sp)
      00853F 72 FB 04         [ 2] 1226 	addw	x, (0x04, sp)
      008542 F6               [ 1] 1227 	ld	a, (x)
                                   1228 ;	lcd_n1202.c: 96: lcdn1202_senddat(fchar); //Send pattern 1 byte at a time
      008543 88               [ 1] 1229 	push	a
      008544 CD 84 59         [ 4] 1230 	call	_lcdn1202_senddat
      008547 84               [ 1] 1231 	pop	a
                                   1232 ;	lcd_n1202.c: 93: for(ci=0;ci<8;ci++)
      008548 0C 01            [ 1] 1233 	inc	(0x01, sp)
      00854A 7B 01            [ 1] 1234 	ld	a, (0x01, sp)
      00854C A1 08            [ 1] 1235 	cp	a, #0x08
      00854E 25 E8            [ 1] 1236 	jrc	00112$
      008550                       1237 00114$:
      008550 5B 0B            [ 2] 1238 	addw	sp, #11
      008552 81               [ 4] 1239 	ret
                                   1240 ;	lcd_n1202.c: 102: void LCD_drawtext(char *text, unsigned char txtrow, unsigned char txtcol)
                                   1241 ;	-----------------------------------------
                                   1242 ;	 function LCD_drawtext
                                   1243 ;	-----------------------------------------
      008553                       1244 _LCD_drawtext:
      008553 52 02            [ 2] 1245 	sub	sp, #2
                                   1246 ;	lcd_n1202.c: 106: while(text[stridx] != 0) //Scan characters in string
      008555 5F               [ 1] 1247 	clrw	x
      008556 1F 01            [ 2] 1248 	ldw	(0x01, sp), x
      008558                       1249 00101$:
      008558 1E 05            [ 2] 1250 	ldw	x, (0x05, sp)
      00855A 72 FB 01         [ 2] 1251 	addw	x, (0x01, sp)
      00855D F6               [ 1] 1252 	ld	a, (x)
      00855E 97               [ 1] 1253 	ld	xl, a
      00855F 4D               [ 1] 1254 	tnz	a
      008560 27 19            [ 1] 1255 	jreq	00104$
                                   1256 ;	lcd_n1202.c: 108: LCD_drawchar(text[stridx],txtrow,txtcol+(8*stridx)); //Display each character
      008562 7B 02            [ 1] 1257 	ld	a, (0x02, sp)
      008564 48               [ 1] 1258 	sll	a
      008565 48               [ 1] 1259 	sll	a
      008566 48               [ 1] 1260 	sll	a
      008567 1B 08            [ 1] 1261 	add	a, (0x08, sp)
      008569 88               [ 1] 1262 	push	a
      00856A 7B 08            [ 1] 1263 	ld	a, (0x08, sp)
      00856C 88               [ 1] 1264 	push	a
      00856D 9F               [ 1] 1265 	ld	a, xl
      00856E 88               [ 1] 1266 	push	a
      00856F CD 84 BF         [ 4] 1267 	call	_LCD_drawchar
      008572 5B 03            [ 2] 1268 	addw	sp, #3
                                   1269 ;	lcd_n1202.c: 109: stridx++;
      008574 1E 01            [ 2] 1270 	ldw	x, (0x01, sp)
      008576 5C               [ 2] 1271 	incw	x
      008577 1F 01            [ 2] 1272 	ldw	(0x01, sp), x
      008579 20 DD            [ 2] 1273 	jra	00101$
      00857B                       1274 00104$:
      00857B 5B 02            [ 2] 1275 	addw	sp, #2
      00857D 81               [ 4] 1276 	ret
                                   1277 ;	lcd_n1202.c: 113: void LCD_drawint(unsigned int num, unsigned char numrow, unsigned char numcol)
                                   1278 ;	-----------------------------------------
                                   1279 ;	 function LCD_drawint
                                   1280 ;	-----------------------------------------
      00857E                       1281 _LCD_drawint:
      00857E 52 0C            [ 2] 1282 	sub	sp, #12
                                   1283 ;	lcd_n1202.c: 121: numb = num;
      008580 1E 0F            [ 2] 1284 	ldw	x, (0x0f, sp)
                                   1285 ;	lcd_n1202.c: 122: while(numb!=0) //Counting digit
      008582 4F               [ 1] 1286 	clr	a
      008583                       1287 00101$:
      008583 5D               [ 2] 1288 	tnzw	x
      008584 27 08            [ 1] 1289 	jreq	00114$
                                   1290 ;	lcd_n1202.c: 124: ndigit++;
      008586 4C               [ 1] 1291 	inc	a
                                   1292 ;	lcd_n1202.c: 125: numb /= 10; 
      008587 90 AE 00 0A      [ 2] 1293 	ldw	y, #0x000a
      00858B 65               [ 2] 1294 	divw	x, y
      00858C 20 F5            [ 2] 1295 	jra	00101$
      00858E                       1296 00114$:
      00858E 6B 0A            [ 1] 1297 	ld	(0x0a, sp), a
                                   1298 ;	lcd_n1202.c: 127: for(nd=0;nd<ndigit;nd++) //Converting each digit
      008590 4F               [ 1] 1299 	clr	a
      008591 96               [ 1] 1300 	ldw	x, sp
      008592 1C 00 03         [ 2] 1301 	addw	x, #3
      008595 1F 0B            [ 2] 1302 	ldw	(0x0b, sp), x
      008597                       1303 00106$:
      008597 11 0A            [ 1] 1304 	cp	a, (0x0a, sp)
      008599 24 27            [ 1] 1305 	jrnc	00104$
                                   1306 ;	lcd_n1202.c: 129: numb = num%10;
      00859B 1E 0F            [ 2] 1307 	ldw	x, (0x0f, sp)
      00859D 90 AE 00 0A      [ 2] 1308 	ldw	y, #0x000a
      0085A1 65               [ 2] 1309 	divw	x, y
      0085A2 17 01            [ 2] 1310 	ldw	(0x01, sp), y
                                   1311 ;	lcd_n1202.c: 130: num = num/10;
      0085A4 1E 0F            [ 2] 1312 	ldw	x, (0x0f, sp)
      0085A6 90 AE 00 0A      [ 2] 1313 	ldw	y, #0x000a
      0085AA 65               [ 2] 1314 	divw	x, y
      0085AB 1F 0F            [ 2] 1315 	ldw	(0x0f, sp), x
                                   1316 ;	lcd_n1202.c: 131: ibuff[ndigit-(nd+1)] = numb + '0'; //Start from last_index-1
      0085AD 4C               [ 1] 1317 	inc	a
      0085AE 6B 09            [ 1] 1318 	ld	(0x09, sp), a
      0085B0 7B 0A            [ 1] 1319 	ld	a, (0x0a, sp)
      0085B2 10 09            [ 1] 1320 	sub	a, (0x09, sp)
      0085B4 5F               [ 1] 1321 	clrw	x
      0085B5 97               [ 1] 1322 	ld	xl, a
      0085B6 72 FB 0B         [ 2] 1323 	addw	x, (0x0b, sp)
      0085B9 7B 02            [ 1] 1324 	ld	a, (0x02, sp)
      0085BB AB 30            [ 1] 1325 	add	a, #0x30
      0085BD F7               [ 1] 1326 	ld	(x), a
                                   1327 ;	lcd_n1202.c: 127: for(nd=0;nd<ndigit;nd++) //Converting each digit
      0085BE 7B 09            [ 1] 1328 	ld	a, (0x09, sp)
      0085C0 20 D5            [ 2] 1329 	jra	00106$
      0085C2                       1330 00104$:
                                   1331 ;	lcd_n1202.c: 133: ibuff[ndigit] = '\0'; //Last character is null
      0085C2 5F               [ 1] 1332 	clrw	x
      0085C3 7B 0A            [ 1] 1333 	ld	a, (0x0a, sp)
      0085C5 97               [ 1] 1334 	ld	xl, a
      0085C6 72 FB 0B         [ 2] 1335 	addw	x, (0x0b, sp)
      0085C9 7F               [ 1] 1336 	clr	(x)
                                   1337 ;	lcd_n1202.c: 135: LCD_drawtext(ibuff,numrow,numcol); //Display number as text
      0085CA 1E 0B            [ 2] 1338 	ldw	x, (0x0b, sp)
      0085CC 7B 12            [ 1] 1339 	ld	a, (0x12, sp)
      0085CE 88               [ 1] 1340 	push	a
      0085CF 7B 12            [ 1] 1341 	ld	a, (0x12, sp)
      0085D1 88               [ 1] 1342 	push	a
      0085D2 89               [ 2] 1343 	pushw	x
      0085D3 CD 85 53         [ 4] 1344 	call	_LCD_drawtext
      0085D6 5B 10            [ 2] 1345 	addw	sp, #16
      0085D8 81               [ 4] 1346 	ret
                                   1347 ;	lcd_n1202.c: 138: void LCD_clear()
                                   1348 ;	-----------------------------------------
                                   1349 ;	 function LCD_clear
                                   1350 ;	-----------------------------------------
      0085D9                       1351 _LCD_clear:
                                   1352 ;	lcd_n1202.c: 140: lcdn1202_sendcom(0xAE);  //Set Display off
      0085D9 4B AE            [ 1] 1353 	push	#0xae
      0085DB CD 84 4E         [ 4] 1354 	call	_lcdn1202_sendcom
      0085DE 84               [ 1] 1355 	pop	a
                                   1356 ;	lcd_n1202.c: 141: lcdn1202_clear(); //Clear display
      0085DF CD 84 87         [ 4] 1357 	call	_lcdn1202_clear
                                   1358 ;	lcd_n1202.c: 142: lcdn1202_sendcom(0xAF); //Set Display on
      0085E2 4B AF            [ 1] 1359 	push	#0xaf
      0085E4 CD 84 4E         [ 4] 1360 	call	_lcdn1202_sendcom
      0085E7 84               [ 1] 1361 	pop	a
      0085E8 81               [ 4] 1362 	ret
                                   1363 ;	lcd_n1202.c: 145: void LCD_clearblock(unsigned char row, unsigned char col_start, unsigned char col_fin)
                                   1364 ;	-----------------------------------------
                                   1365 ;	 function LCD_clearblock
                                   1366 ;	-----------------------------------------
      0085E9                       1367 _LCD_clearblock:
                                   1368 ;	lcd_n1202.c: 149: lcdn1202_setpos(row,col_start); //Set start position
      0085E9 7B 04            [ 1] 1369 	ld	a, (0x04, sp)
      0085EB 88               [ 1] 1370 	push	a
      0085EC 7B 04            [ 1] 1371 	ld	a, (0x04, sp)
      0085EE 88               [ 1] 1372 	push	a
      0085EF CD 84 64         [ 4] 1373 	call	_lcdn1202_setpos
      0085F2 5B 02            [ 2] 1374 	addw	sp, #2
                                   1375 ;	lcd_n1202.c: 150: for(col=col_start;col<=col_fin;col++) //Scan columns
      0085F4 7B 04            [ 1] 1376 	ld	a, (0x04, sp)
      0085F6                       1377 00103$:
      0085F6 11 05            [ 1] 1378 	cp	a, (0x05, sp)
      0085F8 22 0B            [ 1] 1379 	jrugt	00105$
                                   1380 ;	lcd_n1202.c: 152: lcdn1202_senddat(0);	//Send 0 to every pixel in a column
      0085FA 88               [ 1] 1381 	push	a
      0085FB 4B 00            [ 1] 1382 	push	#0x00
      0085FD CD 84 59         [ 4] 1383 	call	_lcdn1202_senddat
      008600 84               [ 1] 1384 	pop	a
      008601 84               [ 1] 1385 	pop	a
                                   1386 ;	lcd_n1202.c: 150: for(col=col_start;col<=col_fin;col++) //Scan columns
      008602 4C               [ 1] 1387 	inc	a
      008603 20 F1            [ 2] 1388 	jra	00103$
      008605                       1389 00105$:
      008605 81               [ 4] 1390 	ret
                                   1391 ;	lcd_n1202.c: 156: void LCD_normal()
                                   1392 ;	-----------------------------------------
                                   1393 ;	 function LCD_normal
                                   1394 ;	-----------------------------------------
      008606                       1395 _LCD_normal:
                                   1396 ;	lcd_n1202.c: 158: lcdn1202_sendcom(0xA6);	//Black Pixel in White Background
      008606 4B A6            [ 1] 1397 	push	#0xa6
      008608 CD 84 4E         [ 4] 1398 	call	_lcdn1202_sendcom
      00860B 84               [ 1] 1399 	pop	a
      00860C 81               [ 4] 1400 	ret
                                   1401 ;	lcd_n1202.c: 161: void LCD_reverse()
                                   1402 ;	-----------------------------------------
                                   1403 ;	 function LCD_reverse
                                   1404 ;	-----------------------------------------
      00860D                       1405 _LCD_reverse:
                                   1406 ;	lcd_n1202.c: 163: lcdn1202_sendcom(0xA7);	//White Pixel in Black Background
      00860D 4B A7            [ 1] 1407 	push	#0xa7
      00860F CD 84 4E         [ 4] 1408 	call	_lcdn1202_sendcom
      008612 84               [ 1] 1409 	pop	a
      008613 81               [ 4] 1410 	ret
                                   1411 ;	lcd_n1202.c: 166: void LCD_BL_ON()
                                   1412 ;	-----------------------------------------
                                   1413 ;	 function LCD_BL_ON
                                   1414 ;	-----------------------------------------
      008614                       1415 _LCD_BL_ON:
                                   1416 ;	lcd_n1202.c: 168: lcdn1202_blon(); //Backlight on
      008614 CC 83 F2         [ 2] 1417 	jp	_lcdn1202_blon
                                   1418 ;	lcd_n1202.c: 171: void LCD_BL_OFF()
                                   1419 ;	-----------------------------------------
                                   1420 ;	 function LCD_BL_OFF
                                   1421 ;	-----------------------------------------
      008617                       1422 _LCD_BL_OFF:
                                   1423 ;	lcd_n1202.c: 173: lcdn1202_bloff(); //Backlight off
      008617 CC 83 FA         [ 2] 1424 	jp	_lcdn1202_bloff
                                   1425 ;	main.c: 28: int main()
                                   1426 ;	-----------------------------------------
                                   1427 ;	 function main
                                   1428 ;	-----------------------------------------
      00861A                       1429 _main:
                                   1430 ;	main.c: 30: clock_init();
      00861A CD 81 4F         [ 4] 1431 	call	_clock_init
                                   1432 ;	main.c: 31: delay_init();
      00861D CD 80 A0         [ 4] 1433 	call	_delay_init
                                   1434 ;	main.c: 32: gpio_init();
      008620 CD 87 12         [ 4] 1435 	call	_gpio_init
                                   1436 ;	main.c: 33: lcdn1202_init();
      008623 CD 84 02         [ 4] 1437 	call	_lcdn1202_init
                                   1438 ;	main.c: 34: LCD_clear();
      008626 CD 85 D9         [ 4] 1439 	call	_LCD_clear
                                   1440 ;	main.c: 36: drawLoadingBar();
      008629 CD 89 AB         [ 4] 1441 	call	_drawLoadingBar
                                   1442 ;	main.c: 38: loop();
      00862C CD 86 31         [ 4] 1443 	call	_loop
                                   1444 ;	main.c: 39: return 0;
      00862F 5F               [ 1] 1445 	clrw	x
      008630 81               [ 4] 1446 	ret
                                   1447 ;	main.c: 45: void loop()
                                   1448 ;	-----------------------------------------
                                   1449 ;	 function loop
                                   1450 ;	-----------------------------------------
      008631                       1451 _loop:
                                   1452 ;	main.c: 47: while(1)
      008631                       1453 00102$:
                                   1454 ;	main.c: 49: drawBytes();
      008631 CD 89 48         [ 4] 1455 	call	_drawBytes
                                   1456 ;	main.c: 50: delay_ms(1000);
      008634 4B E8            [ 1] 1457 	push	#0xe8
      008636 4B 03            [ 1] 1458 	push	#0x03
      008638 5F               [ 1] 1459 	clrw	x
      008639 89               [ 2] 1460 	pushw	x
      00863A CD 80 F5         [ 4] 1461 	call	_delay_ms
      00863D 5B 04            [ 2] 1462 	addw	sp, #4
                                   1463 ;	main.c: 51: LCD_clearblock(3,5,84); //Finish column = 5 + 8*10 - 1
      00863F 4B 54            [ 1] 1464 	push	#0x54
      008641 4B 05            [ 1] 1465 	push	#0x05
      008643 4B 03            [ 1] 1466 	push	#0x03
      008645 CD 85 E9         [ 4] 1467 	call	_LCD_clearblock
      008648 5B 03            [ 2] 1468 	addw	sp, #3
                                   1469 ;	main.c: 52: delay_ms(500);
      00864A 4B F4            [ 1] 1470 	push	#0xf4
      00864C 4B 01            [ 1] 1471 	push	#0x01
      00864E 5F               [ 1] 1472 	clrw	x
      00864F 89               [ 2] 1473 	pushw	x
      008650 CD 80 F5         [ 4] 1474 	call	_delay_ms
      008653 5B 04            [ 2] 1475 	addw	sp, #4
                                   1476 ;	main.c: 53: LCD_clearblock(5,3,86); //Finish column = 3 + 6*14 - 1
      008655 4B 56            [ 1] 1477 	push	#0x56
      008657 4B 03            [ 1] 1478 	push	#0x03
      008659 4B 05            [ 1] 1479 	push	#0x05
      00865B CD 85 E9         [ 4] 1480 	call	_LCD_clearblock
      00865E 5B 03            [ 2] 1481 	addw	sp, #3
                                   1482 ;	main.c: 54: delay_ms(500);
      008660 4B F4            [ 1] 1483 	push	#0xf4
      008662 4B 01            [ 1] 1484 	push	#0x01
      008664 5F               [ 1] 1485 	clrw	x
      008665 89               [ 2] 1486 	pushw	x
      008666 CD 80 F5         [ 4] 1487 	call	_delay_ms
      008669 5B 04            [ 2] 1488 	addw	sp, #4
                                   1489 ;	main.c: 56: drawInt();
      00866B CD 87 13         [ 4] 1490 	call	_drawInt
                                   1491 ;	main.c: 57: delay_ms(1000); 
      00866E 4B E8            [ 1] 1492 	push	#0xe8
      008670 4B 03            [ 1] 1493 	push	#0x03
      008672 5F               [ 1] 1494 	clrw	x
      008673 89               [ 2] 1495 	pushw	x
      008674 CD 80 F5         [ 4] 1496 	call	_delay_ms
      008677 5B 04            [ 2] 1497 	addw	sp, #4
                                   1498 ;	main.c: 58: LCD_clear();
      008679 CD 85 D9         [ 4] 1499 	call	_LCD_clear
                                   1500 ;	main.c: 60: drawAlphanum();
      00867C CD 87 78         [ 4] 1501 	call	_drawAlphanum
                                   1502 ;	main.c: 61: delay_ms(1000); 
      00867F 4B E8            [ 1] 1503 	push	#0xe8
      008681 4B 03            [ 1] 1504 	push	#0x03
      008683 5F               [ 1] 1505 	clrw	x
      008684 89               [ 2] 1506 	pushw	x
      008685 CD 80 F5         [ 4] 1507 	call	_delay_ms
      008688 5B 04            [ 2] 1508 	addw	sp, #4
                                   1509 ;	main.c: 62: LCD_reverse();
      00868A CD 86 0D         [ 4] 1510 	call	_LCD_reverse
                                   1511 ;	main.c: 63: delay_ms(1000);
      00868D 4B E8            [ 1] 1512 	push	#0xe8
      00868F 4B 03            [ 1] 1513 	push	#0x03
      008691 5F               [ 1] 1514 	clrw	x
      008692 89               [ 2] 1515 	pushw	x
      008693 CD 80 F5         [ 4] 1516 	call	_delay_ms
      008696 5B 04            [ 2] 1517 	addw	sp, #4
                                   1518 ;	main.c: 64: LCD_clear();
      008698 CD 85 D9         [ 4] 1519 	call	_LCD_clear
                                   1520 ;	main.c: 65: LCD_normal();
      00869B CD 86 06         [ 4] 1521 	call	_LCD_normal
                                   1522 ;	main.c: 66: drawPunct();
      00869E CD 87 D4         [ 4] 1523 	call	_drawPunct
                                   1524 ;	main.c: 67: delay_ms(1000); 
      0086A1 4B E8            [ 1] 1525 	push	#0xe8
      0086A3 4B 03            [ 1] 1526 	push	#0x03
      0086A5 5F               [ 1] 1527 	clrw	x
      0086A6 89               [ 2] 1528 	pushw	x
      0086A7 CD 80 F5         [ 4] 1529 	call	_delay_ms
      0086AA 5B 04            [ 2] 1530 	addw	sp, #4
                                   1531 ;	main.c: 68: LCD_reverse();
      0086AC CD 86 0D         [ 4] 1532 	call	_LCD_reverse
                                   1533 ;	main.c: 69: delay_ms(1000);
      0086AF 4B E8            [ 1] 1534 	push	#0xe8
      0086B1 4B 03            [ 1] 1535 	push	#0x03
      0086B3 5F               [ 1] 1536 	clrw	x
      0086B4 89               [ 2] 1537 	pushw	x
      0086B5 CD 80 F5         [ 4] 1538 	call	_delay_ms
      0086B8 5B 04            [ 2] 1539 	addw	sp, #4
                                   1540 ;	main.c: 70: LCD_clear();
      0086BA CD 85 D9         [ 4] 1541 	call	_LCD_clear
                                   1542 ;	main.c: 71: LCD_normal();
      0086BD CD 86 06         [ 4] 1543 	call	_LCD_normal
                                   1544 ;	main.c: 73: drawFrame();
      0086C0 CD 87 FC         [ 4] 1545 	call	_drawFrame
                                   1546 ;	main.c: 74: delay_ms(700); 
      0086C3 4B BC            [ 1] 1547 	push	#0xbc
      0086C5 4B 02            [ 1] 1548 	push	#0x02
      0086C7 5F               [ 1] 1549 	clrw	x
      0086C8 89               [ 2] 1550 	pushw	x
      0086C9 CD 80 F5         [ 4] 1551 	call	_delay_ms
      0086CC 5B 04            [ 2] 1552 	addw	sp, #4
                                   1553 ;	main.c: 75: LCD_clearblock(3,36,43); //Finish column = 36 + 8 - 1
      0086CE 4B 2B            [ 1] 1554 	push	#0x2b
      0086D0 4B 24            [ 1] 1555 	push	#0x24
      0086D2 4B 03            [ 1] 1556 	push	#0x03
      0086D4 CD 85 E9         [ 4] 1557 	call	_LCD_clearblock
      0086D7 5B 03            [ 2] 1558 	addw	sp, #3
                                   1559 ;	main.c: 76: delay_ms(700);
      0086D9 4B BC            [ 1] 1560 	push	#0xbc
      0086DB 4B 02            [ 1] 1561 	push	#0x02
      0086DD 5F               [ 1] 1562 	clrw	x
      0086DE 89               [ 2] 1563 	pushw	x
      0086DF CD 80 F5         [ 4] 1564 	call	_delay_ms
      0086E2 5B 04            [ 2] 1565 	addw	sp, #4
                                   1566 ;	main.c: 77: LCD_clear();
      0086E4 CD 85 D9         [ 4] 1567 	call	_LCD_clear
                                   1568 ;	main.c: 78: drawArrow();
      0086E7 CD 88 E4         [ 4] 1569 	call	_drawArrow
                                   1570 ;	main.c: 79: delay_ms(700); 
      0086EA 4B BC            [ 1] 1571 	push	#0xbc
      0086EC 4B 02            [ 1] 1572 	push	#0x02
      0086EE 5F               [ 1] 1573 	clrw	x
      0086EF 89               [ 2] 1574 	pushw	x
      0086F0 CD 80 F5         [ 4] 1575 	call	_delay_ms
      0086F3 5B 04            [ 2] 1576 	addw	sp, #4
                                   1577 ;	main.c: 80: LCD_clearblock(3,36,43); //Finish column = 36 + 8 - 1
      0086F5 4B 2B            [ 1] 1578 	push	#0x2b
      0086F7 4B 24            [ 1] 1579 	push	#0x24
      0086F9 4B 03            [ 1] 1580 	push	#0x03
      0086FB CD 85 E9         [ 4] 1581 	call	_LCD_clearblock
      0086FE 5B 03            [ 2] 1582 	addw	sp, #3
                                   1583 ;	main.c: 81: delay_ms(700);
      008700 4B BC            [ 1] 1584 	push	#0xbc
      008702 4B 02            [ 1] 1585 	push	#0x02
      008704 5F               [ 1] 1586 	clrw	x
      008705 89               [ 2] 1587 	pushw	x
      008706 CD 80 F5         [ 4] 1588 	call	_delay_ms
      008709 5B 04            [ 2] 1589 	addw	sp, #4
                                   1590 ;	main.c: 82: LCD_clear();
      00870B CD 85 D9         [ 4] 1591 	call	_LCD_clear
      00870E CC 86 31         [ 2] 1592 	jp	00102$
      008711 81               [ 4] 1593 	ret
                                   1594 ;	main.c: 87: void gpio_init()
                                   1595 ;	-----------------------------------------
                                   1596 ;	 function gpio_init
                                   1597 ;	-----------------------------------------
      008712                       1598 _gpio_init:
                                   1599 ;	main.c: 90: }
      008712 81               [ 4] 1600 	ret
                                   1601 ;	main.c: 92: void drawInt()
                                   1602 ;	-----------------------------------------
                                   1603 ;	 function drawInt
                                   1604 ;	-----------------------------------------
      008713                       1605 _drawInt:
                                   1606 ;	main.c: 94: LCD_drawint(64, 1, 8);
      008713 4B 08            [ 1] 1607 	push	#0x08
      008715 4B 01            [ 1] 1608 	push	#0x01
      008717 4B 40            [ 1] 1609 	push	#0x40
      008719 4B 00            [ 1] 1610 	push	#0x00
      00871B CD 85 7E         [ 4] 1611 	call	_LCD_drawint
      00871E 5B 04            [ 2] 1612 	addw	sp, #4
                                   1613 ;	main.c: 95: LCD_drawint(-64, 1, 48); //Negative number is not supported
      008720 4B 30            [ 1] 1614 	push	#0x30
      008722 4B 01            [ 1] 1615 	push	#0x01
      008724 4B C0            [ 1] 1616 	push	#0xc0
      008726 4B FF            [ 1] 1617 	push	#0xff
      008728 CD 85 7E         [ 4] 1618 	call	_LCD_drawint
      00872B 5B 04            [ 2] 1619 	addw	sp, #4
                                   1620 ;	main.c: 98: LCD_drawint(100, 3, 8);
      00872D 4B 08            [ 1] 1621 	push	#0x08
      00872F 4B 03            [ 1] 1622 	push	#0x03
      008731 4B 64            [ 1] 1623 	push	#0x64
      008733 4B 00            [ 1] 1624 	push	#0x00
      008735 CD 85 7E         [ 4] 1625 	call	_LCD_drawint
      008738 5B 04            [ 2] 1626 	addw	sp, #4
                                   1627 ;	main.c: 99: LCD_drawchar(SYM_DEGREE, 3, 32);
      00873A 4B 20            [ 1] 1628 	push	#0x20
      00873C 4B 03            [ 1] 1629 	push	#0x03
      00873E 4B 7F            [ 1] 1630 	push	#0x7f
      008740 CD 84 BF         [ 4] 1631 	call	_LCD_drawchar
      008743 5B 03            [ 2] 1632 	addw	sp, #3
                                   1633 ;	main.c: 100: LCD_drawchar('C', 3, 40);
      008745 4B 28            [ 1] 1634 	push	#0x28
      008747 4B 03            [ 1] 1635 	push	#0x03
      008749 4B 43            [ 1] 1636 	push	#0x43
      00874B CD 84 BF         [ 4] 1637 	call	_LCD_drawchar
      00874E 5B 03            [ 2] 1638 	addw	sp, #3
                                   1639 ;	main.c: 102: LCD_drawint(65535, 5, 8); //Max. is 65535
      008750 4B 08            [ 1] 1640 	push	#0x08
      008752 4B 05            [ 1] 1641 	push	#0x05
      008754 4B FF            [ 1] 1642 	push	#0xff
      008756 4B FF            [ 1] 1643 	push	#0xff
      008758 CD 85 7E         [ 4] 1644 	call	_LCD_drawint
      00875B 5B 04            [ 2] 1645 	addw	sp, #4
                                   1646 ;	main.c: 104: LCD_drawint(064, 3, 70); //Octal displayed as Decimal
      00875D 4B 46            [ 1] 1647 	push	#0x46
      00875F 4B 03            [ 1] 1648 	push	#0x03
      008761 4B 34            [ 1] 1649 	push	#0x34
      008763 4B 00            [ 1] 1650 	push	#0x00
      008765 CD 85 7E         [ 4] 1651 	call	_LCD_drawint
      008768 5B 04            [ 2] 1652 	addw	sp, #4
                                   1653 ;	main.c: 105: LCD_drawint(0x64, 5, 70); //Hexadecimal displayed as Decimal
      00876A 4B 46            [ 1] 1654 	push	#0x46
      00876C 4B 05            [ 1] 1655 	push	#0x05
      00876E 4B 64            [ 1] 1656 	push	#0x64
      008770 4B 00            [ 1] 1657 	push	#0x00
      008772 CD 85 7E         [ 4] 1658 	call	_LCD_drawint
      008775 5B 04            [ 2] 1659 	addw	sp, #4
      008777 81               [ 4] 1660 	ret
                                   1661 ;	main.c: 108: void drawAlphanum()
                                   1662 ;	-----------------------------------------
                                   1663 ;	 function drawAlphanum
                                   1664 ;	-----------------------------------------
      008778                       1665 _drawAlphanum:
                                   1666 ;	main.c: 110: LCD_drawtext("ABCDEFGHIJKL",0,0);
      008778 AE 8C 67         [ 2] 1667 	ldw	x, #___str_0+0
      00877B 4B 00            [ 1] 1668 	push	#0x00
      00877D 4B 00            [ 1] 1669 	push	#0x00
      00877F 89               [ 2] 1670 	pushw	x
      008780 CD 85 53         [ 4] 1671 	call	_LCD_drawtext
      008783 5B 04            [ 2] 1672 	addw	sp, #4
                                   1673 ;	main.c: 111: LCD_drawtext("MNOPQRSTUVWX",1,0);
      008785 AE 8C 74         [ 2] 1674 	ldw	x, #___str_1+0
      008788 4B 00            [ 1] 1675 	push	#0x00
      00878A 4B 01            [ 1] 1676 	push	#0x01
      00878C 89               [ 2] 1677 	pushw	x
      00878D CD 85 53         [ 4] 1678 	call	_LCD_drawtext
      008790 5B 04            [ 2] 1679 	addw	sp, #4
                                   1680 ;	main.c: 112: LCD_drawtext("YZ",2,0);	
      008792 AE 8C 81         [ 2] 1681 	ldw	x, #___str_2+0
      008795 4B 00            [ 1] 1682 	push	#0x00
      008797 4B 02            [ 1] 1683 	push	#0x02
      008799 89               [ 2] 1684 	pushw	x
      00879A CD 85 53         [ 4] 1685 	call	_LCD_drawtext
      00879D 5B 04            [ 2] 1686 	addw	sp, #4
                                   1687 ;	main.c: 113: LCD_drawtext("abcdefghijkl",3,0);
      00879F AE 8C 84         [ 2] 1688 	ldw	x, #___str_3+0
      0087A2 4B 00            [ 1] 1689 	push	#0x00
      0087A4 4B 03            [ 1] 1690 	push	#0x03
      0087A6 89               [ 2] 1691 	pushw	x
      0087A7 CD 85 53         [ 4] 1692 	call	_LCD_drawtext
      0087AA 5B 04            [ 2] 1693 	addw	sp, #4
                                   1694 ;	main.c: 114: LCD_drawtext("mnopqrstuvwxyz",4,0);
      0087AC AE 8C 91         [ 2] 1695 	ldw	x, #___str_4+0
      0087AF 4B 00            [ 1] 1696 	push	#0x00
      0087B1 4B 04            [ 1] 1697 	push	#0x04
      0087B3 89               [ 2] 1698 	pushw	x
      0087B4 CD 85 53         [ 4] 1699 	call	_LCD_drawtext
      0087B7 5B 04            [ 2] 1700 	addw	sp, #4
                                   1701 ;	main.c: 115: LCD_drawtext("yz",5,0);	
      0087B9 AE 8C A0         [ 2] 1702 	ldw	x, #___str_5+0
      0087BC 4B 00            [ 1] 1703 	push	#0x00
      0087BE 4B 05            [ 1] 1704 	push	#0x05
      0087C0 89               [ 2] 1705 	pushw	x
      0087C1 CD 85 53         [ 4] 1706 	call	_LCD_drawtext
      0087C4 5B 04            [ 2] 1707 	addw	sp, #4
                                   1708 ;	main.c: 116: LCD_drawtext("0123456789",6,0);
      0087C6 AE 8C A3         [ 2] 1709 	ldw	x, #___str_6+0
      0087C9 4B 00            [ 1] 1710 	push	#0x00
      0087CB 4B 06            [ 1] 1711 	push	#0x06
      0087CD 89               [ 2] 1712 	pushw	x
      0087CE CD 85 53         [ 4] 1713 	call	_LCD_drawtext
      0087D1 5B 04            [ 2] 1714 	addw	sp, #4
      0087D3 81               [ 4] 1715 	ret
                                   1716 ;	main.c: 119: void drawPunct()
                                   1717 ;	-----------------------------------------
                                   1718 ;	 function drawPunct
                                   1719 ;	-----------------------------------------
      0087D4                       1720 _drawPunct:
                                   1721 ;	main.c: 121: LCD_drawtext("<{([+_-=])}>",0,0);
      0087D4 AE 8C AE         [ 2] 1722 	ldw	x, #___str_7+0
      0087D7 4B 00            [ 1] 1723 	push	#0x00
      0087D9 4B 00            [ 1] 1724 	push	#0x00
      0087DB 89               [ 2] 1725 	pushw	x
      0087DC CD 85 53         [ 4] 1726 	call	_LCD_drawtext
      0087DF 5B 04            [ 2] 1727 	addw	sp, #4
                                   1728 ;	main.c: 122: LCD_drawtext("!@#$%^&*`|~?",2,0);
      0087E1 AE 8C BB         [ 2] 1729 	ldw	x, #___str_8+0
      0087E4 4B 00            [ 1] 1730 	push	#0x00
      0087E6 4B 02            [ 1] 1731 	push	#0x02
      0087E8 89               [ 2] 1732 	pushw	x
      0087E9 CD 85 53         [ 4] 1733 	call	_LCD_drawtext
      0087EC 5B 04            [ 2] 1734 	addw	sp, #4
                                   1735 ;	main.c: 123: LCD_drawtext(".\,\"\'\\/ :;",4,0);
      0087EE AE 8C C8         [ 2] 1736 	ldw	x, #___str_9+0
      0087F1 4B 00            [ 1] 1737 	push	#0x00
      0087F3 4B 04            [ 1] 1738 	push	#0x04
      0087F5 89               [ 2] 1739 	pushw	x
      0087F6 CD 85 53         [ 4] 1740 	call	_LCD_drawtext
      0087F9 5B 04            [ 2] 1741 	addw	sp, #4
      0087FB 81               [ 4] 1742 	ret
                                   1743 ;	main.c: 126: void drawFrame()
                                   1744 ;	-----------------------------------------
                                   1745 ;	 function drawFrame
                                   1746 ;	-----------------------------------------
      0087FC                       1747 _drawFrame:
                                   1748 ;	main.c: 130: LCD_drawchar(FRAME_TOP_LEFT,1,startcol);
      0087FC 4B 14            [ 1] 1749 	push	#0x14
      0087FE 4B 01            [ 1] 1750 	push	#0x01
      008800 4B 80            [ 1] 1751 	push	#0x80
      008802 CD 84 BF         [ 4] 1752 	call	_LCD_drawchar
      008805 5B 03            [ 2] 1753 	addw	sp, #3
                                   1754 ;	main.c: 131: LCD_drawchar(FRAME_LINE_HOR,1,startcol+8);
      008807 4B 1C            [ 1] 1755 	push	#0x1c
      008809 4B 01            [ 1] 1756 	push	#0x01
      00880B 4B 89            [ 1] 1757 	push	#0x89
      00880D CD 84 BF         [ 4] 1758 	call	_LCD_drawchar
      008810 5B 03            [ 2] 1759 	addw	sp, #3
                                   1760 ;	main.c: 132: LCD_drawchar(FRAME_TOP,1,startcol+16);
      008812 4B 24            [ 1] 1761 	push	#0x24
      008814 4B 01            [ 1] 1762 	push	#0x01
      008816 4B 81            [ 1] 1763 	push	#0x81
      008818 CD 84 BF         [ 4] 1764 	call	_LCD_drawchar
      00881B 5B 03            [ 2] 1765 	addw	sp, #3
                                   1766 ;	main.c: 133: LCD_drawchar(FRAME_LINE_HOR,1,startcol+24);
      00881D 4B 2C            [ 1] 1767 	push	#0x2c
      00881F 4B 01            [ 1] 1768 	push	#0x01
      008821 4B 89            [ 1] 1769 	push	#0x89
      008823 CD 84 BF         [ 4] 1770 	call	_LCD_drawchar
      008826 5B 03            [ 2] 1771 	addw	sp, #3
                                   1772 ;	main.c: 134: LCD_drawchar(FRAME_TOP_RIGHT,1,startcol+32);
      008828 4B 34            [ 1] 1773 	push	#0x34
      00882A 4B 01            [ 1] 1774 	push	#0x01
      00882C 4B 82            [ 1] 1775 	push	#0x82
      00882E CD 84 BF         [ 4] 1776 	call	_LCD_drawchar
      008831 5B 03            [ 2] 1777 	addw	sp, #3
                                   1778 ;	main.c: 136: LCD_drawchar(FRAME_LINE_VER,2,startcol);
      008833 4B 14            [ 1] 1779 	push	#0x14
      008835 4B 02            [ 1] 1780 	push	#0x02
      008837 4B 8A            [ 1] 1781 	push	#0x8a
      008839 CD 84 BF         [ 4] 1782 	call	_LCD_drawchar
      00883C 5B 03            [ 2] 1783 	addw	sp, #3
                                   1784 ;	main.c: 137: LCD_drawchar(FRAME_LINE_VER,2,startcol+16);
      00883E 4B 24            [ 1] 1785 	push	#0x24
      008840 4B 02            [ 1] 1786 	push	#0x02
      008842 4B 8A            [ 1] 1787 	push	#0x8a
      008844 CD 84 BF         [ 4] 1788 	call	_LCD_drawchar
      008847 5B 03            [ 2] 1789 	addw	sp, #3
                                   1790 ;	main.c: 138: LCD_drawchar(FRAME_LINE_VER,2,startcol+32);
      008849 4B 34            [ 1] 1791 	push	#0x34
      00884B 4B 02            [ 1] 1792 	push	#0x02
      00884D 4B 8A            [ 1] 1793 	push	#0x8a
      00884F CD 84 BF         [ 4] 1794 	call	_LCD_drawchar
      008852 5B 03            [ 2] 1795 	addw	sp, #3
                                   1796 ;	main.c: 140: LCD_drawchar(FRAME_MID_LEFT,3,startcol);
      008854 4B 14            [ 1] 1797 	push	#0x14
      008856 4B 03            [ 1] 1798 	push	#0x03
      008858 4B 83            [ 1] 1799 	push	#0x83
      00885A CD 84 BF         [ 4] 1800 	call	_LCD_drawchar
      00885D 5B 03            [ 2] 1801 	addw	sp, #3
                                   1802 ;	main.c: 141: LCD_drawchar(FRAME_LINE_HOR,3,startcol+8);
      00885F 4B 1C            [ 1] 1803 	push	#0x1c
      008861 4B 03            [ 1] 1804 	push	#0x03
      008863 4B 89            [ 1] 1805 	push	#0x89
      008865 CD 84 BF         [ 4] 1806 	call	_LCD_drawchar
      008868 5B 03            [ 2] 1807 	addw	sp, #3
                                   1808 ;	main.c: 142: LCD_drawchar(FRAME_CENTER,3,startcol+16);
      00886A 4B 24            [ 1] 1809 	push	#0x24
      00886C 4B 03            [ 1] 1810 	push	#0x03
      00886E 4B 84            [ 1] 1811 	push	#0x84
      008870 CD 84 BF         [ 4] 1812 	call	_LCD_drawchar
      008873 5B 03            [ 2] 1813 	addw	sp, #3
                                   1814 ;	main.c: 143: LCD_drawchar(FRAME_LINE_HOR,3,startcol+24);
      008875 4B 2C            [ 1] 1815 	push	#0x2c
      008877 4B 03            [ 1] 1816 	push	#0x03
      008879 4B 89            [ 1] 1817 	push	#0x89
      00887B CD 84 BF         [ 4] 1818 	call	_LCD_drawchar
      00887E 5B 03            [ 2] 1819 	addw	sp, #3
                                   1820 ;	main.c: 144: LCD_drawchar(FRAME_MID_RIGHT,3,startcol+32);
      008880 4B 34            [ 1] 1821 	push	#0x34
      008882 4B 03            [ 1] 1822 	push	#0x03
      008884 4B 85            [ 1] 1823 	push	#0x85
      008886 CD 84 BF         [ 4] 1824 	call	_LCD_drawchar
      008889 5B 03            [ 2] 1825 	addw	sp, #3
                                   1826 ;	main.c: 146: LCD_drawchar(FRAME_LINE_VER,4,startcol);
      00888B 4B 14            [ 1] 1827 	push	#0x14
      00888D 4B 04            [ 1] 1828 	push	#0x04
      00888F 4B 8A            [ 1] 1829 	push	#0x8a
      008891 CD 84 BF         [ 4] 1830 	call	_LCD_drawchar
      008894 5B 03            [ 2] 1831 	addw	sp, #3
                                   1832 ;	main.c: 147: LCD_drawchar(FRAME_LINE_VER,4,startcol+16);
      008896 4B 24            [ 1] 1833 	push	#0x24
      008898 4B 04            [ 1] 1834 	push	#0x04
      00889A 4B 8A            [ 1] 1835 	push	#0x8a
      00889C CD 84 BF         [ 4] 1836 	call	_LCD_drawchar
      00889F 5B 03            [ 2] 1837 	addw	sp, #3
                                   1838 ;	main.c: 148: LCD_drawchar(FRAME_LINE_VER,4,startcol+32);
      0088A1 4B 34            [ 1] 1839 	push	#0x34
      0088A3 4B 04            [ 1] 1840 	push	#0x04
      0088A5 4B 8A            [ 1] 1841 	push	#0x8a
      0088A7 CD 84 BF         [ 4] 1842 	call	_LCD_drawchar
      0088AA 5B 03            [ 2] 1843 	addw	sp, #3
                                   1844 ;	main.c: 150: LCD_drawchar(FRAME_BOT_LEFT,5,startcol);
      0088AC 4B 14            [ 1] 1845 	push	#0x14
      0088AE 4B 05            [ 1] 1846 	push	#0x05
      0088B0 4B 86            [ 1] 1847 	push	#0x86
      0088B2 CD 84 BF         [ 4] 1848 	call	_LCD_drawchar
      0088B5 5B 03            [ 2] 1849 	addw	sp, #3
                                   1850 ;	main.c: 151: LCD_drawchar(FRAME_LINE_HOR,5,startcol+8);
      0088B7 4B 1C            [ 1] 1851 	push	#0x1c
      0088B9 4B 05            [ 1] 1852 	push	#0x05
      0088BB 4B 89            [ 1] 1853 	push	#0x89
      0088BD CD 84 BF         [ 4] 1854 	call	_LCD_drawchar
      0088C0 5B 03            [ 2] 1855 	addw	sp, #3
                                   1856 ;	main.c: 152: LCD_drawchar(FRAME_BOT,5,startcol+16);
      0088C2 4B 24            [ 1] 1857 	push	#0x24
      0088C4 4B 05            [ 1] 1858 	push	#0x05
      0088C6 4B 87            [ 1] 1859 	push	#0x87
      0088C8 CD 84 BF         [ 4] 1860 	call	_LCD_drawchar
      0088CB 5B 03            [ 2] 1861 	addw	sp, #3
                                   1862 ;	main.c: 153: LCD_drawchar(FRAME_LINE_HOR,5,startcol+24);
      0088CD 4B 2C            [ 1] 1863 	push	#0x2c
      0088CF 4B 05            [ 1] 1864 	push	#0x05
      0088D1 4B 89            [ 1] 1865 	push	#0x89
      0088D3 CD 84 BF         [ 4] 1866 	call	_LCD_drawchar
      0088D6 5B 03            [ 2] 1867 	addw	sp, #3
                                   1868 ;	main.c: 154: LCD_drawchar(FRAME_BOT_RIGHT,5,startcol+32);
      0088D8 4B 34            [ 1] 1869 	push	#0x34
      0088DA 4B 05            [ 1] 1870 	push	#0x05
      0088DC 4B 88            [ 1] 1871 	push	#0x88
      0088DE CD 84 BF         [ 4] 1872 	call	_LCD_drawchar
      0088E1 5B 03            [ 2] 1873 	addw	sp, #3
      0088E3 81               [ 4] 1874 	ret
                                   1875 ;	main.c: 156: void drawArrow()
                                   1876 ;	-----------------------------------------
                                   1877 ;	 function drawArrow
                                   1878 ;	-----------------------------------------
      0088E4                       1879 _drawArrow:
                                   1880 ;	main.c: 160: LCD_drawchar(ARROW_UP_LEFT,1,startcol);
      0088E4 4B 14            [ 1] 1881 	push	#0x14
      0088E6 4B 01            [ 1] 1882 	push	#0x01
      0088E8 4B 8F            [ 1] 1883 	push	#0x8f
      0088EA CD 84 BF         [ 4] 1884 	call	_LCD_drawchar
      0088ED 5B 03            [ 2] 1885 	addw	sp, #3
                                   1886 ;	main.c: 161: LCD_drawchar(ARROW_UP,1,startcol+16);
      0088EF 4B 24            [ 1] 1887 	push	#0x24
      0088F1 4B 01            [ 1] 1888 	push	#0x01
      0088F3 4B 8B            [ 1] 1889 	push	#0x8b
      0088F5 CD 84 BF         [ 4] 1890 	call	_LCD_drawchar
      0088F8 5B 03            [ 2] 1891 	addw	sp, #3
                                   1892 ;	main.c: 162: LCD_drawchar(ARROW_UP_RIGHT,1,startcol+32);
      0088FA 4B 34            [ 1] 1893 	push	#0x34
      0088FC 4B 01            [ 1] 1894 	push	#0x01
      0088FE 4B 90            [ 1] 1895 	push	#0x90
      008900 CD 84 BF         [ 4] 1896 	call	_LCD_drawchar
      008903 5B 03            [ 2] 1897 	addw	sp, #3
                                   1898 ;	main.c: 164: LCD_drawchar(ARROW_LEFT,3,startcol);
      008905 4B 14            [ 1] 1899 	push	#0x14
      008907 4B 03            [ 1] 1900 	push	#0x03
      008909 4B 8D            [ 1] 1901 	push	#0x8d
      00890B CD 84 BF         [ 4] 1902 	call	_LCD_drawchar
      00890E 5B 03            [ 2] 1903 	addw	sp, #3
                                   1904 ;	main.c: 165: LCD_drawchar(ARROW_POINT,3,startcol+16);
      008910 4B 24            [ 1] 1905 	push	#0x24
      008912 4B 03            [ 1] 1906 	push	#0x03
      008914 4B 93            [ 1] 1907 	push	#0x93
      008916 CD 84 BF         [ 4] 1908 	call	_LCD_drawchar
      008919 5B 03            [ 2] 1909 	addw	sp, #3
                                   1910 ;	main.c: 166: LCD_drawchar(ARROW_RIGHT,3,startcol+32);
      00891B 4B 34            [ 1] 1911 	push	#0x34
      00891D 4B 03            [ 1] 1912 	push	#0x03
      00891F 4B 8E            [ 1] 1913 	push	#0x8e
      008921 CD 84 BF         [ 4] 1914 	call	_LCD_drawchar
      008924 5B 03            [ 2] 1915 	addw	sp, #3
                                   1916 ;	main.c: 168: LCD_drawchar(ARROW_DOWN_LEFT,5,startcol);
      008926 4B 14            [ 1] 1917 	push	#0x14
      008928 4B 05            [ 1] 1918 	push	#0x05
      00892A 4B 91            [ 1] 1919 	push	#0x91
      00892C CD 84 BF         [ 4] 1920 	call	_LCD_drawchar
      00892F 5B 03            [ 2] 1921 	addw	sp, #3
                                   1922 ;	main.c: 169: LCD_drawchar(ARROW_DOWN,5,startcol+16);
      008931 4B 24            [ 1] 1923 	push	#0x24
      008933 4B 05            [ 1] 1924 	push	#0x05
      008935 4B 8C            [ 1] 1925 	push	#0x8c
      008937 CD 84 BF         [ 4] 1926 	call	_LCD_drawchar
      00893A 5B 03            [ 2] 1927 	addw	sp, #3
                                   1928 ;	main.c: 170: LCD_drawchar(ARROW_DOWN_RIGHT,5,startcol+32);
      00893C 4B 34            [ 1] 1929 	push	#0x34
      00893E 4B 05            [ 1] 1930 	push	#0x05
      008940 4B 92            [ 1] 1931 	push	#0x92
      008942 CD 84 BF         [ 4] 1932 	call	_LCD_drawchar
      008945 5B 03            [ 2] 1933 	addw	sp, #3
      008947 81               [ 4] 1934 	ret
                                   1935 ;	main.c: 173: void drawBytes()
                                   1936 ;	-----------------------------------------
                                   1937 ;	 function drawBytes
                                   1938 ;	-----------------------------------------
      008948                       1939 _drawBytes:
      008948 52 06            [ 2] 1940 	sub	sp, #6
                                   1941 ;	main.c: 177: LCD_setpos(3,5);
      00894A 4B 05            [ 1] 1942 	push	#0x05
      00894C 4B 03            [ 1] 1943 	push	#0x03
      00894E CD 84 AB         [ 4] 1944 	call	_LCD_setpos
      008951 5B 02            [ 2] 1945 	addw	sp, #2
                                   1946 ;	main.c: 178: for(Ts=0;Ts<8;Ts++) //Draw pattern 8 times
      008953 AE 00 02         [ 2] 1947 	ldw	x, #_dsine+0
      008956 1F 03            [ 2] 1948 	ldw	(0x03, sp), x
      008958 0F 01            [ 1] 1949 	clr	(0x01, sp)
                                   1950 ;	main.c: 180: for(ds=0;ds<10;ds++)
      00895A                       1951 00115$:
      00895A 0F 02            [ 1] 1952 	clr	(0x02, sp)
      00895C                       1953 00105$:
                                   1954 ;	main.c: 182: LCD_drawbyte(dsine[ds]);
      00895C 5F               [ 1] 1955 	clrw	x
      00895D 7B 02            [ 1] 1956 	ld	a, (0x02, sp)
      00895F 97               [ 1] 1957 	ld	xl, a
      008960 72 FB 03         [ 2] 1958 	addw	x, (0x03, sp)
      008963 F6               [ 1] 1959 	ld	a, (x)
      008964 88               [ 1] 1960 	push	a
      008965 CD 84 B7         [ 4] 1961 	call	_LCD_drawbyte
      008968 84               [ 1] 1962 	pop	a
                                   1963 ;	main.c: 180: for(ds=0;ds<10;ds++)
      008969 0C 02            [ 1] 1964 	inc	(0x02, sp)
      00896B 7B 02            [ 1] 1965 	ld	a, (0x02, sp)
      00896D A1 0A            [ 1] 1966 	cp	a, #0x0a
      00896F 25 EB            [ 1] 1967 	jrc	00105$
                                   1968 ;	main.c: 178: for(Ts=0;Ts<8;Ts++) //Draw pattern 8 times
      008971 0C 01            [ 1] 1969 	inc	(0x01, sp)
      008973 7B 01            [ 1] 1970 	ld	a, (0x01, sp)
      008975 A1 08            [ 1] 1971 	cp	a, #0x08
      008977 25 E1            [ 1] 1972 	jrc	00115$
                                   1973 ;	main.c: 186: LCD_setpos(5,3);
      008979 4B 03            [ 1] 1974 	push	#0x03
      00897B 4B 05            [ 1] 1975 	push	#0x05
      00897D CD 84 AB         [ 4] 1976 	call	_LCD_setpos
      008980 5B 02            [ 2] 1977 	addw	sp, #2
                                   1978 ;	main.c: 187: for(Ts=0;Ts<6;Ts++) //Draw pattern 6 times
      008982 AE 00 0C         [ 2] 1979 	ldw	x, #_dtri+0
      008985 1F 05            [ 2] 1980 	ldw	(0x05, sp), x
      008987 0F 01            [ 1] 1981 	clr	(0x01, sp)
                                   1982 ;	main.c: 189: for(ds=0;ds<14;ds++)
      008989                       1983 00119$:
      008989 0F 02            [ 1] 1984 	clr	(0x02, sp)
      00898B                       1985 00109$:
                                   1986 ;	main.c: 191: LCD_drawbyte(dtri[ds]);
      00898B 5F               [ 1] 1987 	clrw	x
      00898C 7B 02            [ 1] 1988 	ld	a, (0x02, sp)
      00898E 97               [ 1] 1989 	ld	xl, a
      00898F 72 FB 05         [ 2] 1990 	addw	x, (0x05, sp)
      008992 F6               [ 1] 1991 	ld	a, (x)
      008993 88               [ 1] 1992 	push	a
      008994 CD 84 B7         [ 4] 1993 	call	_LCD_drawbyte
      008997 84               [ 1] 1994 	pop	a
                                   1995 ;	main.c: 189: for(ds=0;ds<14;ds++)
      008998 0C 02            [ 1] 1996 	inc	(0x02, sp)
      00899A 7B 02            [ 1] 1997 	ld	a, (0x02, sp)
      00899C A1 0E            [ 1] 1998 	cp	a, #0x0e
      00899E 25 EB            [ 1] 1999 	jrc	00109$
                                   2000 ;	main.c: 187: for(Ts=0;Ts<6;Ts++) //Draw pattern 6 times
      0089A0 0C 01            [ 1] 2001 	inc	(0x01, sp)
      0089A2 7B 01            [ 1] 2002 	ld	a, (0x01, sp)
      0089A4 A1 06            [ 1] 2003 	cp	a, #0x06
      0089A6 25 E1            [ 1] 2004 	jrc	00119$
      0089A8 5B 06            [ 2] 2005 	addw	sp, #6
      0089AA 81               [ 4] 2006 	ret
                                   2007 ;	main.c: 196: void drawLoadingBar()
                                   2008 ;	-----------------------------------------
                                   2009 ;	 function drawLoadingBar
                                   2010 ;	-----------------------------------------
      0089AB                       2011 _drawLoadingBar:
                                   2012 ;	main.c: 200: LCD_setpos(4,5);
      0089AB 4B 05            [ 1] 2013 	push	#0x05
      0089AD 4B 04            [ 1] 2014 	push	#0x04
      0089AF CD 84 AB         [ 4] 2015 	call	_LCD_setpos
      0089B2 5B 02            [ 2] 2016 	addw	sp, #2
                                   2017 ;	main.c: 202: for(lb=5;lb<91;lb++)
      0089B4 A6 05            [ 1] 2018 	ld	a, #0x05
      0089B6                       2019 00102$:
                                   2020 ;	main.c: 204: LCD_drawbyte(0xFF);
      0089B6 88               [ 1] 2021 	push	a
      0089B7 4B FF            [ 1] 2022 	push	#0xff
      0089B9 CD 84 B7         [ 4] 2023 	call	_LCD_drawbyte
      0089BC 84               [ 1] 2024 	pop	a
      0089BD 84               [ 1] 2025 	pop	a
                                   2026 ;	main.c: 205: delay_ms(10);
      0089BE 88               [ 1] 2027 	push	a
      0089BF 4B 0A            [ 1] 2028 	push	#0x0a
      0089C1 5F               [ 1] 2029 	clrw	x
      0089C2 89               [ 2] 2030 	pushw	x
      0089C3 4B 00            [ 1] 2031 	push	#0x00
      0089C5 CD 80 F5         [ 4] 2032 	call	_delay_ms
      0089C8 5B 04            [ 2] 2033 	addw	sp, #4
      0089CA 84               [ 1] 2034 	pop	a
                                   2035 ;	main.c: 202: for(lb=5;lb<91;lb++)
      0089CB 4C               [ 1] 2036 	inc	a
      0089CC A1 5B            [ 1] 2037 	cp	a, #0x5b
      0089CE 25 E6            [ 1] 2038 	jrc	00102$
                                   2039 ;	main.c: 207: delay_ms(1000);
      0089D0 4B E8            [ 1] 2040 	push	#0xe8
      0089D2 4B 03            [ 1] 2041 	push	#0x03
      0089D4 5F               [ 1] 2042 	clrw	x
      0089D5 89               [ 2] 2043 	pushw	x
      0089D6 CD 80 F5         [ 4] 2044 	call	_delay_ms
      0089D9 5B 04            [ 2] 2045 	addw	sp, #4
                                   2046 ;	main.c: 208: LCD_clearblock(4,5,90); //Start & finish column = start & finish lb
      0089DB 4B 5A            [ 1] 2047 	push	#0x5a
      0089DD 4B 05            [ 1] 2048 	push	#0x05
      0089DF 4B 04            [ 1] 2049 	push	#0x04
      0089E1 CD 85 E9         [ 4] 2050 	call	_LCD_clearblock
      0089E4 5B 03            [ 2] 2051 	addw	sp, #3
      0089E6 81               [ 4] 2052 	ret
                                   2053 	.area CODE
      0089E7                       2054 _font_arr:
      0089E7 00                    2055 	.db #0x00	; 0
      0089E8 00                    2056 	.db #0x00	; 0
      0089E9 00                    2057 	.db #0x00	; 0
      0089EA 00                    2058 	.db #0x00	; 0
      0089EB 00                    2059 	.db #0x00	; 0
      0089EC 00                    2060 	.db #0x00	; 0
      0089ED 00                    2061 	.db #0x00	; 0
      0089EE 5F                    2062 	.db #0x5F	; 95
      0089EF 00                    2063 	.db #0x00	; 0
      0089F0 00                    2064 	.db #0x00	; 0
      0089F1 05                    2065 	.db #0x05	; 5
      0089F2 03                    2066 	.db #0x03	; 3
      0089F3 00                    2067 	.db #0x00	; 0
      0089F4 05                    2068 	.db #0x05	; 5
      0089F5 03                    2069 	.db #0x03	; 3
      0089F6 14                    2070 	.db #0x14	; 20
      0089F7 7F                    2071 	.db #0x7F	; 127
      0089F8 14                    2072 	.db #0x14	; 20
      0089F9 7F                    2073 	.db #0x7F	; 127
      0089FA 14                    2074 	.db #0x14	; 20
      0089FB 24                    2075 	.db #0x24	; 36
      0089FC 2A                    2076 	.db #0x2A	; 42
      0089FD 7F                    2077 	.db #0x7F	; 127
      0089FE 2A                    2078 	.db #0x2A	; 42
      0089FF 12                    2079 	.db #0x12	; 18
      008A00 23                    2080 	.db #0x23	; 35
      008A01 13                    2081 	.db #0x13	; 19
      008A02 08                    2082 	.db #0x08	; 8
      008A03 64                    2083 	.db #0x64	; 100	'd'
      008A04 62                    2084 	.db #0x62	; 98	'b'
      008A05 36                    2085 	.db #0x36	; 54	'6'
      008A06 49                    2086 	.db #0x49	; 73	'I'
      008A07 55                    2087 	.db #0x55	; 85	'U'
      008A08 22                    2088 	.db #0x22	; 34
      008A09 50                    2089 	.db #0x50	; 80	'P'
      008A0A 00                    2090 	.db #0x00	; 0
      008A0B 05                    2091 	.db #0x05	; 5
      008A0C 03                    2092 	.db #0x03	; 3
      008A0D 00                    2093 	.db #0x00	; 0
      008A0E 00                    2094 	.db #0x00	; 0
      008A0F 00                    2095 	.db #0x00	; 0
      008A10 1C                    2096 	.db #0x1C	; 28
      008A11 22                    2097 	.db #0x22	; 34
      008A12 41                    2098 	.db #0x41	; 65	'A'
      008A13 00                    2099 	.db #0x00	; 0
      008A14 00                    2100 	.db #0x00	; 0
      008A15 41                    2101 	.db #0x41	; 65	'A'
      008A16 22                    2102 	.db #0x22	; 34
      008A17 1C                    2103 	.db #0x1C	; 28
      008A18 00                    2104 	.db #0x00	; 0
      008A19 0A                    2105 	.db #0x0A	; 10
      008A1A 04                    2106 	.db #0x04	; 4
      008A1B 1F                    2107 	.db #0x1F	; 31
      008A1C 04                    2108 	.db #0x04	; 4
      008A1D 0A                    2109 	.db #0x0A	; 10
      008A1E 08                    2110 	.db #0x08	; 8
      008A1F 08                    2111 	.db #0x08	; 8
      008A20 3E                    2112 	.db #0x3E	; 62
      008A21 08                    2113 	.db #0x08	; 8
      008A22 08                    2114 	.db #0x08	; 8
      008A23 00                    2115 	.db #0x00	; 0
      008A24 50                    2116 	.db #0x50	; 80	'P'
      008A25 30                    2117 	.db #0x30	; 48	'0'
      008A26 00                    2118 	.db #0x00	; 0
      008A27 00                    2119 	.db #0x00	; 0
      008A28 08                    2120 	.db #0x08	; 8
      008A29 08                    2121 	.db #0x08	; 8
      008A2A 08                    2122 	.db #0x08	; 8
      008A2B 08                    2123 	.db #0x08	; 8
      008A2C 08                    2124 	.db #0x08	; 8
      008A2D 00                    2125 	.db #0x00	; 0
      008A2E 60                    2126 	.db #0x60	; 96
      008A2F 60                    2127 	.db #0x60	; 96
      008A30 00                    2128 	.db #0x00	; 0
      008A31 00                    2129 	.db #0x00	; 0
      008A32 20                    2130 	.db #0x20	; 32
      008A33 10                    2131 	.db #0x10	; 16
      008A34 08                    2132 	.db #0x08	; 8
      008A35 04                    2133 	.db #0x04	; 4
      008A36 02                    2134 	.db #0x02	; 2
      008A37 3E                    2135 	.db #0x3E	; 62
      008A38 51                    2136 	.db #0x51	; 81	'Q'
      008A39 49                    2137 	.db #0x49	; 73	'I'
      008A3A 45                    2138 	.db #0x45	; 69	'E'
      008A3B 3E                    2139 	.db #0x3E	; 62
      008A3C 00                    2140 	.db #0x00	; 0
      008A3D 42                    2141 	.db #0x42	; 66	'B'
      008A3E 7F                    2142 	.db #0x7F	; 127
      008A3F 40                    2143 	.db #0x40	; 64
      008A40 00                    2144 	.db #0x00	; 0
      008A41 42                    2145 	.db #0x42	; 66	'B'
      008A42 61                    2146 	.db #0x61	; 97	'a'
      008A43 51                    2147 	.db #0x51	; 81	'Q'
      008A44 49                    2148 	.db #0x49	; 73	'I'
      008A45 46                    2149 	.db #0x46	; 70	'F'
      008A46 22                    2150 	.db #0x22	; 34
      008A47 41                    2151 	.db #0x41	; 65	'A'
      008A48 49                    2152 	.db #0x49	; 73	'I'
      008A49 49                    2153 	.db #0x49	; 73	'I'
      008A4A 36                    2154 	.db #0x36	; 54	'6'
      008A4B 18                    2155 	.db #0x18	; 24
      008A4C 14                    2156 	.db #0x14	; 20
      008A4D 12                    2157 	.db #0x12	; 18
      008A4E 7F                    2158 	.db #0x7F	; 127
      008A4F 10                    2159 	.db #0x10	; 16
      008A50 27                    2160 	.db #0x27	; 39
      008A51 45                    2161 	.db #0x45	; 69	'E'
      008A52 45                    2162 	.db #0x45	; 69	'E'
      008A53 45                    2163 	.db #0x45	; 69	'E'
      008A54 39                    2164 	.db #0x39	; 57	'9'
      008A55 3E                    2165 	.db #0x3E	; 62
      008A56 49                    2166 	.db #0x49	; 73	'I'
      008A57 49                    2167 	.db #0x49	; 73	'I'
      008A58 49                    2168 	.db #0x49	; 73	'I'
      008A59 32                    2169 	.db #0x32	; 50	'2'
      008A5A 61                    2170 	.db #0x61	; 97	'a'
      008A5B 11                    2171 	.db #0x11	; 17
      008A5C 09                    2172 	.db #0x09	; 9
      008A5D 05                    2173 	.db #0x05	; 5
      008A5E 03                    2174 	.db #0x03	; 3
      008A5F 36                    2175 	.db #0x36	; 54	'6'
      008A60 49                    2176 	.db #0x49	; 73	'I'
      008A61 49                    2177 	.db #0x49	; 73	'I'
      008A62 49                    2178 	.db #0x49	; 73	'I'
      008A63 36                    2179 	.db #0x36	; 54	'6'
      008A64 26                    2180 	.db #0x26	; 38
      008A65 49                    2181 	.db #0x49	; 73	'I'
      008A66 49                    2182 	.db #0x49	; 73	'I'
      008A67 49                    2183 	.db #0x49	; 73	'I'
      008A68 3E                    2184 	.db #0x3E	; 62
      008A69 00                    2185 	.db #0x00	; 0
      008A6A 36                    2186 	.db #0x36	; 54	'6'
      008A6B 36                    2187 	.db #0x36	; 54	'6'
      008A6C 00                    2188 	.db #0x00	; 0
      008A6D 00                    2189 	.db #0x00	; 0
      008A6E 00                    2190 	.db #0x00	; 0
      008A6F 56                    2191 	.db #0x56	; 86	'V'
      008A70 36                    2192 	.db #0x36	; 54	'6'
      008A71 00                    2193 	.db #0x00	; 0
      008A72 00                    2194 	.db #0x00	; 0
      008A73 00                    2195 	.db #0x00	; 0
      008A74 08                    2196 	.db #0x08	; 8
      008A75 14                    2197 	.db #0x14	; 20
      008A76 22                    2198 	.db #0x22	; 34
      008A77 00                    2199 	.db #0x00	; 0
      008A78 14                    2200 	.db #0x14	; 20
      008A79 14                    2201 	.db #0x14	; 20
      008A7A 14                    2202 	.db #0x14	; 20
      008A7B 14                    2203 	.db #0x14	; 20
      008A7C 14                    2204 	.db #0x14	; 20
      008A7D 00                    2205 	.db #0x00	; 0
      008A7E 22                    2206 	.db #0x22	; 34
      008A7F 14                    2207 	.db #0x14	; 20
      008A80 08                    2208 	.db #0x08	; 8
      008A81 00                    2209 	.db #0x00	; 0
      008A82 02                    2210 	.db #0x02	; 2
      008A83 01                    2211 	.db #0x01	; 1
      008A84 51                    2212 	.db #0x51	; 81	'Q'
      008A85 09                    2213 	.db #0x09	; 9
      008A86 06                    2214 	.db #0x06	; 6
      008A87 32                    2215 	.db #0x32	; 50	'2'
      008A88 49                    2216 	.db #0x49	; 73	'I'
      008A89 79                    2217 	.db #0x79	; 121	'y'
      008A8A 41                    2218 	.db #0x41	; 65	'A'
      008A8B 3E                    2219 	.db #0x3E	; 62
      008A8C 7C                    2220 	.db #0x7C	; 124
      008A8D 12                    2221 	.db #0x12	; 18
      008A8E 11                    2222 	.db #0x11	; 17
      008A8F 12                    2223 	.db #0x12	; 18
      008A90 7C                    2224 	.db #0x7C	; 124
      008A91 7F                    2225 	.db #0x7F	; 127
      008A92 49                    2226 	.db #0x49	; 73	'I'
      008A93 49                    2227 	.db #0x49	; 73	'I'
      008A94 49                    2228 	.db #0x49	; 73	'I'
      008A95 36                    2229 	.db #0x36	; 54	'6'
      008A96 3E                    2230 	.db #0x3E	; 62
      008A97 41                    2231 	.db #0x41	; 65	'A'
      008A98 41                    2232 	.db #0x41	; 65	'A'
      008A99 41                    2233 	.db #0x41	; 65	'A'
      008A9A 22                    2234 	.db #0x22	; 34
      008A9B 7F                    2235 	.db #0x7F	; 127
      008A9C 41                    2236 	.db #0x41	; 65	'A'
      008A9D 41                    2237 	.db #0x41	; 65	'A'
      008A9E 22                    2238 	.db #0x22	; 34
      008A9F 1C                    2239 	.db #0x1C	; 28
      008AA0 7F                    2240 	.db #0x7F	; 127
      008AA1 49                    2241 	.db #0x49	; 73	'I'
      008AA2 49                    2242 	.db #0x49	; 73	'I'
      008AA3 49                    2243 	.db #0x49	; 73	'I'
      008AA4 49                    2244 	.db #0x49	; 73	'I'
      008AA5 7F                    2245 	.db #0x7F	; 127
      008AA6 09                    2246 	.db #0x09	; 9
      008AA7 09                    2247 	.db #0x09	; 9
      008AA8 09                    2248 	.db #0x09	; 9
      008AA9 09                    2249 	.db #0x09	; 9
      008AAA 3E                    2250 	.db #0x3E	; 62
      008AAB 41                    2251 	.db #0x41	; 65	'A'
      008AAC 49                    2252 	.db #0x49	; 73	'I'
      008AAD 49                    2253 	.db #0x49	; 73	'I'
      008AAE 3A                    2254 	.db #0x3A	; 58
      008AAF 7F                    2255 	.db #0x7F	; 127
      008AB0 08                    2256 	.db #0x08	; 8
      008AB1 08                    2257 	.db #0x08	; 8
      008AB2 08                    2258 	.db #0x08	; 8
      008AB3 7F                    2259 	.db #0x7F	; 127
      008AB4 00                    2260 	.db #0x00	; 0
      008AB5 41                    2261 	.db #0x41	; 65	'A'
      008AB6 7F                    2262 	.db #0x7F	; 127
      008AB7 41                    2263 	.db #0x41	; 65	'A'
      008AB8 00                    2264 	.db #0x00	; 0
      008AB9 20                    2265 	.db #0x20	; 32
      008ABA 40                    2266 	.db #0x40	; 64
      008ABB 41                    2267 	.db #0x41	; 65	'A'
      008ABC 3F                    2268 	.db #0x3F	; 63
      008ABD 01                    2269 	.db #0x01	; 1
      008ABE 7F                    2270 	.db #0x7F	; 127
      008ABF 08                    2271 	.db #0x08	; 8
      008AC0 14                    2272 	.db #0x14	; 20
      008AC1 22                    2273 	.db #0x22	; 34
      008AC2 41                    2274 	.db #0x41	; 65	'A'
      008AC3 7F                    2275 	.db #0x7F	; 127
      008AC4 40                    2276 	.db #0x40	; 64
      008AC5 40                    2277 	.db #0x40	; 64
      008AC6 40                    2278 	.db #0x40	; 64
      008AC7 40                    2279 	.db #0x40	; 64
      008AC8 7F                    2280 	.db #0x7F	; 127
      008AC9 02                    2281 	.db #0x02	; 2
      008ACA 0C                    2282 	.db #0x0C	; 12
      008ACB 02                    2283 	.db #0x02	; 2
      008ACC 7F                    2284 	.db #0x7F	; 127
      008ACD 7F                    2285 	.db #0x7F	; 127
      008ACE 04                    2286 	.db #0x04	; 4
      008ACF 08                    2287 	.db #0x08	; 8
      008AD0 10                    2288 	.db #0x10	; 16
      008AD1 7F                    2289 	.db #0x7F	; 127
      008AD2 3E                    2290 	.db #0x3E	; 62
      008AD3 41                    2291 	.db #0x41	; 65	'A'
      008AD4 41                    2292 	.db #0x41	; 65	'A'
      008AD5 41                    2293 	.db #0x41	; 65	'A'
      008AD6 3E                    2294 	.db #0x3E	; 62
      008AD7 7F                    2295 	.db #0x7F	; 127
      008AD8 09                    2296 	.db #0x09	; 9
      008AD9 09                    2297 	.db #0x09	; 9
      008ADA 09                    2298 	.db #0x09	; 9
      008ADB 06                    2299 	.db #0x06	; 6
      008ADC 3E                    2300 	.db #0x3E	; 62
      008ADD 41                    2301 	.db #0x41	; 65	'A'
      008ADE 51                    2302 	.db #0x51	; 81	'Q'
      008ADF 21                    2303 	.db #0x21	; 33
      008AE0 5E                    2304 	.db #0x5E	; 94
      008AE1 7F                    2305 	.db #0x7F	; 127
      008AE2 09                    2306 	.db #0x09	; 9
      008AE3 19                    2307 	.db #0x19	; 25
      008AE4 29                    2308 	.db #0x29	; 41
      008AE5 46                    2309 	.db #0x46	; 70	'F'
      008AE6 26                    2310 	.db #0x26	; 38
      008AE7 49                    2311 	.db #0x49	; 73	'I'
      008AE8 49                    2312 	.db #0x49	; 73	'I'
      008AE9 49                    2313 	.db #0x49	; 73	'I'
      008AEA 32                    2314 	.db #0x32	; 50	'2'
      008AEB 01                    2315 	.db #0x01	; 1
      008AEC 01                    2316 	.db #0x01	; 1
      008AED 7F                    2317 	.db #0x7F	; 127
      008AEE 01                    2318 	.db #0x01	; 1
      008AEF 01                    2319 	.db #0x01	; 1
      008AF0 3F                    2320 	.db #0x3F	; 63
      008AF1 40                    2321 	.db #0x40	; 64
      008AF2 40                    2322 	.db #0x40	; 64
      008AF3 40                    2323 	.db #0x40	; 64
      008AF4 3F                    2324 	.db #0x3F	; 63
      008AF5 1F                    2325 	.db #0x1F	; 31
      008AF6 20                    2326 	.db #0x20	; 32
      008AF7 40                    2327 	.db #0x40	; 64
      008AF8 20                    2328 	.db #0x20	; 32
      008AF9 1F                    2329 	.db #0x1F	; 31
      008AFA 3F                    2330 	.db #0x3F	; 63
      008AFB 40                    2331 	.db #0x40	; 64
      008AFC 38                    2332 	.db #0x38	; 56	'8'
      008AFD 40                    2333 	.db #0x40	; 64
      008AFE 3F                    2334 	.db #0x3F	; 63
      008AFF 63                    2335 	.db #0x63	; 99	'c'
      008B00 14                    2336 	.db #0x14	; 20
      008B01 08                    2337 	.db #0x08	; 8
      008B02 14                    2338 	.db #0x14	; 20
      008B03 63                    2339 	.db #0x63	; 99	'c'
      008B04 07                    2340 	.db #0x07	; 7
      008B05 08                    2341 	.db #0x08	; 8
      008B06 70                    2342 	.db #0x70	; 112	'p'
      008B07 08                    2343 	.db #0x08	; 8
      008B08 07                    2344 	.db #0x07	; 7
      008B09 61                    2345 	.db #0x61	; 97	'a'
      008B0A 51                    2346 	.db #0x51	; 81	'Q'
      008B0B 49                    2347 	.db #0x49	; 73	'I'
      008B0C 45                    2348 	.db #0x45	; 69	'E'
      008B0D 43                    2349 	.db #0x43	; 67	'C'
      008B0E 00                    2350 	.db #0x00	; 0
      008B0F 7F                    2351 	.db #0x7F	; 127
      008B10 41                    2352 	.db #0x41	; 65	'A'
      008B11 41                    2353 	.db #0x41	; 65	'A'
      008B12 00                    2354 	.db #0x00	; 0
      008B13 02                    2355 	.db #0x02	; 2
      008B14 04                    2356 	.db #0x04	; 4
      008B15 08                    2357 	.db #0x08	; 8
      008B16 10                    2358 	.db #0x10	; 16
      008B17 20                    2359 	.db #0x20	; 32
      008B18 00                    2360 	.db #0x00	; 0
      008B19 41                    2361 	.db #0x41	; 65	'A'
      008B1A 41                    2362 	.db #0x41	; 65	'A'
      008B1B 7F                    2363 	.db #0x7F	; 127
      008B1C 00                    2364 	.db #0x00	; 0
      008B1D 04                    2365 	.db #0x04	; 4
      008B1E 02                    2366 	.db #0x02	; 2
      008B1F 01                    2367 	.db #0x01	; 1
      008B20 02                    2368 	.db #0x02	; 2
      008B21 04                    2369 	.db #0x04	; 4
      008B22 40                    2370 	.db #0x40	; 64
      008B23 40                    2371 	.db #0x40	; 64
      008B24 40                    2372 	.db #0x40	; 64
      008B25 40                    2373 	.db #0x40	; 64
      008B26 40                    2374 	.db #0x40	; 64
      008B27 00                    2375 	.db #0x00	; 0
      008B28 01                    2376 	.db #0x01	; 1
      008B29 02                    2377 	.db #0x02	; 2
      008B2A 04                    2378 	.db #0x04	; 4
      008B2B 00                    2379 	.db #0x00	; 0
      008B2C 20                    2380 	.db #0x20	; 32
      008B2D 54                    2381 	.db #0x54	; 84	'T'
      008B2E 54                    2382 	.db #0x54	; 84	'T'
      008B2F 54                    2383 	.db #0x54	; 84	'T'
      008B30 78                    2384 	.db #0x78	; 120	'x'
      008B31 7F                    2385 	.db #0x7F	; 127
      008B32 50                    2386 	.db #0x50	; 80	'P'
      008B33 48                    2387 	.db #0x48	; 72	'H'
      008B34 48                    2388 	.db #0x48	; 72	'H'
      008B35 30                    2389 	.db #0x30	; 48	'0'
      008B36 38                    2390 	.db #0x38	; 56	'8'
      008B37 44                    2391 	.db #0x44	; 68	'D'
      008B38 44                    2392 	.db #0x44	; 68	'D'
      008B39 44                    2393 	.db #0x44	; 68	'D'
      008B3A 28                    2394 	.db #0x28	; 40
      008B3B 30                    2395 	.db #0x30	; 48	'0'
      008B3C 48                    2396 	.db #0x48	; 72	'H'
      008B3D 48                    2397 	.db #0x48	; 72	'H'
      008B3E 50                    2398 	.db #0x50	; 80	'P'
      008B3F 7F                    2399 	.db #0x7F	; 127
      008B40 38                    2400 	.db #0x38	; 56	'8'
      008B41 54                    2401 	.db #0x54	; 84	'T'
      008B42 54                    2402 	.db #0x54	; 84	'T'
      008B43 54                    2403 	.db #0x54	; 84	'T'
      008B44 18                    2404 	.db #0x18	; 24
      008B45 08                    2405 	.db #0x08	; 8
      008B46 7E                    2406 	.db #0x7E	; 126
      008B47 09                    2407 	.db #0x09	; 9
      008B48 09                    2408 	.db #0x09	; 9
      008B49 02                    2409 	.db #0x02	; 2
      008B4A 08                    2410 	.db #0x08	; 8
      008B4B 54                    2411 	.db #0x54	; 84	'T'
      008B4C 54                    2412 	.db #0x54	; 84	'T'
      008B4D 54                    2413 	.db #0x54	; 84	'T'
      008B4E 3C                    2414 	.db #0x3C	; 60
      008B4F 7F                    2415 	.db #0x7F	; 127
      008B50 10                    2416 	.db #0x10	; 16
      008B51 08                    2417 	.db #0x08	; 8
      008B52 08                    2418 	.db #0x08	; 8
      008B53 70                    2419 	.db #0x70	; 112	'p'
      008B54 00                    2420 	.db #0x00	; 0
      008B55 48                    2421 	.db #0x48	; 72	'H'
      008B56 7A                    2422 	.db #0x7A	; 122	'z'
      008B57 40                    2423 	.db #0x40	; 64
      008B58 00                    2424 	.db #0x00	; 0
      008B59 20                    2425 	.db #0x20	; 32
      008B5A 40                    2426 	.db #0x40	; 64
      008B5B 48                    2427 	.db #0x48	; 72	'H'
      008B5C 3A                    2428 	.db #0x3A	; 58
      008B5D 00                    2429 	.db #0x00	; 0
      008B5E 7F                    2430 	.db #0x7F	; 127
      008B5F 10                    2431 	.db #0x10	; 16
      008B60 28                    2432 	.db #0x28	; 40
      008B61 44                    2433 	.db #0x44	; 68	'D'
      008B62 00                    2434 	.db #0x00	; 0
      008B63 00                    2435 	.db #0x00	; 0
      008B64 41                    2436 	.db #0x41	; 65	'A'
      008B65 7F                    2437 	.db #0x7F	; 127
      008B66 40                    2438 	.db #0x40	; 64
      008B67 00                    2439 	.db #0x00	; 0
      008B68 7C                    2440 	.db #0x7C	; 124
      008B69 04                    2441 	.db #0x04	; 4
      008B6A 7C                    2442 	.db #0x7C	; 124
      008B6B 04                    2443 	.db #0x04	; 4
      008B6C 78                    2444 	.db #0x78	; 120	'x'
      008B6D 7C                    2445 	.db #0x7C	; 124
      008B6E 08                    2446 	.db #0x08	; 8
      008B6F 04                    2447 	.db #0x04	; 4
      008B70 04                    2448 	.db #0x04	; 4
      008B71 78                    2449 	.db #0x78	; 120	'x'
      008B72 38                    2450 	.db #0x38	; 56	'8'
      008B73 44                    2451 	.db #0x44	; 68	'D'
      008B74 44                    2452 	.db #0x44	; 68	'D'
      008B75 44                    2453 	.db #0x44	; 68	'D'
      008B76 38                    2454 	.db #0x38	; 56	'8'
      008B77 7C                    2455 	.db #0x7C	; 124
      008B78 14                    2456 	.db #0x14	; 20
      008B79 14                    2457 	.db #0x14	; 20
      008B7A 14                    2458 	.db #0x14	; 20
      008B7B 08                    2459 	.db #0x08	; 8
      008B7C 08                    2460 	.db #0x08	; 8
      008B7D 14                    2461 	.db #0x14	; 20
      008B7E 14                    2462 	.db #0x14	; 20
      008B7F 18                    2463 	.db #0x18	; 24
      008B80 7C                    2464 	.db #0x7C	; 124
      008B81 7C                    2465 	.db #0x7C	; 124
      008B82 08                    2466 	.db #0x08	; 8
      008B83 04                    2467 	.db #0x04	; 4
      008B84 04                    2468 	.db #0x04	; 4
      008B85 08                    2469 	.db #0x08	; 8
      008B86 48                    2470 	.db #0x48	; 72	'H'
      008B87 54                    2471 	.db #0x54	; 84	'T'
      008B88 54                    2472 	.db #0x54	; 84	'T'
      008B89 54                    2473 	.db #0x54	; 84	'T'
      008B8A 20                    2474 	.db #0x20	; 32
      008B8B 04                    2475 	.db #0x04	; 4
      008B8C 3F                    2476 	.db #0x3F	; 63
      008B8D 44                    2477 	.db #0x44	; 68	'D'
      008B8E 44                    2478 	.db #0x44	; 68	'D'
      008B8F 20                    2479 	.db #0x20	; 32
      008B90 3C                    2480 	.db #0x3C	; 60
      008B91 40                    2481 	.db #0x40	; 64
      008B92 40                    2482 	.db #0x40	; 64
      008B93 20                    2483 	.db #0x20	; 32
      008B94 7C                    2484 	.db #0x7C	; 124
      008B95 1C                    2485 	.db #0x1C	; 28
      008B96 20                    2486 	.db #0x20	; 32
      008B97 40                    2487 	.db #0x40	; 64
      008B98 20                    2488 	.db #0x20	; 32
      008B99 1C                    2489 	.db #0x1C	; 28
      008B9A 3C                    2490 	.db #0x3C	; 60
      008B9B 40                    2491 	.db #0x40	; 64
      008B9C 38                    2492 	.db #0x38	; 56	'8'
      008B9D 40                    2493 	.db #0x40	; 64
      008B9E 3C                    2494 	.db #0x3C	; 60
      008B9F 44                    2495 	.db #0x44	; 68	'D'
      008BA0 28                    2496 	.db #0x28	; 40
      008BA1 10                    2497 	.db #0x10	; 16
      008BA2 28                    2498 	.db #0x28	; 40
      008BA3 44                    2499 	.db #0x44	; 68	'D'
      008BA4 0C                    2500 	.db #0x0C	; 12
      008BA5 50                    2501 	.db #0x50	; 80	'P'
      008BA6 50                    2502 	.db #0x50	; 80	'P'
      008BA7 50                    2503 	.db #0x50	; 80	'P'
      008BA8 3C                    2504 	.db #0x3C	; 60
      008BA9 44                    2505 	.db #0x44	; 68	'D'
      008BAA 64                    2506 	.db #0x64	; 100	'd'
      008BAB 54                    2507 	.db #0x54	; 84	'T'
      008BAC 4C                    2508 	.db #0x4C	; 76	'L'
      008BAD 44                    2509 	.db #0x44	; 68	'D'
      008BAE 00                    2510 	.db #0x00	; 0
      008BAF 08                    2511 	.db #0x08	; 8
      008BB0 36                    2512 	.db #0x36	; 54	'6'
      008BB1 41                    2513 	.db #0x41	; 65	'A'
      008BB2 00                    2514 	.db #0x00	; 0
      008BB3 00                    2515 	.db #0x00	; 0
      008BB4 00                    2516 	.db #0x00	; 0
      008BB5 7F                    2517 	.db #0x7F	; 127
      008BB6 00                    2518 	.db #0x00	; 0
      008BB7 00                    2519 	.db #0x00	; 0
      008BB8 00                    2520 	.db #0x00	; 0
      008BB9 41                    2521 	.db #0x41	; 65	'A'
      008BBA 36                    2522 	.db #0x36	; 54	'6'
      008BBB 08                    2523 	.db #0x08	; 8
      008BBC 00                    2524 	.db #0x00	; 0
      008BBD 10                    2525 	.db #0x10	; 16
      008BBE 08                    2526 	.db #0x08	; 8
      008BBF 08                    2527 	.db #0x08	; 8
      008BC0 10                    2528 	.db #0x10	; 16
      008BC1 08                    2529 	.db #0x08	; 8
      008BC2 06                    2530 	.db #0x06	; 6
      008BC3 09                    2531 	.db #0x09	; 9
      008BC4 09                    2532 	.db #0x09	; 9
      008BC5 06                    2533 	.db #0x06	; 6
      008BC6 00                    2534 	.db #0x00	; 0
      008BC7 00                    2535 	.db #0x00	; 0
      008BC8 00                    2536 	.db #0x00	; 0
      008BC9 00                    2537 	.db #0x00	; 0
      008BCA F8                    2538 	.db #0xF8	; 248
      008BCB F8                    2539 	.db #0xF8	; 248
      008BCC 18                    2540 	.db #0x18	; 24
      008BCD 18                    2541 	.db #0x18	; 24
      008BCE 18                    2542 	.db #0x18	; 24
      008BCF 18                    2543 	.db #0x18	; 24
      008BD0 18                    2544 	.db #0x18	; 24
      008BD1 18                    2545 	.db #0x18	; 24
      008BD2 F8                    2546 	.db #0xF8	; 248
      008BD3 F8                    2547 	.db #0xF8	; 248
      008BD4 18                    2548 	.db #0x18	; 24
      008BD5 18                    2549 	.db #0x18	; 24
      008BD6 18                    2550 	.db #0x18	; 24
      008BD7 18                    2551 	.db #0x18	; 24
      008BD8 18                    2552 	.db #0x18	; 24
      008BD9 18                    2553 	.db #0x18	; 24
      008BDA F8                    2554 	.db #0xF8	; 248
      008BDB F8                    2555 	.db #0xF8	; 248
      008BDC 00                    2556 	.db #0x00	; 0
      008BDD 00                    2557 	.db #0x00	; 0
      008BDE 00                    2558 	.db #0x00	; 0
      008BDF 00                    2559 	.db #0x00	; 0
      008BE0 00                    2560 	.db #0x00	; 0
      008BE1 00                    2561 	.db #0x00	; 0
      008BE2 FF                    2562 	.db #0xFF	; 255
      008BE3 FF                    2563 	.db #0xFF	; 255
      008BE4 18                    2564 	.db #0x18	; 24
      008BE5 18                    2565 	.db #0x18	; 24
      008BE6 18                    2566 	.db #0x18	; 24
      008BE7 18                    2567 	.db #0x18	; 24
      008BE8 18                    2568 	.db #0x18	; 24
      008BE9 18                    2569 	.db #0x18	; 24
      008BEA FF                    2570 	.db #0xFF	; 255
      008BEB FF                    2571 	.db #0xFF	; 255
      008BEC 18                    2572 	.db #0x18	; 24
      008BED 18                    2573 	.db #0x18	; 24
      008BEE 18                    2574 	.db #0x18	; 24
      008BEF 18                    2575 	.db #0x18	; 24
      008BF0 18                    2576 	.db #0x18	; 24
      008BF1 18                    2577 	.db #0x18	; 24
      008BF2 FF                    2578 	.db #0xFF	; 255
      008BF3 FF                    2579 	.db #0xFF	; 255
      008BF4 00                    2580 	.db #0x00	; 0
      008BF5 00                    2581 	.db #0x00	; 0
      008BF6 00                    2582 	.db #0x00	; 0
      008BF7 00                    2583 	.db #0x00	; 0
      008BF8 00                    2584 	.db #0x00	; 0
      008BF9 00                    2585 	.db #0x00	; 0
      008BFA 1F                    2586 	.db #0x1F	; 31
      008BFB 1F                    2587 	.db #0x1F	; 31
      008BFC 18                    2588 	.db #0x18	; 24
      008BFD 18                    2589 	.db #0x18	; 24
      008BFE 18                    2590 	.db #0x18	; 24
      008BFF 18                    2591 	.db #0x18	; 24
      008C00 18                    2592 	.db #0x18	; 24
      008C01 18                    2593 	.db #0x18	; 24
      008C02 1F                    2594 	.db #0x1F	; 31
      008C03 1F                    2595 	.db #0x1F	; 31
      008C04 18                    2596 	.db #0x18	; 24
      008C05 18                    2597 	.db #0x18	; 24
      008C06 18                    2598 	.db #0x18	; 24
      008C07 18                    2599 	.db #0x18	; 24
      008C08 18                    2600 	.db #0x18	; 24
      008C09 18                    2601 	.db #0x18	; 24
      008C0A 1F                    2602 	.db #0x1F	; 31
      008C0B 1F                    2603 	.db #0x1F	; 31
      008C0C 00                    2604 	.db #0x00	; 0
      008C0D 00                    2605 	.db #0x00	; 0
      008C0E 00                    2606 	.db #0x00	; 0
      008C0F 18                    2607 	.db #0x18	; 24
      008C10 18                    2608 	.db #0x18	; 24
      008C11 18                    2609 	.db #0x18	; 24
      008C12 18                    2610 	.db #0x18	; 24
      008C13 18                    2611 	.db #0x18	; 24
      008C14 18                    2612 	.db #0x18	; 24
      008C15 18                    2613 	.db #0x18	; 24
      008C16 18                    2614 	.db #0x18	; 24
      008C17 00                    2615 	.db #0x00	; 0
      008C18 00                    2616 	.db #0x00	; 0
      008C19 00                    2617 	.db #0x00	; 0
      008C1A FF                    2618 	.db #0xFF	; 255
      008C1B FF                    2619 	.db #0xFF	; 255
      008C1C 00                    2620 	.db #0x00	; 0
      008C1D 00                    2621 	.db #0x00	; 0
      008C1E 00                    2622 	.db #0x00	; 0
      008C1F 18                    2623 	.db #0x18	; 24
      008C20 0C                    2624 	.db #0x0C	; 12
      008C21 06                    2625 	.db #0x06	; 6
      008C22 FF                    2626 	.db #0xFF	; 255
      008C23 FF                    2627 	.db #0xFF	; 255
      008C24 06                    2628 	.db #0x06	; 6
      008C25 0C                    2629 	.db #0x0C	; 12
      008C26 18                    2630 	.db #0x18	; 24
      008C27 18                    2631 	.db #0x18	; 24
      008C28 30                    2632 	.db #0x30	; 48	'0'
      008C29 60                    2633 	.db #0x60	; 96
      008C2A FF                    2634 	.db #0xFF	; 255
      008C2B FF                    2635 	.db #0xFF	; 255
      008C2C 60                    2636 	.db #0x60	; 96
      008C2D 30                    2637 	.db #0x30	; 48	'0'
      008C2E 18                    2638 	.db #0x18	; 24
      008C2F 18                    2639 	.db #0x18	; 24
      008C30 3C                    2640 	.db #0x3C	; 60
      008C31 7E                    2641 	.db #0x7E	; 126
      008C32 DB                    2642 	.db #0xDB	; 219
      008C33 99                    2643 	.db #0x99	; 153
      008C34 18                    2644 	.db #0x18	; 24
      008C35 18                    2645 	.db #0x18	; 24
      008C36 18                    2646 	.db #0x18	; 24
      008C37 18                    2647 	.db #0x18	; 24
      008C38 18                    2648 	.db #0x18	; 24
      008C39 18                    2649 	.db #0x18	; 24
      008C3A 99                    2650 	.db #0x99	; 153
      008C3B DB                    2651 	.db #0xDB	; 219
      008C3C 7E                    2652 	.db #0x7E	; 126
      008C3D 3C                    2653 	.db #0x3C	; 60
      008C3E 18                    2654 	.db #0x18	; 24
      008C3F 7F                    2655 	.db #0x7F	; 127
      008C40 7F                    2656 	.db #0x7F	; 127
      008C41 0F                    2657 	.db #0x0F	; 15
      008C42 1F                    2658 	.db #0x1F	; 31
      008C43 3B                    2659 	.db #0x3B	; 59
      008C44 73                    2660 	.db #0x73	; 115	's'
      008C45 E3                    2661 	.db #0xE3	; 227
      008C46 40                    2662 	.db #0x40	; 64
      008C47 40                    2663 	.db #0x40	; 64
      008C48 E3                    2664 	.db #0xE3	; 227
      008C49 73                    2665 	.db #0x73	; 115	's'
      008C4A 3B                    2666 	.db #0x3B	; 59
      008C4B 1F                    2667 	.db #0x1F	; 31
      008C4C 0F                    2668 	.db #0x0F	; 15
      008C4D 7F                    2669 	.db #0x7F	; 127
      008C4E 7F                    2670 	.db #0x7F	; 127
      008C4F FE                    2671 	.db #0xFE	; 254
      008C50 FE                    2672 	.db #0xFE	; 254
      008C51 F0                    2673 	.db #0xF0	; 240
      008C52 F8                    2674 	.db #0xF8	; 248
      008C53 DC                    2675 	.db #0xDC	; 220
      008C54 CE                    2676 	.db #0xCE	; 206
      008C55 C7                    2677 	.db #0xC7	; 199
      008C56 02                    2678 	.db #0x02	; 2
      008C57 02                    2679 	.db #0x02	; 2
      008C58 C7                    2680 	.db #0xC7	; 199
      008C59 CE                    2681 	.db #0xCE	; 206
      008C5A DC                    2682 	.db #0xDC	; 220
      008C5B F8                    2683 	.db #0xF8	; 248
      008C5C F0                    2684 	.db #0xF0	; 240
      008C5D FE                    2685 	.db #0xFE	; 254
      008C5E FE                    2686 	.db #0xFE	; 254
      008C5F 3C                    2687 	.db #0x3C	; 60
      008C60 42                    2688 	.db #0x42	; 66	'B'
      008C61 81                    2689 	.db #0x81	; 129
      008C62 99                    2690 	.db #0x99	; 153
      008C63 99                    2691 	.db #0x99	; 153
      008C64 81                    2692 	.db #0x81	; 129
      008C65 42                    2693 	.db #0x42	; 66	'B'
      008C66 3C                    2694 	.db #0x3C	; 60
      008C67                       2695 ___str_0:
      008C67 41 42 43 44 45 46 47  2696 	.ascii "ABCDEFGHIJKL"
             48 49 4A 4B 4C
      008C73 00                    2697 	.db 0x00
      008C74                       2698 ___str_1:
      008C74 4D 4E 4F 50 51 52 53  2699 	.ascii "MNOPQRSTUVWX"
             54 55 56 57 58
      008C80 00                    2700 	.db 0x00
      008C81                       2701 ___str_2:
      008C81 59 5A                 2702 	.ascii "YZ"
      008C83 00                    2703 	.db 0x00
      008C84                       2704 ___str_3:
      008C84 61 62 63 64 65 66 67  2705 	.ascii "abcdefghijkl"
             68 69 6A 6B 6C
      008C90 00                    2706 	.db 0x00
      008C91                       2707 ___str_4:
      008C91 6D 6E 6F 70 71 72 73  2708 	.ascii "mnopqrstuvwxyz"
             74 75 76 77 78 79 7A
      008C9F 00                    2709 	.db 0x00
      008CA0                       2710 ___str_5:
      008CA0 79 7A                 2711 	.ascii "yz"
      008CA2 00                    2712 	.db 0x00
      008CA3                       2713 ___str_6:
      008CA3 30 31 32 33 34 35 36  2714 	.ascii "0123456789"
             37 38 39
      008CAD 00                    2715 	.db 0x00
      008CAE                       2716 ___str_7:
      008CAE 3C 7B 28 5B 2B 5F 2D  2717 	.ascii "<{([+_-=])}>"
             3D 5D 29 7D 3E
      008CBA 00                    2718 	.db 0x00
      008CBB                       2719 ___str_8:
      008CBB 21 40 23 24 25 5E 26  2720 	.ascii "!@#$%^&*`|~?"
             2A 60 7C 7E 3F
      008CC7 00                    2721 	.db 0x00
      008CC8                       2722 ___str_9:
      008CC8 2E 2C                 2723 	.ascii ".,"
      008CCA 22                    2724 	.db 0x22
      008CCB 27                    2725 	.ascii "'"
      008CCC 5C                    2726 	.db 0x5C
      008CCD 2F 20 3A 3B           2727 	.ascii "/ :;"
      008CD1 00                    2728 	.db 0x00
                                   2729 	.area INITIALIZER
      008E39                       2730 __xinit__dsine:
      008E39 18                    2731 	.db #0x18	; 24
      008E3A 06                    2732 	.db #0x06	; 6
      008E3B 01                    2733 	.db #0x01	; 1
      008E3C 01                    2734 	.db #0x01	; 1
      008E3D 06                    2735 	.db #0x06	; 6
      008E3E 18                    2736 	.db #0x18	; 24
      008E3F 60                    2737 	.db #0x60	; 96
      008E40 80                    2738 	.db #0x80	; 128
      008E41 80                    2739 	.db #0x80	; 128
      008E42 60                    2740 	.db #0x60	; 96
      008E43                       2741 __xinit__dtri:
      008E43 08                    2742 	.db #0x08	; 8
      008E44 04                    2743 	.db #0x04	; 4
      008E45 02                    2744 	.db #0x02	; 2
      008E46 01                    2745 	.db #0x01	; 1
      008E47 02                    2746 	.db #0x02	; 2
      008E48 04                    2747 	.db #0x04	; 4
      008E49 08                    2748 	.db #0x08	; 8
      008E4A 10                    2749 	.db #0x10	; 16
      008E4B 20                    2750 	.db #0x20	; 32
      008E4C 40                    2751 	.db #0x40	; 64
      008E4D 80                    2752 	.db #0x80	; 128
      008E4E 40                    2753 	.db #0x40	; 64
      008E4F 20                    2754 	.db #0x20	; 32
      008E50 10                    2755 	.db #0x10	; 16
                                   2756 	.area CABS (ABS)
