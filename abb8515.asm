;***************************************************************************
;* 
;* 
;* File Name            :"app8515.asm"
;* Title                :
;* Date                 :
;* Version              :2.0
;* Target MCU           :AT90S8515
;* AUTHOR		:
;* 			 
;* 			 
;* 			 
;*
;* DESCRIPTION:
;* 
;*
;*
;*
;* Copyright (c) 		 Jarl&Jarmo
;* All rights reserved
;*
;*  
;*
;***************************************************************************
  
  .include "8515def.inc"
  ;.include "m8515def.inc"
      
.equ	InitBaudRate		=11059200/16/9600-1	;Calculate  baud-rate for hardware-UART (9600, crystal 12MHz=12000000Hz)
;.equ	InitBaudRate		=11059200/16/115200-1	;Calculate  baud-rate for hardware-UART (115200, crystal 12MHz=12000000Hz)
.equ 	BUFSIZE			=120
.equ	Str6Size		=6
.equ	LongIntSize		=2
.equ	LongWordSize		=4

;******************
;* SUART Constants *
;******************

;**** Constant declarations Data Rate ****

;.equ	N = 95			; 115200 BAUD when R=1 and XTAL=11.059MHz

;.equ	N = 31			; 57600 BAUD when R=2 and XTAL=11.059MHz
;.equ	N = 43			; 38400 BAUD when R=2 and XTAL=11.059MHz
; .equ	N = 79			; 19200 BAUD when R=2 and XTAL=11.059MHz
 
;t .equ	N = 170  		; 9600	BAUD wheb R=2,N1=12,N2=18 and XTAL=11.059MHz
;.equ	N = 33			; 19200 BAUD when R=2 and XTAL=4.00MHz
;.equ	N = 102			; 38400 BAUD when R=1 and XTAL=4.00MHz
;t.equ	R = 2
;t.equ	N1=13 ;17;13
;t.equ	N2=45 ;76;45

;**** SUART transmit pin in PORTD ****

.equ	STXPIN = 3;1
.equ	SRXPIN = 2;0		; Receive pin must be external interrupt !!

;**** Bit positions in SUART Status Register ****

.equ	STXC = 0				; Transmit
.equ	SRXC = 1				; Receive

.equ	SCK  = 7
.equ	MISO = 6
.equ	MOSI = 5
.equ	SS   = 4
.equ	SS1  = 0;7
.equ	SS2  = 1;6


.def 	flag			=R25
.def    rxhead                  =R16
.def	rxtail			=R17		
.def	tempchar		=R18		
.def	temp1			=R19		;temporary register
.def	temp0			=R20		;temporary register
.def	u_data			=R21		; UART data r20
.def	bit_cnt			=R23		; Bit count used by SUART routine
.def	u_stat			=R24		; r24Status byte used by SUART routine

.dseg
 Buf:    	.byte BUFSIZE
 SUARTbuf:	.byte BUFSIZE
 TempBuf:	.byte BUFSIZE
 Str1:		.byte 1
 Str6:		.byte Str6Size
 LongInt:	.byte LongIntSize
 LongWord:	.byte LongWordSize
 Mode:		.byte 1
 BL1:		.byte 1				;bit-length 1
 BL15:		.byte 1				;bit-length 1.5
 DelayV:	.byte 3
 Phoneno:	.byte 15
 TempPhoneno:	.byte 15
 ;CrFlag:	.byte 1
 ;SmsFlag:	.byte 1
 ;pollFlag:	.byte 1
 Count:		.byte 1
 Scount:	.byte 1				;Source counter
 Mcount:	.byte 1				;Moutef counter

 TimeOutCount:	.byte 3
 lampStatus:	.byte 1
 ;cmgrFlag:	.byte 1
 SmsCount:	.byte 1
 SmsNo:		.byte 2
 ;Svolume:	.byte 1
 Svolume:	.byte 2
 RefTemp:	.byte 1
 Bflags:	.byte 1				;bit0=pollFlag
 						;bit1=SmsFlag
 						;bit2=
 						;bit3=StartFlag	
 						;bit4=CrFlag
 						;bit5=ButtonLockFlag
 						
 year:		.byte 1
 day:		.byte 2
 hour:		.byte 1
 minute:	.byte 1
 seconds:	.byte 1
 weekday:	.byte 1
 
 GTemperature:	.byte 2
 		

;------------------------------------------------------------------------------------------
;********************************************************************
;* Interrupt table
;********************************************************************
.cseg
;------------------------------------------------------------------------------------------
.org 0						;Reset
		rjmp reset	

.org INT0addr	
		rjmp	INT_0
		;ldi	temp0,0<<INT0		;Disable external interrupt INT0
		;out	GIMSK,temp0
		;ldi	u_data,65
		;rcall	putc
		;ldi	temp0,1<<INT0		;Enable external interrupt INT0
		;out	GIMSK,temp0
		;reti

.org OVF0addr					;Overflow0 Interrupt 
		rjmp	TIM0_OVF		;Timer0 Overflow Handle
		
.org OVF1addr	
		rjmp	TIM1_OVF	
;------------------------------------------------------------------------------------------
.org URXCaddr					;prijem zo seriovej linky
		push	r1
		in	r1,SREG				; preserve main OS status reg.
		push	r2
		push	zl
		push	zh
		clr	r2
		ldi	zl,low(Buf)			; ptr to our rxbuffer... z = &Buf[rxtail]
		ldi	zh,high(Buf)
		add	zl,rxtail	
		adc	zh,r2
		in	r2,UDR				; get the incomming char 
		;----------------------------
		lds	tempchar,Mode			; get modeno
		cpi	tempchar,'2'
		breq	URXCaddr0
		mov	temp0,r2
		cpi	temp0,13     ; 'cr' 
 		breq	w2
		;---------------------------
		;push	temp0
	        ;in	temp0,UDR			;nacitaj do temp0 prijate data z UART-u
	 	;sei					;povol interrupty na obsluhu USB
URXCaddr0:	;in	backupSREGTimer,SREG		;zaloha SREG
		cbi	UCR,RXCIE			;AT90s8515,zakazat interrupt od prijimania UART		
		;cbi	UCSRB,udre			;register name in Atmega 8515
		st	z,r2				; Buf[rxtail++] = UDR
		inc	rxtail
		cpi	rxtail,BUFSIZE
		brlo	Rs232inx
		clr	rxtail				; Circle around to the first buffer pos.
Rs232inx:
		
		
		
		;cpi	RS232BufferFull,MAXRS232LENGTH-4
		;brcc	NoIncRS232BufferFull
		;push	RS232BufptrX
		;lds	RS232BufptrX,RS232BufferBegin+2	;nastavenie sa na zaciatok buffera zapisu RS232 kodu : 3.byte hlavicky (dlzka kodu + citanie + zapis + rezerva)
		;st	X+,temp0			;a uloz ho do buffera
		;cpi	RS232BufptrX,RS232BufferBegin+MAXRS232LENGTH+1	;ak sa nedosiahol maximum RS232 buffera
		;brne	NoUARTBufferOverflow		;tak pokracuj
		;ldi	RS232BufptrX,RS232BufferBegin+4	;inak sa nastav na zaciatok buffera
 NoUARTBufferOverflow:
		;sts	RS232BufferBegin+2,RS232BufptrX	;ulozenie noveho offsetu buffera zapisu RS232 kodu : 3.byte hlavicky (dlzka kodu + citanie + zapis + rezerva)
		;inc	RS232BufferFull			;zvys dlzku RS232 Buffera
		;pop	RS232BufptrX
 NoIncRS232BufferFull:
		;pop	temp0
		;out	SREG,backupSREGTimer		;obnova SREG
		
		;;;;out	udr,r2 ;temp0	;send the recived valuable
w1:
		;mov     temp1,r2
		;sts	CrFlag,temp1
;***** Xoff calcs go here *****

		pop	zh
		pop	zl
		pop	r2
		out	SREG,r1				; restore previous status reg
		pop	r1
		cli					;
		sbi	UCR,RXCIE			;
		;sbi	UCSRB,RXCIE			;
;send:	
;	sbi	ucr,txen	;set sender bit
;	sbis	usr,udre	;wait till register is cleared
;	rjmp	send
	;out	udr,temp0	;send the recived valuable
;	cbi	ucr,txen	;clear sender bit
		reti
w2:
		;sbi	PORTD,6
		;;ldi	temp1,1
		;;sts	CrFlag,temp1
		rcall	SetCrFlag		;Set CrFlag
		;lds	temp1,Bflags
		;sbr	temp1,16
		;sts	Bflags,temp1
		rjmp	URXCaddr0;w1

;------------------------------------------------------------------------------------------
;***************************************************************************
;*
;* INTERRUPT
;*	TIM0_OVF - Software SUART Service Routine
;*
;***************************************************************************

TIM0_OVF:
	in	r0,SREG		; store SREG
	;ldi	temp1,(256-N+N1+8) ;(256-N+8)
	lds	temp1,BL1	; load T/C0
	out	TCNT0,temp1	; reset T/C0 to one bit lenght
	inc	bit_cnt		; increment bit counter
	sbrs	u_stat,STXC	; if (transmit complete flag clear)
	rjmp	transmit	;    goto transmit

to_0:	sec			; set carry
	sbis	PIND,SRXPIN	; if (RxD == LOW)
	clc			;	clear carry
	ror	u_data		; shift carry into u_data

	cpi	bit_cnt,8	; if (bit_cnt == 8)
	brne	to_1		; {
	clr	temp1		;    disable T/C0 Overflow Interrupt
	out	TIMSK,temp1
	sbr	u_stat,1<<SRXC	;    set receive complete
to_1:				; }
	out	SREG,r0	        ; restore SREG
	reti			; exit

transmit:
	cpi	bit_cnt,1	; if (bit_cnt == 1)	\\ start bit
	brne	to_2		; {
	cbi	PORTB,STXPIN	;	generate start bit
	rjmp	to_1		;	exit
to_2:				; }
	cpi	bit_cnt,10	; if (bit_cnt == 10)	\\ stop bit
	brne	to_3		; {
	sbi	PORTB,STXPIN	;	generate stop bit
	clr	temp1		;	disable TC0 overflow interrupt
	out	TIMSK,temp1
	sbr	u_stat,1<<STXC	;	set transmit complete bit
	rjmp	to_1		;	exit
to_3:				; }
	sbrc	u_data,0	; if (LSB set)
	sbi	PORTB,STXPIN	;	PD3 = HIGH
	sbrs	u_data,0	; if (LSB clear)
	cbi	PORTB,STXPIN	;	PD3 = LOW
	lsr	u_data		; shift left u_data
	rjmp	to_1		; exit
	
	
;------------------------------------------------------------------------------------------
;***************************************************************************
;*
;* INTERRUPT
;*	TIM1_OVF - *************** Routine
;*
;***************************************************************************

TIM1_OVF:
	;sbi	PORTD,6	
	push	tempchar
	lds	tempchar,Count
	inc	tempchar
	sts	Count,tempchar
	pop	tempchar
	;ldi	tempchar,65
	;rcall	PutcHardUart
	;ldi	u_data,65
	;rcall	putc
	;rcall	timer1_init
	;ldi	temp1,1<<TOV1	; enable T/C0 overflow interrupt
	;out	TIMSK,temp1	
	;cbi	PORTD,6	
	reti
;-----------------------------------------------------------------------------------------
INT_0:
	push	r1
	in	r1,SREG	
	sbi	PORTD,6                 ;Set RTS ON
	ldi	temp0,0<<INT0		;Disable external interrupt INT0
	out	GIMSK,temp0
	;ldi	u_data,65
	;rcall	putc
	out	SREG,r1			; restore previous status reg
	pop	r1
	ldi	temp0,1<<INT0		;Enable external interrupt INT0
	out	GIMSK,temp0
	ldi	flag,1
	reti
;----------------------------------------------------------------------------------------
;********************************************************************
;* Init program
;********************************************************************
;------------------------------------------------------------------------------------------
reset:		
	;----------------------Init Stack Pointer-----------------------------
		ldi temp0,$02			; Stack Pointer Setup
		out SPH,temp0			; Stack Pointer High Byte 
		ldi temp0,$5F			; Stack Pointer Setup 
		out SPL,temp0			; Stack Pointer Low Byte 
			
;		rcall	LongDelay
;		rcall	LongDelay
		;ldi 	zl,low(ireg)		;Load strinpointer
		;ldi 	zh,high(ireg)
		;rcall	EERead			;Get initreg ireg
		;mov	temp0,tempchar
		
		;sbrc	temp0,0
		;rcall	initPoll
	
;--------------------------------test-------------------------------------------------	
;		ldi	yl,low(phoneno)
;		ldi	yh,high(phoneno)
		;adiw	yl,1
		;sbiw	yl,1				
;		ldi	temp0,'+'
;		st	y+,temp0
;		ldi	temp0,'4'
;		st	y+,temp0
;		ldi	temp0,'6'
;		st	y+,temp0
;		ldi	temp0,'0'
;		st	y+,temp0
;		ldi	temp0,'7'
;		st	y+,temp0
	
;		ldi	yl,low(phoneno)
;		ldi	yh,high(phoneno)
		
;		ldi	zl,low(pslistE)
;		ldi	zh,high(pslistE)		
;		rcall	StrSearchEEP			;Check for matching phoneno in list

	
;---------------------------------------------------------------------------------------------	
	
	
		
		sbi	DDRA,0			;Config PORTA pin 0 as an output
	;-----------------Config pins for Buttons function-------------------------------	
		sbi	DDRA,5			;Config pin 5 as an output
		sbi	DDRA,6			;Config pin 6 as an output
		;sbi 	DDRA,7			;Config pin 7 as an input
		
		cbi	DDRC,0			;Config pin 0 as an input
		cbi	DDRC,1			;Config pin 1 as an input
		cbi	DDRC,2			;Config pin 2 as an input
		cbi	DDRC,3			;Config pin 3 as an input
		cbi	DDRC,4			;Config pin 4 as an input
		cbi	DDRC,5			;Config pin 5 as an input
		;sbi	DDRC,4			;Config pin 4 as an output
		;sbi	DDRC,5			;Config pin 5 as an output
		sbi 	DDRC,6			;Config pin 6 as an output
		sbi	DDRC,7			;Config pin 7 as an output
		
		;cbi	PORTC,4			;Set pin4 Off
		;cbi	PORTC,5			;Set pin5 Off
		cbi	PORTC,6			;Set pin6 Off
		cbi 	PORTC,7			;Set pin7 Off
		
		
		sbi	DDRD,6			;init rts (as an output)
		cbi	PORTD,6			;set rts off
		sbi	PORTD,0			;nahodit pull-up na RxD vstupe
		ldi	temp0,InitBaudRate	;Set baude-rate for hardware-UART

		sbi	DDRD,7			;Config pin 7 as an output
		cbi	PORTD,7			;Set pin 7 off

;		ldi 	zl,low(baudrate)	;Load strinpointer
;		ldi 	zh,high(baudrate)
;		rcall	EERead			;Get Baud-rate
;		mov	temp0,tempchar
		out	UBRR,temp0		;Set baude-rate for hardware-Uart
		sbi	UCR,TXEN		;AT90s8515,TX enable hardware-UART
		;sbi	UCSRB,TXEN		;ATmega8515,TX enable hardware-UART
		sbi	UCR,RXEN		;AT90s8515,RX enable hardware-UART
		;sbi	UCSRB,RXEN		;ATmega8515,RX enable hardware-UART
		sbi	UCR,RXCIE		;AT90s8515,RX complete interrupt enable for hardware-UART
		;sbi	UCSRB,RXCIE		;ATmega8515,RX complete interrupt enable for hardware-UART
		ldi	temp0,2;0x0F		;INT0 - sens control (falling edge)
		out	MCUCR,temp0		;
		ldi	temp0,1<<INT0		;Enable external interrupt INT0
		out	GIMSK,temp0
		;tldi	temp0,R
		;tsts	str6,temp0
		;trcall	u_init			;Initialize SUART (software uart)
		;rcall	WriteTempBuf
		
		
		ldi 	zl,low(ireg)		;Load strinpointer
		ldi 	zh,high(ireg)
		rcall	EERead			;Get initreg ireg
		cbr	tempchar,2		;Set SmsFlag=0
		cbr	tempchar,16		;Set CrFlag=0
		sts	Bflags,tempchar
		ldi	flag,0			;init flag
		clr	temp0
		;;sts	SmsFlag,temp0
		sts	Scount,temp0		;Clear source counter
		sts	Mcount,temp0		;Clear moute counter
;--------------Init hardware-UART---------------------------------------------
		clr	temp0
		mov	rxhead,temp0
		mov	rxtail,temp0
		
;-------------------Init softuart----------------------------------------------		
		ldi 	zl,low(baudrates)		;Load strinpointer
		ldi 	zh,high(baudrates)
		rcall	EERead				;Get Baud-rate
		mov	temp0,tempchar
		sts	str6,temp0
		rcall	u_init			;Initialize SUART (software uart)

		rcall 	spi_initM		;init spi

		rcall	GetTemp			;init temp
;----BitLength------------------------
		adiw	zl,1				;increase z
		rcall	EERead	
		;ldi	temp0,(256-N+N1+8) 		;(256-N+8)
		mov	temp0,tempchar
		sts	BL1,temp0			; reset T/C0 to one bit lenght
		;ldi	temp0,(256-(N+N/2)+8+12+N2)	;(256-(N+N/2)+8+12);
		adiw	zl,1				;increase z
		rcall	EERead	
		mov	temp0,tempchar
		sts	BL15,temp0 			; preset T/C0 to 1.5 bit lengths
;-----Buffer--------------------------		
		ldi	temp0,0
		sts	SUARTbuf,temp0		;init softuart buffer
		
;------------------Init modeno------------------------------------------------
		ldi	temp0,'2'
		;ldi	temp0,'0'  ;test
		sts	Mode,temp0
;-----------------Init volume-------------------------------------------------
		;ldi	temp0,20
		;sts	Svolume,temp0
;test********************
	
	;	rcall	timer1_init
;********************************************************************
;* Main program
;********************************************************************

		;sei					;povolit interrupty globalne
;------------------Init phone-------------------------------------------------
		;sbi	UCR,RXCIE		;RX complete interrupt enable for hardware-UART
		;BUTTON
		;----------init phone---------------------------------------------	
;rev		ldi 	zl,low(ireg)		;Load strinpointer
;rev		ldi 	zh,high(ireg)
;rev		rcall	EERead			;Get initreg ireg
		;;mov	temp0,tempchar
		;;sts	Bflags,tempchar		;Init box flags reg
;rev		sbrc	tempchar,1
;		rcall	PhoneInit00      ;rcall 	PhoneInit
	
		;rcall	LongDelay
		;rcall	LongDelay
		;rcall	LongDelay
		;rcall	LongDelay
			;----Set Delay Value---------------------
	;	ldi	temp0,0x20
	;	sts	DelayV,temp0
	;	ldi	temp0,0x50
	;	sts	DelayV+1,temp0
	;	ldi	temp0,0x01
	;	sts	DelayV+2,temp0
		rcall	ClearTempBuf
		rcall	ClearBuf
		rcall	ClearSuartBuf
		;clr	u_data
		clr	temp0
		mov	rxhead,temp0
		mov	rxtail,temp0
		sts	TempBuf,temp0
		;ldi	temp0,'0' ;0
		ldi	temp0,'1'
		sts	Mode,temp0
		;;ldi	temp0,'0'
		;;sts	CrFlag,temp0
		rcall	ClearCrFlag		;Clear CrFlag
		;--Clear all SMS----------------------------
		ldi 	zl,low(ireg)		;Load strinpointer
		ldi 	zh,high(ireg)
		rcall	EERead			;Get initreg ireg
		mov	temp0,tempchar
		sbrc	temp0,1
		rjmp	initphone00
		rjmp	initphone01
initphone00:
		;ssldi 	zl,low(2*atcmgd)			;Load strinpointer
		;ssldi 	zh,high(2*atcmgd)
		sei
		rcall	LongDelay
		;ssrcall	PrintStr
		rjmp	initphone02
		;Clear SoftUart-------------------------------------------------
initphone01:
		sei
initphone02:
		clr	u_Data
		rcall	putc
		;test********************
		clr	temp0			 
		sts	Count,temp0		;clear time counter		
		;;sts	pollFlag,temp0		;clear pollFlag	
		lds	temp0,Bflags
		cbr	temp0,1
		sts	Bflags,temp0
		clr	temp0
		sts	lampStatus,temp0	;clear lampStatus
		;rcall	timer1_init
		;init cmgr-par**********************************
	;	sts	cmgrFlag,temp0
		sts	SmsCount,temp0
		
		
	;----------init poll---------------------------------------------	
		ldi 	zl,low(ireg)		;Load strinpointer
		ldi 	zh,high(ireg)
		rcall	EERead			;Get initreg ireg
		mov	temp0,tempchar
		sbrc	temp0,0
		rcall	initPoll
		;clr	temp0
		;sts	Count,temp0		;clear time counter
		;ldi	temp0,1
		;sts	pollFlag,temp0		;set pollFlag	
		;rcall	timer1_init
		ldi 	temp0,0b00001111	;Set watchdog
		out 	wdtcr,temp0		;time out 1,0 s




	;---------set pump in hot mode-----------------------------------	
		ldi	tempchar,'b'
		cbi	portb,ss
		rcall	rw_spi
		sbi	portb,ss
	;---------Load RefTemp------------------------------------------	
		ldi 	zl,low(tempref)		;Load strinpointer
		ldi 	zh,high(tempref)
		rcall	EERead	
		sts	RefTemp,tempchar		
		
	;-----------End poll init----------------------------------------
	
Main:
	wdr						;watchdog reset
	ldi 	zl,low(2*ButtonIoA)				;Load strinpointer to commandlist 
	ldi 	zh,high(2*ButtonIoA)
	rcall	Ds2406_Status	
	sbrs	tempchar,2	
	;rcall	iotest
	rcall	PumpOnOff	
	
	ldi 	zl,low(2*ButtonIoA0)				;Load strinpointer to commandlist 
	ldi 	zh,high(2*ButtonIoA0)
	rcall	Ds2406_Status	
	sbrs	tempchar,2	
	;rcall	iotest	
	rcall	SetRefTemp
	
;	lds	temp1,Bflags
;	sbrc	temp1,4
;	rcall	send
;	rjmp	wsend
;	cpi	flag,1
;	breq	wint0	
;	sbrc	temp1,5
;	rjmp	main				;Jump if ButtonLockFlag is set 
	lds	temp1,Count			;Load poll count


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	cpi	temp1,3;50			
	brne	lPoll
	rcall	SharplPoll
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;	brge	lPoll		;breq	Lpoll
lPoll:	
	;--------Button poll------------
	in	temp1,pinc			;read portc
	andi	temp1,56			;get bit 0,1,2,3   1+2+4+8=15, If not 15 then Button 
;;;	andi	temp1,63;15			;get bit 0,1,2,3,4,5  1+2+4+8+16+32=63, If not 63 then Button 
;;;	ldi	temp0,63;15
	ldi	temp0,56
	cpse	temp1,temp0
	rcall	Button
	;-------------------------------
	
	rjmp	main



;lPoll: 
	;rcall	ViewSonicpoll
;	rcall	SharplPoll
	;rcall	Barcopoll
	;rcall	Panasonicpoll
;	rjmp	main	






;wsend:			;t
;	rcall	send	;t
;	rjmp	main	;t


wint0:
;	lds	temp1,BL15
;	out	TCNT0,temp1 			; preset T/C0 to 1.5 bit lengths
;	ldi	temp1,1<<TOIE0
;	out	TIFR,temp1			; clear T/C0 overflow flag
;	out	TIMSK,temp1			; enable T/C0 overflow Interrupt
;	clr	bit_cnt				; clear bit counter
;wgetc1:	sbrs	u_stat,SRXC			; wait for Receive Complete
;	rjmp	wgetc1
;	cbr	u_stat,1<<SRXC			; clear SRXC
;	lds	temp1,SUARTbuf			;load buf length counter
;	inc	temp1				;increase counter
;	sts	SUARTbuf,temp1			;store counter
;	ldi	zl,low(SUARTbuf)
;	ldi	zh,high(SUARTbuf)
;	adc	zl,temp1
;	brcc	wgetcj
;	inc	zh
;wgetcj:	st	z,u_data			;store char in buffer		
;	breq	wflag
;	cpi	u_data,13    			;'cr' 
;	breq	wgetc2
;wflag:
;	ldi	flag,0
;	cbi	PORTD,6			;RTS Off
;	rjmp	main
;wgetc2:
;	
;	ldi	zl,low(SUARTbuf)			;load stringpointer
;	ldi	zh,high(SUARTbuf)
;	inc	temp1
;	adc	zl,temp1
;	brcc	wgetcj0
;	inc	zh
;wgetcj0:
;	clr	temp0					;terminate buffer	
;	st	z,temp0
;	ldi	zl,low(SUARTbuf)
;	ldi	zh,high(SUARTbuf)
;	adiw	zl,1
;	clr 	temp0
;	sts	SUARTbuf,temp0				;clear counter
;	ldi	flag,0
;	cbi	PORTD,6					;RTS Off
	sbi	PORTD,6	
	rcall	ReadSoftUart
	cbi	PORTD,6
	;-------Test for poll status----------------------------------------------------
	;lds	tempchar,
	;clr	temp0
	;cpse	tempchar,temp0
	;rcall	lampTest
	;lds	tempchar,pollFlag
	;cpi	tempchar,1
	;breq	wgetcj1
	;-------Send SMS if smsFlag is on else print-out buffer to hardware uart--------
	ldi	yl,low(TempBuf)
	ldi	yh,high(TempBuf)
	rcall	MoveString
	ldi	zl,low(TempBuf)
	ldi	zh,high(TempBuf)
	;;lds	tempchar,SmsFlag		;Load SmsFlag
	;;ldi	temp0,1
	;;cpse	tempchar,temp0
	lds	tempchar,Bflags
	sbrs	tempchar,1
	rcall	PrintBuf			;print-out buffer to uart
	;;clr	temp0
	;;cpse	tempchar,temp0			;Send SMS if SmsFlag on
	sbrc	tempchar,1
	rcall	SendSms
wgetcj1:
	;---------------Init timer1 if pollFlag=1---------------------------------------------------
	;;lds	tempchar,pollFlag
	;;clr	temp0
	;;cpse	tempchar,temp0
	lds	tempchar,Bflags
	sbrc	tempchar,0
	rcall	timer1_init	
	;-------------------------------------------------------------------
	;;clr	tempchar
	;;sts	SmsFlag,tempchar		;Set SmsFlag off
	;lds	tempchar,Bflags
	;cbr 	tempchar,2
	;sts	Bflags,tempchar
	rcall	ClearSmsFlag			;Clear SmsFlag	
	;-----------------------------------------------------------------------------
	;sts	pollFlag,tempchar		;Clear pollFlag
	rjmp	main

wtest:
	rcall	test
	rjmp	main
wwritetempbuf:	
;	rcall	WriteTempBuf
	rjmp	main
	
;wsend:	
;	rcall	send
;	rjmp	main
	
	;	rcall	test
	;	rcall	GetStringData
	
	
	
	;	ldi 	zl,low(2*Ver)			;Load stringpointer source
	;	ldi 	zh,high(2*Ver)
	;	ldi	yl,low(TempBuf)			;Load stringpointer destination
	;	ldi	yh,high(TempBuf)
	;	rcall	StrSearch
		rjmp	Main
;********************************************************************
;* Main program END
;********************************************************************

;***************************************************************************
;*
;* FUNCTION
;*	SPI_initM
;*
;* DESCRIPTION
;*	Initialize SPI.
;*
;***************************************************************************

spi_initM:	
	cbi	ddrb,miso
	sbi	ddrb,mosi
	sbi	ddrb,sck
	sbi	ddrb,ss		
	;sbi	ddrd,ss1
	;sbi	ddrd,ss2
	sbi	ddrc,ss1
	sbi	ddrc,ss2
;	cbi	portb,miso
;	sbi	portb,mosi
;	sbi	portb,sck
;	sbi	portb,ss
	


;	in	temp0,portb
;	andi	temp0,0b11000011
;	ori	temp0,0b00101100
;	out	portb,temp0
	
	
;	in	temp0,ddrb
;	andi	temp0,0b11000011
;	ori	temp0,0b00101100
;	out	ddrb,temp0

;	cbi	ddrb,miso
;	sbi	ddrb,mosi
;	sbi	ddrb,sck
;	sbi	ddrb,ss	

;	sbi	spcr,spie			;Slave Spi Interrupt Enable
;	cbi	spcr,spie			;Master 	
;	sbi	spcr,spe
;	cbi	spcr,dord
;	sbi	spcr,mstr			;Master mode
;	cbi	spcr,mstr			;Slave mode
;	cbi	spcr,cpol
;	cbi	spcr,cpha
;	sbi	spcr,spr1
;	sbi	spcr,spr0

;	ldi	temp0,0b01010011
;	out	spcr,temp0


;	cbi	spsr,spif
	;ldi	temp0,0B11011011
;	ldi	temp0,0B01011111		;Set up spi control register for master
;	ldi	temp0,0B11001011		;Set up spi control register for slave
;	out	spcr,temp0
	
;	in	temp0,spsr
;	in	temp0,spdr

	ret



;***************************************************************************
;*
;* FUNCTION
;*	u_init
;*
;* DESCRIPTION
;*	Initialize SUART.
;*
;***************************************************************************

u_init:	ldi	u_stat,1<<STXC	; set STXC
	lds	temp1,Str6	; set clock rate
	;ldi	temp1,R		
	out	TCCR0,temp1
	sbi	DDRB,STXPIN	; initialize SUART pins
	cbi	DDRD,SRXPIN
	ret

;-------------------------------------------------------------------------------
;***************************************************************************
;*
;* FUNCTION
;*	timer1_init
;*
;* DESCRIPTION
;*	Initialize timer1
;*
;***************************************************************************
timer1_init:	

	;lds	temp1,Str6	; set clock rate
	ldi	temp1,5	
	out	TCCR1b,temp1

	;*****************************
	ldi	temp1,1<<TOV1	; enable T/C0 overflow interrupt
	out	TIMSK,temp1
	;******************************
	ldi	temp1,0;(256-(N+N/2)+8+12);
	out	TCNT1H,temp1 			; preset T/C0 to 1.5 bit lengths
	ldi	temp1,256
	out	TCNT1L,temp1
	ldi	temp1,1<<TOIE1
	out	TIFR,temp1			; clear T/C0 overflow flag
	out	TIMSK,temp1			; enable T/C0 overflow Interrupt
	ret

;-------------------------------------------------------------------------------



;***************************************************************************
;*
;* FUNCTION
;*	putc
;*
;* DESCRIPTION
;*	Send a character on the UART Tx line.
;*
;***************************************************************************

putc:	
	clr	u_stat		; clear UART status flags
	clr	bit_cnt		; clear bit counter
	ldi	temp1,1<<TOV0	; enable T/C0 overflow interrupt
	out	TIMSK,temp1
putc0:	sbrs	u_stat,STXC	; while (!(u_stat & STXC)); // Wait for STXC
	rjmp	putc0
	ret
;---------------------------------------------------------------------------------
;---------------------------------------------------------------------------------------------
;***********************************************************************
;*
;* FUNCTION
;*    PutcHardUart
;*
;* DESCRIPTION
;*    Send a character on the Hard Uart Tx line
;*
;* PARAMETER
;*   tempchar
;*
;***********************************************************************
PutcHardUart: 
		sbi	ucr,txen		;AT90s8515,set sender bit
		;sbi	ucsrb,txen		;ATmega8515,set sender bit
		sbis	usr,udre		;AT90s8515,wait till register is cleared
		;sbis	ucsra,udre		;ATmega8515,wait till register is cleared
		rjmp	PutcHardUart
		out	udr,tempchar		;send the byte

		ret
;---------------------------------------------------------------------------------------------

;***************************************************************************
;*
;* FUNCTION
;*	getc
;*
;* DESCRIPTION
;*	Wait for start bit and receive a character on the SUART Rx line.
;*
;***************************************************************************

getc:	sbis	PIND,SRXPIN
	rjmp	getc
getc0:	sbic	PIND,SRXPIN
	rjmp	getc0
	;tldi	temp1,(256-(N+N/2)+8+12);
	lds	temp1,BL15
	out	TCNT0,temp1 			; preset T/C0 to 1.5 bit lengths
	ldi	temp1,1<<TOIE0
	out	TIFR,temp1			; clear T/C0 overflow flag
	out	TIMSK,temp1			; enable T/C0 overflow Interrupt
	clr	bit_cnt				; clear bit counter
getc1:	sbrs	u_stat,SRXC			; wait for Receive Complete
	rjmp	getc1
	cbr	u_stat,1<<SRXC			; clear SRXC
	ret


;---------------------------------------------------------------------------------
send:	
		sbi	PORTD,6
		;;clr	temp1
		;;sts	CrFlag,temp1
		rcall	ClearCrFlag		;Clear CrFlag
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
lsend:
		;ld	tempchar,y+		;Load byte from program memory into tempchar
		;tst	tempchar		;Test if we have reached the end of the string
		;brne	lsend			;If so, go to lsend
		;dec	yl
		
lsend0:		
		rcall 	Rs232GetByte
lsend1:					;----------
	;	sbi	ucr,txen	;set sender bit
	;	sbis	usr,udre	;wait till register is cleared
	;	rjmp	lsend1
	;	out	udr,tempchar	;send the recived valuable
		;-------------------------------------------
		st	y+,tempchar
		
	;cbi	ucr,txen	;clear sender bit
;		rcall	Rs232GetByte	
;		out	udr,tempchar	;send the recived valuable
;WaitForRS232Send:
;		sbis	UCR,TXEN			;ak nie je povoleny UART vysielac
;		rjmp	OneZeroAnswer			;tak skonci - ochrana kvoli zacykleniu v AT90S2323/2343
;		sbis	USR,TXC				;pockat na dovysielanie bytu
;		rjmp	WaitForRS232Send	
;OneZeroAnswer:	
		cpi	tempchar,0     
		brne	lsend0
lsend2:
                           
                
                ;------Test for suart=----------------------------------------
;Testforsuart:	
		ldi 	zl,low(2*SUART)			;Load stringpointer source
		ldi 	zh,high(2*SUART)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		;tst	yl;T	
		;breq	Testformo;T
		sbiw	yl,4
		pop	tempchar
		cp	yl,tempchar
		brne	Testforsettempref
		adiw	yl,4
		rjmp	ActionSuart
		;------Test for SetTempRef=----------------------------------------------------
Testforsettempref:
		ldi 	zl,low(2*settempref)		;Load stringpointer source
		ldi 	zh,high(2*settempref)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,10
		pop	tempchar
		cp	yl,tempchar
		brne	Testforstxetx
		adiw	yl,10
		rjmp	ActionSetTempRef

		;------Test for stxetx=-----------------------------------------------------
Testforstxetx:
		ldi 	zl,low(2*stxetx)		;Load stringpointer source
		ldi 	zh,high(2*stxetx)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,6
		pop	tempchar
		cp	yl,tempchar
		brne	Testforstxetx0
		adiw	yl,6
		rjmp	ActionStxEtx
		;------Test for stxetx0=--------------------------------------------------------------------
Testforstxetx0:
		ldi 	zl,low(2*stxetx0)		;Load stringpointer source
		ldi 	zh,high(2*stxetx0)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,7
		pop	tempchar
		cp	yl,tempchar
		brne	TestforSetTargetPno
		adiw	yl,7
		rjmp	ActionStxEtx10	
		;------Test for SetTargetPno=---------------------------------------------------------------
TestforSetTargetPno:
		ldi 	zl,low(2*settargetpno)		;Load stringpointer source
		ldi 	zh,high(2*settargetpno)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,12
		pop	tempchar
		cp	yl,tempchar
		brne	TestforSetPassPno
		adiw	yl,12
		rjmp	ActionSetTargetPno			
		;-------Test for SetPassPno=------------------------------------------------------
TestforSetPassPno:
		ldi 	zl,low(2*setpasspno)		;Load stringpointer source
		ldi 	zh,high(2*setpasspno)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,10
		pop	tempchar
		cp	yl,tempchar
		;brne	Testforabsent
		brne	TestforSetCustomerNo
		adiw	yl,10
		rjmp	ActionSetPassPno
		;------Testfor SetCustomerNo=-----------------------------------------------------
TestforSetCustomerNo:				
		ldi 	zl,low(2*setcustomerno)		;Load stringpointer source
		ldi 	zh,high(2*setcustomerno)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,13
		pop	tempchar
		cp	yl,tempchar
		brne	Testforabsent
		adiw	yl,13
		rjmp	ActionSetCustomerno
		
		;------Test for absent------------------------------------------------------
Testforabsent:	
		ldi 	zl,low(2*sspi)			;Load stringpointer source
		ldi 	zh,high(2*sspi)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,4
		pop	tempchar
		cp	yl,tempchar
		brne	Testformo
		adiw	yl,4
		rjmp	ActionSspi;ActionAbsent
		;-----Test for mo=----------------------------------------------
Testformo:		
		ldi 	zl,low(2*modeno)		;Load stringpointer source
		ldi 	zh,high(2*modeno)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl	
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,2
		pop	tempchar
		cp	yl,tempchar
		;tst	yl	
		;breq	Testmodeno
		brne	TestforBaud
		adiw	yl,2
		rjmp	ActionModeno
		;------Test for Baud-Rate-------------------------------------------		
TestforBaud:	
		ldi 	zl,low(2*baud)			;Load stringpointer source
		ldi 	zh,high(2*baud)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,4
		pop	tempchar
		cp	yl,tempchar
		brne	TestforLamp
		adiw	yl,4
		rjmp	ActionBaud
		;------Test for Lamp-Time-------------------------------------------		
TestforLamp:	
		ldi 	zl,low(2*lamptime)			;Load stringpointer source
		ldi 	zh,high(2*lamptime)
		ldi	yl,low(TempBuf)				;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,4
		pop	tempchar
		cp	yl,tempchar
		brne	TestforPoll
		adiw	yl,4
		rjmp	ActionLamp
		;-----Test for poll--------------------------------------------------
TestforPoll:
		ldi 	zl,low(2*poll)				;Load stringpointer source
		ldi 	zh,high(2*poll)
		ldi	yl,low(TempBuf)				;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,3
		pop	tempchar
		cp	yl,tempchar
		brne	TestforCMTI
		adiw	yl,3
		rjmp	ActionPoll
		
                ;------Test for +CMTI------------------------------------------------
TestforCMTI:                
                ldi 	zl,low(2*cmti)			;Load stringpointer source
		ldi 	zh,high(2*cmti)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		breq	TestforCMTI2
		rjmp	ActionCMTI

                ;------Test for +CMTI2---------------------------------------------------
TestforCMTI2:   
		            
                ldi 	zl,low(2*cmti)			;Load stringpointer source
		ldi 	zh,high(2*cmti)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		breq	TestforCMGL
		rjmp	ActionCMTI2		
		
		;------Test for +CMGL: 1---------------------------------------------
TestforCMGL:		
		;ldi 	zl,low(2*cmgl1)			;Load stringpointer source
		;ldi 	zh,high(2*cmgl1)
		ldi 	zl,low(2*cmgr)			;Load stringpointer source
		ldi 	zh,high(2*cmgr)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		breq	TestforTrimBits
		rjmp	ActionCMGL
                ;------Test for TrimBits---------------------------------------------
TestforTrimBits:		
		ldi 	zl,low(2*TrimBits)		;Load stringpointer source
		ldi 	zh,high(2*TrimBits)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		breq	Testfornocarrier
		rjmp	ActionTrimBits
		;------Test for NO CARRIER----------------------------------------
Testfornocarrier:	
		ldi 	zl,low(2*nocarrier)			;Load stringpointer source
		ldi 	zh,high(2*nocarrier)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		breq	TestforRing
		rjmp	ActionNoCarrier
		;------Test for RING----------------------------------------
TestforRing:	
		ldi 	zl,low(2*ring)			;Load stringpointer source
		ldi 	zh,high(2*ring)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		breq	TestforEEprom
		rjmp	ActionRing
		;------Test for EEprom------------------------------------------
TestforEEprom:
		;ldi 	zl,low(2*eeprom)		;Load stringpointer source
		;ldi 	zh,high(2*eeprom)
		;ldi	yl,low(TempBuf)			;Load stringpointer destination
		;ldi	yh,high(TempBuf)
		;rcall	StrSearch
		;tst	yl
		;breq	TestforConnect
		;rjmp	ActionEEprom 
		ldi 	zl,low(2*eeprom)			;Load stringpointer source
		ldi 	zh,high(2*eeprom)
		ldi	yl,low(TempBuf)				;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		sbiw	yl,3
		pop	tempchar
		cp	yl,tempchar
		brne	TestforConnect
		adiw	yl,3
		rjmp	ActionEEprom
	
		;------Test for CONNECT----------------------------------------
TestforConnect:	
		ldi 	zl,low(2*connect)		;Load stringpointer source
		ldi 	zh,high(2*connect)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		breq	TestforClock
		rjmp	ActionConnect				
		;-------Test for Clock----------------------------------------
TestforClock:		
		ldi 	zl,low(2*Clock)			;Load stringpointer source
		ldi 	zh,high(2*Clock)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		brne	ActionClock		
		;------Test for ClearAC-----------------------------------------------
		ldi 	zl,low(2*ClearAC)		;Load stringpointer source
		ldi 	zh,high(2*ClearAC)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		brne	ActionClearAC
		;------Test for ReadCC--------------------------------------------
		ldi 	zl,low(2*ReadCC)		;Load stringpointer source
		ldi 	zh,high(2*ReadCC)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		brne	ActionReadCC
		;------Test for ClearCC-----------------------------------------------
		ldi 	zl,low(2*ClearCC)		;Load stringpointer source
		ldi 	zh,high(2*ClearCC)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		brne	ActionClearCC
		;--------Test for Ver------------------------------------		
		ldi 	zl,low(2*Ver)			;Load stringpointer source
		ldi 	zh,high(2*Ver)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		push	yl
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		;lds	tempchar,smsFlag
		;clr	temp0
		;cpse	tempchar,temp0
		;sbiw	yl,2
		sbiw	yl,2
		pop	tempchar
		cp	yl,tempchar
		brne	Testmodeno						
		rjmp	ActionVer

		
		
		;------Test for getsubuf----------------------------------------------
TestforGetsubuf:		
	;	ldi 	zl,low(2*getsubuf)		;Load stringpointer source
	;	ldi 	zh,high(2*getsubuf)
	;	ldi	yl,low(TempBuf)			;Load stringpointer destination
	;	ldi	yh,high(TempBuf)
	;	rcall	StrSearch
	;	tst	yl	
	;	brne	Actiongetsubuf
		
		;----------------Test modeno-------------------------------
Testmodeno:	
	;;	lds	tempchar,SmsFlag
	;;	cpi	tempchar,1
	;;	breq	TestModeSms
		lds	tempchar,Bflags
		sbrc	tempchar,1
		rjmp	TestModeSms
		lds	tempchar,Mode
		cpi	tempchar,'1'
		breq	TestmodenoExit	;brne
		rjmp	ConvertToSoftU
TestmodenoExit:	
		rjmp	sendexit
TestmodeSms:
		rjmp	TestModeSmsExit
		
		;-------------------------------------------------------------


;---------------------Load protocolregister Ds 1603------------------------------------


ActionTrimBits:
		ldi	temp0,216
		rjmp	ActionClock0
ActionClock:
		ldi	temp0,65			
		rjmp	ActionClock0
ActionReadCC:
		ldi	temp0,129
		rjmp	ActionClock0
ActionClearCC:
		ldi	temp0,4		
		rjmp	ActionClock0
ActionClearAC:		
		ldi	temp0,2

ActionClock0:		
		ldi	xl,low(LongWord)
		ldi	xh,high(LongWord)		;Load pointer to clock return value
		rcall	DsClock
		

		;---------Print high word-----------------
		ldi	zl,low(LongWord)
		ldi	zh,high(LongWord)
		ldi	xl,low(Str6)
		ldi	xh,high(Str6)
		rcall	Itoa
		ldi 	zl,low(Str6)			;Load strinpointer
		ldi 	zh,high(Str6)
		;rcall	PrintBuf
		ldi	yl,low(TempBuf)			;t
		ldi	yh,high(TempBuf)		;t
		rcall	MoveString			;t
		ldi 	zl,low(2*Text1)			;Load strinpointer
		ldi 	zh,high(2*Text1)
		;rcall	PrintStr
		sbiw	yl,1				;t
		rcall	MFTR				;t
		push	yl				;t
		push	yh				;t
		;--------Print low word----------------
		ldi	zl,low(LongWord)
		ldi	zh,high(LongWord)
		adiw	zl,1
		adiw	zl,1
		ldi	xl,low(Str6)
		ldi	xh,high(Str6)
		rcall	Itoa
		ldi 	zl,low(Str6)			;Load strinpointer
		ldi 	zh,high(Str6)
		;rcall	PrintBuf
		pop	yh				;t
		pop	yl				;t
		sbiw	yl,1				;t
		rcall	MoveString			;t
		ldi 	zl,low(2*Text2)			;Load strinpointer
		ldi 	zh,high(2*Text2)
		;rcall	PrintStr
		sbiw	yl,1				;t
		rcall	MFTR				;t
		
		;rjmp	sendexit0
		rjmp	sendexit1			;t		
		
		
ActionVer:	
		
		
		;ldi 	zl,low(Text4)			;Load strinpointer
		;ldi 	zh,high(Text4)
		
		;ldi 	zl,$02			;Load strinpointer
		;ldi 	zh,$00
		;rcall	PrintEEtoSoftU
		
		
		ldi 	zl,low(2*Text)			;Load strinpointer
		ldi 	zh,high(2*Text)
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		rcall	MFTR
		;ldi	zl,low(TempBuf)
		;ldi	zh,high(TempBuf)
		;lds	tempchar,SmsFlag
		;cpi	tempchar,1
		;breq	ActionVer1
		;rcall	PrintBuf
		;rcall	PrintStr
		rjmp	sendexit1
;ActionVer1:
		
		
		;rcall	PrintSoftU
		;rcall	SendSms
		;clr	tempchar
		;sts	SmsFlag,tempchar

		;rjmp	sendexit0
		
ActionSuart:	
		mov	zl,yl
		mov	zh,yh
		adiw	zl,1
		;;lds	tempchar,SmsFlag
		;;cpi	tempchar,0
		;;breq	ActionSuart1
		lds	tempchar,Bflags
		sbrs	tempchar,1
		rjmp	ActionSuart1
		;----Eliminate cr and linefeed----------------------------------------
		push	zl
		push	zh
ActionSuart0:	
		ld	tempchar,z+		;Load byte from program memory into tempchar
		tst	tempchar		;Test if we have reached the end of the string
		brne	ActionSuart0		;If not so, go to ActionSuart0
		sbiw	zl,4
		clr	tempchar
		st	z,tempchar
		pop	zh
		pop	zl
		;----End of eliminat cr and linefeed-----------------------------------
ActionSuart1:				
		
		rcall	PrintSoftU
		rcall	ReadSoftUart			;t
		clr 	temp0				;t
		sts	Tempbuf,temp0			;t
		ldi	yl,low(TempBuf)			;t
		ldi	yh,high(TempBuf)		;t
		ldi	zl,low(SUARTbuf)		;t
		ldi	zh,high(SUARTbuf)		;t
		adiw	zl,1
		;pop	yh				;t
		;pop	yl				;t
		;sbiw	yl,1				;t
		rcall	MoveString			;t
;		rcall 	LongDelay
		rcall	SendSms0
		rcall	ClearSmsFlag		;ClearSmsFlag 
		clr	temp0
		sts	Count,temp0		;clear time counter	
		lds	temp0,Bflags
		sbr	temp0,1
;		cbr	temp0,4
		sts	Bflags,temp0
		clr	flag
		rcall	timer1_init
		
		
		rjmp	sendexit0	
		
	        ;rjmp	sendexit0


;------------------------------------------------------------------------------------------
ActionSetTempRef:
	

		
	
		adiw	yl,1	
		ld	temp0,y
		adiw	yl,1
		ld	tempchar,y
		clc
		subi	tempchar,48
		clc
		subi	temp0,48
		clc
		add	tempchar,temp0
		add	tempchar,temp0
		add	tempchar,temp0
		add	tempchar,temp0
		add	tempchar,temp0
		add	tempchar,temp0
		add	tempchar,temp0
		add	tempchar,temp0
		add	tempchar,temp0
		add	tempchar,temp0
	
	;	rcall	PutcHardUart
	
	

		ldi 	zl,low(tempref)		;Load strinpointer
		ldi 	zh,high(tempref)
		rcall	EEWrite	
				
		
		
		rcall	LongDelay
		ldi	tempchar,'T'
		rcall	PutcHardUart
		ldi	tempchar,'e'
		rcall	PutcHardUart
		ldi	tempchar,'m'
		rcall	PutcHardUart
		ldi	tempchar,'p'
		rcall	PutcHardUart
		ldi	tempchar,'r'
		rcall	PutcHardUart
		ldi	tempchar,'e'
		rcall	PutcHardUart
		ldi	tempchar,'f'
		rcall	PutcHardUart
		ldi	tempchar,':'
		rcall	PutcHardUart
		clr	tempchar
		sts	LongWord,tempchar
		ldi 	zl,low(tempref)		;Load strinpointer
		ldi 	zh,high(tempref)
		rcall	EERead	
		sts	LongWord+1,tempchar
		;---------Print low word-----------------
		ldi	zl,low(LongWord)
		ldi	zh,high(LongWord)
		ldi	xl,low(Str6)
		ldi	xh,high(Str6)
		rcall	Itoa
		lds	tempchar,str6+3
		rcall	PutcHardUart
		lds	tempchar,str6+4
		rcall	PutcHardUart
		ldi	tempchar,13
		rcall	PutcHardUart
	
	
ActionSetTempRefExit:		
	
	
	
	
	

		rcall	ClearSmsFlag		;ClearSmsFlag 
		clr	temp0
		sts	Count,temp0		;clear time counter	
		lds	temp0,Bflags
		sbr	temp0,1
		sts	Bflags,temp0
		rcall	timer1_init
		rcall	ClearTempBuf
		rcall	ClearBuf
		rcall	ClearSuartBuf
		clr	temp0
		mov	rxhead,temp0
		mov	rxtail,temp0
		rjmp	sendexit0
;----------------------------------------------------------------------------
				
ActionStxEtx:
		ldi	tempchar,48
		rcall	PutcHardUart
		mov	zl,yl
		mov	zh,yh
		adiw	zl,1
		;lds	tempchar,SmsFlag
		;cpi	tempchar,0
		;breq	ActionStxEtx1
		lds	tempchar,Bflags
		sbrs	tempchar,1
		rjmp	ActionStxEtx1
		;----Eliminate cr and linefeed----------------------------------------
		push	zl
		push	zh
ActionStxEtx0:	
		ld	tempchar,z+		;Load byte from program memory into tempchar
		tst	tempchar		;Test if we have reached the end of the string
		brne	ActionStxEtx0		;If not so, go to ActionSuart0
		sbiw	zl,4
		clr	tempchar
		st	z,tempchar
		pop	zh
		pop	zl
		;----End of eliminat cr and linefeed-----------------------------------
ActionStxEtx1:
		ldi	u_data,02				;Load byte
		rcall	putc	
		rcall	PrintSoftU
		ldi	u_data,03				;Load byte
		rcall	putc	
		;;clr	tempchar
		;;sts	SmsFlag,tempchar
		;lds	tempchar,Bflags
		;cbr	tempchar,2
		;sts	Bflags,tempchar
		;ldi 	zl,low(ireg)		;Load strinpointer
		;ldi 	zh,high(ireg)
		;rcall	EERead			;Get initreg ireg
		rcall	ClearSmsFlag		;Clear SmsFlag
		ldi	flag,0
		clr	temp0			 
		sts	Count,temp0		;clear time counter
		;;lds	tempchar,pollFlag
		;;cpse	tempchar,temp0
		lds	tempchar,Bflags
		sbrc	tempchar,0
		rcall	timer1_init	
		rjmp	sendexit0
		



ActionStxEtx10:	
		mov	zl,yl
		mov	zh,yh
		adiw	zl,1
		;lds	tempchar,SmsFlag
		;cpi	tempchar,0
		;breq	ActionStxEtx12
		lds	tempchar,Bflags
		sbrs	tempchar,1
		rjmp	ActionStxEtx12
		;----Eliminate cr and linefeed----------------------------------------
		push	zl
		push	zh	
ActionStxEtx11:		
		ld	tempchar,z+		;Load byte from program memory into tempchar
		tst	tempchar		;Test if we have reached the end of the string
		brne	ActionStxEtx11		;If not so, go to ActionSuart0
		sbiw	zl,4
		clr	tempchar
		st	z,tempchar
		pop	zh
		pop	zl
		;----End of eliminat cr and linefeed-----------------------------------		
ActionStxEtx12:		
		ldi	u_data,02				;Load byte
		rcall	putc	
		rcall	PrintSoftU
		ldi	u_data,03				;Load byte
		rcall	putc	
	        rjmp	sendexit0




ActionSetPassPno:
		mov	zl,yl
		mov	zh,yh
		adiw	zl,1
		ldi 	yl,low(pslistE)			;Load strinpointer
		ldi 	yh,high(pslistE)
		lds	tempchar,Bflags
		sbrs	tempchar,1
;		rjmp	ActionSetTargetPno2
;		rjmp 	ActionSetTargetPno00
		rjmp	ActionSetPassPno2
		rjmp	ActionSetPassPno1
ActionSetCustomerNo:
		mov	zl,yl
		mov	zh,yh
		adiw	zl,1
		ldi 	yl,low(customernoE)			;Load strinpointer
		ldi 	yh,high(customernoE)
		lds	tempchar,Bflags
		sbrs	tempchar,1
		rjmp	ActionSetTargetPno2
		rjmp 	ActionSetTargetPno00



ActionSetTargetPno:
		mov	zl,yl
		mov	zh,yh
		adiw	zl,1
		;;lds	tempchar,SmsFlag
		;;cpi	tempchar,0
		;;breq	ActionSetTargetPno1
		ldi 	yl,low(targetPno)			;Load strinpointer
		ldi 	yh,high(targetPno)
		lds	tempchar,Bflags
		sbrs	tempchar,1
		rjmp	ActionSetTargetPno1
		

		
		;----Eliminate cr and linefeed----------------------------------------
ActionSetTargetPno00:
		push	zl
		push	zh
ActionSetTargetPno0:
		ld	tempchar,z+		;Load byte from program memory into tempchar
		tst	tempchar		;Test if we have reached the end of the string
		brne	ActionSetTargetPno0	;If not so, go to ActionSuart0
		;sbiw	zl,4
		sbiw	zl,5
		clr	tempchar
		st	z,tempchar
		pop	zh
		pop	zl
		;----End of eliminat cr and linefeed-----------------------------------	
ActionSetTargetPno1:
		;ldi	u_data,02				;Load byte
		;rcall	putc	
		;rcall	PrintSoftU
		;ldi	u_data,03				;Load byte
		;rcall	putc
		
		
	        

;---------------------------------------------------------------------------
		
;		ldi 	yl,low(targetPno)			;Load strinpointer
;		ldi 	yh,high(targetPno)
		;sbi	PORTD,6			;Set RTS On
ActionSetTargetPno2:
		ld	tempchar,z		;Load byte from program memory into tempchar
		rcall	EEWriteY
		tst	tempchar		;Test if we have reached the end of the string
		breq	ActionSetTargetPno3	;If so, quit
		;mov	u_data,tempchar		;Load byte
		;rcall	putc			;Send byte to soft-uart
		adiw	zl,1			;increase z
		adiw	yl,1			;increase y
		rjmp	ActionSetTargetPno2	;
ActionSetTargetPno3:	
		;cbi	PORTD,6			;Set RTS Off
		;;clr	tempchar
		;;sts	SmsFlag,tempchar
		;lds	tempchar,Bflags
		;cbr	tempchar,2
		;sts	Bflags,tempchar
		rcall	ClearSmsFlag		
		rjmp	sendexit0
;----------------------------------------------------------------------------
		;----Eliminate cr and linefeed----------------------------------------
ActionSetPassPno1:

LineFeed:

		push	zl
		push	zh
LineFeed0:
		ld	tempchar,z+		;Load byte from program memory into tempchar
		tst	tempchar		;Test if we have reached the end of the string
		brne	LineFeed0		;If not so, go to ActionSuart0
		sbiw	zl,5
		clr	tempchar
		st	z,tempchar
		pop	zh
		pop	zl
		;----End of eliminat cr and linefeed-----------------------------------	


ActionSetPassPno2:

		ld	tempchar,z
		subi	tempchar,48

		adiw	zl,1
ActionSetPassPnoLoop:
		tst	tempchar
		breq	ActionSetPassPno3		
		adiw	yl,13
		dec	tempchar
		rjmp	ActionSetPassPnoLoop
ActionSetPassPno3:


		ld	tempchar,z		;Load byte from program memory into tempchar
		tst	tempchar		;Test if we have reached the end of the string
		breq	ActionSetPassPno4	;If so, quit
		rcall	EEWriteY
		adiw	zl,1			;increase z
		adiw	yl,1			;increase y
		rjmp	ActionSetPassPno3	;
ActionSetPassPno4:	
		ldi	tempchar,32
		rcall	EEWriteY
		;cbi	PORTD,6			;Set RTS Off
		rcall	ClearSmsFlag		
		rjmp	sendexit0


;---------------------------------------------------------------------------------------------------------



	        
;ActionAbsent:
ActionSspi:
		
		
		
		adiw	yl,1
		ld	tempchar,y	
		cpi	tempchar,48
		breq	ActionSspi_10
		cpi	tempchar,49
		breq	ActionSspi_20
		cpi	tempchar,50
		breq	ActionSspi_30
		
ActionSspi_10:		
		adiw	yl,1
		ld	tempchar,y	
		cbi	portb,ss
		rcall	rw_spi		
		sbi	portb,ss
		
		
		ldi	tempchar,'v'
		cbi	portb,ss
		rcall	rw_spi
		sbi	portb,ss
		;mov	u_data,tempchar
		;rcall	putc
		rcall	PutcHardUart
		ldi	tempchar,'v'
		cbi	portb,ss
		rcall	rw_spi
		sbi	portb,ss
		;mov	u_data,tempchar
		;rcall	putc
		rcall	PutcHardUart
		rjmp	ActionSspiexit	
		
ActionSspi_20:		
		adiw	yl,1
		ld	tempchar,y	
		cbi	portc,ss1
		rcall	rw_spi		
		sbi	portc,ss1
		rjmp	ActionSspiexit
		
ActionSspi_30:		
		adiw	yl,1
		ld	tempchar,y	
		cbi	portc,ss2
		rcall	rw_spi		
		sbi	portc,ss2
		rjmp	ActionSspiexit				
		
		
ActionSspiexit:		
		;ldi 	zl,low(2*atcmgd)	;Clear all sms
		;ldi 	zh,high(2*atcmgd)
		;rcall	PrintStr
		;rcall	longdelay


		rcall	ClearSmsFlag		;ClearSmsFlag 
		clr	temp0
		sts	Count,temp0		;clear time counter
		;;ldi	temp0,1
		;;sts	pollFlag,temp0		;set pollFlag	
		lds	temp0,Bflags
		sbr	temp0,1
		sts	Bflags,temp0
		;sts	lampStatus,temp0	;clear lampStatus
		rcall	timer1_init
		rcall	ClearTempBuf
		rcall	ClearBuf
		rcall	ClearSuartBuf
		clr	temp0
		mov	rxhead,temp0
		mov	rxtail,temp0
		rjmp	sendexit0

;		sbi	PORTD,6
;		mov	zl,yl
;		mov	zh,yh
;		adiw	zl,1
		;;lds	tempchar,SmsFlag
		;;cpi	tempchar,0
		;;breq	ActionAbsent4
;		lds	tempchar,Bflags
;		sbrs	tempchar,1
;		rjmp	ActionAbsent4	
		;----Eliminate cr and linefeed----------------------------------------
;		push	zl
;		push	zh
;ActionAbsent0:	
;		ld	tempchar,z+		;Load byte from program memory into tempchar
;		tst	tempchar		;Test if we have reached the end of the string
;		brne	ActionAbsent0		;If not so, go to ActionAbsent0
;		sbiw	zl,5
;		ldi	tempchar,'/'
;		st	z,tempchar
;		adiw	zl,1
		;-----Add phoneno-----------------------------------------
;		mov	yl,zl
;		mov	yh,zh
;		ldi	zl,low(Phoneno)
;		ldi	zh,high(Phoneno)
;		rcall	MoveString
		;clr	tempchar
		;st	z,tempchar
;		pop	zh
;		pop	zl
		;-----Add space--------------------------------------------
;		push	zl
;		push	zh
;		clr	temp0
;ActionAbsent2:		
;		ld	tempchar,z+		;Load byte from program memory into tempchar
;		inc	temp0			;Increace counter
;		tst	tempchar		;Test if we have reached the end of the string
;		brne	ActionAbsent2		;If not so, go to ActionAbsent2
;		sbiw	zl,1
;		dec	temp0
;ActionAbsent3:		
;		inc	temp0
;		ldi	tempchar,' '
;		st	z,tempchar
;		adiw	zl,1
;		cpi	temp0,32
;		brne	ActionAbsent3
;		clr	tempchar
;		st	z,tempchar	
;		pop	zh
;		pop	zl
		;----End of eliminat cr and linefeed-----------------------------------
;ActionAbsent4:		
		;sbi	PORTD,6				;Set RTS On
;		rcall	PrintSoftU
	        ;cbi	PORTD,6				;Set RTS Off
;	        sbi	PORTD,6
	        ;rjmp	sendexit0
;	        rjmp	sendexit2
	        
;Actiongetsubuf:
;		ldi	zl,low(SUARTbuf)		;load stringpointer
;		ldi	zh,high(SUARTbuf)
;		lds	temp1,SUARTbuf			;load buf length counter
;		inc	temp1
;		add	zl,temp1
;		ldi	temp0,0				;terminate buffer	
;		st	z,temp0
;		sub	zl,temp1
;		adiw	zl,1
;		rcall	PrintBuf			;print-out buffer to uart
;		clr 	temp0
;		sts	SUARTbuf,temp0			;clear counter

;M		rjmp	sendexit0
ActionCMTI:
		;ejldi 	zl,low(TempBuf)			;Load strinpointer
		;ejldi 	zh,high(TempBuf)
		;mov	zl,yl
		;mov	zh,yh
		;adiw	zl,1
		;rcall	PrintSoftU
		;rcall printbuf
		;ldi	u_data,'J'
	        ;rcall	putc
	        ;clr	tempchar
		;sts	TempBuf,tempchar
		cbi	PORTD,6
		ldi	temp0,1
		sts	SmsCount,temp0
	;******************************
	;	ldi 	zl,low(2*unread)			;Load strinpointer source
	;	ldi 	zh,high(2*unread)
	;	rcall	StrSearch
	;	ldi	xl,low(phoneno)
	;	ldi	xh,high(phoneno)
;ActionCMGL0:
		adiw	yl,1	
		ld	tempchar,y+
		;mov	u_data,tempchar
		;rcall	putc
		sts	SmsNo,tempchar
		ld	tempchar,y+
		;mov	u_data,tempchar
		;rcall	putc
		sts	SmsNo+1,tempchar
		;ld	tempchar,y+
		;mov	u_data,tempchar
		;rcall	putc
		;ld	tempchar,y+
		;mov	u_data,tempchar
		;rcall	putc
;		st	x+,tempchar
;		cpi	tempchar,'"'
;		brne	ActionCMGL0
;		clr	tempchar
;		st	-x,tempchar
		;******************************
		rcall	ReadSms00
	        ;ldi 	zl,low(2*atcmgl)			;Load strinpointer
		;ldi 	zh,high(2*atcmgl)
		;tldi 	zl,low(2*atcmgr1)			;Load strinpointer
		;tldi 	zh,high(2*atcmgr1)
		;trcall	PrintStr
		;cbi	PORTD,6
		;trcall	LongDelay
		;trcall	LongDelay
		;trcall	LongDelay
		;trcall	LongDelay
	
	        rjmp	sendexit0
ActionCMTI2:
		;cbi	PORTD,6
		;ldi 	zl,low(2*atcmgr2)			;Load strinpointer
		;ldi 	zh,high(2*atcmgr2)
		;rcall	PrintStr
		;rcall	LongDelay
		;rcall	LongDelay
		;rcall	LongDelay
		;rcall	LongDelay
	        rjmp	sendexit0	        
	        
ActionCMGL:
		;------Get SMS Phoneno from sender-------------------------------------
		;cbi	PORTD,6					;Set RTS On
		ldi 	zl,low(2*unread)			;Load strinpointer source
		ldi 	zh,high(2*unread)
		rcall	StrSearch
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
ActionCMGL0:	
		ld	tempchar,y+
		st	x+,tempchar
		cpi	tempchar,'"'
		brne	ActionCMGL0
		clr	tempchar
		st	-x,tempchar
		;------Securety check---------------------------------------------------
		push	yl				;Save SMS message pointer on stack
		push	yh
		ldi	yl,low(phoneno)
		ldi	yh,high(phoneno)
;		ldi	zl,low(2*pslist)
;		ldi	zh,high(2*pslist)		
;		rcall	StrSearchEE			;Check for matching phoneno in list	

		ldi	zl,low(pslistE)
		ldi	zh,high(pslistE)		
		rcall	StrSearchEEP			;Check for matching phoneno in list

		tst	zl	
		brne	S00
		pop	yh
		pop	yl
		ldi 	zl,low(2*atcmgd)		;Clear all sms
		ldi 	zh,high(2*atcmgd)
		rcall	PrintStr
		rcall	longdelay
		rjmp	sendexit0
S00:		
		;------If Mode=0 printout Phoneno to softuart---------------------------
		lds	tempchar,Mode
		cpi	tempchar,'0'
		brne	GetSmsMessage
		ldi	zl,low(phoneno)
		ldi	zh,high(phoneno)
		;ldi 	zl,low(2*comma2)			;Load stringpointer source
		;ldi 	zh,high(2*comma2)
		;rcall	StrSearch
		
		;mov	zl,yl
		;mov	zh,yh
		;adiw	zl,1
		rcall	PrintSoftU
		;clr	tempchar
		;sts	TempBuf,tempchar
		;rcall printbuf
		ldi	u_data,13
	        rcall	putc
	        ;------Get SMS message------------------------------
GetSmsMessage:
		pop	yh;securety
		pop	yl;securety
		mov	zl,yl
		mov	zh,yh
		adiw	zl,24
		push	zl				;Save SMS message pointer on stack
		push	zh
		;----get end of SMS message----
		ldi	zl,low(2*ok)
		ldi	zh,high(2*ok)
		rcall	StrSearch
		clr	tempchar
		st	-y,tempchar
		st	-y,tempchar
		;-----print SMS message to SoftUart-----
		pop	zh
		pop	zl
		;------------------------------
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		sbiw	yl,2	
		rcall	MoveString
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		;;ldi	tempchar,1
		;;sts	SmsFlag,tempchar		;Set SmsFlag on
		rcall	SetSmsFlag			;Set SmsFlag 
		lds	tempchar,Bflags
		sbr	tempchar,2
		sts	Bflags,tempchar
		lds	tempchar,TempBuf;+2		
		cbi	PORTD,6				;Set RTS Off
		ldi	temp0,'#'
		cpse	tempchar,temp0
		rjmp	lsend2
		;------------------------------
		;sbi	PORTD,6				;Set RTS On
		ldi	zl,low(TempBuf)
		ldi	zh,high(TempBuf)
		adiw	zl,1;3
		;rcall	PrintSoftU
		rcall	PrintBuf
		;rcall	LongDelay
		ldi	tempchar,13
		rcall	PutcHardUart
		rcall	LongDelay;t
		;rcall	LongDelay;t
		;ldi	tempchar,0
		;sts	SmsFlag,tempchar		;Set SmsFlag on
		;--Clear all SMS----------------------------
		;ldi 	zl,low(2*atcmgd)			;Load strinpointer
		;ldi 	zh,high(2*atcmgd)
		;rcall	PrintStr
	        ;trjmp	sendexit0
	        ;cbi	PORTD,6				;Set RTS on
	        rjmp	send	        

ActionModeno:
		
		adiw	yl,1
		ld	tempchar,y			;Get modeno
		cpi	tempchar,'A'			;Set toggle source video,data1
		breq	ActionModeno1
		cpi	tempchar,'B'			;Set toggle source video,data1,data2
		breq	ActionModeno2
		cpi	tempchar,48
		brlo	ExitModenoError
		cpi	tempchar,51
		brsh	ExitModenoError	
		sts	Mode,tempchar			;Store modeno
		ldi 	zl,low(2*OK)			;Load strinpointer
		ldi 	zh,high(2*OK)
		rjmp	ExitModenoOk
ActionModeno1:
		ldi 	zl,low(buttonFlag)		;Load strinpointer
		ldi 	zh,high(buttonFlag)
		ldi	tempchar,0
		rcall	EEWrite	
		ldi 	zl,low(2*OK)			;Load strinpointer
		ldi 	zh,high(2*OK)
		rjmp	ExitModenoOk
ActionModeno2:	
		ldi 	zl,low(buttonFlag)		;Load strinpointer
		ldi 	zh,high(buttonFlag)
		ldi	tempchar,1
		rcall	EEWrite	
		ldi 	zl,low(2*OK)			;Load strinpointer
		ldi 	zh,high(2*OK)
		rjmp	ExitModenoOk	
		
ExitModenoError:
		ldi	zl,low(2*Error)
		ldi	zh,high(2*Error)
ExitModenoOk:		
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		rcall	MFTR
		rjmp	sendexit1	
		
ActionBaud:
		adiw	yl,1
		ld	tempchar,y			;Get modeno
		cpi	tempchar,48
		breq	Action9600
		cpi	tempchar,49
		breq	Action19200
		cpi	tempchar,50
		breq	Action38400
		cpi	tempchar,51
		breq	Action57600
		cpi	tempchar,52
		;breq	Action115200
		breq	Action115200T
		ldi	zl,low(2*Error)
		ldi	zh,high(2*Error)
		rjmp	ExitBaudError
Action115200T:
		rjmp	Action115200
Action9600:;----BitLength------------------------	
		ldi 	zl,low(baudrates)		;Load strinpointer
		ldi 	zh,high(baudrates)
		ldi	tempchar,2
		rcall	EEWrite	
		ldi	temp0,2
		sts	str6,temp0
		rcall	u_init				;Initialize SUART (software uart)	
		;tldi	temp0,(256-N+N1+8) 		;(256-N+8)
		ldi	tempchar,(256-170+13+8)
		adiw	zl,1				;increase z
		rcall	EEWrite
		sts	BL1,tempchar			; reset T/C0 to one bit lenght
		;tldi	temp0,(256-(N+N/2)+8+12+N2)	;(256-(N+N/2)+8+12);
		ldi	tempchar,(256-(170+170/2)+8+12+45)	;(256-(N+N/2)+8+12);
		adiw	zl,1
		rcall	EEWrite
		sts	BL15,tempchar 			; preset T/C0 to 1.5 bit lengths		
		rjmp	ExitBaudOk
Action19200:;----BitLength---------------------------
		ldi 	zl,low(baudrates)			;Load strinpointer
		ldi 	zh,high(baudrates)
		ldi	tempchar,2
		rcall	EEWrite	
		;ldi	temp0,2
		sts	str6,tempchar
		rcall	u_init					;Initialize SUART (software uart)
		ldi	tempchar,184;(256-79+8)
		adiw	zl,1					;increase z
		rcall	EEWrite
		sts	BL1,tempchar				; reset T/C0 to one bit lenght
		ldi	tempchar,155;(256-(79+79/2)+8+12);
		adiw	zl,1
		rcall	EEWrite
		sts	BL15,tempchar 				; preset T/C0 to 1.5 bit lengths				
		rjmp	ExitBaudOk

Action38400:;----BitLength------------------------		
		ldi 	zl,low(baudrates)			;Load strinpointer
		ldi 	zh,high(baudrates)
		ldi	tempchar,2
		rcall	EEWrite	
		;ldi	temp0,2
		sts	str6,tempchar
		rcall	u_init					;Initialize SUART (software uart)
		ldi	tempchar,(256-43+8)
		adiw	zl,1					;increase z
		rcall	EEWrite
		sts	BL1,tempchar				; reset T/C0 to one bit lenght
		ldi	tempchar,(256-(43+43/2)+8+12);
		adiw	zl,1
		rcall	EEWrite
		sts	BL15,tempchar 				; preset T/C0 to 1.5 bit lengths		
		rjmp	ExitBaudOk
Action57600:;----BitLength------------------------
		ldi 	zl,low(baudrates)			;Load strinpointer
		ldi 	zh,high(baudrates)
		ldi	tempchar,2
		rcall	EEWrite	
		sts	str6,tempchar
		rcall	u_init					;Initialize SUART (software uart)
		ldi	tempchar,(256-31+8)
		adiw	zl,1					;increase z
		rcall	EEWrite
		sts	BL1,tempchar				; reset T/C0 to one bit lenght			; reset T/C0 to one bit lenght
		ldi	tempchar,(256-(31+31/2)+8+12);
		adiw	zl,1
		rcall	EEWrite
		sts	BL15,tempchar 				; preset T/C0 to 1.5 bit lengths				
		rjmp	ExitBaudOk
Action115200:;----BitLength------------------------
		ldi 	zl,low(baudrates)			;Load strinpointer
		ldi 	zh,high(baudrates)
		ldi	tempchar,1
		rcall	EEWrite	
		sts	str6,tempchar
		rcall	u_init					;Initialize SUART (software uart)
		ldi	tempchar,(256-95+8)
		adiw	zl,1					;increase z
		rcall	EEWrite
		sts	BL1,tempchar				; reset T/C0 to one bit lenght	
		ldi	tempchar,(256-(95+95/2)+8+12);
		adiw	zl,1
		rcall	EEWrite
		sts	BL15,tempchar 				; preset T/C0 to 1.5 bit lengths				
		rjmp	ExitBaudOk		
ExitBaudOk:;--------------------------------------		
		ldi 	zl,low(2*OK)				;Load strinpointer
		ldi 	zh,high(2*OK)
ExitBaudError:
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		rcall	MFTR
		rjmp	sendexit1						
		
ActionNoCarrier:
		ldi	tempchar,'1'
		sts	Mode,tempchar
		rjmp	sendexit0
		

ActionRing:	
		rjmp	sendexit0
		
ActionEEprom:
		adiw	yl,1
		ld	tempchar,y			;Get value (On=1,Off=0)
		cpi	tempchar,48
		breq	ActionGetBoxStatus
		cpi	tempchar,49	
		breq	ActionReset
		cpi	tempchar,50
		breq	ActionPumpOn
		;breq	ActionButtonLock
		cpi	tempchar,51
		;breq	ActionButtonUnLock
		breq	ActionPumpOff
;		cpi	tempchar,52
;		breq	ActionCustomerNoOn
;		cpi	tempchar,53
;		breq	ActionCustomerNoOff
		
		ldi	zl,low(2*Error)
		ldi	zh,high(2*Error)
		rjmp	ExitBoxTest	
ActionReset:	
		rjmp	ActionReset


;ActionButtonLock:
;		ldi 	zl,low(ireg)		;Load strinpointer
;		ldi 	zh,high(ireg)
		;set
		;bld	tempchar,0			;Set bit 0 in ireg		
;		ldi	tempchar,39
;		rcall	EEWrite
;		rjmp	ActionReset
		
;ActionButtonUnLock:
;		ldi 	zl,low(ireg)		;Load strinpointer
;		ldi 	zh,high(ireg)
;		ldi	tempchar,7
;		rcall	EEWrite
;		rjmp	ActionReset
		
ActionCustomerNoOn:
		ldi 	zl,low(fieldFlag)		;Load strinpointer
		ldi 	zh,high(fieldFlag)
		ldi	tempchar,1
		rcall	EEWrite
		rjmp	ActionReset


ActionCustomerNoOff:		
		ldi 	zl,low(fieldFlag)		;Load strinpointer
		ldi 	zh,high(fieldFlag)
		ldi	tempchar,0
		rcall	EEWrite
		rjmp	ActionReset			


ActionPumpOn:
	
		ldi 	zl,low(pumpflag)		;Load strinpointer
		ldi 	zh,high(pumpflag)
		ldi	tempchar,1
		rcall	EEWrite	
		rcall	LongDelay
		ldi	tempchar,'O'
		rcall	PutcHardUart
		ldi	tempchar,'n'
		rcall	PutcHardUart
		rjmp	ActionPumpExit		
	
	
		
	
ActionPumpOff:	
		
		ldi	tempchar,0
		ldi 	zl,low(pumpflag)		;Load strinpointer
		ldi 	zh,high(pumpflag)
		rcall	EEWrite	
		rcall	LongDelay	
		ldi	tempchar,'O'
		rcall	PutcHardUart
		ldi	tempchar,'f'
		rcall	PutcHardUart
		ldi	tempchar,'f'
		rcall	PutcHardUart
ActionPumpExit:	
		ldi	tempchar,13
		rcall	PutcHardUart
		rcall	ClearSmsFlag		;ClearSmsFlag 
		clr	temp0
		sts	Count,temp0		;clear time counter	
		lds	temp0,Bflags
		sbr	temp0,1
		sts	Bflags,temp0
		rcall	timer1_init
		rcall	ClearTempBuf
		rcall	ClearBuf
		rcall	ClearSuartBuf
		clr	temp0
		mov	rxhead,temp0
		mov	rxtail,temp0
		rjmp	sendexit0


ActionGetBoxStatus:		
		rcall	LongDelay
		ldi	tempchar,'T'
		rcall	PutcHardUart
		ldi	tempchar,'e'
		rcall	PutcHardUart
		ldi	tempchar,'m'
		rcall	PutcHardUart
		ldi	tempchar,'p'
		rcall	PutcHardUart
		ldi	tempchar,':'
		rcall	PutcHardUart


		
				
		wdr
		clr	tempchar
		sts	LongWord,tempchar
		rcall	ReadDs18s20
		sts	LongWord+1,tempchar
		;---------Print low word-----------------
		ldi	zl,low(LongWord)
		ldi	zh,high(LongWord)
		ldi	xl,low(Str6)
		ldi	xh,high(Str6)
		rcall	Itoa
		lds	tempchar,str6+3
		rcall	PutcHardUart
		lds	tempchar,str6+4
		rcall	PutcHardUart
		ldi	tempchar,'/'
		rcall	PutcHardUart
		
		
		ldi	tempchar,'T'
		rcall	PutcHardUart
		ldi	tempchar,'e'
		rcall	PutcHardUart
		ldi	tempchar,'m'
		rcall	PutcHardUart
		ldi	tempchar,'p'
		rcall	PutcHardUart
		ldi	tempchar,'r'
		rcall	PutcHardUart
		ldi	tempchar,'e'
		rcall	PutcHardUart
		ldi	tempchar,'f'
		rcall	PutcHardUart
		ldi	tempchar,':'
		rcall	PutcHardUart
		clr	tempchar
		sts	LongWord,tempchar
		ldi 	zl,low(tempref)		;Load strinpointer
		ldi 	zh,high(tempref)
		rcall	EERead	
		sts	LongWord+1,tempchar
		;---------Print low word-----------------
		ldi	zl,low(LongWord)
		ldi	zh,high(LongWord)
		ldi	xl,low(Str6)
		ldi	xh,high(Str6)
		rcall	Itoa
		lds	tempchar,str6+3
		rcall	PutcHardUart
		lds	tempchar,str6+4
		rcall	PutcHardUart
		ldi	tempchar,13
		rcall	PutcHardUart
	



;		ldi	zl,low(Str6)
;		ldi	zh,high(Str6)
		
		
		
;		adiw	zl,1
;		pop	yh				;t
;		pop	yl				;t
;		sbiw	yl,1				;t
;		rcall	MoveString			;t

;		sbiw	yl,1
;		ldi	temp0,'/'
;		st	y+,temp0
;		adiw	yl,1
;		push	yl
;		push	yh

;		clr	tempchar
;		sts	LongWord,tempchar
;		lds	tempchar,hour
;		sts	LongWord+1,tempchar
		;---------Print low word-----------------
;		ldi	zl,low(LongWord)
;		ldi	zh,high(LongWord)
;		ldi	xl,low(Str6)
;		ldi	xh,high(Str6)
;		rcall	Itoa
		
		
		
		
;		ldi	zl,low(Str6)
;		ldi	zh,high(Str6)
		
		
		
;		adiw	zl,1
;		pop	yh				;t
;		pop	yl				;t
;		sbiw	yl,1				;t
;		rcall	MoveString			;t
	
;		sbiw	yl,1
;		ldi	temp0,'/'
;		st	y+,temp0
;		adiw	yl,1
;		push	yl
;		push	yh

;		clr	tempchar
;		sts	LongWord,tempchar
;		lds	tempchar,minute
;		sts	LongWord+1,tempchar
		;---------Print low word-----------------
;		ldi	zl,low(LongWord)
;		ldi	zh,high(LongWord)
;		ldi	xl,low(Str6)
;		ldi	xh,high(Str6)
;		rcall	Itoa
		
		
		
		
;		ldi	zl,low(Str6)
;		ldi	zh,high(Str6)
		
		
		
;		adiw	zl,1
;		pop	yh				;t
;		pop	yl				;t
;		sbiw	yl,1				;t
;		rcall	MoveString			;t
		
;		sbiw	yl,1
;		ldi	temp0,'/'
;		st	y+,temp0	
	
;		adiw	yl,1
;		push	yl
;		push	yh

;		clr	tempchar
;		sts	LongWord,tempchar
;		lds	tempchar,seconds
;		sts	LongWord+1,tempchar
		;---------Print low word-----------------
;		ldi	zl,low(LongWord)
;		ldi	zh,high(LongWord)
;		ldi	xl,low(Str6)
;		ldi	xh,high(Str6)
;		rcall	Itoa
		
		
		
		
;		ldi	zl,low(Str6)
;		ldi	zh,high(Str6)
		
		
		
;		adiw	zl,1
;		pop	yh				;t
;		pop	yl				;t
;		sbiw	yl,1				;t
;		rcall	MoveString			;t
		
;		sbiw	yl,1
;		ldi	temp0,'/'
;		st	y+,temp0
		
		
		
;		adiw	yl,1
;		push	yl
;		push	yh
	

		;---------Print Low  word------------------
;		lds	tempchar,Svolume
;		sts	LongWord+1,tempchar
		;---------Print high word-----------------
;		lds	tempchar,Svolume+1
;		sts	LongWord,tempchar
		
		
;		ldi	zl,low(LongWord)
;		ldi	zh,high(LongWord)
;		ldi	xl,low(Str6)
;		ldi	xh,high(Str6)
;		rcall	Itoa
		
		
		
		
;		ldi	zl,low(Str6)
;		ldi	zh,high(Str6)
		
		
		
;		adiw	zl,1
;		pop	yh				;t
;		pop	yl				;t
;		sbiw	yl,1				;t
;		rcall	MoveString			;t
	
;		sbiw	yl,1
;		ldi	temp0,'/'
;		st	y+,temp0
	
		
;		adiw	yl,1
;		push	yl
;		push	yh
	
	
		

		
			;---------Print Low  word------------------
;		lds	tempchar,GTemperature
;		sts	LongWord+1,tempchar
		;---------Print high word-----------------
;		lds	tempchar,GTemperature+1
;		sts	LongWord,tempchar
		
;		ldi	zl,low(LongWord)
;		ldi	zh,high(LongWord)
;		ldi	xl,low(Str6)
;		ldi	xh,high(Str6)
;		rcall	Itoa
		
		
		
		
;		ldi	zl,low(Str6)
;		ldi	zh,high(Str6)
		
		
		
;		adiw	zl,1
;		pop	yh				;t
;		pop	yl				;t
;		sbiw	yl,1				;t
;		rcall	MoveString			;t
;		
;		sbiw	yl,1
;		ldi	temp0,'/'
;		st	y+,temp0
	
		
		
;		ldi	zl,low(customernoE)
;		ldi	zh,high(customernoE)
;		rcall	MoveEEtoRam
;		sbiw	yl,1
		
;		ldi	temp0,'/'
;		st	y+,temp0
;		ldi	temp0,'T'
;		st	y+,temp0
;		ldi	temp0,'e'
;		st	y+,temp0
;		ldi	temp0,'s'
;		st	y+,temp0
;		ldi	temp0,'t'
;		st	y+,temp0
	
;		clr	temp0
;		st	y+,temp0
	
;		wdr
;		rcall	SendSms
;		wdr
		rcall	ClearSmsFlag		;ClearSmsFlag 
		clr	temp0
		sts	Count,temp0		;clear time counter	
		lds	temp0,Bflags
		sbr	temp0,1
		sts	Bflags,temp0
	
		rcall	timer1_init							
		rjmp	sendexit0	


ExitBoxTest:
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		rcall	MFTR
		rjmp	sendexit1	


ActionLamp:
		;mov	zl,yl
		;mov	zh,yh
		adiw	yl,1
		ld	tempchar,y			;Get value (On=1,Off=0)
		cpi	tempchar,48
		breq	ActionLamptOff
		cpi	tempchar,49
		breq	ActionLamptOn
		ldi	zl,low(2*Error)
		ldi	zh,high(2*Error)
		rjmp	ExitLamptError
		
ActionLamptOff:
		ldi	temp1,0	
		out	TCCR1b,temp1		;Stop timer
		;;clr	temp1
		;;sts	pollFlag,temp1		;clear pollFlag	
		lds	temp1,Bflags
		cbr	temp1,1
		sts	Bflags,temp1
		rjmp	ExitLamptOk
ActionLamptOn:
		;clr	temp0			 
		;sts	Count,temp0		;clear time counter
		;sts	pollFlag,temp0		;clear pollFlag	
		;sts	lampStatus,temp0	;clear lampStatus
		;rcall	timer1_init
		ldi	zl,low(Phoneno)		;Source string
		ldi	zh,high(Phoneno)
		
		ldi	yl,low(TempPhoneno)	;Target string
		ldi	yh,high(TempPhoneno)
		rcall	MoveString		;Copy Phoneno to TempPhoneno

		
		clr 	temp0
		sts	SUARTbuf,temp0			
		ldi 	zl,low(2*tlps)				;Load strinpointer
		ldi 	zh,high(2*tlps)
		rcall	PrintSoftUstr
		rcall	ReadSoftUart
		
		ldi	zl,low(SUARTbuf)
		ldi	zh,high(SUARTbuf)
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		ldi	temp0,'$'
		st	y+,temp0
		ldi	temp0,'L'
		st	y+,temp0
		ldi	temp0,'S'
		st	y+,temp0
		adiw	zl,1
		rcall	MoveString
		;-------Test--------------------------
		;ldi	zl,low(TempPhoneno)	;Target string
		;ldi	zh,high(TempPhoneno)
		;adiw	zl,1
		;rcall	MoveString
		;-------------------------------------
		;ld	tempchar,z		;Load byte from program memory into tempchar
		lds	tempchar,TempBuf+3
		sts	lampStatus,tempchar	;Save lampStatus
		rcall	SendSms
		;;clr	temp0
		;;sts	SmsFlag,temp0		;Set SmsFlag off			 
		;lds	temp0,Bflags
		;cbr	temp0,2
		;sts	Bflags,temp0
		rcall	ClearSmsFlag		;ClearSmsFlag 
		clr	temp0
		sts	Count,temp0		;clear time counter
		;;ldi	temp0,1
		;;sts	pollFlag,temp0		;set pollFlag	
		lds	temp0,Bflags
		sbr	temp0,1
		sts	Bflags,temp0
		;sts	lampStatus,temp0	;clear lampStatus
		rcall	timer1_init
		rjmp	sendexit0
		;rjmp	ExitLamptOk
		
ExitLamptOk:;--------------------------------------		
		ldi 	zl,low(2*OK)			;Load strinpointer
		ldi 	zh,high(2*OK)
ExitLamptError:
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		rcall	MFTR
		rjmp	sendexit1			
		
		
		
		

;------------------------------------------------------------------------------------
ActionPoll:
		adiw	yl,1
		ld	tempchar,y			;Get value (On=1,Off=0)
		cpi	tempchar,48
		breq	ActionPollOff
		cpi	tempchar,49
		breq	ActionPollOn
		cpi	tempchar,50
		breq	ActionPollinitFlagOn
		cpi	tempchar,51
		breq	ActionPollinitFlagOff
		cpi	tempchar,52
		breq	ActionPollinitTargetPhoneNo
		ldi	zl,low(2*Error)
		ldi	zh,high(2*Error)
		rjmp	ExitPollError
		
ActionPollOff:
		ldi	temp1,0	
		out	TCCR1b,temp1		;Stop timer
		;;clr	temp1
		;;sts	pollFlag,temp1		;clear pollFlag
		lds	temp1,Bflags
		cbr	temp1,1
		sts	Bflags,temp1	
		rjmp	ExitPollOk
ActionPollOn:
		ldi	zl,low(Phoneno)		;Source string
		ldi	zh,high(Phoneno)
		
		ldi	yl,low(TempPhoneno)	;Target string
		ldi	yh,high(TempPhoneno)
		rcall	MoveString		;Copy Phoneno to TempPhoneno

		
		;clr 	temp0
		;sts	SUARTbuf,temp0			
		;ldi 	zl,low(2*tlps)				;Load strinpointer
		;ldi 	zh,high(2*tlps)
		;rcall	PrintSoftUstr
		;rcall	ReadSoftUart
		
		;ldi	zl,low(SUARTbuf)
		;ldi	zh,high(SUARTbuf)
		;ldi	yl,low(TempBuf)
		;ldi	yh,high(TempBuf)
		;ldi	temp0,'$'
		;st	y+,temp0
		;ldi	temp0,'L'
		;st	y+,temp0
		;ldi	temp0,'S'
		;st	y+,temp0
		;adiw	zl,1
		;rcall	MoveString



		;lds	tempchar,TempBuf+3
		;sts	lampStatus,tempchar		;Save lampStatus
		;rcall	SendSms
		clr	temp0
	;	sts	SmsFlag,temp0			;Set SmsFlag off			 
		sts	Count,temp0			;clear time counter
		;;ldi	temp0,1
		;;sts	pollFlag,temp0			;set pollFlag	
		lds	temp0,Bflags
		sbr	temp0,1				;set pollFlag
		cbr	temp0,2				;Set SmsFlag off
		sts	Bflags,temp0
		rcall	timer1_init
		;rjmp	sendexit0
		rjmp	ExitPollOk
ActionPollinitFlagOn:
		ldi 	zl,low(ireg)			;Load strinpointer
		ldi 	zh,high(ireg)
		rcall	EERead
		set
		bld	tempchar,0			;Set bit 0 in ireg		
		;ldi	tempchar,1
		rcall	EEWrite	
		rjmp	ExitPollOk
ActionPollinitFlagOff:
		ldi 	zl,low(ireg)			;Load strinpointer
		ldi 	zh,high(ireg)
		rcall	EERead
		clt					;Clear bit 0 in ireg
		bld	tempchar,0
		;ldi	tempchar,0
		rcall	EEWrite	
		rjmp	ExitPollOk
ActionPollinitTargetPhoneNo:
		rcall	initPoll
		rjmp	ExitPollOk
ExitPollOk:;--------------------------------------		
		ldi 	zl,low(2*OK)			;Load strinpointer
		ldi 	zh,high(2*OK)
ExitPollError:
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		rcall	MFTR
		rjmp	sendexit1			
		
	

;------------------------------------------------------------------------------------


				

ActionConnect:
		clr	tempchar
		sts	Mode,tempchar
		rjmp	sendexit0



ConvertToSoftU:
		ldi	zl,low(TempBuf)
		ldi	zh,high(TempBuf)
		rcall	PrintSoftU
		;clr	tempchar
		;sts	TempBuf,tempchar
		rjmp	sendexit0		
		
		
sendexit:
		cbi	PORTD,6
		ret
sendexit0:
		clr	tempchar
		sts	TempBuf,tempchar
		rjmp	sendexit
sendexit1:
		ldi	zl,low(TempBuf)
		ldi	zh,high(TempBuf)
		;;lds	tempchar,SmsFlag		;Load SmsFlag
		;;ldi	temp0,1
		;;cpse	tempchar,temp0
		lds	tempchar,Bflags
		sbrs	tempchar,1
		rcall	PrintBuf
		;clr	temp0
		;cpse	tempchar,temp0			;Send SMS if SmsFlag on
		sbrc	tempchar,1
		rcall	SendSms
		;;clr	tempchar
		;;sts	SmsFlag,tempchar		;Set SmsFlag off
		;lds	tempchar,Bflags
		;cbr	tempchar,2
		;sts	Bflags,tempchar
		rcall	ClearSmsFlag			;Clear SmsFlag			;C
		rjmp	sendexit	
sendexit2:
		clr	tempchar
		sts	TempBuf,tempchar
		ret
					
TestModeSmsExit:
		;ldi	yl,low(TempBuf)
		;ldi	yh,high(TempBuf)
		;rcall	MoveString
		;-------If Mode=0 print-out to softuart-----------------------------------
		lds	tempchar,Mode
		cpi	tempchar,'0'
		brne	TME0	
		ldi	zl,low(TempBuf)
		ldi	zh,high(TempBuf)
		ldi	u_data,65
	        rcall	putc
	        rcall	PrintSoftu
	        ldi	u_Data,66
	        rcall	putc
	        ;tlds	tempchar,TempBuf
	        ;tcpi	tempchar,13
	        ;tbreq	TestModeExit0
TME0:	        ;------Send Sms---------------
		rcall	SendSms
		;;clr	tempchar
		;;sts	SmsFlag,tempchar
		;lds	tempchar,Bflags
		;cbr	tempchar,2
		;sts	Bflags,tempchar
		rcall	ClearSmsFlag			;Clear SmsFlag
TestModeExit0:
		rjmp	sendexit
		
;--------------------------------------------------------------------------------------------


;
;**************************
;* The RS232 FIFO manager *
;**************************
Rs232GetByte:
	push	r30
	push	r31
	clr	tempchar
	cp	rxhead,rxtail
	breq	Rs232GetBytexit
	clr	r2
	ldi	zl,low(Buf)	; Pointer to the rxbuffer (FIFO)
	ldi	zh,high(Buf)
	add	zl,rxhead
	adc	zh,tempchar	; add address offset...  z = &Buf[rxhead]
	ld	tempchar,z	; tempchar = Buf[rxhead++]
	inc	rxhead
	cpi	rxhead,BUFSIZE	; test for the end of the data buffer
	brlo	Rs232Gbx
	clr	rxhead		; Circle around to the first buffer pos.
Rs232Gbx:
;***** Xon calcs go here *****
	sec			; This is not need for ASCII data
	pop	r30		; as a zero value returned in tempchar works
	pop	r31
	ret			; But, for binary data, zero may be valid!
Rs232GetBytexit:		; Return 'No' bytes in yet.
	clc			
	pop	r30
	pop	r31
	ret
;----------------------------------------------------------------------------------------------
GetStringData:
		clr	temp1
		clr	tempchar
		ldi	zl,low(2*Ver)
		ldi	zh,high(2*Ver)
		ldi	tempchar,1
		add	zl,tempchar
		clr	tempchar
		;adc	zh,tempchar
	
		;lpm	tempchar,z
		lpm

		;st	z+,tempchar
		;st	z+,tempchar
		;out	udr,tempchar	;send the recived valuable
send2:	
		sbi	ucr,txen	;AT90s8515,set sender bit
		;sbi	ucsrb,txen	;ATmega8515,set sender bit
		sbis	usr,udre	;AT90s8515,wait till register is cleared
		;sbis	ucsra,udre	;ATmega8515,wait till register is cleared
		rjmp	send2
		out	udr,r0	;send the recived valuable
		ret
		
;----------------------------------------------------------------------------------------------
test:
		clr	temp1
	;	ldi 	zl,low(2*Text)	;Load strinpointer
	;	ldi 	zh,high(2*Text)
		ldi	zl,low(TempBuf)
		ldi	zh,high(TempBuf)
	;	rcall	PrintStr
		rcall	printBuf

		ret

;----------------------------------------------------------------------------------------------
;WriteTempBuf:
;		ldi	xl,low(TempBuf)			;Load stringpointer destination
;		ldi	xh,high(TempBuf)
;		ldi	tempchar,'R'	
;		st	x+,tempchar
;		ldi	tempchar,'e'
;		st	x+,tempchar
;		ldi	tempchar,'a'
;		st	x+,tempchar
;		ldi	tempchar,'d'
;		st	x+,tempchar
;		ldi	tempchar,'C'
;		st	x+,tempchar
;		ldi	tempchar,'C'
;		st	x+,tempchar
;		clr	tempchar
;		st	x+,tempchar

;		ret
;----------------------------------------------------------------------------------------------		

;***********************************************************************
;*PrintStr
;*
;*z pointer to the string (zl,zh)
;*
;***********************************************************************
PrintStr:
		lpm				;Load byte from program memory into r0
		tst	r0			;Test if we have reached then end of the string
		breq	quitPrintStr		;If so, quit
PrintStr1:
		sbi	ucr,txen		;AT90s8515,set sender bit
		;sbi	ucsrb,txen		;ATmega8515,set sender bit
		sbis	usr,udre		;AT90s8515,wait till register is cleared
		;sbis	ucsra,udre		;ATmega8515,wait till register is cleared
		rjmp	PrintStr1
		out	udr,r0			;send the byte
		adiw	zl,1			;increase z
		rjmp	PrintStr
quitPrintStr:
		ret
;---------------------------------------------------------------------------------------------
;***********************************************************************
;*PrintBuf
;*
;*z pointer to the string (zl,zh)
;*
;***********************************************************************
PrintBuf:
		ld	tempchar,z		;Load byte from program memory into r0
		tst	tempchar		;Test if we have reached then end of the string
		breq	quitPrintBuf		;If so, quit
lPrintBuf:
		sbi	ucr,txen		;AT90s8515,set sender bit
		;sbi	ucsrb,txen		;ATmega8515,set sender bit
		sbis	usr,udre		;AT90s8515,wait till register is cleared, tegister name in AT90s8515
		;sbis	ucsra,udre		;ATmega8515,wait till register is cleared, register name in Atmega 8515
		rjmp	lPrintBuf
		out	udr,tempchar		;send the byte
		adiw	zl,1			;increase z
		rjmp	PrintBuf
quitPrintBuf:
		ret
;---------------------------------------------------------------------------------------------

;---------------------------------------------------------------------------------------------
;***********************************************************************
;*PrintSoftU
;*
;*z pointer to the string (zl,zh)
;*
;***********************************************************************
PrintSoftU:

		sbi	PORTD,6			;Set RTS On
		ld	tempchar,z		;Load byte from program memory into tempchar
		tst	tempchar		;Test if we have reached the end of the string
		breq	quitPrintSoftU		;If so, quit
		mov	u_data,tempchar		;Load byte
		rcall	putc			;Send byte to soft-uart
		adiw	zl,1			;increase z
		rjmp	PrintSoftU
quitPrintSoftU:
		;ldi	u_data,13
		;rcall	putc
		cbi	PORTD,6			;Set RTS Off
		ret
;--------------------------------------------------------------------------------------------
;***********************************************************************
;*PrintSoftUstr
;*
;*z pointer to the string (zl,zh)
;*
;***********************************************************************
PrintSoftUstr:

		lpm				;Load byte from program memory into r0
		tst	r0			;Test if we have reached the end of the string
		breq	quitPrintSoftUstr	;If so, quit
		mov	u_data,r0		;Load byte
		rcall	putc			;Send byte to soft-uart
		adiw	zl,1			;increase z
		rjmp	PrintSoftUstr
quitPrintSoftUstr:
		;ldi	u_data,13
		;rcall	putc
		ret
;--------------------------------------------------------------------------------------------
;***********************************************************************
;*SendPacketSoftUstr
;*
;*z pointer to the string (zl,zh)
;*temp0 packet length
;*
;***********************************************************************
SendPacketSoftUstr:
		;ldi	temp0,13
SendPacketSoftUstr0:
		lpm				;Load byte from program memory into r0
		tst	temp0;r0			;Test if we have reached the end of the string
		breq	quitSendPacketSoftUstr	;If so, quit
		mov	u_data,r0		;Load byte
		rcall	putc			;Send byte to soft-uart
		adiw	zl,1			;increase z
		dec	temp0			;decrease temp0
		rjmp	SendPacketSoftUstr0
quitSendPacketSoftUstr:
		;ldi	u_data,13
		;rcall	putc
		ret
;--------------------------------------------------------------------------------------------







;***********************************************************************
;*PrintEESoftU
;*
;*z pointer to the string (zl,zh)
;*
;***********************************************************************
PrintEEtoSoftU:

		sbi	PORTD,6				;Set RTS On
		;ld	tempchar,z			;Load byte from program memory into tempchar
		rcall	EERead
		tst	tempchar			;Test if we have reached the end of the string
		breq	quitPrintEEtoSoftU		;If so, quit
		mov	u_data,tempchar			;Load byte
		rcall	putc				;Send byte to soft-uart
		adiw	zl,1				;increase z
		rjmp	PrintEEtoSoftU
quitPrintEEtoSoftU:
		;ldi	u_data,13
		;rcall	putc
		cbi	PORTD,6				;Set RTS Off
		ret
;--------------------------------------------------------------------------------------------
;------------------------------------------------------------------------------
;Routine to search for one string within another
;y=pointer to the destination string (1)
;z=pointer to the source string(2)
;
StrSearch:
		mov	xl,zl			;Save z in x
		mov	xh,zh
		
		
StrSearchL:
		lpm				;Load byte from source string
		ld	tempchar,y		;Load byte from destination string
		cp	r0,tempchar		;if r0[z]=tempchar[y] then 
		breq	StrSearch1
		
		mov	zl,xl			;else
		mov	zh,xh			;Pointer to beginning of source string
		sbiw	yl,1
		lpm
		ld	tempchar,y
		cp	r0,tempchar
		breq	StrSearch2
StrSearch01:	
		adiw	yl,1			;Increase y		
						;end else
StrSearch0:
		adiw	yl,1			;Increase y
		ld	tempchar,y
		tst	tempchar		;If tempchar=0 then exit	
		breq	StrSearchNosucess
		rjmp	StrSearchL

StrSearch1:
		adiw	zl,1			;Increase z
		lpm	
		tst	r0			;If r0=0 then exit
		breq	StrSearchExit		;(a match was found)
		rjmp	StrSearch0	
		
StrSearch2:
		adiw	zl,1			;Increase z
		rjmp	StrSearch01

StrSearchNosucess:
		clr	yl
		clr	yh
		rjmp	StrSearchExit


StrSearchExit:
;	POP	CX				;Restore registers
;	POP	DI			
;	POP	SI
		ret


;------------------------------------------------------------------------------
;*******************************************************************************
;*FUNCTION
;* StrSearchEE
;*
;*DESCRIPTION
;*
;*Routine to search for one string within another
;*y=pointer to the destination string (1)
;*z=pointer to the source string(2)
;*
;*
;*
;*******************************************************************************

StrSearchEE:
		;push	yl
		;push	yh
		mov	xl,yl			;Save z in x
		mov	xh,yh
		
		
StrSearchEEL:
		ld	tempchar,y		;Load byte from source string
		lpm				;Load byte from destination string
		;cp	r0,tempchar		;if r0[z]=tempchar[y] then 
		cp	tempchar,r0		;if tempchar[z]=r0[y] then 
		breq	StrSearchEE1
		
		mov	yl,xl			;else
		mov	yh,xh			;Pointer to beginning of source string
		sbiw	zl,1
		ld	tempchar,y
		lpm		
		;cp	r0,tempchar
		cp	tempchar,r0
		breq	StrSearchEE2
StrSearchEE01:	
		adiw	zl,1			;Increase y		
						;end else
StrSearchEE0:
		adiw	zl,1			;Increase y
		lpm				;ld	tempchar,y
		;tst	tempchar		;If tempchar=0 then exit
		tst	r0			;If r0=0 then exit	
		breq	StrSearchEENosucess
		rjmp	StrSearchEEL

StrSearchEE1:
		adiw	yl,1			;Increase z
		ld	tempchar,y	
		;tst	r0			;If r0=0 then exit
		tst	tempchar		;If tempchar=0 then exit
		breq	StrSearchEEExit		;(a match was found)
		rjmp	StrSearchEE0	
		
StrSearchEE2:
		adiw	yl,1			;Increase z
		rjmp	StrSearchEE01

StrSearchEENosucess:
		clr	zl
		clr	zh
		rjmp	StrSearchEEExit


StrSearchEEExit:
		;pop	yh
		;pop	yl	

		ret

;------------------------------------------------------------------------------
;*******************************************************************************
;*FUNCTION
;* StrSearchEEP
;*
;*DESCRIPTION
;*
;*Routine to search for one string within another
;*y=pointer to the destination string (1)
;*z=pointer to the source string(2)
;*
;*
;*
;*******************************************************************************

StrSearchEEP:
		;push	yl
		;push	yh
		mov	xl,yl			;Save z in x
		mov	xh,yh
		
		
StrSearchEEPL:
;		ld	tempchar,y		;Load byte from source string
		ld	r0,y			;Load byte from source string
		rcall	EERead
;		lpm				;Load byte from destination string
		cp	r0,tempchar		;if r0[z]=tempchar[y] then 
;		cp	tempchar,r0		;if tempchar[z]=r0[y] then 
		breq	StrSearchEEP1
		
		mov	yl,xl			;else
		mov	yh,xh			;Pointer to beginning of source string
		sbiw	zl,1
;		ld	tempchar,y
		ld	r0,y
;		lpm		
		rcall	EERead
		cp	r0,tempchar
;		cp	tempchar,r0
		breq	StrSearchEEP2
StrSearchEEP01:	
		adiw	zl,1			;Increase y		
						;end else
StrSearchEEP0:
		adiw	zl,1			;Increase y
;		lpm				;ld	tempchar,y
		rcall 	EERead
		tst	tempchar		;If tempchar=0 then exit
;		tst	r0			;If r0=0 then exit	
		breq	StrSearchEEPNosucess
		rjmp	StrSearchEEPL

StrSearchEEP1:
		adiw	yl,1			;Increase z
;		ld	tempchar,y	
		ld	r0,y
		tst	r0			;If r0=0 then exit
;		tst	tempchar		;If tempchar=0 then exit
		breq	StrSearchEEPExit		;(a match was found)
		rjmp	StrSearchEEP0	
		
StrSearchEEP2:
		adiw	yl,1			;Increase z
		rjmp	StrSearchEEP01

StrSearchEEPNosucess:
		clr	zl
		clr	zh
		rjmp	StrSearchEEPExit


StrSearchEEPExit:
		;pop	yh
		;pop	yl	

		ret

;------------------------------------------------------------------------------


;*******************************************************************************
;*FUNCTION
;* Itoa
;*
;*DESCRIPTION
;*Convert a 16-bit integer to a nullterminated string (min 6byte)
;*
;*Input parameter  z=Pointer to number to be converted (2byte)
;*Output parameter x=Pointer to the string with the converted integer
;*
;*******************************************************************************
Itoa:
		clc
		ld	temp0,z+		;high load nuber to be converted
		ld	temp1,z			;low
		mov	zl,temp1
		mov	zh,temp0
		ldi	temp1,0			;init digit counter
		ldi	temp0,4			;init exponent counter
		ldi	yl,16			;load exponent 10000
		ldi	yh,39	
itoal:
		inc	temp1			;increase digit counter
		sub	zl,yl			;Subtract low bytes
		sbc	zh,yh			;Subtract high byte with carry
		brcc	itoal
		
		

itoa1xxxx:
		clc
		dec	temp1			;decrease digit counter
		add	zl,yl
		adc	zh,yh
		dec	temp0			;decrease exponent counter
		
		ldi	tempchar,48	
		add	tempchar,temp1
		st	x+,tempchar		;store digit
		ldi	temp1,0			;clear exponent counter
		cpi	temp0,1
		breq	itoa10
		cpi	temp0,2
		breq	itoa100
		cpi	temp0,3
		breq	itoa1000
			
		rjmp	itoa0
		
;------------------------------------------------------


itoa1000:
		ldi	yl,232
		ldi	yh,3
		rjmp	itoal
itoa100:	
		ldi	yl,100
		ldi	yh,0
		rjmp	itoal
		
itoa10:		
		ldi	yl,10 
		ldi	yh,0
		rjmp	itoal
itoa0:		
		mov	temp1,zl
		ldi	tempchar,48
		add	tempchar,temp1
		st	x+,tempchar
		ldi	tempchar,0
		st	x,tempchar		;store last digit
		ret
;-----------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  Delay
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*Input: DelayV
;************************************************************************************
Delay:
		push	temp0
		push	temp1
		push	tempchar
		;ldi	temp0,0x20    ;0x30		;load counter
		lds	temp0,DelayV
delayl00:	
		;ldi	temp1,0x50		;load counter
		lds	temp1,DelayV+1
		dec	temp0			;decrease counter
delayl01:
		;ldi	tempchar,0x01;0x90		;
		lds	tempchar,DelayV+2
		dec	temp1			;decrease counter
delayl02:					;	
		wdr				;watchdog reset
		dec	tempchar		;
		tst	tempchar		;
		brne	delayl02		;
		tst	temp1
		brne	delayl01
		tst	temp0
		brne	delayl00
		pop	tempchar
		pop	temp1
		pop	temp0
		ret

;-------------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  ShortDelay
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*Input:
;************************************************************************************
ShortDelay:;----Set Delay Value---------------------
		push	tempchar
		ldi	tempchar,0x20
		sts	DelayV,tempchar
		ldi	tempchar,0x50
		sts	DelayV+1,tempchar
		ldi	tempchar,0x01
		sts	DelayV+2,tempchar
		
		rcall	Delay
		pop	tempchar
		ret
;-----------------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  LongDelay
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*Input:
;************************************************************************************
LongDelay:;----Set Delay Value---------------------
		push	tempchar
		ldi	tempchar,0xff
		sts	DelayV,tempchar
		ldi	tempchar,0xff
		sts	DelayV+1,tempchar
		ldi	tempchar,0x8f
		sts	DelayV+2,tempchar
		rcall	Delay
		pop	tempchar
		ret
;--------------------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  DsClock:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*input temp0=protocol register
;*input x=pointer to the clock return parameter (4byte)
;************************************************************************************
DsClock:
	
		;----Set Delay Value---------------------
		ldi	temp1,0x20
		sts	DelayV,temp1
		ldi	temp1,0x50
		sts	DelayV+1,temp1
		ldi	temp1,0x01
		sts	DelayV+2,temp1
		;------------------------------------------
		adiw	xl,3			;increase iput pointer
		sbi	DDRB,0
		sbi	DDRB,1
		sbi	DDRB,2
		sbi	portb,0			;set portb bit0=1 (reset)
		cbi	portb,2			;set portb bit2=0 (clock)
		ldi	temp1,8			;load counter
		
DsClockl01:		
	;-------------Send  bits to protocol register (DS1603)-------------------	
		ror	temp0
		brcs	DsClock03
		cbi	portb,1			;set portb bit1=0  (protocolregister)

DsClock00:	
	
		dec	temp1			;decrease counter
		cbi	portb,2			;set portb bit2=0  (clock)
		rcall	Delay			;delay
		sbi	portb,2			;set portb bit2=1  (clock)
		rcall	Delay			;delay
		tst	temp1
		brne	DsClockl01
	     ;----------------------------------------------	
		rcall	Delay			;delay
		cbi	portb,1			;set portb bit1=0  
		rcall	Delay			;delay
	     ;---------------------------------------------	
		cbi	DDRB,1			;Config pin 1 as an input
		sbi	portb,0			;reset
		cbi	portb,2			;clock
		ldi	temp1,32		;Load counter
		ldi	tempchar,0		;clear tempchar
	;-----------Read data-bits from DS1603--------------------------------------
DsClockl02:		
		rcall	Delay			;delay
		cbi	portb,2			;set portb bit2=0  (clock)
		rcall	Delay			;delay
		sbi	portb,2			;set portb bit2=1  (clock)
		in	temp0,pinb		;read portb
		clc				;clear carry
		andi	temp0,2			;get bit 1
		brne	DsClock01
		rjmp	DsClock02
DsClock01:
		sec				;set carry
DsClock02:		
		ror	tempchar		;ror carry into tempchar
		dec	temp1			;decrease counter
		cpi	temp1,24
		breq	DsClock04
		cpi	temp1,16
		breq	DsClock04
		cpi	temp1,8
		breq	DsClock04		
		cpi	temp1,0
		brne	DsClockl02
	    ;-----------Store Clock high word (bit25-32)-----------------	
	    	st	x,tempchar		;store last byte  
		cbi	portb,0			;reset
		ret				;exit
		
DsClock03:  ;----------Set portb bit0=1----------------------
		sbi	portb,1			;set portb bit1=1  (protocolregister)	
		clc		
		rjmp	DsClock00

DsClock04:  ;--Store Clock byte low word(bit1-8),(bit9-16) and high word (17-24),(25-32)---	
		st	x,tempchar		;store byte
		sbiw	xl,1			
		ldi	tempchar,0		;clear tempchar
		rjmp	DsClockl02

;---------------------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  ClearTempBuf:
;*
;*DESCRIPTION Fill TempBuf with 0
;*
;*PARAMETER
;*
;*
;************************************************************************************

ClearTempBuf:
		push	temp1
		clr	temp1		
		ldi	yl,low(TempBuf)			;Load TempBuf pointer
		ldi	yh,high(TempBuf)
		clr	tempchar			;Clear tempchar
ClearTempBuf1:
		st	y+,tempchar
		inc	temp1
		cpi	temp1,BUFSIZE
		brne	ClearTempBuf1
		pop	temp1
		ret
;---------------------------------------------------------------------------------------------
;---------------------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  ClearBuf:
;*
;*DESCRIPTION Fill Buf with 0
;*
;*PARAMETER
;*
;*
;************************************************************************************

ClearBuf:
		;push	temp1
		;clr	temp1		
		;ldi	yl,low(Buf)			;Load Buf pointer
		;ldi	yh,high(Buf)
		;clr	tempchar			;Clear tempchar
	
ClearBuf1:
		rcall 	Rs232GetByte
		cpi	tempchar,0     
		brne	ClearBuf1
		;st	y+,tempchar
		;inc	temp1
		;cpi	temp1,BUFSIZE
		;brne	ClearBuf1
		;pop	temp1
		ret
;---------------------------------------------------------------------------------------------

;---------------------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  ClearSuartBuf:
;*
;*DESCRIPTION Fill SuartBuf with 0
;*
;*PARAMETER
;*
;*
;************************************************************************************

ClearSuartBuf:
		push	temp1
		clr	temp1		
		ldi	yl,low(SUARTbuf)		;Load SUARTbuf pointer
		ldi	yh,high(SUARTbuf)
		clr	tempchar			;Clear tempchar
ClearSuartBuf1:
		st	y+,tempchar
		inc	temp1
		cpi	temp1,BUFSIZE
		brne	ClearSuartBuf1
		pop	temp1
		ret
;---------------------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  MoveString:
;*
;*DESCRIPTION Moves string from source positon to target positon
;*            z points at source positon and y points at target positon
;*	      Sorce string must be null terminated.
;*
;*PARAMETER
;*  z source pointer, y target pointer
;*
;*
;************************************************************************************

MoveString:
		ld	tempchar,z+
		st	y+,tempchar
		cpi	tempchar,0
		brne	MoveString
		ret
;---------------------------------------------------------------------------------------------

;---------------------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  MFTR:
;*
;*DESCRIPTION Moves data from flash memory to ram memory
;*            z points at source positon and y points at target positon
;*	      Sorce string must be null terminated.
;*
;*PARAMETER
;*  z source pointer, y target pointer
;*
;*
;************************************************************************************

MFTR:
		lpm	
		adiw	zl,1
		st	y+,r0
		mov	tempchar,r0
		cpi	tempchar,0
		brne	MFTR
		ret
;---------------------------------------------------------------------------------------------

;************************************************************************************
;*FUNCTION
;*  PhoneInit:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************

PhoneInit:
		;----Set echo off------------------------
		ldi 	zl,low(2*ate0)				;Load strinpointer
		ldi 	zh,high(2*ate0)
		rcall	PrintStr
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		rcall	LongDelay
		;----Set Delay Value---------------------
		ldi	temp0,0x20
		sts	DelayV,temp0
		ldi	temp0,0x50
		sts	DelayV+1,temp0
		ldi	temp0,0x01
		sts	DelayV+2,temp0
		;----------------------------------------
		clr	temp0					;Init counter
	
PhoneInit0:
		;rcall	ClearBuf
		;ldi	rxhead,0
		;ldi	rxtail,0
		clr	tempchar
		sts	TempBuf,tempchar
		cpi	temp0,0
		breq	PhoneInit_ate0
		cpi	temp0,1
		breq	PhoneInit_atcicb
		cpi	temp0,2
		breq	PhoneInit_ats0;PhoneInit_atcpin
		cpi	temp0,3
		breq	PhoneInit_atcpin ;PhoneInit_ats0
		cpi	temp0,4
		breq	PhoneInit_atcpa;PhoneInit_atcmgd
		cpi	temp0,5
		breq	PhoneInit_atcmgd
		cpi	temp0,6
		breq	PhoneTinitExit
		rjmp	PhoneInit1
PhoneTinitExit: 
		rjmp	PhoneInitExit	
;----------------------------------------------------------------------------------			
PhoneInit_atcpa:;-----------------------------------------------------
		ldi 	zl,low(2*atcpina)			;Load strinpointer
		ldi 	zh,high(2*atcpina)
		rcall	PrintStr
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
			;----Set Delay Value---------------------
	;	ldi	tempchar,0xff
	;	sts	DelayV,tempchar
	;	ldi	tempchar,0xff
	;	sts	DelayV+1,tempchar
	;	ldi	tempchar,0xff
	;	sts	DelayV+2,tempchar
		;----------------------------------------
		;rcall	Delay
		rcall	LongDelay
		rcall	ShortDelay
			;----Set Delay Value---------------------
	;	ldi	tempchar,0x20
	;	sts	DelayV,tempchar
	;	ldi	tempchar,0x50
	;	sts	DelayV+1,tempchar
	;	ldi	tempchar,0x01
	;	sts	DelayV+2,tempchar
		;----------------------------------------
		
		rjmp	PhoneInit1
PhoneInit_ats0:;-----------------------------------------------
		ldi 	zl,low(2*ats01)				;Load strinpointer
		ldi 	zh,high(2*ats01)
		rcall	PrintStr
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
			;----Set Delay Value---------------------
		
		;----------------------------------------
		;rcall	Delay
		rcall	LongDelay
		rcall	ShortDelay
		
		rjmp	PhoneInit1
PhoneInit_atcpin:;-----------------------------------------------
		ldi 	zl,low(2*atcpin)			;Load strinpointer
		ldi 	zh,high(2*atcpin)
		rcall	PrintStr
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		rcall	LongDelay
		rcall	ShortDelay
		;rcall	Delay
		rjmp	PhoneInit1
PhoneInit_ate0:;----------Print ate0----------------------------
		ldi 	zl,low(2*ate0)				;Load strinpointer
		ldi 	zh,high(2*ate0)
		rcall	PrintStr
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)			
				;----Set Delay Value---------------------
		;ldi	tempchar,0xff
		;sts	DelayV,tempchar
		;ldi	tempchar,0xff
		;sts	DelayV+1,tempchar
		;ldi	tempchar,0xff
		;sts	DelayV+2,tempchar
		;----------------------------------------
		rcall	LongDelay
		rcall	ShortDelay
			;----Set Delay Value---------------------
		;ldi	tempchar,0x20
		;sts	DelayV,tempchar
		;ldi	tempchar,0x50
		;sts	DelayV+1,tempchar
		;ldi	tempchar,0x01
		;sts	DelayV+2,tempchar
		;----------------------------------------
		rjmp	PhoneInit1
PhoneInit_atcicb:;-----------------------------------------------
		ldi 	zl,low(2*atcicb)			;Load strinpointer
		ldi 	zh,high(2*atcicb)
		rcall	PrintStr
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		;rcall	Delay
		rcall	LongDelay
		rcall	ShortDelay
		rjmp	PhoneInit1
PhoneInit_atcmgd:;-----------------------------------------------
		ldi 	zl,low(2*atcmgd)			;Load strinpointer
		ldi 	zh,high(2*atcmgd)
		rcall	PrintStr
		;rcall	Delay
		ldi	yl,low(TempBuf)
		ldi	yh,high(TempBuf)
		rcall	LongDelay
		rcall	ShortDelay
		rjmp	PhoneInit1													
;---------------------------------------------------------------------------------
PhoneInit1:	
		rcall	Delay	
		rcall 	Rs232GetByte				;Get char
		;cpi	tempchar,13
		;breq	PhoneInit1;PhoneInit1
		cpi	tempchar,10
		breq	PhoneInit2
		cpi	tempchar,0				
		breq 	PhoneInit1
		st	y+,tempchar
		rjmp	PhoneInit1
PhoneInit2:
		clr	tempchar
		st	y+,tempchar
		;ldi	rxhead,0
		;ldi	rxtail,0
;----------------Test--------------------------------------------------------

;---------------Test for OK--------------------------------------------------
		ldi 	zl,low(2*OK)			;Load stringpointer source
		ldi 	zh,high(2*OK)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		breq	PhoneInit3 ;PhoneInitExit     ;TestforClock
		rjmp	ActionOk
;---------------Test for ERROR--------------------------------------------------				
PhoneInit3:
		ldi 	zl,low(2*Error)			;Load stringpointer source
		ldi 	zh,high(2*Error)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		breq	PhoneInit4 ;PhoneInitExit     ;TestforClock
		rjmp	ActionError				
		;rcall	Rs232GetByte				;Get char
		;cpi	tempchar,13
		;brne	PhoneInit3
;---------------Test for +CPIN: READY------------------------------------------------------		
PhoneInit4:
		ldi 	zl,low(2*cpiready)			;Load stringpointer source
		ldi 	zh,high(2*cpiready)
		ldi	yl,low(TempBuf)			;Load stringpointer destination
		ldi	yh,high(TempBuf)
		rcall	StrSearch
		tst	yl	
		breq	PhoneTinit0 ;PhoneInitExit     ;TestforClock
		rjmp	ActionCpinready	
		;rcall	Rs232GetByte				;Get char
		;cpi	tempchar,10
		;brne	PhoneInit4		
		;inc	temp0					;Increase counter
		;rjmp	PhoneInit0
PhoneTinit0:
		;rcall	Rs232GetByte
		;cpi	tempchar,0
		;brne	PhoneTinit0
		clr	tempchar
		sts	TempBuf,tempchar
		rjmp	PhoneInit0		
PhoneInitExit:	
		ret
		


	

	
	
;---------------------Phone_Action--------------------------------------------------------		
ActionOk:
		inc	temp0					;Increase counter
		clr	tempchar
		sts	TempBuf,tempchar
		;rcall	ClearTempBuf
		rjmp	PhoneInit0					
;------------------------------------------------------------------------------------------
ActionError:
		clr	tempchar
		sts	TempBuf,tempchar
		;rcall	ClearTempBuf
		rjmp	PhoneInit0
;------------------------------------------------------------------------------------------
ActionCpinready:
		inc	temp0
		clr	tempchar
		sts	TempBuf,tempchar
		rjmp	PhoneInit0		
;-------------------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  PhoneInit00:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************

PhoneInit00:
		;----To start modem set pin 16 high (power ON)--------------------- 
		sbi	porta,0
		rcall	LongDelay
		rcall	LongDelay
		rcall	LongDelay
		cbi	porta,0
		;----Set echo off------------------------
		ldi 	zl,low(2*ate0)				;Load strinpointer
		ldi 	zh,high(2*ate0)
		rcall	PrintStr
		rcall	LongDelay
		rcall	LongDelay
		;---Set incoming call type to data------------
		ldi 	zl,low(2*atcicb)			;Load strinpointer
		ldi 	zh,high(2*atcicb)
		rcall	PrintStr
		rcall	LongDelay	
		;--Set pin code--------------------------------
		ldi 	zl,low(2*atcpin)				;Load strinpointer
		ldi 	zh,high(2*atcpin)
		rcall	PrintStr
		rcall	LongDelay
		;rcall	LongDelay
		;rcall	LongDelay
		;--Automatic answer after 2 rings-------------
		;ldi 	zl,low(2*ats01)				;Load strinpointer
		;ldi 	zh,high(2*ats01)
		
		;--No automatic answer---------------------------------------
		ldi 	zl,low(2*ats00)				;Load strinpointer
		ldi 	zh,high(2*ats00)
		rcall	PrintStr
		rcall	LongDelay
		rcall	LongDelay
		;--Clear all SMS----------------------------
		;ldi 	zl,low(2*atcmgd)			;Load strinpointer
		;ldi 	zh,high(2*atcmgd)
		;rcall	PrintStr
		;rcall 	Rs232GetByte	
		ret
		
;************************************************************************************
;*FUNCTION
;*  '
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************

SendSms:
		;sbi	PORTD,6
		;----------------------------
		ldi 	zl,low(2*atcmgf1)				;Load strinpointer
		ldi 	zh,high(2*atcmgf1)
		rcall	PrintStr
		rcall	LongDelay
		;trcall	LongDelay
		;--Set nuber to Sms-station-------------------
		;ldi 	zl,low(2*atcsca)			;Load strinpointer
		;ldi 	zh,high(2*atcsca)
		;rcall	PrintStr
		;rcall	LongDelay	
		;----------------------------------
		ldi 	zl,low(2*atcmgs)				;Load strinpointer
		ldi 	zh,high(2*atcmgs)
		rcall	PrintStr
		ldi	zl,low(phoneno)
		ldi	zh,high(phoneno)
		rcall	PrintBuf
		ldi	tempchar,34
		rcall	PutcHardUart
		;rcall	LongDelay
		ldi	tempchar,13
		rcall 	PutcHardUart
		rcall	LongDelay
		;---------------
	
		;--Write SMS-message---------------------------
		ldi 	zl,low(TempBuf)			;Load strinpointer
		ldi 	zh,high(TempBuf)
		rcall	PrintBuf
		rcall	LongDelay	;t
		ldi	tempchar,26
		rcall	PutcHardUart
		rcall	LongDelay
		rcall	LongDelay
		;----------------------------------
		ldi 	zl,low(2*atcmgd0)				;Load strinpointer
		ldi 	zh,high(2*atcmgd0)
		;ldi 	zl,low(2*atcmgd)				;Load strinpointer
		;ldi 	zh,high(2*atcmgd)
		rcall	PrintStr
		rcall	LongDelay
		;cbi	PORTD,6
		;---------------
		rcall	LongDelay
		rcall	LongDelay					;test
		rcall	LongDelay
		
		;ldi 	zl,low(2*atcmgl)				;test Load strinpointer
		;ldi 	zh,high(2*atcmgl)				;test
		;rcall	PrintStr
		;tldi 	zl,low(2*atcmgr2)			;Load strinpointer
		;tldi 	zh,high(2*atcmgr2)
		;trcall	PrintStr
		;cbi	PORTD,6
		;trcall	LongDelay
		;trcall	LongDelay
		;trcall	LongDelay
		;trcall	LongDelay
		rcall	ReadSms					;test
		
		ret
		
;************************************************************************************
;*FUNCTION
;*  SendSms:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************

SendSms0:
		;sbi	PORTD,6
		;----------------------------

		;----------------------------
		ldi 	zl,low(2*atcmgf1)				;Load strinpointer
		ldi 	zh,high(2*atcmgf1)
		rcall	PrintStr
		rcall	LongDelay
		;trcall	LongDelay
		;--Set nuber to Sms-station-------------------
		;ldi 	zl,low(2*atcsca)			;Load strinpointer
		;ldi 	zh,high(2*atcsca)
		;rcall	PrintStr
		;rcall	LongDelay	
		;----------------------------------
		ldi 	zl,low(2*atcmgs)				;Load strinpointer
		ldi 	zh,high(2*atcmgs)
		rcall	PrintStr
		ldi	zl,low(phoneno)
		ldi	zh,high(phoneno)
		rcall	PrintBuf
		ldi	tempchar,34
		rcall	PutcHardUart
		;rcall	LongDelay
		ldi	tempchar,13
		rcall 	PutcHardUart
		rcall	LongDelay
		;---------------
	
		;--Write SMS-message---------------------------
		ldi 	zl,low(TempBuf)			;Load strinpointer
		ldi 	zh,high(TempBuf)
		rcall	PrintBuf
		rcall	LongDelay	;t
		ldi	tempchar,26
		rcall	PutcHardUart
		rcall	LongDelay
		rcall	LongDelay
		;----------------------------------
		ldi 	zl,low(2*atcmgd0)				;Load strinpointer
		ldi 	zh,high(2*atcmgd0)
		;ldi 	zl,low(2*atcmgd)				;Load strinpointer
		;ldi 	zh,high(2*atcmgd)
		rcall	PrintStr
		rcall	LongDelay
		;cbi	PORTD,6
		;---------------
		rcall	LongDelay
		rcall	LongDelay					;test
		rcall	LongDelay
		
		;ldi 	zl,low(2*atcmgl)				;test Load strinpointer
		;ldi 	zh,high(2*atcmgl)				;test
		;rcall	PrintStr
		;tldi 	zl,low(2*atcmgr2)			;Load strinpointer
		;tldi 	zh,high(2*atcmgr2)
		;trcall	PrintStr
		;cbi	PORTD,6
		;trcall	LongDelay
		;trcall	LongDelay
		;trcall	LongDelay
		;trcall	LongDelay
		rcall	ReadSms					;test
		rcall	LongDelay
		rcall	LongDelay
		rcall	LongDelay
;		rcall	LongDelay
;		rcall	LongDelay
;		rcall	LongDelay
;		rcall	LongDelay
;		rcall	LongDelay
;		rcall	LongDelay
;		rcall	ClearCrFlag		;Clear CrFlag
;		rcall 	ClearBuf
;		rcall	ClearTempBuf
;		ldi	rxhead,0
;		ldi	rxtail,0
		ret		
;---------------------------------------------------------------------------------------------

;************************************************************************************
;*FUNCTION
;*  lampTest:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************
						
						
lampTest:		
		;ldi	zl,low(TempBuf)
		;ldi	zh,high(TempBuf)
		;ld	tempchar,z		;Load byte from program memory into tempchar
		
		lds	tempchar,TempBuf+3
		;mov	u_data,tempchar
	        ;rcall	putc
		;push	tempchar
		lds	temp0,lampStatus	;Load lampStatus
		;mov	u_data,temp0
		;rcall	putc
		cp	temp0,tempchar
		breq	lampTestExit
		;clr	temp0
		;sts	pollFlag,temp0
		;ldi	temp0,1
		;sts	SmsFlag,temp0
		;-------------------------------------------------------
		;----Copy TempPhoneno to Phoneno--
		ldi	zl,low(TempPhoneno)
		ldi	zh,high(TempPhoneno)
		ldi	yl,low(Phoneno)
		ldi	yh,high(Phoneno)
		;adiw	zl,1
		rcall	MoveString
		;-----------------------------------
		;-------------------------------------------
		rcall	SendSms
		;;clr	temp0
		;;sts	SmsFlag,temp0		;Set SmsFlag off
		;lds	temp0,Bflags
		;cbr	temp0,2
		;sts	Bflags,temp0
		rcall	ClearSmsFlag		;Clear SmsFlag
		;--------------------------------------------------------
lampTestExit:
		;pop	tempchar		
		lds	tempchar,TempBuf+3
		sts	lampStatus,tempchar	;Save lampStatus
		

		clr	temp0			 
		sts	Count,temp0		;clear time counter
		sts	TempBuf,temp0
		rcall	timer1_init	
		ret
		
;************************************************************************************
;*FUNCTION
;*  ReadSms:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************
						
						
ReadSms:
		lds	temp0,SmsCount
		cpi	temp0,1 
		breq	ReadSms1
		cpi	temp0,2
		breq	ReadSms2
		cpi	temp0,3
		breq	ReadSms3
		cpi	temp0,4
		breq	ReadSms4
		cpi	temp0,5
		breq	ReadSms5
		clr	temp0
		sts	SmsCount,temp0
		rjmp	ReadSmsExit0
		
		
ReadSms1:	
		
		ldi 	zl,low(2*atcmgr1)			;Load strinpointer
		ldi 	zh,high(2*atcmgr1)
		rjmp	ReadSmsExit
ReadSms2:	
		ldi 	zl,low(2*atcmgr2)			;Load strinpointer
		ldi 	zh,high(2*atcmgr2)
		rjmp	ReadSmsExit
ReadSms3:
		ldi 	zl,low(2*atcmgr3)			;Load strinpointer
		ldi 	zh,high(2*atcmgr3)
		rjmp	ReadSmsExit
ReadSms4:
		ldi 	zl,low(2*atcmgr4)			;Load strinpointer
		ldi 	zh,high(2*atcmgr4)
		rjmp	ReadSmsExit
ReadSms5:
		ldi 	zl,low(2*atcmgr5)			;Load strinpointer
		ldi 	zh,high(2*atcmgr5)
		rjmp	ReadSmsExit		
		
		
ReadSmsExit:			
		
		rcall	PrintStr
		lds	temp0,SmsCount
		inc	temp0
		sts	SmsCount,temp0
		clr	temp0
		mov	rxhead,temp0
		mov	rxtail,temp0
		rcall	LongDelay
		;rcall	LongDelay
		;rcall	LongDelay
		;rcall	LongDelay
ReadSmsExit0:
		ret


;************************************************************************************
;*FUNCTION
;*  ReadSms:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************
						
						
ReadSms00:	
		
		ldi 	zl,low(2*atcmgr)			;Load strinpointer
		ldi 	zh,high(2*atcmgr)
		rcall	PrintStr
		lds	tempchar,SmsNo
		;ldi	tempchar,13
		rcall	PutcHardUart
		lds	tempchar,SmsNo+1
		cpi	tempchar,13 
		breq	ReadSms00Exit
		ldi	tempchar,13
		;rcall	LongDelay;t
		
		
ReadSms00Exit:	
		rcall	PutcHardUart		
		lds	temp0,SmsCount
		inc	temp0
		sts	SmsCount,temp0
		clr	temp0
		mov	rxhead,temp0
		mov	rxtail,temp0
		rcall	LongDelay
		ret
		
		
;************************************************************************************
;*FUNCTION
;*  ReadSoftUart:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************

ReadSoftUart:
		clr	temp0
		sts	TimeOutCount,temp0
		sts	TimeOutCount+1,temp0
		sts	TimeOutCount+2,temp0
		
		;cpi	flag,1	
		;brne	ReadSoftUart

ReadSoftUart00: ;------TimeOut loop--------------------------------
		wdr					;watchdog reset
		cpi	flag,1
		breq	ReadSoftUart0
		inc	temp0
		cpi	temp0,255
		brne	ReadSoftUart00
		lds	temp1,TimeOutCount
		inc	temp1
		sts	TimeOutCount,temp1
		cpi	temp1,255
		brne	ReadSoftUart00
		
		
		lds	temp1,TimeOutCount+1
		inc	temp1
		sts	TimeOutCount+1,temp1
		cpi	temp1,255
		brne	ReadSoftUart00
		
		rjmp	ReadSoftUart5
		;-----End TimeOutLoop---------------------------------
;wint0:
ReadSoftUart0:
		lds	temp1,BL15
		out	TCNT0,temp1 				; preset T/C0 to 1.5 bit lengths
		ldi	temp1,1<<TOIE0
		out	TIFR,temp1				; clear T/C0 overflow flag
		out	TIMSK,temp1				; enable T/C0 overflow Interrupt
		clr	bit_cnt					; clear bit counter
;wgetc1:		
ReadSoftUart1:
		sbrs	u_stat,SRXC				; wait for Receive Complete
		;rjmp	wgetc1
		rjmp	ReadSoftUart1
		cbr	u_stat,1<<SRXC				; clear SRXC
		cpi	u_data,0				;test to ingnore null
		breq	ReadSoftUart3				;test
		lds	temp1,SUARTbuf				;load buf length counter
		inc	temp1					;increase counter
		sts	SUARTbuf,temp1				;store counter
		ldi	zl,low(SUARTbuf)
		ldi	zh,high(SUARTbuf)
		adc	zl,temp1
		;brcc	wgetcj
		brcc	ReadSoftUart2
		inc	zh
;wgetcj:		
ReadSoftUart2:		
		st	z,u_data				;store char in buffer		
		;breq	wflag
		breq	ReadSoftUart3
		cpi	u_data,13    				;'cr' 
		;breq	wgetc2
		breq	ReadSoftUart4
		cpi	u_data,3				;ETX
		breq	ReadSoftUart4				
		;cpi	u_data,0				;test to ingnore null
		;brne	ReadSoftUart3				;test
		;dec	temp1					;test
		;ldi	u_data,' '				;test
		
;wflag:
ReadSoftUart3:
		ldi	flag,0
		;cbi	PORTD,6					;RTS Off
		;rjmp	main
		rjmp	ReadSoftUart
;wgetc2:
ReadSoftUart4:	
		ldi	zl,low(SUARTbuf)			;load stringpointer
		ldi	zh,high(SUARTbuf)
		inc	temp1
		adc	zl,temp1
		;brcc	wgetcj0
		brcc	ReadSoftUart5
		inc	zh





;wgetcj0:
ReadSoftUart5:
		clr	temp0					;terminate buffer	
		st	z,temp0
		ldi	zl,low(SUARTbuf)
		ldi	zh,high(SUARTbuf)
		adiw	zl,1
		clr 	temp0
		sts	SUARTbuf,temp0				;clear counter
		ldi	flag,0
		cbi	PORTD,6	




		ret
		
;************************************************************************************
;*FUNCTION
;*  ReadPacketSoftUart:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************

ReadPacketSoftUart:
		clr	temp0
		sts	TimeOutCount,temp0
		sts	TimeOutCount+1,temp0
		sts	TimeOutCount+2,temp0
		
		;cpi	flag,1	
		;brne	ReadPacketSoftUart
ReadPacketSoftUart00: ;------TimeOut loop--------------------------------	
		cpi	flag,1
		breq	ReadPacketSoftUart0
		;push	temp0
		;push	temp1
		;in	temp1,pinc			;read portc
		;andi	temp1,63;15			;get bit 0,1,2,3,4,5  1+2+4+8+16+32=63, If not 63 then Button 
		;ldi	temp0,63;15
		;cpse	temp1,temp0
		;rjmp	ReadPacketSoftUartExit
		;pop	temp1
		;pop	temp0
		inc	temp0
		cpi	temp0,255
		brne	ReadPacketSoftUart00
		lds	temp1,TimeOutCount
		inc	temp1
		sts	TimeOutCount,temp1
		cpi	temp1,255
		brne	ReadPacketSoftUart00
		
		
		lds	temp1,TimeOutCount+1
		inc	temp1
		sts	TimeOutCount+1,temp1
		cpi	temp1,255
		brne	ReadPacketSoftUart00
		
		rjmp	ReadPacketSoftUart5
		;-----End TimeOutLoop---------------------------------
;wint0:
ReadPacketSoftUart0:
		lds	temp1,BL15
		out	TCNT0,temp1 				; preset T/C0 to 1.5 bit lengths
		ldi	temp1,1<<TOIE0
		out	TIFR,temp1				; clear T/C0 overflow flag
		out	TIMSK,temp1				; enable T/C0 overflow Interrupt
		clr	bit_cnt					; clear bit counter
;wgetc1:		
ReadPacketSoftUart1:
		sbrs	u_stat,SRXC				; wait for Receive Complete
		;rjmp	wgetc1
		rjmp	ReadPacketSoftUart1
		cbr	u_stat,1<<SRXC				; clear SRXC
		;cpi	u_data,0				;test to ingnore null
		;breq	ReadPacketSoftUart3				;test
		lds	temp1,SUARTbuf				;load buf length counter
		inc	temp1					;increase counter
		sts	SUARTbuf,temp1				;store counter
		ldi	zl,low(SUARTbuf)
		ldi	zh,high(SUARTbuf)
		adc	zl,temp1
		;brcc	wgetcj
		brcc	ReadPacketSoftUart2
		inc	zh
;wgetcj:		
ReadPacketSoftUart2:		
		st	z,u_data				;store char in buffer		
		breq	ReadPacketSoftUart3
		cp	tempchar,temp1
		;cpi	temp1,2
		;breq	ReadPacketSoftUartExit1			;test
		breq	ReadPacketSoftUart4
		;cpi	u_data,13    				;'cr' 
		;breq	ReadPacketSoftUart4
		;cpi	u_data,3				;ETX
		;breq	ReadPacketSoftUart4				
		;cpi	u_data,0				;test to ingnore null
		;brne	ReadSoftUart3				;test
		;dec	temp1					;test
		;ldi	u_data,' '				;test
		
;wflag:
ReadPacketSoftUart3:
		ldi	flag,0
		;cbi	PORTD,6					;RTS Off
		;rjmp	main
		rjmp	ReadPacketSoftUart
;wgetc2:
ReadPacketSoftUart4:	
		ldi	zl,low(SUARTbuf)			;load stringpointer
		ldi	zh,high(SUARTbuf)
		inc	temp1
		adc	zl,temp1
		;brcc	wgetcj0
		brcc	ReadPacketSoftUart5
		inc	zh





;wgetcj0:
ReadPacketSoftUart5:
		clr	temp0					;terminate buffer	
		st	z,temp0
		ldi	zl,low(SUARTbuf)
		ldi	zh,high(SUARTbuf)
		adiw	zl,1
		clr 	temp0
		sts	SUARTbuf,temp0				;clear counter
		ldi	flag,0
		cbi	PORTD,6	
		rjmp	ReadPacketSoftUartExit1	

ReadPacketSoftUartExit:
		lds	temp1,Count			;Load poll count
		cpi	temp1,3;50			
		brsh	ReadPacketSoftUartExit0
		rjmp	ReadPacketSoftUart00	
ReadPacketSoftUartExit0:
		
		ldi	flag,0
		cbi	PORTD,6
		clr	temp0
		sts	Count,temp0
		rjmp	main	
ReadPacketSoftUartExit1:		
		ret		

;************************************************************************************
;*FUNCTION
;*  SharpButton:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************						

SharpButton:

	;	sbi	PORTD,6	
		;-----Button_1--------
	;	sbis	pinc,0
	;	rjmp	SharpButton1
		;----Button_2----------
	;	sbis	pinc,1
	;	rjmp	SharpButton2
		;----Button_3----------
	;	sbis	pinc,2
	;	rjmp	SharpButton3
		;----Button_4----------
		sbis	pinc,3
		rjmp	SharpButton4
		;----Button_5----------
		sbis	pinc,4
		rjmp	SharpButton5
		;----Button_6----------
		sbis	pinc,5
		rjmp	SharpButton6
		rjmp	SharpButtonExit0

SharpButton1:
		rcall	Ds2406_pioaON
		rjmp	SharpButtonExit1
	;	ldi	temp0,2
	;	ldi	xl,low(LongWord)
	;	ldi	xh,high(LongWord)		;Load pointer to clock return value
	;	wdr
	;	rcall	DsClock
	;	wdr
		rjmp	SharpButton1_1
		
		
;		cbi	portb,sck
;		cbi	portb,mosi
;		cbi	portb,ss
		;ldi	tempchar,0x06
		;ldi	tempchar,130
		;ldi	tempchar,0x4
		;ldi	tempchar,146
		;ldi	tempchar,130
		;ldi	tempchar,0x02
;		ldi	tempchar,194
;		rcall	rw_spi
		;ldi	u_data,65
;		mov	u_data,tempchar
;		rcall	putc
		;rjmp	SharpButton1_1		
;		rcall	delay
;		ldi	tempchar,0x04
		;ldi	tempchar,20
;		rcall	rw_spi
;		mov	u_data,tempchar
;		rcall	putc
;		rcall	delay
;		rjmp	SharpButton1_1
;		ldi	tempchar,0x00
		;ldi	tempchar,16
;		rcall	rw_spi
;		mov	u_data,tempchar
;		rcall	putc
;		rjmp	SharpButton1_1
;		rcall 	Delay
;		ldi	tempchar,146
;		rcall	rw_spi
;		mov	u_data,tempchar
;		rcall	putc
;		rjmp	SharpButton1_1


	;	ldi	tempchar,0x00
	;	rcall	rw_spi
	;	ldi	tempchar,0x82
		;ldi	tempchar,0x00
		;ldi	tempchar,129
	;	rcall	rw_spi
	;	rjmp	SharpButton1_1
		
		
;		ldi 	zl,low(2*powr1)				;Load strinpointer to commandlist 
;		ldi 	zh,high(2*powr1)
		
		
;		rcall	PrintSoftUstr				;Send command to projector
;		rcall	ReadSoftUart				;Read projector respons

;		ldi	zl,low(2*ok)				;respons sharp
;		ldi	zh,high(2*ok)				;respons sharp
		
		
;		ldi	yl,low(SUARTbuf)			;Load stringpointer destination
;		ldi	yh,high(SUARTbuf)
;		adiw	yl,1
;		rcall	StrSearch
;		tst	yl					;Test for OK	
;		breq	SharpButton1_1
		;ldi 	zl,low(2*powr1)				;Load strinpointer
		;ldi 	zh,high(2*powr1)
		;rcall	PrintStr
		;ldi	yl,low(TempBuf)
		;ldi	yh,high(TempBuf)
		;sbi	PORTC,6;4				;Set led button 1 On
;		cbi	PORTC,7;5
;		rcall	LedFlashC6
;		sbi	PORTA,5					;Set led Source Button On
		rjmp	SharpButtonExit1	 
SharpButton1_1:		
		cbi	PORTC,6;4				;Set led button 1 Off 
		rjmp	SharpButtonExit1
		
SharpButton2:	
		rcall	Ds2406_pioaOFF
		rjmp	SharpButtonExit1
;		ldi 	zl,low(2*powr0)				;sharp
;		ldi 	zh,high(2*powr0)			;sharp
;		rcall	PrintSoftUstr				;toshiba
;		rcall	ReadSoftUart		
;		ldi	zl,low(2*ok)
;		ldi	zh,high(2*ok)
;		ldi	yl,low(SUARTbuf)			;Load stringpointer destination
;		ldi	yh,high(SUARTbuf)
;		adiw	yl,1
;		rcall	StrSearch
;		tst	yl					;Test for OK	
;		breq	SharpButton2_1
		;sbi	PORTC,7;5				;Set led button 1 On
;		clr	temp0
;		sts	Mcount,temp0				;clear Moute counter
;		cbi	PORTA,6					;Set led Mute Button Off
;		cbi	PORTC,6;4
;		cbi	PORTA,5					;Set led Source Button Off
;		rcall	LedFlashC7
		;		cbi	PORTA,5
		;		cbi	PORTA,6
		;ldi 	zl,low(2*powr0)				;Load strinpointer
		;ldi 	zh,high(2*powr0)
		;rcall	PrintStr
		;ldi	yl,low(TempBuf)
		;ldi	yh,high(TempBuf)
		rjmp	SharpButtonExit1	 
SharpButton2_1:		
		cbi	PORTC,7;5					;Set led button 1 Off 
		rjmp	SharpButtonExit1
	
	
SharpButton3:	
	
		;----------------Mute Button toggle On_Off-----------------------------------------------------------------			
		;lds	temp0,Mcount				;Load Mount counter
		;cpi	temp0,1
		;breq	SharpButton3_2

		;---------------End mute Button toggle---------------------------------------------------------------------
	
		;ldi	temp0,1
		;sts	Mcount,temp0				;increase Moute counter
		;rcall	ClearSuartBuf
		;ldi 	zl,low(2*muteOn)			;sharp xr-10x,xr-20x
		;ldi 	zh,high(2*muteOn)			;sharp xr-10x,xr-20x
		;rcall	PrintSoftUstr				;Send command to projector
		;rcall	ReadSoftUart				;Read projector respons
		;ldi	zl,low(2*ok)
		;ldi	zh,high(2*ok)
		;ldi	yl,low(SUARTbuf)			;Load stringpointer destination
		;ldi	yh,high(SUARTbuf)
		;adiw	yl,1
		;rcall	StrSearch
		;tst	yl					;Test for OK	
		;breq	SharpButton3_1
		;cbi	PORTA,5
		;rcall	LedFlashA6
		rjmp	SharpButtonExit1	 
SharpButton3_1:		
		cbi	PORTA,6					;Set led button 1 Off 
		rjmp	SharpButtonExit1
		
SharpButton3_2:
;		clr	temp0
;		sts	Mcount,temp0				;clear Moute counter
;		ldi 	zl,low(2*muteOff)			;sharp xr-10x,xr-20x
;		ldi 	zh,high(2*muteOff)			;sharp xr-10x,xr-20x
;		rcall	PrintSoftUstr				;Send command to projector
;		rcall	ReadSoftUart				;Read projector respons
;		ldi	zl,low(2*ok)
;		ldi	zh,high(2*ok)
;		ldi	yl,low(SUARTbuf)			;Load stringpointer destination
;		ldi	yh,high(SUARTbuf)
;		adiw	yl,1
;		rcall	StrSearch
;		tst	yl					;Test for OK	
;		breq	SharpButton3_3
;		cbi	PORTA,5
;		rcall	LedFlashA6
;		cbi	PORTA,6					;Set led Mute Button Off
;		sbi	PORTA,5					;Set led Source Button On
;		rjmp	SharpButtonExit1	 
SharpButton3_3:		
;		cbi	PORTA,6					;Set led button 1 Off 
;		rjmp	SharpButtonExit1
	
		
SharpButton4:		
		rcall	GetDs64BitCode
		rjmp	SharpButtonExit1
		
	;	ldi	xl,low(phoneno)
	;	ldi	xh,high(phoneno)
		
	;	ldi	tempchar,0x55
	;	st	x+,tempchar
	;	ldi	tempchar,0x12
	;	st	x+,tempchar
	;	ldi	tempchar,0x2f
	;	st	x+,tempchar
	;	ldi	tempchar,0x22
	;	st	x+,tempchar
	;	ldi	tempchar,0x73	
	;	st	x+,tempchar
	;	ldi	tempchar,0x00
	;	st	x+,tempchar
	;	ldi	tempchar,0x00
	;	st	x+,tempchar
	;	ldi	tempchar,0x00
	;	st	x+,tempchar
	;	ldi	tempchar,0x48
	;	st	x+,tempchar
	
	
	;	ldi	tempchar,0x55	;
	;	st	x+,tempchar
	;	ldi	tempchar,0x07	;
	;	st	x+,tempchar
	;	ldi	tempchar,0x00	
	;	st	x+,tempchar
	;	ldi	tempchar,0x0e	;
	;	st	x+,tempchar
		
	;	ldi	xl,low(phoneno)
	;	ldi	xh,high(phoneno)
		
	
	;	rcall	Ds2406_pioaON
		
		
		
		rjmp	SharpButtonExit1

		;-------------------Source Button toggle---------------------------------------------------------------------------
	;	lds	temp0,Scount				;Load source counter
	;	cpi	temp0,1
	;	breq	SharpButton4_2	
	;	cpi	temp0,2
	;	breq	SharpButton4_4	
		;----------------------End Button toggle--------------------------------------------------	
	;	ldi	temp0,1
	;	sts	Scount,temp0				;increase Scounter
		
	;	ldi 	zl,low(2*svideo)				;sharp xr-10x, xr-20x
	;	ldi 	zh,high(2*svideo)				;sharp xr-10x, xr-20x
	;	rcall	PrintSoftUstr				;Send command to projector
	;	rcall	ReadSoftUart				;Read projector respons
	;	ldi	zl,low(2*ok)
	;	ldi	zh,high(2*ok)
	;	ldi	yl,low(SUARTbuf)			;Load stringpointer destination
	;	ldi	yh,high(SUARTbuf)
	;	adiw	yl,1
	;	rcall	StrSearch
	;	tst	yl					;Test for OK	
	;	breq	SharpButton4_1
	;	cbi	PORTA,6
	;	rcall 	LedFlashA5
	;	rjmp	SharpButtonExit1	 
SharpButton4_1:		
	;	cbi	PORTA,5					;Set led button 1 Off 
	;	rjmp	SharpButtonExit1

SharpButton4_2:
	;	ldi 	zl,low(buttonFlag)			;Load strinpointer
	;	ldi 	zh,high(buttonFlag)
	;	rcall	EERead					;Get buttonFlag
	;	clr 	temp0
	;	sbrc	tempchar,0				;Skip if bit0 is cleared
	;	ldi	temp0,2					
	;	sts	Scount,temp0				;increase Scounter							
	;	ldi 	zl,low(2*rgb1)				;sharp xr-10x, xr-20x
	;	ldi 	zh,high(2*rgb1)				;sharp xr-10x, xr-20x
	;	rcall	PrintSoftUstr				;Send command to projector
	;	rcall	ReadSoftUart				;Read projector respons
	;	ldi	zl,low(2*ok)
	;	ldi	zh,high(2*ok)
	;	ldi	yl,low(SUARTbuf)			;Load stringpointer destination
	;	ldi	yh,high(SUARTbuf)
	;	adiw	yl,1
	;	rcall	StrSearch
	;	tst	yl					;Test for OK	
	;	breq	SharpButton4_3
	;	cbi	PORTA,6
	;	rcall 	LedFlashA5
	;	rjmp	SharpButtonExit1	 
SharpButton4_3:		
	;	cbi	PORTA,5					;Set led button 1 Off 
	;	rjmp	SharpButtonExit1


SharpButton4_4:
	;	clr	temp0
	;	sts	Scount,temp0				;increase Scounter							
	;	ldi 	zl,low(2*rgb2)				;sharp xr-10x, xr-20x
	;	ldi 	zh,high(2*rgb2)				;sharp xr-10x, xr-20x
	;	rcall	PrintSoftUstr				;Send command to projector
	;	rcall	ReadSoftUart				;Read projector respons
	;	ldi	zl,low(2*ok)
	;	ldi	zh,high(2*ok)
	;	ldi	yl,low(SUARTbuf)			;Load stringpointer destination
	;	ldi	yh,high(SUARTbuf)
	;	adiw	yl,1
	;	rcall	StrSearch
	;	tst	yl					;Test for OK	
	;	breq	SharpButton4_5
	;	cbi	PORTA,6
	;	rcall 	LedFlashA5
	;	rjmp	SharpButtonExit1	 
SharpButton4_5:		
	;	cbi	PORTA,5					;Set led button 1 Off 
	;	rjmp	SharpButtonExit1
					
		
SharpButton5:
		;rcall	GetDs64BitCode
	
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
		
		ldi	tempchar,0x55
		st	x+,tempchar
		ldi	tempchar,0x12
		st	x+,tempchar
		ldi	tempchar,0xa0
		st	x+,tempchar
		ldi	tempchar,0x1f
		st	x+,tempchar
		ldi	tempchar,0x73	
		st	x+,tempchar
		ldi	tempchar,0x00
		st	x+,tempchar
		ldi	tempchar,0x00
		st	x+,tempchar
		ldi	tempchar,0x00
		st	x+,tempchar
		ldi	tempchar,0xee
		st	x+,tempchar
	
	
		ldi	tempchar,0x55	;
		st	x+,tempchar
		ldi	tempchar,0x07	;
		st	x+,tempchar
		ldi	tempchar,0x00	
		st	x+,tempchar
		ldi	tempchar,0x0e	;
		st	x+,tempchar
		
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
		
	
		rcall	Ds2406_pioaON
		rjmp	SharpButtonExit1
	;	ldi	tempchar,0xff
	;	sts	DelayV,tempchar
	;	ldi	tempchar,0xff
	;	sts	DelayV+1,tempchar
	;	ldi	tempchar,0x4
	;	sts	DelayV+2,tempchar
	;	rcall	Delay
	;	ldi	tempchar,0
	;	sts	LongWord,tempchar
		;lds	tempchar,Svolume
	;	tst	tempchar
	;	breq	SharpButton5_2
	;	ldi	temp0,2
	;	sub	tempchar,temp0
		;sts	Svolume,tempchar
	;	sts	LongWord+1,tempchar
	;	ldi	zl,low(LongWord)
	;	ldi	zh,high(LongWord)
	;	ldi	xl,low(Str6)
	;	ldi	xh,high(Str6)
	;	rcall	Itoa
		
		
	;	ldi 	zl,low(2*volume)			;Load low stringpointer sharp
	;	ldi 	zh,high(2*volume)			;Load high stringpointer sharp
		
		
	;	rcall	PrintSoftUstr				;Send command to projector
	;	ldi 	zl,low(Str6)				;Load strinpointer
	;	ldi 	zh,high(Str6)
	;	adiw	zl,3
	;	rcall	PrintSoftU
	;	ldi	u_data,13				;Load byte
	;	rcall	putc					;Send byte to soft-uart
	;	rcall	ReadSoftUart				;Read projector response

	;	ldi	zl,low(2*ok)				;Load low stringpointer for matching word
	;	ldi	zh,high(2*ok)				;Loas high stringpointer for matching word
	;	ldi	yl,low(SUARTbuf)			;Load stringpointer destination
	;	ldi	yh,high(SUARTbuf)
	;	adiw	yl,1
	;	rcall	StrSearch
	;	tst	yl					;Test for OK	
	;	breq	SharpButton5_1
		
	;	rjmp	SharpButtonExit1	 


SharpButton5_1:		 
		;lds 	tempchar,Svolume
		;ldi	temp0,2
		;add	tempchar,temp0
		;sts	Svolume,tempchar

SharpButton5_2:		
	;	rjmp	SharpButtonExit1
		


SharpButton6:
;		rcall	Ds2406_pioaOFF
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
		
		ldi	tempchar,0x55
		st	x+,tempchar
		ldi	tempchar,0x12
		st	x+,tempchar
		ldi	tempchar,0xa0
		st	x+,tempchar
		ldi	tempchar,0x1f
		st	x+,tempchar
		ldi	tempchar,0x73	
		st	x+,tempchar
		ldi	tempchar,0x00
		st	x+,tempchar
		ldi	tempchar,0x00
		st	x+,tempchar
		ldi	tempchar,0x00
		st	x+,tempchar
		ldi	tempchar,0xee
		st	x+,tempchar
	
	
		ldi	tempchar,0x55	;
		st	x+,tempchar
		ldi	tempchar,0x07	;
		st	x+,tempchar
		ldi	tempchar,0x00	
		st	x+,tempchar
		ldi	tempchar,0x6d	;
		st	x+,tempchar
		
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
		rcall	Ds2406_pioaON
		
		
		
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
		
		ldi	tempchar,0x55
		st	x+,tempchar
		ldi	tempchar,0x12
		st	x+,tempchar
		ldi	tempchar,0x2f
		st	x+,tempchar
		ldi	tempchar,0x22
		st	x+,tempchar
		ldi	tempchar,0x73	
		st	x+,tempchar
		ldi	tempchar,0x00
		st	x+,tempchar
		ldi	tempchar,0x00
		st	x+,tempchar
		ldi	tempchar,0x00
		st	x+,tempchar
		ldi	tempchar,0x48
		st	x+,tempchar
	
	
		ldi	tempchar,0x55	;
		st	x+,tempchar
		ldi	tempchar,0x07	;
		st	x+,tempchar
		ldi	tempchar,0x00	
		st	x+,tempchar
		ldi	tempchar,0x6d	;
		st	x+,tempchar
		
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
		
	
		rcall	Ds2406_pioaON
		
		rjmp	SharpButtonExit1
		;ldi	tempchar,0xff
		;sts	DelayV,tempchar
		;ldi	tempchar,0xff
		;sts	DelayV+1,tempchar
		;ldi	tempchar,0x4
		;sts	DelayV+2,tempchar
		;rcall	Delay
		
		;ldi	tempchar,0
		;sts	LongWord,tempchar
		;lds	tempchar,Svolume
		;cpi	tempchar,60
		;breq	SharpButton6_2
		;ldi	temp0,2
		;add	tempchar,temp0
		;sts	Svolume,tempchar
		;sts	LongWord+1,tempchar
		;ldi	zl,low(LongWord)
		;ldi	zh,high(LongWord)
		;ldi	xl,low(Str6)
		;ldi	xh,high(Str6)
		;rcall	Itoa
		
		
		;ldi 	zl,low(2*volume)			;Load low stringpointer sharp
		;ldi 	zh,high(2*volume)			;Load high stringpointer sharp
		
		
		;rcall	PrintSoftUstr				;Send command to projector
		;ldi 	zl,low(Str6)				;Load strinpointer
		;ldi 	zh,high(Str6)
		;adiw	zl,3
		;rcall	PrintSoftU
		;ldi	u_data,13				;Load byte
		;rcall	putc					;Send byte to soft-uart
		;rcall	ReadSoftUart				;Read projector response

		;ldi	zl,low(2*ok)				;Load low stringpointer for matching word
		;ldi	zh,high(2*ok)				;Loas high stringpointer for matching word
		;ldi	yl,low(SUARTbuf)			;Load stringpointer destination
		;ldi	yh,high(SUARTbuf)
		;adiw	yl,1
		;rcall	StrSearch
		;tst	yl					;Test for OK	
		;breq	SharpButton6_1
		
		;rjmp	SharpButtonExit1	 	 

SharpButton6_1:		 
		;lds 	tempchar,Svolume
		;ldi	temp0,2
		;sub	tempchar,temp0
		;sts	Svolume,tempchar



SharpButton6_2:		
		 
		;rjmp	SharpButtonExit1				
		
SharpButtonExit:		
		;rcall	PrintSoftUstr
		;rcall	ReadSoftUart
		;rcall	LongDelay
		;---------------Init timer1 if pollFlag=1---------------------------------------------------
SharpButtonExit1:
		;
		ldi	flag,0
		clr	temp0			 
		sts	Count,temp0		;clear time counter
		sts	TempBuf,temp0
		;lds	tempchar,pollFlag
		;clr	temp0
		;cpse	tempchar,temp0
		lds	tempchar,Bflags
		sbrc	tempchar,0
		rcall	timer1_init	
		;-------------------------------------------------------------------
SharpButtonExit0:
		;cbi	PORTD,6
		ret






	
	
;-----------------------------------------------------------------------------------

	
	
;-----------------------------------------------------------------------------------




;------------------------------------------------------------------------------------------------------------------												
;************************************************************************************
;*FUNCTION
;*  LedFlashA5
;*
;*DESCRIPTION
;*		LedFlash for PORTA,5
;*PARAMETER
;*
;*Input:
;************************************************************************************
;LedFlashA5:;----Set Delay Value---------------------
		
;		ldi	temp0,15
		
;LedFlashA50:
;		ldi	tempchar,0xff
;		sts	DelayV,tempchar
;		ldi	tempchar,0xff
;		sts	DelayV+1,tempchar
;		ldi	tempchar,0xa
;		sts	DelayV+2,tempchar
;		cbi	PORTA,5					;Set led button On
;		rcall	Delay
;		sbi	PORTA,5					;Set led button Off
;		rcall	Delay
;		dec	temp0
;		tst	temp0
;		brne	LedFlashA50
;		ret
;--------------------------------------------------------------------------------------------
	
;------------------------------------------------------------------------------------------------------------------												
;************************************************************************************
;*FUNCTION
;*  LedFlashA6
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*Input:
;************************************************************************************
;LedFlashA6:;----Set Delay Value---------------------
		
;		ldi	temp0,15
		
;LedFlashA60:
;		ldi	tempchar,0xff
;		sts	DelayV,tempchar
;		ldi	tempchar,0xff
;		sts	DelayV+1,tempchar
;		ldi	tempchar,0xa
;		sts	DelayV+2,tempchar
;		cbi	PORTA,6					;Set led button On
;		rcall	Delay
;		sbi	PORTA,6					;Set led button Off
;		rcall	Delay
;		dec	temp0
;		tst	temp0
;		brne	LedFlashA60
;		ret
;--------------------------------------------------------------------------------------------
;************************************************************************************
;*FUNCTION
;*  LedFlashC6
;*
;*DESCRIPTION
;*		LedFlash for PORTC,6
;*PARAMETER
;*
;*Input:
;************************************************************************************
;LedFlashC6:;----Set Delay Value---------------------
		
;		ldi	temp0,50
		
;LedFlashC60:
;		ldi	tempchar,0xff
;		sts	DelayV,tempchar
;		ldi	tempchar,0xff
;		sts	DelayV+1,tempchar
;		ldi	tempchar,0xa
;		sts	DelayV+2,tempchar
;		cbi	PORTC,6					;Set led button On
;		rcall	Delay
;		sbi	PORTC,6					;Set led button Off
;		rcall	Delay
;		dec	temp0
;		tst	temp0
;		brne	LedFlashC60
;		ret
;--------------------------------------------------------------------------------------------

;------------------------------------------------------------------------------------------------------------------		
;************************************************************************************
;*FUNCTION
;*  LedFlashC7
;*
;*DESCRIPTION
;*		LedFlash for PORTC,7
;*PARAMETER
;*
;*Input:
;************************************************************************************
;LedFlashC7:;----Set Delay Value---------------------
		
;		ldi	temp0,90
		
;LedFlashC70:
;		ldi	tempchar,0xff
;		sts	DelayV,tempchar
;		ldi	tempchar,0xff
;		sts	DelayV+1,tempchar
;		ldi	tempchar,0xa
;		sts	DelayV+2,tempchar
;		cbi	PORTC,7					;Set led button On
;		rcall	Delay
;		sbi	PORTC,7					;Set led button Off
;		rcall	Delay
;		dec	temp0
;		tst	temp0
;		brne	LedFlashC70
;		ret
;--------------------------------------------------------------------------------------------
	
		
;************************************************************************************
;*FUNCTION
;*  Button:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************						

Button:
		
		;rcall	ViewSonicButton
		rcall	SharpButton
		;rcall	BarcoButton
		;rcall  PanasonicButton
		ret
		
;***************************************************************************
;* 
;* EEWrite
;*
;* This subroutine waits until the EEPROM is ready to be programmed, then
;* programs the EEPROM with register variable "tempchar" at address "zl:zh"
;*
;* Number of words	:8515 ; 7 + return
;* Number of cycles	:8515 ; 11 + return (if EEPROM is ready)
;* Low Registers used	:zl
;* High Registers used:	;zh
;*
;***************************************************************************

;***** Subroutine register variables

;.def	EEdwr	=r16		;data byte to write to EEPROM
;.def	EEawr	=r17		;address low byte to write to
;.def	EEawrh	=r18		;address high byte to write to

;***** Code

EEWrite:
	sbic	EECR,EEWE	;if EEWE not clear
	rjmp	EEWrite		;    wait more


; the two following lines must be replaced with the line above if 1200 is used
	out 	EEARH,zh;EEawrh	;output address high for 8515
	out	EEARL,zl;EEawr	;output address low for 8515
		

	out	EEDR,tempchar	;output data
	sbi 	EECR,EEMWE	;set master write enable, remove if 1200 is used	
	sbi	EECR,EEWE	;set EEPROM Write strobe
				;This instruction takes 4 clock cycles since
				;it halts the CPU for two clock cycles
	ret
	
;***************************************************************************
;* 
;* EEWriteY
;*
;* This subroutine waits until the EEPROM is ready to be programmed, then
;* programs the EEPROM with register variable "tempchar" at address "zl:zh"
;*
;* Number of words	:8515 ; 7 + return
;* Number of cycles	:8515 ; 11 + return (if EEPROM is ready)
;* Low Registers used	:yl
;* High Registers used:	;yh
;*
;***************************************************************************

;***** Subroutine register variables

;.def	EEdwr	=r16		;data byte to write to EEPROM
;.def	EEawr	=r17		;address low byte to write to
;.def	EEawrh	=r18		;address high byte to write to

;***** Code

EEWriteY:
	sbic	EECR,EEWE	;if EEWE not clear
	rjmp	EEWriteY	;wait more


; the two following lines must be replaced with the line above if 1200 is used
	out 	EEARH,yh;EEawrh	;output address high for 8515
	out	EEARL,yl;EEawr	;output address low for 8515
		

	out	EEDR,tempchar	;output data
	sbi 	EECR,EEMWE	;set master write enable, remove if 1200 is used	
	sbi	EECR,EEWE	;set EEPROM Write strobe
				;This instruction takes 4 clock cycles since
				;it halts the CPU for two clock cycles
	ret	
	
	
;***************************************************************************
;* 
;* EERead
;*
;* This subroutine waits until the EEPROM is ready to be programmed, then
;* reads the register variable "tempchar" from address "zh:zl"
;*
;* Number of words	:8515 ; 6 + return
;*			:8515 ; 9 + return (if EEPROM is ready)
;* Low Registers used	:1 (zl)
;* High Registers used:	:2 (zh)
;*
;***************************************************************************

;***** Subroutine register variables

;.def	EEdrd	=r0		;result data byte
;.def	EEard	=r17		;address low to read from
;.def	EEardh	=r18		;address high to read from

;***** Code

EERead:
	sbic	EECR,EEWE	;if EEWE not clear
	rjmp	EERead		;    wait more
;


	out 	EEARH,zh;EEardh	;output address high for 8515
	out	EEARL,zl;EEard	;output address low for 8515


	sbi	EECR,EERE	;set EEPROM Read strobe
				;This instruction takes 4 clock cycles since
				;it halts the CPU for two clock cycles
	in	tempchar,EEDR	;get data
	ret	
	
;***************************************************************************
;* 
;* EEReadY
;*
;* This subroutine waits until the EEPROM is ready to be programmed, then
;* reads the register variable "tempchar" from address "zh:zl"
;*
;* Number of words	:8515 ; 6 + return
;*			:8515 ; 9 + return (if EEPROM is ready)
;* Low Registers used	:1 (yl)
;* High Registers used:	:2 (yh)
;*
;***************************************************************************

;***** Subroutine register variables

;.def	EEdrd	=r0		;result data byte
;.def	EEard	=r17		;address low to read from
;.def	EEardh	=r18		;address high to read from

;***** Code

EEReadY:
	sbic	EECR,EEWE	;if EEWE not clear
	rjmp	EEReadY		;    wait more
;


	out 	EEARH,yh;EEardh	;output address high for 8515
	out	EEARL,yl;EEard	;output address low for 8515


	sbi	EECR,EERE	;set EEPROM Read strobe
				;This instruction takes 4 clock cycles since
				;it halts the CPU for two clock cycles
	in	tempchar,EEDR	;get data
	ret			
	
;----------------------------------------------------------------------------------------------
;WriteEE:
;		ldi	zl,$00
;		ldi	zh,$00
;		ldi	tempchar,'J'	
;		rcall	EEWrite
;		adiw	zl,1
;		ldi	tempchar,'a'
;		rcall	EEWrite
;		adiw	zl,1
;		ldi	tempchar,'r'
;		rcall	EEWrite
;		adiw	zl,1
;		ldi	tempchar,'l'
;		rcall	EEWrite
;		adiw	zl,1
;		ldi	tempchar,13
;		rcall	EEWrite
;		adiw	zl,1
;		ldi	tempchar,0
;		rcall	EEWrite
;		adiw	zl,1
;		rcall	EEWrite

;		ret
;----------------------------------------------------------------------------------------------			
;************************************************************************************
;*FUNCTION
;*  GetSource:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************						

;GetSource:
;	rcall	ClearSuartBuf
;	ldi 	zl,low(2*gss)				;Load strinpointer
;	ldi 	zh,high(2*gss)
;	rcall	PrintSoftUstr
;	ldi	zl,low(2*gssin1)
;	ldi	zh,high(2*gssin1)
;	ldi	yl,low(SUARTbuf)			;Load stringpointer destination
;	ldi	yh,high(SUARTbuf)
;	adiw	yl,1
;	rcall	StrSearch
;	tst	yl					;Test for OK	
;	breq	lpollin11
;	sbi	PORTA,5					;Set led button 1 On
;	cbi	PORTA,6
;	rjmp	lpollin4	 
;lpollin11:		
;	cbi	PORTA,5					;Set led button 1 Off 		 
	
;lpollin4:
;		rcall	ClearSuartBuf
;		ldi 	zl,low(2*gss)				;pioneer-status
;		ldi 	zh,high(2*gss)				;pioneer-status
;		rcall	PrintSoftUstr				;Send command to projector
;		rcall	ReadSoftUart				;Read projector respons
;		ldi	zl,low(2*gssin4)
;		ldi	zh,high(2*gssin4)
		
;		ldi	yl,low(SUARTbuf)			;Load stringpointer destination
;		ldi	yh,high(SUARTbuf)
;		adiw	yl,1
;		rcall	StrSearch
;		tst	yl					;Test for OK	
;		breq	lpollin41
;		sbi	PORTA,6					;Set led button 1 On
;		cbi	PORTA,5
;		rjmp	lpollin42	 
;lpollin41:		
;		cbi	PORTA,6					;Set led button 1 Off 
;lpollin42:	

;rcall	ClearSuartBuf
;ret

;************************************************************************************
;*FUNCTION
;*  GetSourceVS:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************						

;GetSourceVS:
;		rcall	ClearSuartBuf
;		ldi 	zl,low(2*vsgetsource)			;
;		ldi 	zh,high(2*vsgetsource)			;
;		ldi	temp0,13				;Set packet length
;		rcall	SendPacketSoftUstr
;		ldi	tempchar,3				;Set packet length
;		rcall	ReadPacketSoftUart
;		lds	tempchar,SUARTbuf+2
;		clc
;		cpi	tempchar,0x01				;if rgb in1 jump
;		breq	GetSourceVS00
;		cpi	tempchar,0x02				;if rgb in2 jump
;		breq	GetSourceVS01
;		cpi	tempchar,0x04				;if video jump
;		breq	GetSourceVS02
;		cbi	PORTA,5
;		cbi	PORTA,6
;		rjmp	GetSourceVSexit
		
		
;GetSourceVS00:						;rgb1  input1
;	ldi	temp0,1					
;	sts	Scount,temp0				;increase Scounter
;	sbi	PORTA,5
;	cbi	PORTA,6
;	rjmp	GetSourceVSexit
;GetSourceVS01:						;rgb2  input2
;	ldi	temp0,2					
;	sts	Scount,temp0				;increase Scounter
;	sbi	PORTA,5
;	cbi	PORTA,6
;	rjmp	GetSourceVSexit

;GetSourceVS02:						;video input4				
;	clr	temp0
;	sts	Scount,temp0				;increase Scounter
;	sbi	PORTA,5
;	cbi	PORTA,6
;	rjmp	GetSourceVSexit



;GetSourceVSexit:

;ret




;************************************************************************************
;*FUNCTION
;*  GetSourceSH:
;*
;*DESCRIPTION
;* Only sharp xr10
;* Implemented width Mute function
;*
;*PARAMETER
;*
;*
;************************************************************************************						

;GetSourceSH:
;		rcall	ClearSuartBuf
;		ldi 	zl,low(2*ichk)				;Load strinpointer
;		ldi 	zh,high(2*ichk)
;		rcall	PrintSoftUstr
;		rcall	ReadSoftUart	
;		lds	tempchar,SUARTbuf+4
;		clc
;		cpi	tempchar,'4'				;if video jump
;		breq	GetSourceSH02
		;cpi	tempchar,0x00				;if rgb in1 jump
		;breq	GetSourceSH00
;		cpi	tempchar,'1'				;if rgb in2 jump
;		breq	GetSourceSH00
;		cpi	tempchar,'2'
;		breq	GetSourceSH01
;		cbi	PORTA,5
;		cbi	PORTA,6
;		rjmp	GetSourceSHexit
		
		
;GetSourceSH00:
;	clr	temp0
	;ldi	temp0,1
;	sts	Scount,temp0				;increase Scounter
;	rjmp	GetSourceSH00_01
;GetSourceSH01:
;	clr	temp0
;	sts	Scount,temp0				;increase Scounter
;GetSourceSH00_01:	
;	sbi	PORTA,5
;	cbi	PORTA,6
;	rjmp	GetSourceSHexit
;GetSourceSH02:
;	ldi	temp0,1
;	sts	Scount,temp0
;	sbi	PORTA,5
;	cbi	PORTA,6
;	rjmp	GetSourceSHexit


;GetSourceSHexit:

;ret




;************************************************************************************
;*FUNCTION
;*  GetTargetNo:
;*
;*DESCRIPTION Moves data from EEPROM memory to ram memory
;*            z points at source positon and y points at target positon
;*	      Sorce string must be null terminated.
;*
;*PARAMETER
;*z pointer to the source positon in (zl,zh) in EEPROM memory
;*y pointer to the target positon  (yl,yh) in Ram memory
;*
;************************************************************************************						

MoveEEtoRam:

		sbi	PORTD,6				;Set RTS On
		rcall	EERead
		tst	tempchar			;Test if we have reached the end of the string
		breq	quitMoveEEtoRam			;If so, quit
		mov	r0,tempchar
		st	y+,r0				;Store byte in flash memory
		adiw	zl,1				;increase z
		rjmp	MoveEEtoRam
quitMoveEEtoRam:
		clr	r0
		st	y+,r0
		cbi	PORTD,6				;Set RTS Off
		ret


ret


;************************************************************************************
;*FUNCTION
;*  GetTargetPhoneNo:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************
GetTargetPhoneNo:
	ldi	zl,low(targetPno)
	ldi	zh,high(targetPno)
	ldi	yl,low(TempPhoneno)
	ldi	yh,high(TempPhoneno)
	rcall	MoveEEtoRam
ret


;************************************************************************************
;*FUNCTION
;*  initPoll:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************
initPoll:
	rcall	GetTargetPhoneNo
	lds	temp0,Bflags
	cbr	temp0,2
	sts	Bflags,temp0
	clr	temp0
	;;sts	SmsFlag,temp0		;Set SmsFlag off			 
	sts	Count,temp0		;clear time counter
	;;ldi	temp0,1
	;;sts	pollFlag,temp0		;set pollFlag	
	lds	temp0,Bflags
	sbr	temp0,1
	sts	Bflags,temp0
	rcall	timer1_init
ret


;************************************************************************************
;*FUNCTION
;*  Sharppoll:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************

SharplPoll: 	

	;wdr				;watchdog reset
;	sbi	PORTD,6	
	;----------------------------poll gsm-modul-------------------------------------------
	;rcall	ClearCrFlag		;Clear CrFlag
	;rcall 	ClearBuf

	;	ldi	yl,low(suartbuf)
	;	ldi	yh,high(suartbuf)	
	;	ldi	temp0,32
	;	st	y+,temp0
	;	ldi	temp0,32
	;	st	y+,temp0
	;	ldi	temp0,49
	;	st	y+,temp0
	;	ldi	temp0,55
	;	st	y+,temp0
		;ldi	temp0,13
		;st	y+,temp0
	;	ldi	temp0,0
	;	st	y+,temp0
		
		
		

	;	ldi	yl,low(suartbuf)
	;	ldi	yh,high(suartbuf)

	;	adiw	yl,1	
	;	rcall	atol
	;	lds	u_data,str6
	;	rcall	putc
		
		;lds	tempchar,str6
		;rcall	Ds2406				;test
		;rjmp	Sharplpollexit00		;test
		ldi 	zl,low(tempref)		;Load strinpointer
		ldi 	zh,high(tempref)
		rcall	EERead	
		mov	temp0,tempchar
		lds	tempchar,reftemp
		
		cp	temp0,tempchar
		breq	sharppolltempref	
		ldi 	zl,low(tempref)		;Load strinpointer
		ldi 	zh,high(tempref)
		rcall	EEWrite	
		
		ldi	u_data,254
		rcall	putc
		ldi	u_data,88
		rcall	putc
		ldi	u_data,'U'
		rcall	putc
		ldi	u_data,'p'
		rcall	putc
		ldi	u_data,'d'
		rcall	putc
		ldi	u_data,'a'
		rcall	putc
		ldi	u_data,'t'
		rcall	putc
		ldi	u_data,'e'
		rcall	putc
		rjmp	sharppolltempref0

sharppolltempref:
		
		
		
		
		
		
		ldi 	zl,low(pumpflag)		;Load strinpointer
		ldi 	zh,high(pumpflag)
		rcall	EERead	
		sbrs	tempchar,0			;Skip if bit0 is set
		rjmp	pump_off
		
		
		ldi	u_data,254
		rcall	putc
		ldi	u_data,88
		rcall	putc

		ldi 	zl,low(tempref)		;Load strinpointer
		ldi 	zh,high(tempref)
		rcall	EERead	
		mov	temp0,tempchar
		push	temp0
		
		ldi	temp0,25	
		rcall	printff	
		
	;	ldi	xl,low(phoneno)
	;	ldi	xh,high(phoneno)
		
	;	ldi	tempchar,0x55
	;	st	x+,tempchar
	;	ldi	tempchar,0x10
	;	st	x+,tempchar
	;	ldi	tempchar,0xaf
	;	st	x+,tempchar
	;	ldi	tempchar,0x26
	;	st	x+,tempchar
	;	ldi	tempchar,0x15	
	;	st	x+,tempchar
	;	ldi	tempchar,0x02
	;	st	x+,tempchar
	;	ldi	tempchar,0x08
	;	st	x+,tempchar
	;	ldi	tempchar,0x00
	;	st	x+,tempchar
	;	ldi	tempchar,0x67
	;	st	x+,tempchar
		;ldi	tempchar,0x33
		;st	x+,tempchar
		;ldi	tempchar,0x44
		;st	x+,tempchar
	;	ldi	xl,low(phoneno)
	;	ldi	xh,high(phoneno)
		
		ldi 	zl,low(2*tempa0)				;Load strinpointer to commandlist 
		ldi 	zh,high(2*tempa0)
		rcall	ReadDs18s20
		pop	temp0
		sub	temp0,tempchar
		breq	TempDifEq
		brpl	TempDifPlus
		brmi	TempDifNeg
	
	
TempDifPlus:
		cpi	temp0,2
		brsh	TempDifPlus_00
		mov	u_data,temp0
		rcall	putc
		
		ldi	u_data,'p'
		rcall	putc
		
		ldi	u_data,13
		rcall	putc	
		ldi	u_data,10
		rcall	putc
		
		ldi	tempchar,'j'
		cbi	portb,ss
		rcall	rw_spi
		sbi	portb,ss
		
	rjmp	Sharplpollexit00
TempDifPlus_00:
		mov	u_data,temp0
		rcall	putc
		
		ldi	u_data,'P'
		rcall	putc
		
		ldi	u_data,13
		rcall	putc	
		ldi	u_data,10
		rcall	putc
		ldi	tempchar,'u'
		cbi	portb,ss
		rcall	rw_spi
		sbi	portb,ss	
	
	rjmp	Sharplpollexit00

TempDifNeg:
		cpi	temp0,254
		brlo	TempDifNeg_00	
		mov	u_data,temp0
		rcall	putc
		
		ldi	u_data,'n'
		rcall	putc
		
		ldi	u_data,13
		rcall	putc
		ldi	u_data,10
		rcall	putc		
		ldi	tempchar,'k'
		cbi	portb,ss
		rcall	rw_spi
		sbi	portb,ss
	
	rjmp	Sharplpollexit00
TempDifNeg_00:	
		mov	u_data,temp0
		rcall	putc
		
		ldi	u_data,'N'
		rcall	putc
		
		ldi	u_data,13
		rcall	putc
		ldi	u_data,10
		rcall	putc		
		ldi	tempchar,'i'
		cbi	portb,ss
		rcall	rw_spi
		sbi	portb,ss
	
	rjmp	Sharplpollexit00

TempDifEq:	

	
		mov	u_data,temp0
		rcall	putc
		
		ldi	u_data,'E'
		rcall	putc
		
		ldi	u_data,13
		rcall	putc
		ldi	u_data,10
		rcall	putc	
	rjmp	Sharplpollexit00

pump_off:	
		;ldi	tempchar,'i'
		;cbi	portb,ss
		;rcall	rw_spi
		;sbi	portb,ss
		;mov	u_data,tempchar
		;ldi	u_data,'Z'
		;rcall	putc
		
		ldi	tempchar,'k'	
		cbi	portb,ss
		rcall	rw_spi		
		sbi	portb,ss
		
		
		ldi	tempchar,'v'
		cbi	portb,ss
		rcall	rw_spi
		sbi	portb,ss
		;mov	u_data,tempchar
		;rcall	putc
		ldi	tempchar,'v'
		cbi	portb,ss
		rcall	rw_spi
		sbi	portb,ss
		mov	u_data,tempchar
		rcall	putc
		
	
	rjmp	Sharplpollexit00
;	rjmp    Sharplpoll90
	;ldi	rxhead,0
	;ldi	rxtail,0
;	ldi	zl,low(2*atcpinask)			;Load low stringpointer for matching word
;	ldi	zh,high(2*atcpinask)			;Loas high stringpointer for matching word
;	rcall	PrintStr	
	;rcall	ReadSoftUart	
SharplPoll85:	
;	lds	temp1,Bflags
;	sbrc	temp1,4
;	rjmp	SharplPoll86
;	lds	temp1,Count			;Load poll count
;	cpi	temp1,10			
;	brge	SharplPoll86	
;	rjmp	SharplPoll85
SharplPoll86:
;	rcall	ClearCrFlag		;Clear CrFlag
;	cp	rxhead,rxtail
;	breq	SharplPoll87
;	ldi 	zl,low(2*cpinready)			;Load stringpointer source
;	ldi 	zh,high(2*cpinready)
;	ldi	yl,low(Buf)			;Load stringpointer destination
;	ldi	yh,high(Buf)
;	rcall	StrSearch
;	tst	yl	
	;brne	ActionClock	
	
	
	
	
	
	;ldi	zl,low(2*cpinready)			;Load low stringpointer for matching word
	;ldi	zh,high(2*cpinready)			;Loas high stringpointer for matching word
	;ldi	yl,low(SUARTbuf)			;Load stringpointer destination
	;ldi	yh,high(SUARTbuf)
	;adiw	yl,1
	;rcall	StrSearch
	;tst	yl
;	brne	Sharplpoll90
	;sbi	porta,0
	;cbi	porta,0
	;ldi	u_data,'&'
	;rcall	putc
	;rjmp	reset
SharplPoll87:
;	rcall 	PhoneInit00
;	rjmp	Sharplpollexit00
Sharplpoll90:
	;------------------------------------------------------------------------------
;	rcall	GetSourceSH				;Only sharp xr-10x
;	rcall	ClearTempBuf
;	rcall	ClearSuartBuf
;	ldi 	zl,low(2*tlps)				;Load strinpointer
;	ldi 	zh,high(2*tlps)
;	rcall	PrintSoftUstr

;	rcall	ReadSoftUart	
;	ldi	zl,low(2*err)				;Load low stringpointer for matching word
;	ldi	zh,high(2*err)				;Loas high stringpointer for matching word
;	ldi	yl,low(SUARTbuf)			;Load stringpointer destination
;	ldi	yh,high(SUARTbuf)
;	adiw	yl,1
;	rcall	StrSearch
;	tst	yl					;Test for Error	

	;;;;cbi	portb,sck




;	ldi	tempchar,'s'
;	cbi	portb,ss
;	rcall	rw_spi
;	sbi	portb,ss
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'i'
;	cbi	portb,ss
;	rcall	rw_spi
;	sbi	portb,ss
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'v'
;	cbi	portb,ss
;	rcall	rw_spi
;	sbi	portb,ss
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'t'
;	cbi	portb,ss
;	rcall	rw_spi
;	sbi	portb,ss
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'u'
;	cbi	portb,ss
;	rcall	rw_spi
;	sbi	portb,ss
;	mov	u_data,tempchar
;	rcall	putc
	
	
;	ldi	u_data,13
;	rcall	putc
	
	


;	ldi	tempchar,'s'
;	cbi	portc,ss1
;	rcall	rw_spi
;	sbi	portc,ss1
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'i'
;	cbi	portc,ss1
;	rcall	rw_spi
;	sbi	portc,ss1
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'v'
;	cbi	portc,ss1
;	rcall	rw_spi
;	sbi	portc,ss1
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'t'
;	cbi	portc,ss1
;	rcall	rw_spi
;	sbi	portc,ss1
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'u'
;	cbi	portc,ss1
;	rcall	rw_spi
;	sbi	portc,ss1
;	mov	u_data,tempchar
;	rcall	putc
	
	
;	ldi	u_data,13
;	rcall	putc	
	
	
	
;	ldi	tempchar,'s'
;	cbi	portc,ss2
;	rcall	rw_spi
;	sbi	portc,ss2
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'i'
;	cbi	portc,ss2
;	rcall	rw_spi
;	sbi	portc,ss2
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'v'
;	cbi	portc,ss2
;	rcall	rw_spi
;	sbi	portc,ss2
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'t'
;	cbi	portc,ss2
;	rcall	rw_spi
;	sbi	portc,ss2
;	mov	u_data,tempchar
;	rcall	putc
;	ldi	tempchar,'u'
;	cbi	portc,ss2
;	rcall	rw_spi
;	sbi	portc,ss2
;	mov	u_data,tempchar
;	rcall	putc
	
	
;	ldi	u_data,13
;	rcall	putc	
	
	
	
rjmp	Sharplpollexit00

	;--------------------------------------
		;cbi	portb,sck
		;ldi	tempchar,194
		;rcall	rw_spi
		;mov	u_data,tempchar
		;rcall	putc		
		;rcall	delay
		;ldi	tempchar,0x04
		;rcall	rw_spi
		;mov	u_data,tempchar
		;rcall	putc
		;rcall	delay
		;ldi	tempchar,0x00
		;rcall	rw_spi
		;mov	u_data,tempchar
		;rcall	putc
	;--------------------------------------
		;sbi	PORTD,6	
;		wdr
;		ldi	temp0,1 		;Switch to AC clock
;		ldi	temp1,2			;Svitch to time function
;		ldi	tempchar,0
;		rcall	GetTimeCC		
			
;		lds	temp0,str6		;Seconds
;		sts	str1,temp0
;		sts	seconds,temp0
		
;		lds	tempchar,day		;print day
;		ldi	temp0,65
;		rcall	printff
		
		
;		lds	tempchar,hour		;print hour
;		ldi	temp0,65
;		rcall	printff
		
;		lds	tempchar,minute		;print minute
;		ldi	temp0,65
;		rcall	printff
		
;		lds	tempchar,str1		;print seconds
;		ldi	temp0,65
;		rcall	printff
		;-------Print Seconds-------------------
		
;		rcall	GetTemp
						;set bit0
;		ldi	temp0,1   		;switch on the temp function
;		ldi	zl,low(2*tempC)
;		ldi	zh,high(2*tempC)
;		rcall	GetTime
;		lds	temp0,LongWord+1
;		sts	Svolume,temp0		;store tempvalue in Svolume low byte
;		lds	temp0,LongWord
;		sts	Svolume+1,temp0		;store tempvalue in Svolume high byte
;		ldi	zl,low(LongWord)
;		ldi	zh,high(LongWord)
;		ldi	temp0,66		;print tempvalue
;		rcall	printff
		
		
		
;		lds	temp0,day
;		cpi	temp0,1
;		brge	SharplpollTemp_100_2

		
;		lds	temp0,hour
;		cpi	temp0,3
;		brge	SharplpollTemp_100	
		
;-------------------Tempinterval 0-100  -------------------------------------		
	
	
;		ldi	temp0,1 		;Switch to AC clock
;		ldi	temp1,1			;Switch to time function
;		ldi	tempchar,0
;		rcall	GetTimeCC		
		
		
						;set bit0
;		ldi	temp0,1   		;switch on the temp function
;		ldi	zl,low(2*tempGC0)
;		ldi	zh,high(2*tempGC0)
;		rcall	GetTime

;		rcall	TempCompare

;		ldi	zl,low(LongWord)
;		ldi	zh,high(LongWord)
		
		
;		ldi	temp0,130		;print tempgraf_value
;		rcall	printff
;		rjmp	Sharplpollexit00
			

;SharplpollTemp_100:;------------------Tempinterval 100----------------------------
;		lds	temp0,hour
;		cpi	temp0,12
;		brge	SharplpollTemp_100_1
;		ldi	temp0,97
;		sts	LongWord+1,temp0	
;		clr	temp0
;		sts	LongWord,temp0
;		rcall	TempCompare
		
;		ldi	tempchar,100			;print temp=100	
;		ldi	temp0,129	
;		rcall	printff			
;		rjmp	Sharplpollexit00
;SharplpollTemp_100_1:;---------------Tempinterval 100-600  ------------------------		
		
		;ldi	u_data,65
		;rcall	putc
		
;		ldi	temp0,1 		;Switch to AC clock
;		ldi	temp1,1			;Switch to time function
;		ldi	tempchar,1
;		rcall	GetTimeCC			
						;set bit0
;		ldi	temp0,1   		;switch on the temp function
;		ldi	zl,low(2*tempGC1)
;		ldi	zh,high(2*tempGC1)
;		rcall	GetTime
;		lds	temp0,LongWord+1
;		ldi	temp1,100		;Add 100 to LongWord
;		add	temp0,temp1
;		sts	LongWord+1,temp0
;		lds	temp0,LongWord
;		brcc	Temp_Add	
;		inc	temp0
;Temp_Add:		
;		sts	LongWord,temp0
		
;		rcall	TempCompare
;		ldi	zl,low(LongWord)
;		ldi	zh,high(LongWord)
		
;		ldi	temp0,130		;print tempgraf_value
;		rcall	printff
		
		
		
;		rjmp	Sharplpollexit00


;SharplpollTemp_100_2:;----------Tempinterval 600-----------------------------------
;		lds	temp0,hour
;		cpi	temp0,10
;		brge	SharplpollTemp_100_3
		
						;print temp=600	
;		ldi	temp0,2
;		sts	LongWord,temp0
;		ldi	temp0,88
;		sts	LongWord+1,temp0
;		clr	temp0
;		sts	LongWord+2,temp0
;		sts	LongWord+3,temp0
		
;		rcall	TempCompare
;		ldi	zl,low(LongWord)
;		ldi	zh,high(LongWord)
;		ldi	temp0,130	
;		rcall	printff	
;		rjmp	Sharplpollexit00
		
;SharplpollTemp_100_3:;----------Tempinterval 600-1000 -----------------------------------		
;		lds	temp0,hour
;		cpi	temp0,18
;		brge	SharplpollTemp_100_4
		
		
		
;		ldi	temp0,1 		;Switch to AC clock
;		ldi	temp1,1			;Switch to time function
;		ldi	tempchar,2		;Switch to BreakPoint1
;		rcall	GetTimeCC			
						;set bit0
;		ldi	temp0,1   		;switch on the temp function
;		ldi	zl,low(2*tempGC2)
;		ldi	zh,high(2*tempGC2)
;		rcall	GetTime
;		lds	temp0,LongWord+1
;		ldi	temp1,88		;Add 600=88,2 to LongWord
;		add	temp0,temp1
;		sts	LongWord+1,temp0
;		lds	temp0,LongWord
;		brcc	Temp_Add0
		
		
;		inc	temp0
;Temp_Add0:
;		inc	temp0
;		inc	temp0
;		sts	LongWord,temp0
;		
;		
;		rcall	TempCompare
;		ldi	zl,low(LongWord)
;		ldi	zh,high(LongWord)
		
;		ldi	temp0,130		;print tempgraf_value
;		rcall	printff
;		rjmp	Sharplpollexit00

;SharplpollTemp_100_4:;-----------------------Temp 1000 Off------------------------
		
		;clr	temp0			;Lundsbrunn
		;sts	SUARTbuf+2,temp0	;Lundsbrunn
		;ldi	zl,low(SUARTbuf)
		;ldi	zh,high(SUARTbuf)
		;ldi	yl,low(TempBuf)
		;ldi	yh,high(TempBuf)
		;ldi	temp0,'$'
		;st	y+,temp0
		;ldi	temp0,'L'
		;st	y+,temp0
		;ldi	temp0,'S'
		;st	y+,temp0
		;adiw	zl,1
		;rcall	MoveString	
		;sbiw	yl,1
		;ldi	temp0,'/'
		;st	y+,temp0
		;clr	temp0
		;st	y+,temp0
		;rcall	lampTest
		
		
	
	rjmp	Sharplpollexit00
Sharplpoll00:
;-----------------------------------------------------------------------------
	;lds	tempchar,SUARTbuf+1
;	lds	tempchar,SUARTbuf+4			;Only sharp xr-10x
;	cpi	tempchar,48
;	breq	Sharplpoll01
;	cpi	tempchar,49
;	breq	Sharplpoll02
;	cpi	tempchar,50
;	breq	Sharplpoll03
;	cpi	tempchar,51
;	breq	Sharplpoll04
;	cpi	tempchar,52
;	breq	Sharplpoll05
;	rjmp	Sharplpoll06
	;rjmp	Sharplpollexit
Sharplpoll01:	
;	ldi	tempchar,48
;	rjmp Sharplpoll07

Sharplpoll02:
;	cbi	PORTC,7;4					;Set led button 1 On
;	sbi	PORTC,6;5
;	ldi	tempchar,49
;	sts	SUARTbuf+1,tempchar
;	rjmp	Sharplpollexit



	
Sharplpoll03:
;	ldi	tempchar,50
;	rjmp Sharplpoll07
Sharplpoll04:
;	ldi	tempchar,51
;	rjmp Sharplpoll07
Sharplpoll05:
;	ldi	tempchar,52
;	rjmp Sharplpoll07
Sharplpoll06:
;	ldi	tempchar,'?'
;	rjmp Sharplpoll08					;Only sharp xr-10x
	;rjmp	Sharplpollexit00				;Lunsbrunn
Sharplpoll07:
;	sbi	PORTC,7;5					;Set led button 1 On
;	cbi	PORTC,6;4
;	cbi	PORTA,5
;	cbi	PORTA,6	
;	sts	SUARTbuf+1,tempchar
;	rjmp	Sharplpollexit
Sharplpoll08:	
;	cbi	PORTC,7;5
;	cbi	PORTC,6;4
;	cbi	PORTA,5
;	cbi	PORTA,6	
;	sts	SUARTbuf+1,tempchar
Sharplpollexit:

;-------------------------------------------------------------------------------
	;clr	temp0			;Lundsbrunn
	;sts	SUARTbuf+2,temp0	;Lundsbrunn
	;ldi	zl,low(SUARTbuf)
	;ldi	zh,high(SUARTbuf)
	;ldi	yl,low(TempBuf)
	;ldi	yh,high(TempBuf)
	;ldi	temp0,'$'
	;st	y+,temp0
	;ldi	temp0,'L'
	;st	y+,temp0
	;ldi	temp0,'S'
	;st	y+,temp0
	;adiw	zl,1
	;rcall	MoveString	
	;sbiw	yl,1
	;ldi	temp0,'/'
	;st	y+,temp0
	;clr	temp0
	;st	y+,temp0
	;push	yl
	;push	yh
	;ldi 	zl,low(2*tltt)				;Load strinpointer
	;ldi 	zh,high(2*tltt)
	;rcall	PrintSoftUstr				;Get lamptime
;	wdr						;watchdog reset
	;rcall	ReadSoftUart	
	;ldi	zl,low(SUARTbuf)
	;ldi	zh,high(SUARTbuf)
	;adiw	zl,1
	;pop	yh				;t
	;pop	yl				;t
	;sbiw	yl,1				;t
	;rcall	MoveString			;t
	;sbiw	yl,2
;-----------------------------------------------------------------------------------
	;ldi 	zl,low(fieldFlag)			;Load strinpointer
	;ldi 	zh,high(fieldFlag)
	;rcall	EERead					;Get fieldFlag
	;clr 	temp0
	;sbrc	tempchar,0				;Skip if bit0 is cleared
	;rcall	PrintCustomerNo
	
;	sbiw	yl,2
;	ldi	temp0,'/'
;	st	y+,temp0
;	ldi	zl,low(customernoE)
;	ldi	zh,high(customernoE)
;	rcall	MoveEEtoRam
;	sbiw	yl,1
;-----------------------------------------------------------------------------------


	;ldi 	zl,low(ireg)		;Load strinpointer
	;ldi 	zh,high(ireg)
	;rcall	EERead			;Get initreg ireg
	;mov	temp0,tempchar
;	lds	temp0,Bflags
;	sbrs	temp0,2
;	rjmp	Sharplpoll09
;	cbr	temp0,4			;clear bit 3
	;mov	tempchar,temp0
	;rcall	EEWrite
;	sts	Bflags,temp0
;	sbiw	yl,2
;	ldi	temp0,'/'
;	st	y+,temp0
;	ldi	temp0,'S'
;	st	y+,temp0
;	ldi	temp0,'t'
;	st	y+,temp0
;	ldi	temp0,'a'
;	st	y+,temp0
;	ldi	temp0,'r'
;	st	y+,temp0
;	ldi	temp0,'t'
;	st	y+,temp0
;	clr	temp0
;	st	y+,temp0
	
Sharplpoll09:
	rcall	lampTest		;Only modul

ret					;Only modul

Sharplpollexit00:
		;ldi	xl,low(phoneno)
		;ldi	xh,high(phoneno)
		
	;	ldi	tempchar,0x55
	;	st	x+,tempchar
	;	ldi	tempchar,0x10
	;	st	x+,tempchar
	;	ldi	tempchar,0x9d
	;	st	x+,tempchar
	;	ldi	tempchar,0x4c
	;	st	x+,tempchar
	;	ldi	tempchar,0x15	
	;	st	x+,tempchar
	;	ldi	tempchar,0x02
	;	st	x+,tempchar
	;	ldi	tempchar,0x08
	;	st	x+,tempchar
	;	ldi	tempchar,0x00
	;	st	x+,tempchar
	;	ldi	tempchar,0x48
	;	st	x+,tempchar
	;	ldi	xl,low(phoneno)
	;	ldi	xh,high(phoneno)
		
		ldi 	zl,low(2*tempa0)				;Load strinpointer to commandlist 
		ldi 	zh,high(2*tempa0)
		rcall	ReadDs18s20
		ldi	temp0,49	
		rcall	printff	
		
		
		
		ldi 	zl,low(2*tempa1)				;Load strinpointer to commandlist 
		ldi 	zh,high(2*tempa1)
		rcall	ReadDs18s20
		ldi	temp0,49	
		rcall	printff	
		
;		ldi	xl,low(phoneno)
;		ldi	xh,high(phoneno)
		
;		ldi	tempchar,0x55
;		st	x+,tempchar
;		ldi	tempchar,0x10
;		st	x+,tempchar
;		ldi	tempchar,0x97
;		st	x+,tempchar
;		ldi	tempchar,0x18
;		st	x+,tempchar
;		ldi	tempchar,0x15	
;		st	x+,tempchar
;		ldi	tempchar,0x02
;		st	x+,tempchar
;		ldi	tempchar,0x08
;		st	x+,tempchar
;		ldi	tempchar,0x00
;		st	x+,tempchar
;		ldi	tempchar,0x0d
;		st	x+,tempchar
;		ldi	xl,low(phoneno)
;		ldi	xh,high(phoneno)
		
		ldi 	zl,low(2*tempa2)				;Load strinpointer to commandlist 
		ldi 	zh,high(2*tempa2)
		rcall	ReadDs18s20
		
	;	ldi	tempchar,100			;print temp=100	
		ldi	temp0,25	
		rcall	printff	
		
;		ldi 	zl,low(tempref)		;Load strinpointer
;		ldi 	zh,high(tempref)
;		rcall	EERead	
;		mov	temp0,tempchar
;		lds	tempchar,reftemp
		
;		cp	temp0,tempchar
;		breq	sharppolltempref	
;		ldi 	zl,low(tempref)		;Load strinpointer
;		ldi 	zh,high(tempref)
;		rcall	EEWrite	
;		ldi	u_data,'X'
;		rcall	putc



sharppolltempref0:
	;	clr	temp0
	;	sts	LongWord,temp0
	;	sts	LongWord+1,temp0
	;	sts	LongWord+2,temp0
	;	sts	LongWord+3,temp0
	;	clc
	;	ldi	tempchar,64
	;	sts	LongWord+1,tempchar
	;	ldi	zl,low(LongWord)
	;	ldi	zh,high(LongWord)
	;	ldi	xl,low(Str6)
	;	ldi	xh,high(Str6)
	;	rcall	Itoa
		
	;	ldi 	zl,low(Str6)				;Load strinpointer
	;	ldi 	zh,high(Str6)
	;	adiw	zl,3
	;	rcall	PrintSoftU
	;	ldi	u_data,65;13				;Load byte
	;	rcall	putc					;Send byte to soft-uart




;***************Not used with modul*********************************	
	clr	temp0			 
	sts	Count,temp0		;clear time counter
	sts	TempBuf,temp0
	rcall	timer1_init
;*******************************************************************	
;	cbi	PORTD,6	
ret







;************************************************************************************
;*FUNCTION
;*  Securety:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************
;Securety:






;		ret
;------------------------------------------------------------------------------------------------------------------												
;************************************************************************************
;*FUNCTION
;*  SetSmsFlag:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************
SetSmsFlag:
		lds	temp0,Bflags
		sbr	temp0,2
		sts	Bflags,temp0


	ret

;************************************************************************************
;*FUNCTION
;*  SetCrFlag:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************
SetCrFlag:
		lds	temp0,Bflags
		sbr	temp0,16
		sts	Bflags,temp0


	ret


;************************************************************************************
;*FUNCTION
;*  ClearSmsFlag:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************




ClearSmsFlag:

		lds	temp0,Bflags
		cbr 	temp0,2
		sts	Bflags,temp0
		
	ret	



;************************************************************************************
;*FUNCTION
;*  ClearCrFlag:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************




ClearCrFlag:

		lds	temp0,Bflags
		cbr 	temp0,16
		sts	Bflags,temp0
		
	ret	




;************************************************************************************
;*FUNCTION
;*  PrintCustomerNo:
;*
;*DESCRIPTION
;*
;*PARAMETER
;*
;*
;************************************************************************************

PrintCustomerNo:
		;sbiw	yl,2
		ldi	temp0,'/'
		st	y+,temp0
		ldi	zl,low(customernoE)
		ldi	zh,high(customernoE)
		rcall	MoveEEtoRam
		sbiw	yl,1

	ret

;***************************************************************************
;*
;* FUNCTION
;*	sub_32
;*
;* DESCRIPTION
;*	str6=str6-LongWord
;*
;* USAGE
;*	Call this function 
;*	
;*
;* RETURN
;*	str6,r22
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************
sub_32:

	;ldi	zl,low(2*dayC)
	;ldi	zh,high(2*dayC)	
	;lpm				;Load byte from program memory into r0
	;sts	LongWord,r0
	;inc	zl
	;lpm
	;sts	LongWord+1,r0
	;inc	zl
	;lpm
	;sts	LongWord+2,r0
	;inc	zl
	;lpm
	;sts	LongWord+3,r0
	
	
	;ldi	temp0,0
	;sts	str6,temp0
	
	;ldi	temp0,0
	;sts	str6+1,temp0
	
	
	;ldi	temp0,2
	;sts	str6+2,temp0
	
	;ldi	temp0,0
	;sts	str6+3,temp0
;------------------------------------------------------------------------------------------------
	ldi	r22,00
	;ldi	temp0,200
	;sts	str6,temp0
	;ldi	temp0,220
	;sts	LongWord,temp0
	
	;ldi	temp0,0
	;sts	str6+1,temp0
	;ldi	temp0,0
	;sts	LongWord+1,temp0
	
	
	;ldi	temp0,1
	;sts	str6+2,temp0
	;ldi	temp0,0
	;sts	LongWord+2,temp0
	
	;ldi	temp0,0
	;sts	str6+3,temp0
	;ldi	temp0,0
	;sts	LongWord+3,temp0
;******Check if Str6>LongWord *************************************************************
	lds	temp1,Str6+3
	lds	temp0,LongWord+3
	cp	temp0,temp1
	brlo	sub_32_00
	in	r22,SREG
	sbrs	r22,1
	ret
	;-------------------------
	lds	temp1,Str6+2
	lds	temp0,LongWord+2
	cp	temp0,temp1
	brlo	sub_32_00
	in	r22,SREG
	sbrs	r22,1
	ret
	;--------------------------
	lds	temp1,Str6+1
	lds	temp0,LongWord+1
	cp	temp0,temp1
	brlo	sub_32_00
	in	r22,SREG
	sbrs	r22,1
	ret
	;--------------------------
	lds	temp1,Str6
	lds	temp0,LongWord
	cp	temp0,temp1
	brlo	sub_32_00
	in	r22,SREG
	;sbrs	r22,1
	;clr	r22
	;ldi	r22,253
	sbrc	r22,1
	rjmp sub_32_00
	ret
;*****************************************************************************************************	
sub_32_00:
	;lds	temp0,Str6
	;lds	temp1,LongWord
	clr	tempchar			;init indexcounter
	ldi	yl,low(Str6)
	ldi	yh,high(Str6)
	ldi	zl,low(LongWord)
	ldi	zh,high(LongWord)
	mov	tempchar,yl
sub_32_01:
	ld	temp0,y
	ld	temp1,z
	clc
;sub_32_02:
	;sec
	;ldi	temp0,0
	;ldi	temp1,0
	sbc	temp0,temp1
	brcs	sub_32_borrow1
sub_32_02:
	st	y,temp0
	adiw	yl,1	;inc	yl
	adiw	zl,1	;inc	zl
	inc	tempchar
	cpi	tempchar,low(str6)+3
	brne	sub_32_01
	in	r22,SREG
	
	;lds	temp0,str6
	;lds	temp0,str6+1
	;lds	temp0,str6+2
	;lds	temp0,str6+3
	ret
sub_32_borrow1:	
	;clc
	;ldi	temp1,256
	;adc	temp0,temp1
	;sts	Str6,temp1
sub_32_borrow_01:
;rev	inc	tempchar   ;test
	adiw 	yl,1		;inc	yl	
	;lds	temp1,Str6+1
	ld	temp1,y
	cpi	temp1,0
	breq	sub_32_borrow_01
	;cln
	;???????????????????????????????????
	
	;dec	temp1
	;inc	tempchar   ;*************
	
	dec	temp1
	st	y,temp1
;rev	dec	tempchar
	sbiw	yl,1	;dec	yl
	
	;???????????????????????????????????????
sub_32_borrow_02:
	;st	y,temp1
	;dec	tempchar    ;test
	;dec	yl
	ld	temp1,y
	cpi	temp1,0
	brne	sub_32_borrow_04
;rev	cpi	tempchar,0	;test
;rev	breq	sub_32_borrow_04 ;test
	cp	tempchar,yl
	breq	sub_32_borrow_04 	
	;dec	tempchar    ;******
	;dec	yl		;****
	ldi	temp1,0xff
	st	y,temp1
;rev	dec	tempchar
	sbiw	yl,1	;dec	yl
	rjmp	sub_32_borrow_02
sub_32_borrow_04:
	;inc	yl      ;*************
	clc
;	sec
	ld	temp0,y
	ld	temp1,z
	sbc	temp0,temp1
	;inc	temp0
	;------------------
	;clc
	;ldi	temp1,256
	;adc	temp0,temp1
	;--------------------
	rjmp	sub_32_02	
	ret


;***************************************************************************
;*
;* FUNCTION
;*	GetTemp
;*
;* DESCRIPTION
;*	
;*
;* USAGE
;*	Call this function 
;*	
;*
;* RETURN
;*	
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

GetTemp:
	
		cbi	portb,sck
		ldi	tempchar,194
		rcall	rw_spi
		;mov	u_data,tempchar
		;rcall	putc		
		sts	LongWord,tempchar		;low
		rcall	delay
		ldi	tempchar,0x04
		rcall	rw_spi
		;mov	u_data,tempchar
		;rcall	putc
		sts	LongWord+1,tempchar
		rcall	delay
		ldi	tempchar,0x00
		rcall	rw_spi
		;mov	u_data,tempchar
		;rcall	putc	
		sts	LongWord+2,tempchar		;high

		ldi	temp1,103		;4,3X24	load TempOffset
		
		
		clr	temp0
		sts	str6,temp0
		sts	str6+1,temp0
		sts	str6+2,temp0
		sts	str6+3,temp0
		
		;------------------Add TempOffset
		lds	tempchar,LongWord		;low
		clc
		adc	temp1,tempchar
		sts	LongWord,temp1
		lds	tempchar,LongWord+2		;high
		clr	temp1
		adc	temp1,tempchar
		sts	LongWord+2,temp1
		

		;------------------Multiply with 10		
GetTemp0:
		lds	tempchar,LongWord		;low
		lds	temp1,str6
		clc
		adc	temp1,tempchar
		sts	str6,temp1
		lds	tempchar,LongWord+2		;high
		lds	temp1,str6+1
		adc	temp1,tempchar
		sts	str6+1,temp1
		inc	temp0
		cpi	temp0,10
		brne	GetTemp0
		
	
;		lds	tempchar,str6
;		lds	temp0,str6+1		
		
	ret
	
;***************************************************************************
;*
;* FUNCTION
;*	GetTime
;*
;* DESCRIPTION
;*	
;*
;* USAGE
;*	Call this function 
;*
;* PARAMETER	
;*	temp0   bit0=tempFlag     switch on the temp function
;*		bit1=timeFlag 	  switch on the time function
;*
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************
GetTime:
	push	temp0			;stor switchFlags on stack
;	ldi	temp0,16;15;16;140
;	sts	str6,temp0
;	ldi	temp0,0
;	sts	str6+1,temp0
;	ldi	temp0,1
;	sts	str6+2,temp0
;	ldi	temp0,0
;	sts	str6+3,temp0
	;clr	temp0
	;sts	str1,temp0
	clr	temp1
	sts	str1,temp1

	
;	ldi	zl,low(2*tempC)
;	ldi	zh,high(2*tempC)
	sbrc	temp0,0
	rjmp	GetTimeStart
	;---------------------Init year----------------------------------------------------
	ldi	zl,low(2*yearC)
	ldi	zh,high(2*yearC)	
	rjmp	GetTimeStart
	;----------------------Init day----------------------------------------------------
GetTimeInitDay0:		
	;lds	temp0,LongInt			;store year
	
	sts	year,temp1

;	rjmp	GetTime_02
	ldi	zl,low(2*dayC)
	ldi	zh,high(2*dayC)	
	rjmp	GetTimeStart
	;----------Init Hour--------------------------------------------------------------
GetTimeInitHour:
	;lds	temp0,LongInt
	sts	day,temp1			;store day 
	lds	temp0,LongInt+1
	sts	day+1,temp0
	ldi	zl,low(2*hourC)
	ldi	zh,high(2*hourC)	
	rjmp	GetTimeStart
	;----------init Minute--------------------------------------------------------------
GetTimeInitMinute:	
	;lds	temp0,LongInt
	sts	hour,temp1			;store hour 
	ldi	zl,low(2*minuteC)
	ldi	zh,high(2*minuteC)
GetTimeStart:
	lpm				;Load byte from program memory into r0
	sts	LongWord,r0
	inc	zl
	lpm
	sts	LongWord+1,r0
	inc	zl
	lpm
	sts	LongWord+2,r0
	inc	zl
	lpm
	sts	LongWord+3,r0
	
	clr	temp0
	sts	LongInt,temp0
	sts	LongInt+1,temp0
	
GetTime_00:
	rcall	sub_32
	;lds	temp0,str6
	;lds	temp0,str6+1
	;lds	temp0,str6+2
	;lds	temp0,str6+3

	sbrs	r22,1
	rjmp	GetTime_01
	ldi	temp1,1
	lds	temp0,LongInt
	adc	temp0,temp1

	sts	LongInt,temp0
	brcc	GetTime_00
	lds	temp0,LongInt+1
	inc	temp0
	sts	LongInt+1,temp0
	rjmp	GetTime_00
GetTime_01:
	;lds	temp0,LongInt
	;lds	temp0,LongInt+1
	lds	temp1,LongInt
	
	lds	temp0,str1
	inc	temp0
	sts	str1,temp0
	cpi	temp0,1
	breq	GetTimeInitDay
	cpi	temp0,2
	breq	GetTimeInitHour
	cpi	temp0,3
	breq	GetTimeInitMinute
	lds	temp0,LongInt
	sts	minute,temp0			;store minute
ret
	;lds	temp0,minute
	;lds	temp0,hour
	;lds	temp0,day
	;lds	temp0,year
GetTime_02:
	;lds	temp0,LongInt+1
	;sts	day,temp0
	lds	temp0,LongInt
	sts	LongWord+1,temp0
	lds	temp0,LongInt+1
	sts	LongWord,temp0	
ret	

GetTimeInitDay:
	pop	temp0
	sbrc	temp0,0
	rjmp	GetTime_02
	rjmp	GetTimeInitDay0
;-------------------------------------------------------------------------------------------
;***************************************************************************
;*
;* FUNCTION
;*	GetTimeCC
;*
;* DESCRIPTION
;*	
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	temp0     bit0=CCFlag 	  switch on the CC clock
;*		  bit1=ACFlag 	  switch on the AC clock
;*	
;*	temp1	  bit0
;*		  bit1
;*
;*	tempchar  bit0
;*
;*
;*
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************
GetTimeCC:	
		
		push	temp1
		push	tempchar
		sbrc	temp0,0
		ldi	temp0,65
		sbrc	temp0,1
		ldi	temp0,129
		ldi	xl,low(LongWord)
		ldi	xh,high(LongWord)		;Load pointer to clock return value
		rcall	DsClock
		lds	temp0,LongWord+3
		sts	str6,temp0
		lds	temp0,LongWord+2
		sts	str6+1,temp0
		lds	temp0,LongWord+1
		sts	str6+2,temp0
		lds	temp0,LongWord
		sts	str6+3,temp0
		;-----------------------------------
		pop	temp0
		sbrc	temp0,0
		rjmp	GetTimeCC00
		sbrc	temp0,1
		rjmp	GetTimeCC01
		rjmp	GetTimeCC03	
		
GetTimeCC00:		
		ldi	zl,low(2*BreakPoint0)
		ldi	zh,high(2*BreakPoint0)
		rjmp	GetTimeCC03	
		
GetTimeCC01:		
		ldi	zl,low(2*BreakPoint1)
		ldi	zh,high(2*BreakPoint1)
		
GetTimeCC02:		
		rcall	sub_32
	
		
GetTimeCC03:
		;-------------------------------------------
		;----------------------------------------------
;		pop	temp0
;		sbrs	temp0,0
;		rjmp	GetTimeCC00	
;		ldi	zl,low(2*BreakPoint0)
;		ldi	zh,high(2*BreakPoint0)		
;		rcall	sub_32
;GetTimeCC00:
		;------------------------------------------------
		
		
		
		pop	temp0
		;ldi	temp0,2					;set bit1 switch on the time function
		rcall	GetTime
		clr	temp0
		sts	LongWord,temp0
		sts	LongWord+1,temp0
		sts	LongWord+2,temp0
		sts	LongWord+3,temp0
		lds	temp0,str6
ret

;-------------------------------------------------------------------------------------------
;***************************************************************************
;*
;* FUNCTION
;*	printff
;*
;* DESCRIPTION
;*	Print to the rs232-output
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	temchar   print out parameter	bit0
;*	z pointer			bit1
;*	
;*	temp0	bit0
;*		bit1
;*		bit2	
;*		bit3=1		no end char		
;*		bit4=1		increase pointer z=z+2	
;*		bit5=1		/	
;*		bit6=1		space
;*		bit7=1		Cr
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************
printff:
		push	temp0
		sbrc	temp0,0
		rjmp	printff_00
		sbrc	temp0,1
		rjmp	printff_01
		rjmp	printff_exit
printff_00:		
		sts	LongWord+1,tempchar
		clr	tempchar
		sts	LongWord+2,tempchar
		sts	LongWord+3,tempchar
		sts	LongWord,tempchar
		ldi	zl,low(LongWord)
		ldi	zh,high(LongWord)
		
printff_01:	
	
		ldi	xl,low(Str6)
		ldi	xh,high(Str6)
		rcall	Itoa
		ldi 	zl,low(Str6)			;Load strinpointer
		ldi 	zh,high(Str6)
		adiw	zl,1
		pop	temp0
		push	temp0
		sbrc	temp0,4
		adiw	zl,2
		rcall	PrintSoftU
		
	
		pop	temp0
		sbrc	temp0,5
		ldi	u_data,'/'
		sbrc	temp0,6
		ldi	u_data,32
		sbrc	temp0,7
		ldi	u_data,13
		sbrs	temp0,3
		rcall	putc

printff_exit:

ret

;***************************************************************************
;*
;* FUNCTION
;*	TempCompare
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************
TempCompare:
		;lds	temp0,LongWord
		
		;mov	u_data,temp0
		;rcall	putc
		;lds	temp1,Svolume+1
		;mov	u_data,temp1
		;rcall	putc
		lds	temp0,LongWord+1
		sts	GTemperature,temp0		;Store lowbyte
		
		
		
		lds	temp0,LongWord
		sts	GTemperature+1,temp0		;Store higbyte
		lds	temp1,Svolume+1
		cp	temp1,temp0
		brlo	tempCompare00_2		
		brne	tempCompare00_1	
		
		;mov	u_data,temp1
			
		;rcall	putc	
;tempCompare00_0:		
		lds	temp0,LongWord+1
		lds	temp1,Svolume
		cp	temp1,temp0
		brlo	tempCompare00_2
tempCompare00_1:
		cbi	portd,7		
		rjmp	tempCompare00_3
tempCompare00_2:
		sbi	portd,7
tempCompare00_3:


ret



;************************************************************************************
;*FUNCTION
;*  strtol:
;*
;*DESCRIPTION	Converts string to long integer  
;*
;*PARAMETER	Input: Pointer to string y, Output: str6 low byte,  str6+1  high byte
;*
;*
;************************************************************************************


atol:







;	clr	temp0
;	sts	str6,tempchar					;clear low byte
;	sts	str6+1,tempchar					;clear high byte
;	ldi 	zl,low(2*exponent)				;10 Exponent
;	ldi 	zh,high(2*exponent)



atol_10:


	ld	tempchar,y+		;Load byte from program memory into tempchar
	tst	tempchar		;Test if we have reached the end of the string
	breq	atol_20			;If not so, go to atol_20	
	breq	atolExit
	cpi	tempchar,48
	brlo	atol_10
	cpi	tempchar,58
	brsh	atol_10
	subi	tempchar,48
	push	tempchar
	inc	temp0
	rjmp	atol_10



	
atol_20:	
	clr	tempchar
	sts	str6,tempchar					;clear low byte
	sts	str6+1,tempchar					;clear high byte
	ldi 	zl,low(2*exponent)				;10 Exponent
	ldi 	zh,high(2*exponent)
	clc
atol_30:
	tst	temp0
	breq	atolExit
	pop	tempchar
	tst	tempchar
	breq	atol_40
	dec	tempchar
	push	tempchar
	;ld	tempchar,y+					;load exponent low byte
	lpm
	mov	tempchar,r0
	lds	temp1,str6
	adc	temp1,tempchar
	sts	str6,temp1
	;adiw	zl,1
	inc	zl
	lpm
	mov	tempchar,r0
	;ld	tempchar,y+					;loaad exponent high byte
	lds	temp1,str6+1
	adc	temp1,tempchar
	sts	str6+1,temp1
	sbiw	zl,1	
	rjmp	atol_30
		
	;tst	temp0
	;breq	atolExit
	;pop	temp1			;x10
	;add	tempchar,temp1
atol_40:
	dec	temp0
	adiw	zl,2
	rjmp	atol_30
	;rjmp	atol_10

atolExit:

	ret



;***************************************************************************
;*
;* FUNCTION
;*	rw_spi
;*
;* DESCRIPTION
;*	Write a word out on SPI while simultaneously reading in a word.
;*	Data is sent MSB-first, and info read from SPI goes into
;*	the same buffer that the write data is going out from.
;*	Make sure data, clock and /SS are init'd before coming here.
;*	SCK high time is ((delay * 3) + 1) AVR clock cycles.
;*
;*	If 8-bit use is needed, change  LDI TEMP,16  to ,8  and also
;*	eliminate the ROL SPI_HI statement.
;*
;* CODE SIZE:
;*	21 words
;* NUMBER OF CYCLES:
;*	Overhead = 8, loop = 16 * (16 + (2* (delay_value*3)))
;	(With call + return + delay=4, it is about 648 cycles.)
;*
;***************************************************************************

rw_spi:	
			;----Set Delay Value---------------------
	ldi	temp1,0x20	;0x20
	sts	DelayV,temp1
	ldi	temp1,0x50
	sts	DelayV+1,temp1
	;ldi	temp1,0x50	;0x01
	ldi	temp1,0x01
	sts	DelayV+2,temp1

;;			ldi	temp,16		;init loop counter to 16 bits
       	ldi	temp0,8		;use THIS line instead if 8-bit desired
	cbi	portb,sck
;;;;;;;;	cbi	portb,ss
;	rcall	delay
	;

spi_loop:
	lsl	tempchar	;	lsl	spi_lo		;move 0 into D0, all other bits UP one slot,
	

;;			rol	spi_hi		; and C ends up being first bit to be sent.
 ;If 8-bit desired, also comment out the preceding ROL SPI_HI statement
	;
	brcc	lo_mosi
	sbi	portb,mosi	;mosi_hi
	rjmp	mosi_done	;this branch creates setup time on MOSI
lo_mosi:
	cbi	portb,mosi	;mosi_lo
;	nop			;also create setup time on MOSI
mosi_done:
	;
	rcall	delay		;mod
;	set_delay temp0,7
;	time_hi:
;	inc_delay temp0		;inc upper nibble until it rolls over; then,
;	brcs	time_hi		; C gets CLEARED, & temp has original value
	sbi	portb,sck	;sck_hi
	;
 ;must now time the hi pulse - not much else we can do here but waste time
	;
	;set_delay temp0,4	;(4 * 3) cycle delay; range is from 1 to 7!
;	set_delay temp0,7
;time_hi:
;	inc_delay temp0		;inc upper nibble until it rolls over; then,
;	brcs	time_hi		; C gets CLEARED, & temp has original value

	rcall delay

;	set_delay temp0,7
;	time_lo:
;	inc_delay temp0
;	brcs	time_lo
	
	sbic	pinb,miso	;after delay, read in SPI bit & put into D0
	inc	tempchar	;inc	spi_lo		;we FORCED D0=0, so use INC to set D0.
	
	cbi	portb,sck	;sck_lo			;drop clock line low
	
 ;must now delay before reading in SPI data on MISO
	;
	;set_delay temp0,4
;	set_delay temp0,7
;time_lo:
;	inc_delay temp0
;	brcs	time_lo
;	rcall delay	
;	sbic	pinb,miso	;after delay, read in SPI bit & put into D0
;	inc	tempchar	;inc	spi_lo		;we FORCED D0=0, so use INC to set D0.
	
	dec	temp0
	brne	spi_loop
	;cbi	portb,mosi
;	ldi	temp1,0x20	;0x20
;	sts	DelayV,temp1
;	ldi	temp1,0x50
;	sts	DelayV+1,temp1
;	ldi	temp1,0x10	;0x01
;	sts	DelayV+2,temp1
;	rcall	delay
;;;;;;;	sbi	portb,ss
;	rcall	delay
;	cbi	portb,ss
;	rcall	delay
;	sbi	portb,ss
	ret
	
	
	
;***************************************************************************
;*
;* FUNCTION
;*	delay_us
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

delay_us:
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
ret	


;***************************************************************************
;*
;* FUNCTION
;*	delay_100us
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

delay_100us:
	push	temp1
		
	ldi	temp1,0x19	
	sts	DelayV,temp1
	ldi	temp1,0x03
	sts	DelayV+1,temp1
	ldi	temp1,0x01
	sts	DelayV+2,temp1
	
	rcall	delay
	
	pop	temp1
ret

;***************************************************************************
;*
;* FUNCTION
;*	OneWireReset
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

OneWireReset:
	ldi	temp1,0x20	
	sts	DelayV,temp1
	ldi	temp1,0x13
	sts	DelayV+1,temp1
	ldi	temp1,0x01
	sts	DelayV+2,temp1
	sbi	ddra,4
	cbi	porta,4
	rcall	delay
	


	sbi	porta,4			
master_relaxed:				
	sbis	pina,4			
	rjmp	master_relaxed		
slave_pulse_start:			
	sbic	pina,4			
	rjmp	slave_pulse_start  		
slave_relaxed:				
	sbis	pina,4			
	rjmp	slave_relaxed		
	rcall	delay			

	ret
	
;***************************************************************************
;*
;* FUNCTION
;*	OneWireWrite
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

OneWireWrite:
	ldi	temp1,0x19	
	sts	DelayV,temp1
	ldi	temp1,0x03
	sts	DelayV+1,temp1	
	ldi	temp1,0x01	
	sts	DelayV+2,temp1
	sbi	ddra,4	

       	ldi	temp0,8		


OneWireWrite_loop:
	lsr	tempchar	;	lsr		;move 0 into D0, all other bits 
	

	brcc	lo_wwdata
	cbi	porta,4
	rcall	delay_us
	sbi	porta,4		;wwdata_hi
	rcall delay		

	rjmp	wwdata_done	
lo_wwdata:
	cbi	porta,4		;wwdata_lo
	rcall	delay
	sbi	porta,4
	rcall	delay_us

		
wwdata_done:
	dec	temp0
	brne	OneWireWrite_loop
	ret
	
	
;***************************************************************************
;*
;* FUNCTION
;*	OneWireRead
;*
;* DESCRIPTION
;*
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

OneWireRead:

	clr	tempchar

       	ldi	temp0,8		


OneWireRead_loop:
	sbi	ddra,4
	cbi	porta,4
	rcall	delay_us
	cbi	ddra,4		
	cbi	porta,4
	rcall	delay_us
	rcall	delay_us	
	rcall	delay_us
	clc
	sbis	pina,4
	rjmp	OneWireReadDataDone
	sec

OneWireReadDataDone:
	ror	tempchar
	rcall	delay_100us	
	dec	temp0
	brne	OneWireRead_loop
	ret		

;***************************************************************************
;*
;* FUNCTION
;*	GetDs64BitCode
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

GetDs64BitCode:
		rcall	OneWireReset
		ldi	tempchar,0x33
		rcall	OneWireWrite
		
		rcall	OneWireRead
		mov	u_data,tempchar
		
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc
		rcall	OneWireRead
		mov	u_data,tempchar
		
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc
		rcall	OneWireRead
		mov	u_data,tempchar
		
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc
		rcall	OneWireRead
		mov	u_data,tempchar
		
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc
		ldi	tempchar,13
		mov	u_data,tempchar
		rcall	putc
		ret


	
;***************************************************************************
;*
;* FUNCTION
;*	ReadDs18s20
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

ReadDs18s20:	
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
		
		lpm				;Load byte from program memory into r0		
		st	x+,r0
		adiw	zl,1			;increase z
		lpm
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
		
		
		push	xl
		push	xh
		rcall	OneWireReset
;		ldi	tempchar,0xcc	;0xcc,0x33
;		rcall	OneWireWrite
;		ldi	tempchar,0x44	;0x44
;		rcall	OneWireWrite
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		ldi	temp0,9	;13
ReadDs18s20_loop0:		
		push	temp0
		ld	tempchar,x+
		rcall	OneWireWrite
		pop	temp0
		dec	temp0
		brne	ReadDs18s20_loop0
		
		ldi	tempchar,0x44	
		rcall	OneWireWrite



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;		
		
		
		
		
		
ReadDs18s20_loop:
		rcall	OneWireRead
		tst	tempchar
		breq	ReadDs18s20_loop
				
		rcall	OneWireReset
	

	;	ldi	tempchar,0xcc		
	;	rcall	OneWireWrite
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
		pop	xh
		pop	xl
	;	ldi	xl,low(phoneno)
	;	ldi	xh,high(phoneno)
		
		ldi	temp0,9	;13
ReadDs18s20_loop1:		
		push	temp0
		ld	tempchar,x+
		rcall	OneWireWrite
		pop	temp0
		dec	temp0
		brne	ReadDs18s20_loop1
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;		
		
		
		ldi	tempchar,0xbe
		rcall	OneWireWrite

		rcall	OneWireRead
		clc
		ror	tempchar
		push	tempchar
		mov	u_data,tempchar
	
		;;;;;;;;rcall	putc
		
		rcall	OneWireRead
		mov	u_data,tempchar
		
		;:::rcall	putc
		
		pop	tempchar

ret		
	;	rcall	OneWireRead
	;	mov	u_data,tempchar
		
	;	rcall	putc

	;	rcall	OneWireRead
	;	mov	u_data,tempchar
	;	rcall	putc

	;	rcall	OneWireRead
	;	mov	u_data,tempchar
	;	rcall	putc

	;	rcall	OneWireRead
	;	mov	u_data,tempchar
	;	rcall	putc

	;	rcall	OneWireRead
	;	mov	u_data,tempchar
	;	rcall	putc

	;	rcall	OneWireRead
	;	mov	u_data,tempchar
	;	rcall	putc
		
;ret
;		rcall	OneWireRead
;		mov	u_data,tempchar
;		rcall	putc
	;	rcall	OneWireRead
	;	mov	u_data,tempchar
	;	rcall	putc
;ret


;***************************************************************************
;*
;* FUNCTION
;*	Ds2406
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

Ds2406:	
		rcall	OneWireReset
		ldi	tempchar,0xcc	;0x33
		rcall	OneWireWrite
		ldi	tempchar,0x55	;0x44
		rcall	OneWireWrite
		
		ldi	tempchar,0x07	;test
		rcall	OneWireWrite
		ldi	tempchar,0x00	;test
		rcall	OneWireWrite
		ldi	tempchar,0x0e	;0x3f	;test
		rcall	OneWireWrite
		
		rcall	OneWireRead
		mov	u_data,tempchar
		
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc
		
		
		ret
		
		
Ds2406_loop:
	;	rcall	OneWireRead
	;	tst	tempchar
	;	breq	Ds2406_loop	
	
	;	rcall	OneWireReset
	;	ldi	tempchar,0xcc		
	;	rcall	OneWireWrite
	;	ldi	tempchar,0xbe
	;	rcall	OneWireWrite

	;	rcall	OneWireRead
	;	clc
	;	ror	tempchar
	;	push	tempchar
	;	mov	u_data,tempchar
	
	;	rcall	putc
		
	;	rcall	OneWireRead
	;	mov	u_data,tempchar
		
	;	rcall	putc
		
	;	pop	tempchar

;ret		
		rcall	OneWireRead
		mov	u_data,tempchar
		
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc
		
;ret
		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc
		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc
ret




;***************************************************************************
;*
;* FUNCTION
;*	Ds2406_pioaON
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

Ds2406_pioaON:	
		rcall	OneWireReset
		
		ldi	temp0,13	
		;ldi	tempchar,0xcc	;
		;rcall	OneWireWrite
	;	ldi	tempchar,0x55	;
	;	rcall	OneWireWrite
		
		
		
	;	ldi	tempchar,0x12	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0xa0	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0x1f	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0x73	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0x00	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0x00	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0x00	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0xee	;
	;	rcall	OneWireWrite
		
		
	;	ldi	xl,low(phoneno)
	;	ldi	xh,high(phoneno)	
Ds_loop:		
		push	temp0
		ld	tempchar,x+
		rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite	
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite


	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite


	
		
;		ldi	tempchar,0x55	;
;		rcall	OneWireWrite
		
;		ldi	tempchar,0x07	;
;		rcall	OneWireWrite
;		ldi	tempchar,0x00	;
;		rcall	OneWireWrite
;		ldi	tempchar,0x0e	;
;		rcall	OneWireWrite
		pop	temp0
		dec	temp0
		brne	Ds_loop
		rcall	OneWireRead
		mov	u_data,tempchar
		
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc
		
		
		ret			
		
		
;***************************************************************************
;*
;* FUNCTION
;*	Ds2406_pioaOFF
;*
;* DESCRIPTION
;*
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

Ds2406_pioaOFF:	
		rcall	OneWireReset
		ldi	tempchar,0xcc	;
		rcall	OneWireWrite
		ldi	tempchar,0x55	;
		rcall	OneWireWrite
		
		ldi	tempchar,0x07	;
		rcall	OneWireWrite
		ldi	tempchar,0x00	;
		rcall	OneWireWrite
		ldi	tempchar,0x6d	;
		rcall	OneWireWrite
		
		rcall	OneWireRead
		mov	u_data,tempchar
		
		rcall	putc

		rcall	OneWireRead
		mov	u_data,tempchar
		rcall	putc
		
		
		ret	
		
		
;***************************************************************************
;*
;* FUNCTION
;*	Ds2406_Status
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

Ds2406_Status:	
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
		
		lpm				;Load byte from program memory into r0		
		st	x+,r0
		adiw	zl,1			;increase z
		lpm
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		adiw	zl,1			;increase z
		lpm	
		st	x+,r0
		
		
		ldi	xl,low(phoneno)
		ldi	xh,high(phoneno)
		
		rcall	OneWireReset
		
		ldi	temp0,11 ;12;13	
		;ldi	tempchar,0xcc	;
		;rcall	OneWireWrite
	;	ldi	tempchar,0x55	;
	;	rcall	OneWireWrite
		
		
		
	;	ldi	tempchar,0x12	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0xa0	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0x1f	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0x73	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0x00	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0x00	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0x00	;
	;	rcall	OneWireWrite
	;	ldi	tempchar,0xee	;
	;	rcall	OneWireWrite
		
		
	;	ldi	xl,low(phoneno)
	;	ldi	xh,high(phoneno)	
DsStatus_loop:		
		push	temp0
		ld	tempchar,x+
		rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite	
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite


	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite
	;	ld	tempchar,x+
	;	rcall	OneWireWrite


	
		
;		ldi	tempchar,0x55	;
;		rcall	OneWireWrite
		
;		ldi	tempchar,0x07	;
;		rcall	OneWireWrite
;		ldi	tempchar,0x00	;
;		rcall	OneWireWrite
;		ldi	tempchar,0x0e	;
;		rcall	OneWireWrite
		pop	temp0
		dec	temp0
		brne	DsStatus_loop
		
	
		
		rcall	OneWireRead
;		mov	u_data,tempchar
		
;		rcall	putc

		rcall	OneWireRead
		;mov	temp1,tempchar
		;mov	u_data,tempchar
		;rcall	putc
;		rcall	OneWireRead
;		mov	u_data,tempchar
		
;		rcall	putc

;		rcall	OneWireRead
;		mov	u_data,tempchar
;		rcall	putc
		
		
	
	;	rcall	OneWireRead
;		ldi	tempchar,0x0d
;		mov	u_data,tempchar
		
;		rcall	putc

	;	rcall	OneWireRead
	;	mov	u_data,tempchar
	;	rcall	putc
	;	rcall	OneWireRead
	;	mov	u_data,tempchar
		
	;	rcall	putc

	;	rcall	OneWireRead
	;	mov	u_data,tempchar
	;	rcall	putc
		ret		
		

;***************************************************************************
;*
;* FUNCTION
;*	PumpOnOff
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

PumpOnOff:

		ldi 	zl,low(pumpflag)		;Load strinpointer
		ldi 	zh,high(pumpflag)
		rcall	EERead	
		sbrs	tempchar,0			;Skip if bit0 is set
		rjmp	PumpOnOff_00
;-------------------PumpOff----------------------------------------------------
		ldi	tempchar,0
		ldi 	zl,low(pumpflag)		;Load strinpointer
		ldi 	zh,high(pumpflag)
		rcall	EEWrite	
		rcall	LongDelay	
		ldi	u_data,'O'
		rcall	Putc
		ldi	u_data,'f'
		rcall	Putc
		ldi	u_data,'f'
		rcall	Putc

		rjmp	PumpOnOff_Exit	
;-------------------PumpOn------------------------------------------------------
PumpOnOff_00:
		ldi 	zl,low(pumpflag)		;Load strinpointer
		ldi 	zh,high(pumpflag)
		ldi	tempchar,1
		rcall	EEWrite	
		rcall	LongDelay
		ldi	u_data,'O'
		rcall	Putc
		ldi	u_data,'n'
		rcall	Putc
		rjmp	PumpOnOff_Exit	
	
	
PumpOnOff_Exit:	

		clr	temp0			 
		sts	Count,temp0		;clear time counter
		sts	TempBuf,temp0
		rcall	timer1_init

	ret

;***************************************************************************
;*
;* FUNCTION
;*	SetRefTemp
;*
;* DESCRIPTION
;*t
;*
;* USAGE
;*	Call this function 
;*	
;*
;*   PARAMETER	
;*	
;*	
;*	
;*	
;*		
;*		
;*			
;*		
;*			
;*	
;*		
;* RETURN
;*
;*
;* NOTE
;*	
;*
;*
;* COMMENT
;*	
;*
;***************************************************************************

SetRefTemp:
	ldi	tempchar,0xff
	sts	DelayV,tempchar
	ldi	tempchar,0xff
	sts	DelayV+1,tempchar
	ldi	tempchar,0x4
	sts	DelayV+2,tempchar
	rcall	delay
	ldi	u_data,254
	rcall	putc
	ldi	u_data,88
	rcall	putc
	ldi	u_data,'R'
	rcall	putc
	ldi	u_data,'e'
	rcall	putc
	ldi	u_data,'f'
	rcall	putc
	ldi	u_data,':'
	rcall	putc
	lds 	tempchar,RefTemp
	inc	tempchar
	cpi	tempchar,41	
	brlo	SetRefTemp_0
	ldi	tempchar,10
	
	
	
SetRefTemp_0:
	sts	RefTemp,tempchar
	
	ldi	temp0,81;17;49	
	rcall	printff	
	
	clr	temp0			 
	sts	Count,temp0		;clear time counter
	sts	TempBuf,temp0
	rcall	timer1_init
	
	ret

	
;----------------------------------------------------------------------------------		
;iotest:
;		ldi	u_data,65
;		rcall	putc	
;		clr	temp0			 
;		sts	Count,temp0		;clear time counter
;		sts	TempBuf,temp0
;		rcall	timer1_init
;	ret							
;-------------------------------------------------------------------------------------------







Clock:	  	.db "clock",0
ClearAC:  	.db "ClearAC",0
ReadCC:   	.db "ReadCC",0
ClearCC: 	.db "ClearCC",0
TrimBits: 	.db "TrimBits",0
Ver:      	.db "ver",0
;eeprom:		.db "eeprom",0
eeprom:		.db "test",0
SUART:	  	.db "send=",0 ;su=
;sendhex:	.db "sendhex=",0
baud:		.db "baud=",0
modeno:   	.db "mo=",0 ;#m=
getsubuf: 	.db "#getsubuf",0
OK:       	.db "OK",13,0
Error:    	.db "ERROR",0
cpiready: 	.db "+CPIN: READY",0 
cmti:	  	.db "+CMTI: ",34,"SM",34,",",0
cmti2:		.db "+CMTI: ",34,"SM",34,",2",0
;cmgl1:    	.db "+CMGL: 1",0
cmgl1:    	.db "+CMGL: ",0
cmgr:		.db "+CMGR:",0
unread:	  	.db "UNREAD",34,",",34,"+",0	
comma2:   	.db 34,",,",34,0
nocarrier:	.db "NO CARRIER",0
ring:		.db "RING",0
connect:	.db "CONNECT",0
lamptime:	.db "lampt",0
;absent:	.db "absent=",0
sspi:		.db "sspi=",0
stxetx:		.db "stxetx=",0
stxetx0:	.db "stxetx0=",0
ack:		.db 13
poll:		.db "poll",0
settargetpno:	.db "SetTargetPno=",0
setpasspno:	.db "SetPassPno=",0
setcustomerno:	.db "SetCustomerNo=",0	
settempref:	.db "SetTempRef=",0
;---------------Text--------------------------------------------------------------------
Text:	  	.db "Copyright (c) 2004 Jarl&Jarmo",13,0
;Text:	  	.db "20081021",13,0
;Text:	  	.db "Billingehus Version 1.00",13,0
;Text:	  	.db "Hjortviken Version 1.00",13,0
;Text:	  	.db "Lundsbrunn Version 1.00",13,0
;Text:	  	.db "Smögen Version 1.00",13,0
;Text:	  	.db "SAS Radison Version 1.00",13,0
;Text:	  	.db "Chalmers Version 1.1",13,0
Text1:	  	.db " H   ",0
Text2:	  	.db " L",13,0

;----------------At Commands-----------------------------------------------------------
ate0:     	.db "ate0",13,0	 
atcicb:   	.db "at+cicb=0",13,0
;atcpin:   	.db "at+cpin=",34,"2978",34,13,0
;atcsca:	.db "at+csca=",34,"46705008999",34,13,0		;Sms-central Telia
atcpin:   	.db "at+cpin=",34,"1200",34,13,0
atcpinask:	.db "at+cpin?",13,0
atcsca:		.db "at+csca=",34,"+46708000708",34,13,0	;Sms-central Vodafon
atcmgs:		.db "at+cmgs=",34,0
ats01:	  	.db "ats0=2",13,0				;Automatic answer after 2 rings
ats00:		.db "ats0=0",13,0				;No Automatic answer
atcmgd:   	.db "at+cmgd=1,4",13,0
atcmgd0:	.db "at+cmgd=1,2",13,0				;1,3
atcpina:  	.db "at+cpin?",13,0
atcmgf1:	.db "at+cmgf=1",13,0
atcmgl:	  	.db "at+cmgl=",34,"ALL",34,13,0	
atcmgr1:	.db "at+cmgr=1",13,0
atcmgr2:	.db "at+cmgr=2",13,0
atcmgr3:	.db "at+cmgr=3",13,0
atcmgr4:	.db "at+cmgr=4",13,0
atcmgr5:	.db "at+cmgr=5",13,0
atcmgr:		.db "at+cmgr=",0
atcpof:		.db "at+cpof",13,0
cpinready:	.db "+CPIN: READY",13,10,0
;----------------SHARP Cammands----------------------------------------------------------------
err:		.db "ERR",13,0					;Error code
tlps:		.db "TLPS   1",13,0				;Get LampStatus
tltt:		.db "TLTT   1",13,0				;Get usetime
powr1:		.db "POWR   1",13,0				;Power on
powr0:		.db "POWR   0",13,0				;Power off
rgb1:		.db "IRGB   1",13,0				;RGB1
rgb2:		.db "IRGB   2",13,0				;RGB2
video:		.db "IVED   1",13,0				;Video
svideo:		.db "IVED   2",13,0				;S-Video
muteOn:		.db "IMBK   1",13,0				;Mute on
muteOff:	.db "IMBK   0",13,0				;Mute off
getived:	.db "IVED????",13,0				;Get input source (3,4)
getirgb:	.db "IRGB????",13,0				;Get input source (1,2)
volume:		.db "VOLA  ",0					;Volume 0-60
ichk:		.db "ICHK????",13,0				;Get input source
;---------------TOSHIBA and PANASONIC Commands----------------------------------------------------------------
;powon:		.db 02,"PON",03,0				;Power on
;powoff:		.db 02,"POF",03,0				;Power off
;rgbt:		.db 02,"IN1",03,0				;RGB (Toshiba)
;videot:		.db 02,"IN3",03,0				;Video (Toshiba)
;panvideo:	.db 02,"IIS:VID",03,0				;Video (Panasonic)
;panrg1:		.db 02,"IIS:RG1",03,0				;RG1 (Panasonic)
;panGetstatus:	.db 02,"Q$S",03,0				;Get status (Panasonic)
;---------------Pioneer Commands-------------------------------------------------------------------------
;pon:		.db 02,"01PON",03,0				;Turn power on
;pof:		.db 02,"01POF",03,0				;Turn power off
;in1:		.db 02,"01IN1",03,0				;Selects Input1-Data
;in4:		.db 02,"01IN4",03,0				;Selects Input4-Data
;gso:		.db 02,"01GSO",03,0				;Get status opton
;gss:		.db 02,"01GSS",03,0				;Get status set up
;vol:		.db 02,"01VOL0",0				;Adjusts audio volume 0-60
;pwn:		.db "P",0;PWN1
;pwn:		.db "PLASMA",0
;gssin1:		.db "IN1",0					;Data
;gssin4:		.db "IN4",0					;Video		
;---------------VievSonic Commands-------------------------------------------------------------------------------
;vson:		.db 0xbe,0xef,0x03,0x06,0x00,0xba,0xd2,0x01,0x00,0x00,0x60,0x01,0x00	;Turn on
;vsof:		.db 0xbe,0xef,0x03,0x06,0x00,0x2a,0xd3,0x01,0x00,0x00,0x60,0x00,0x00	;Turn off
;vsrgb1:		.db 0xbe,0xef,0x03,0x06,0x00,0xfe,0xd2,0x01,0x00,0x00,0x20,0x00,0x00	;Select source RGB IN1
;vsrgb2:		.db 0xbe,0xef,0x03,0x06,0x00,0x3e,0xd0,0x01,0x00,0x00,0x20,0x04,0x00	;Select source RGB IN2
;vsvideo:	.db 0xbe,0xef,0x03,0x06,0x00,0x6e,0xd3,0x01,0x00,0x00,0x20,0x01,0x00	;Select source VIDEO
;vsvolumeinc:	.db 0xbe,0xef,0x03,0x06,0x00,0x57,0xd3,0x04,0x00,0x01,0x20,0x00,0x00	;Icrement volume
;vsvolumedec:	.db 0xbe,0xef,0x03,0x06,0x00,0x86,0xd2,0x05,0x00,0x01,0x20,0x00,0x00	;Decrement volume
;vsgetOnOff:	.db 0xbe,0xef,0x03,0x06,0x00,0x19,0xd3,0x02,0x00,0x00,0x60,0x00,0x00	;Get On/Off status
;vsgetsource:	.db 0xbe,0xef,0x03,0x06,0x00,0xcd,0xd2,0x02,0x00,0x00,0x20,0x00,0x00	;Get source
;----------------Barco Commands---------------------------------------------------------------------------
;barcoOn:	.db 0xfe,0x01,0x65,0x66,0xff						;Set projector on 
;barcoOff:	.db 0xfe,0x01,0x66,0x67,0xff						;Set the projector off
;barcoVoldec:	.db 0xfe,0x01,0x23,0x07,0x2b,0xff					;Decrement volume
;barcoVolincr:	.db 0xfe,0x01,0x22,0x07,0x2a,0xff					;Increment volume
;barcoGetstatus:	.db 0xfe,0x01,0x67,0x68,0xff						;Read the projector status
;barcoSetslot2:	.db 0xfe,0x01,0x31,0x02,0x33,0xff					;Select source 2 of projector
;barcoSetslot3:	.db 0xfe,0x01,0x31,0x03,0x34,0xff					;Select source 3 of projector
;barcoSetslot2:	.db 0xfe,0x01,0x33,0x04,0x01,0x39,0xff					;Select source 2 of projector
;barcoSetslot2:	.db 0xfe,0x01,0x33,0x03,0x00,0x37,0xff					;Select source 2 of projector
;barcoSetslot3:	.db 0xfe,0x01,0x33,0x01,0x05,0x3a,0xff					;Select source 3 of projector
;barcoSetslot3:	.db 0xfe,0x01,0x33,0x02,0x00,0x36,0xff					;Select source 3 of projector
;------------------TempSensor Adress-------------------------------------------------
tempa0:		.db 0x55,0x10,0x84,0xef,0xf2,0x01,0x08,0x00,0xd3
tempa1:		.db 0x55,0x10,0x9a,0x3b,0x15,0x02,0x08,0x00,0xf1
tempa2:		.db 0x55,0x10,0x78,0x1b,0x15,0x02,0x08,0x00,0x4e
;-------------------Button Address---------------------------------------------------
ButtonIoA:	.db 0x55,0x12,0xd0,0x23,0x73,0x00,0x00,0x00,0xd3,0xf5,0xf2
ButtonIoA0:	.db 0x55,0x12,0xa0,0x1f,0x73,0x00,0x00,0x00,0xee,0xf5,0xf2
ButtonIoA1:	.db 0x55,0x12,0x2f,0x22,0x73,0x00,0x00,0x00,0x48,0xf5,0xf2
;---------------Time parameter---------------------------------------------------------------------------
yearC:		.db 128,51,225,1	;1,225,51,128		;365 days=31536000
dayC:		.db 128,81,1,0					;86400 sec 
hourC:		.db 16,14,0,0					;3600 sec
minuteC:	.db 60,0,0,0					;60 sec
weekC:		.db 7,0,0,0					;7 days=week

tempC:		.db 42,0,0,0
tempGC0:	.db 108,0,0,0					;temp graf constant 100
tempGC1:	.db 86,0,0,0
tempGC2:	.db 72,0,0,0

;breakPoint0:	.db 192,168,0,0
;breakPoint0:	.db 40,135,0,0
breakPoint0:	.db 192,168,0,0
breakPoint1:	.db 32,222,1,0
;------------------10 exponent----------------------------------------------------
exponent:	.db 1,0,10,0,100,0,232,3
;-----------------Securety---------------------------------------------------------------------------------------
pslist:		.db "+46703962707 +46734430336 +46761260395 +46704226260 +46703204141 +46707434488 +46735081012 +46705189115 +46761048668 +46761281610 +46733582926 +46761307803",0
;pslist:		.db "+46703962707 +46734430336 +46733582926",0

;----------------EEPROM DATA---------------------------------------------------------------------
.eseg
;.org 0x190
;.org 0x50
;		.db 0xff,0xff,0xff,0xff
;----------------Securety-------------------------------------------------------------


baudrate:	.db 5			;Default baud-rate for hardware uart 115200,11059200/16/115200-1=5 (Calculate  baud-rate for hardware-UART,crystal 11.059200Mhz)
;-------------Parameters for sofuart-------------------------------------------------------
					;Default baud-rate for softuart 9600 
baudrates:	.db 2,184,155		;R,BL1,BL15  	19200 
;baudrates:	.db 2,107,66		;R,BL1,BL15 	9600

					;R=2
					;N=170
					;N1=13
					;N2=45
					;BL1=(256-N+N1+8)=107
					;BL15=256-(N+N/2)+8+12+N2)=66
					
;-------------Initregister bit0 to 7-------------------------------------------------------
ireg:		.db 5;7;39;7;3;3;1;0		;bit0=0 poll off, bit0=1 poll on
						;bit1=0 phoneinit inactive, bit1=1 phoneinit active
						;bit2 start up flag
						;bit5 ButtonLockFlag bit5=0 Button inactive, bit5=1 Button active
;-------------ButtonFlag bit0 to 7-------------------------------------------------------------------------------
buttonFlag:     .db 0				;bit0=0 data2 off, bit0=1 data2 on		
;------------FieldOutputFlag-----------------------------------------------------------
fieldFlag:	.db 1				;bit0=0 CustomerNo off,bit0=1 CustomerNo on 						
;-----------Target Phoneno---------------------------------------------------------------------------------------
;targetPno:	.db "+46703962707",0		;Target Phoneno
;targetPno:	.db "+46703962707",0,0,0,0,0	;Target Phoneno
targetPno:	.db "+46733582926",0,0,0,0,0	;Target Phoneno panel1
;targetPno:	.db "+46761030969",0,0,0,0,0	;Target Phoneno
;targetPno:	.db "+46761048668",0,0,0,0,0	;Target Phoneno panel2
;targetPno:	.db "+46761260395",0,0,0,0,0	;Target Phoneno panel3
;targetPno:	.db "+46761307803",0,0,0,0,0	;Target Phoneno panel4
;targetPno:	.db "+46761063382",0,0,0,0,0	;Target Phoneno panel5
;targetPno:	.db "+46761259649",0,0,0,0,0	;Target Phoneno panel6
;-------------------------------pumpflag---------------------------------------------
pumpflag:	.db 1				;bit0=0 off,bit0=1 on
;-------------------------------tempref----------------------------------------------
tempref:	.db 25
;------------------------------------------------------------------------------------------
EEText1:	.db "POWR   1",13,0
EEText2:	.db "POWR   0",13,0
EEText3:	.db "IVED   1",13,0
EEText4:	.db "IRGB   1",13,0
EEText5:	.db "",13,0
EEText6:	.db "",13,0
;-----------------------------Securety List--------------------------------------

;pslistE:	.db "+46703962707 +46733582926 +46761281610",0,0,0,0,0,0,0,0	;Panel1
;pslistE:	.db "+46703962707 +46761048668 +46761281610",0,0,0,0,0,0,0,0	;Panel2
;pslistE:	.db "+46703962707 +46761260395 +46761281610",0,0,0,0,0,0,0,0	;Panel3
;pslistE:	.db "+46703962707 +46761307803 +46761281610",0,0,0,0,0,0,0,0	;Panel4
;pslistE:	.db "+46703962707 +46761063382 +46761281610",0,0,0,0,0,0,0,0	;Panel5
;pslistE:	.db "+46703962707 +46761259649 +46761281610",0,0,0,0,0,0,0,0	;Panel6
;pslistE:	.db "+46703962707 +46705189115 +46761259649 +46761063382 +46761281610 +46761307803 +46761260395 +46761048668 +46733582926",0,0,0,0,0,0,0,0
pslistE:	.db "+46703962707 " 
		.db "+46705189115 "
		.db "+46761259649 "
		.db "+46761063382 "
		.db "+46761281610 "
		.db "+46761307803 "
		.db "+46761260395 "
		.db "+46761048668 "
		.db "+46733582926 "
		.db "             ",0
;-----------------------------CustomerNo-----------------------------------------
customernoE:	.db "CustomerNo",0,0,0,0,0,0,0,0,0,0	

;Obs!!!!! Modifiera send= , lägg till ReadSoftUart

;********************************************************************
;* End of file
;********************************************************************