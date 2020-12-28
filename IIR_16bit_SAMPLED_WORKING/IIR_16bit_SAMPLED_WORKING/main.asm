.include "./m328Pdef.inc"		;assembly inst set for 328p
.equ F_CPU = 16000000			;speed of processor (16M)
.equ	baud	= 9600			; baudrate
.equ	bps	= (F_CPU/16/baud) - 1	; baud prescale

///////REGISTER DEFINITIONS/////////////

.DEF AC0 = R3	;LOW BYTE ACCUMULATOR
.DEF AC1 = R4	;MIDDLE
.DEF AC2 = R5	;MIDDLE
.DEF AC3 = R6	;HIGH
.DEF IR0 = R16	;for intermediate results of multiplication low byte
.DEF IR1 = R17
.DEF IR2 = R18
.DEF IR3 = R19	;highiest byte for intermediate results
.DEF DATAL = R20 ;LOW BYTE OF DATA FROM ADC
.DEF DATAH = R21 ;HIGH BYTE OF DATA
.DEF COEFFL = R22 ; LOW BYTE OF COEFFICIENT
.DEF COEFFH = R23 ; HIGH BYTE OF COEFFICIENT
.DEF XNL = R14	;used to hold current x[n] value
.def XNH = R15	
.DEF LOOPING = R8	;LOOPS FOR NUMBER OF SAMPLES
.DEF EMPTY = R2	  ; CLEARED REGISTER
.def tempr = r24
.DEF OUT_VAL = r13
.DEF FIRST_CHECK = R25

.equ number_of_samples = 100
.equ number_of_tx = number_of_samples*2
.equ data_size_bytes = 2
.equ I0 = 0x0100						;I0 is pointer to adress of input data  HIGH BYTE X[N-2]
.equ I1 = 0x0200				     	;I1 is pointer to adress of outputs HIGH BYTE Y[N-2]
.equ I1_YN = I1 + 2*data_size_bytes		;POINT TO FIRST RESULT CALCULATED
.equ I0_XN = I0 + 2*data_size_bytes		;POINT TO FIRST DATA VALUE RECEIVED


/////////////INTERRUPT VECTOR DEFINITION///////////
.cseg 
.org 0						//on reset interrupt
	jmp reset	
.org ADCCaddr				//on adc complete conversion interupt
	jmp ISR_ADC_COMPLETE

reset:
	CLR R2

/////////////MACRO DEFINITIONS/////////////
.macro InReg 
    .if @1 < 0x40 
        in @0, @1 
    .elif ((@1 >= 0x60) && (@1 < SRAM_START))  
        lds @0,@1 
    .else 
       .error "InReg: Invalid I/O register address" 
    .endif 
.endmacro 

; usage: OUTREG addr, reg 
.macro OUTREG 
    .if @0 < 0x40 
        out @0, @1
    .elif ((@0 >= 0x60) && (@0 < SRAM_START))
        sts @0,@1 
    .else 
       .error "OUTREG: Invalid I/O register address" 
    .endif 
.endmacro

.macro send_c 
	wait3:
		LDS tempr, UCSR0A			//load Uart status REG
		SBRS tempr, UDRE0			//Check empty bit
		rjmp wait3    				//if not empty keep waiting

	OUTREG UDR0, OUT_VAL
.endm 

.MACRO MUL16_32_SIGNED
	MULS COEFFH, DATAH		;signed byte of coeff * signed byte of data
	MOVW IR3:IR2, R1:R0
	MUL COEFFL, DATAL		;unsigned byte of coeff and data
	MOVW IR1:IR0, R1:R0
	MULSU COEFFH, DATAL		;signed byte coefficient * unsigned byte data
	SBC  IR3, EMPTY
	ADD	IR1, R0
	ADC	IR2, R1
	ADC	IR3, EMPTY
	MULSU	DATAH, COEFFL	; signed byte data * unsigned byte coeff
	SBC	IR3, EMPTY
	ADD	IR1, R0
	ADC	IR2, R1
	ADC	IR3, EMPTY
.ENDM 

//////////////////SET UP POINTERS AND CLEAN DATA/////////////////

;memory arranged as high-low-high-low
//CLEAN MEMORY LOCATIONS FOR INPUT DATA
	LDI XL,LOW(I0)		;make X point to x[n-2] High byte
	LDI XH,HIGH(I0)

	ST X+, EMPTY		;CLEAR X[N-2]
	ST X+, EMPTY
	ST X+, EMPTY		;CLEAR X[N-1]
	ST X+, EMPTY		;X NOW POINTS TO HIGH BYTE X[N]

//CLEAN MEMORY LOCATIONS FOR OUTPUT
	LDI YL,LOW(I1)		;make Y point to y[n-2] High byte	
	LDI YH,HIGH(I1)

	ST Y+, EMPTY		;CLEAR Y[N-2]
	ST Y+, EMPTY
	ST Y+, EMPTY		;CLEAR Y[N-1] 
	ST Y+, EMPTY		;Y NOW POINTS TO HIGH BYTE Y[N]

/////////// DEFINE COEFFICIENTS VIA POLE/ZERO PLACEMENT//////////

/*
//LOW PASS /

// ALPHA0 = -0.9 +J0			ALPHA1 = -0.9 - J0
// BETA0 = 0.3738945891 + j 0.3638925310	BETA1 = 0.3738945891 - j 0.3638925310
//a=-0.9 b=0 c= 0.3738945891  d= 0.3638925310
// a[1] = 2a  a[2] = e = a^2 + b^2
// b[1] = 2c	b[2] = f

// a[1] = 1.8  a[2] = 0.81
// b[1] = 0.7477891782	b[2] = 0.272214937875

// if we use 16bit signed coeffcients we can represent -2^15 to 2^15
// we can't scale by 2^15 becouse 1.8*2^15 is outside the range
//so we need to multiply by 2^14  

// scaled coefficients
// a[1] = 29491  a[2] = 13271
// b[1] = 12252	b[2] = 4460

	.equ A1 = 29491
	.equ A2 = 13271
	.equ B1 = 12252
	.equ B2 = 4460 
*/
	.equ A1 = -29491			//positive --> high pass, negative --> lowpass
	.equ A2 = 13271
	.equ B1 = 12252
	.equ B2 = 4460

//////////////SET UP UART, ADC AND TIMER ////////////////

;INIT UART
	LDI R25, HIGH(bps)
	LDI R24, LOW(bps)

	OUTREG UBRR0H, R25		// Set Baud rate
	OUTREG UBRR0L, R24		
	
	ldi r24, (1<< TXEN0) | 	(1<< RXEN0)		  // Enable transmitter and receivier 
	OUTREG UCSR0B,r24

	ldi r24, (0<<USBS0) | (1<<UCSZ01)	|	(1<<UCSZ00)	  // frame format: 8data, 1 stop bit
	OUTREG UCSR0C,r24

;Turnoff digital buffer at ADC pin
	LDI tempr,(1<<ADC0D)
	STS DIDR0, tempr

;SET_UP_ADC_W_TRIGGER

	LDI tempr, (1<<ADPS2) | (0<<ADPS1) |(1<<ADPS0)		//set prescaler (110 for clk/64)   (101 for /32)  (100 for /16)
	STS ADCSRA, tempr

	LDI tempr, (1<<REFS0) | (0<<ADLAR)		//set reference voltage, select channel and justified
	STS ADMUX, tempr

	LDS tempr, ADCSRA 
	ORI tempr, (1<<ADATE) 					// enable autotriggering
	STS ADCSRA, tempr

	LDS tempr, ADCSRA						//enable ADC
	ORI tempr, (1<<ADEN)
	STS ADCSRA, tempr

	LDS tempr, ADCSRA			//enable trigger for ADC conversion complete
	ORI tempr, (1<<ADIE)
	STS ADCSRA, tempr

	LDI tempr, (0<<ADTS2) | (1<<ADTS1) | (1 << ADTS0)	//set source that decides when an new ADC conversiion starts  (011 for timer)  (000 free running)
	STS ADCSRB, tempr 

;Init timer 0 and compA

	ldi tempr, 0x00			//clear contents of timer 0
	OUT TCNT0, tempr

	ldi tempr, 55			//set required value to output compare register
	OUT OCR0A,tempr

	ldi tempr, (1<<WGM01)	//set timer for ctc (clear timer when matches compare) mode (WGM01 = 1)
	OUT TCCR0A,tempr	

;START_TIMER0

	LDI tempr, (0<<CS02) | (1<<CS01) | (0<<CS00)	//select prescalar using bits 2,1,0 of TCCR0B and start timer.   (011 FOR CLK/64) (010 for /8)
	OUT TCCR0B, tempr

	
;START FIRST CONVERSION ADC

	LDS tempr, ADCSRA									
	ORI tempr, (1<<ADSC)
	STS ADCSRA, tempr

;enable global interrupts

	SEI

;MAIN loop while all samples are taken
	LDI FIRST_CHECK, 0X01
	LDI tempr, number_of_samples
	MOV looping, TEMPR
main:
	
	CP looping, EMPTY
	
BRNE main

;Disable global interrupts
	CLI

;Disable ADC 
	LDI tempr,(0<<ADEN) | (0<<ADATE) | (0<<ADIE) 
	STS ADCSRA, tempr
	
;Stop timer
	LDI tempr, (0<<CS00) | (0<<CS01) | (0<<CS02)	
	OUT TCCR0B, tempr

;SEND RESULTS via USART

	LDI YL, LOW(I1_YN)		//make Y point to first Y[n] again
	LDI YH, HIGH(I1_YN)

	LDI R16, 0				//R16 holds how many results we have sent
	LDI R17, number_of_tx	//R17 holds total number of results taken

start_TX:

	wait:
		CP	R16, R17
		BREQ finish_TX
		LDS tempr, UCSR0A			//load Uart status REG
		SBRS tempr, UDRE0			//Check empty bit
		rjmp wait    				//if not empty keep waiting

	LD OUT_VAL, Y+				//load value pointed by y and send it out
	STS UDR0, OUT_VAL
	 
	INC R16				//Increment the "number of results sent" counter
	rjmp start_TX

finish_TX:

;SEND DISTINGUISH BITS via USART

	LDI tempr, 0xAB
	mov OUT_VAL, tempr
	LDI tempr, 20
	mov looping, tempr
	loopdist:
		send_c
		DEC looping
	BRNE loopdist

;SEND data via USART

	LDI XL, LOW(I0_XN)		//make x point to first x[n] again
	LDI XH, HIGH(I0_XN)

	LDI R16, 0				//R16 holds how many results we have sent
	LDI R17, number_of_tx	//R17 holds total number of results taken

start_TX2:

	wait2:
		CP	R16, R17
		BREQ finish_TX2
		LDS tempr, UCSR0A			//load Uart status REG
		SBRS tempr, UDRE0			//Check empty bit
		rjmp wait2    				//if not empty keep waiting

	LD OUT_VAL, X+				//load value pointed by x and send it out
	STS UDR0, OUT_VAL
	 
	INC R16				//Increment the "number of results sent" counter
	rjmp start_TX2

finish_TX2:

//infinite loop. end of program
	end_all:		

	jmp end_all


 //////////ISR DEFINITIONS/////////////

 ISR_ADC_COMPLETE:

  LDI tempr, (1<<OCF0A)		//clear timer interrupt flag
  OUTREG TIFR0, tempr 

  CPI FIRST_CHECK, 1			//check if it's first conversion
  BREQ first_conversion
  rjmp second_conversion_and_highier
	
first_conversion:
	LDS DATAL, ADCL
	LDS DATAH, ADCH		;dummy load
	INC FIRST_CHECK
	rjmp return_ISR

second_conversion_and_highier:
  LDS XNL, ADCL
  LDS XNH, ADCH		;load high and low adc bytes
	
  ldi tempr, low(512)
  SUB XNL, tempr
  ldi tempr, high(512)
  SBC XNH, tempr

;store new data value into x[n]. X now points to x[n+1] High byte
  ST X+, XNH		
  ST X+, XNL

;for each new data value we calculate a single new value of y[n]
;clear accumulator
	CLR AC0				
	CLR AC1
	CLR AC2
	CLR AC3

;difference equation is given as:
; y[n] = A0*x[n] - A1*x[n-1] + A2*x[n-2] + B1*y[n-1] - B2*y[n-2]

;we get x[n]

	LD XNL, -X			
	LD XNH, -X		;DATA now holds x[n] x is now pointing to x[n] high byte

;we get x[n-1]

	LD DATAL, -X			
	LD DATAH, -X		;DATA now holds x[n-1] x is now pointing to x[n-1] HIGH byte

;get a[1]
	LDI COEFFL, LOW(A1)
	LDI COEFFH, HIGH(A1)

	MUL16_32_SIGNED

	SUB AC0, IR0	//USE SUBSTRACT BECOUSE A1 SHOULD BE NEGATIVE
	SBC AC1, IR1
	SBC AC2, IR2
	SBC AC3, IR3

;get x[n-2]	

	LD DATAL, -X			
	LD DATAH, -X		;DATA now holds x[n-2] x is now pointing to x[n-2] HIGH byte

;get a[2]
	LDI COEFFL, LOW(A2)
	LDI COEFFH, HIGH(A2)

	MUL16_32_SIGNED

	ADD AC0, IR0
	ADC AC1, IR1
	ADC AC2, IR2
	ADC AC3, IR3

;get y[n-1]
	LD DATAL, -Y			
	LD DATAH, -Y		;DATA now holds Y[n-1] Y is now pointing to Y[n-1] HIGH byte

;get b[1]
	LDI COEFFL, LOW(B1)
	LDI COEFFH, HIGH(B1)

	MUL16_32_SIGNED

	ADD AC0, IR0
	ADC AC1, IR1
	ADC AC2, IR2
	ADC AC3, IR3

;get y[n-2]
	LD DATAL, -Y			
	LD DATAH, -Y		;DATA now holds Y[n-2] Y is now pointing to Y[n-2] HIGH byte

;get b[2]
	LDI COEFFL, LOW(B2)
	LDI COEFFH, HIGH(B2)

	MUL16_32_SIGNED

	SUB AC0, IR0		//USE SUBSTRACT
	SBC AC1, IR1
	SBC AC2, IR2
	SBC AC3, IR3
	
//DOWNSCALE RESULT
	;2^14 CASE
	LDI tempr, 6
	Downslp:
		LSR AC3
		ROR AC2
		ROR AC1
	dec tempr
	BRNE Downslp
	
;now that we downsample we can add x[n]	
	ADD AC1, XNL
	ADC AC2, XNH

LDI tempr, 2		// A/4
loop_div:
	ASR AC2
	ROR AC1
	dec tempr
BRNE loop_div		

;make Y point to y[n] again
	ldi tempr, 2*data_size_bytes
	ADD YL, tempr

;store value of Y
	st Y+, AC2	;store high part first
	st Y+, AC1 ;store in y[n], Y now points to y[n+1] High byte which will be y[n] on next loop

;MAKE X point to x[n+1] which will be x[n] in next loop
	ldi tempr, 3*data_size_bytes
	ADD XL, tempr
  
  dec looping
return_ISR:
reti