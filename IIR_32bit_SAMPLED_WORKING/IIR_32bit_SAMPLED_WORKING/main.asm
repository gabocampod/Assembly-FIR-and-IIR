.include "./m328Pdef.inc"		;assembly inst set for 328p
.equ F_CPU = 16000000			;speed of processor (16M)
.equ	baud	= 9600			; baudrate
.equ	bps	= (F_CPU/16/baud) - 1	; baud prescale

///////REGISTER DEFINITIONS/////////////

.DEF EMPTY = R2	   ; CLEARED REGISTER
.DEF LOOPING = R15 ;FOR COUNTING # OF SAMPLES

.DEF RES0 = R3  	;LOWEST BYTE OF ACCUMULATOR
.DEF RES1 = R4	;
.DEF RES2 = R5	;
.DEF RES3 = R6	;
.DEF RES4 = R7	;
.DEF RES5 = R8	;	HIGHIEST BYTE ACCUMULATOR

.DEF COEFFL = R10	;LOWEST BYTE OF COEFFICIENT
.DEF COEFFML = R11
.DEF COEFFMH = R12 
.DEF COEFFH = R13  ;HIGHIEST BYTE OF COEFFICIENT
.DEF DATAL = R22   ;LOW BYTE OF DATA FROM ADC
.DEF DATAH = R23   ;HIGH BYTE OF DATA
.DEF IR0 = R16	;for intermediate results of multiplication low byte
.DEF IR1 = R17
.DEF IR2 = R18
.DEF IR3 = R19	;highiest byte f intermediate results


.DEF DATA_EXTRA = R14

.DEF XNL = R9

.def tempr = r24						;tempr register for intermediate values


.equ number_of_samples = 50
.equ data_size_bytes = 4
.equ I0 = 0x0100						;I0 is pointer to adress of input data  HIGH BYTE X[N-2]
.equ I1 = 0x0200				     	;I1 is pointer to adress of outputs HIGH BYTE Y[N-2]
.equ I1_YN = I1 + 2*data_size_bytes		;POINT TO FIRST RESULT CALCULATED
.equ I0_XN = I0 + 2*data_size_bytes		;POINT TO FIRST DATA VALUE RECEIVED
.equ number_of_tx = number_of_samples*data_size_bytes

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

	OUTREG UDR0, r15
.endm 


.MACRO MUL16_32_SIGNED

	clr	r2
	muls	r23, r21		;signed byte of coeff * signed byte of data
	movw	r19:r18, r1:r0
	mul	r22, r20		;unsigned byte of coeff and unsigned byte of data
	movw	r17:r16, r1:r0
	mulsu	r23, r20		;signed byte coefficient * unsigned byte data
	sbc	r19, r2
	add	r17, r0
	adc	r18, r1
	adc	r19, r2
	mulsu	r21, r22		; signed byte data * unsigned byte coeff
	sbc	r19, r2
	add	r17, r0
	adc	r18, r1
	adc	r19, r2
.ENDM 

.MACRO MUL16_32_UNSIGNED
	mul	r23, r21		; ah * bh
	movw	r19:r18, r1:r0
	mul	r22, r20		; al * bl
	movw	r17:r16, r1:r0
	mul	r23, r20		; ah * bl
	add	r17, r0
	adc	r18, r1
	adc	r19, r2
	mul	r21, r22		; bh * al
	add	r17, r0
	adc	r18, r1
	adc	r19, r2
.ENDM

.MACRO MUL16_32_SIGNED_X_UNSIGNED
	MULSU R23,R21			//signed ah* unsigned bh  
	movw r19:r18, r1:r0

	MUL R22, R20			//unsigned al* unsigned bl  
	movw r17:r16, r1:r0

	MULSU R23, R20			//signed ah* unsigned bl  
	sbc	r19, r2
	add	r17, r0
	adc	r18, r1
	adc	r19, r2

	MUL R22, R21			//unsigned al* unsigned bl
	add	r17, r0
	adc	r18, r1
	adc	r19, r2
.ENDM

.macro MUL_SIGNED_32X32_48
	
	MOV R25, DATAH
	MOV R24, DATAL

	movw R23:R22, COEFFH:COEFFMH			//signed A3:A2 * signed B3:B2
	mov R21, DATA_EXTRA						//movw R21:R20, DATA_EXTRA:DATA_EXTRA
	MOV R20, DATA_EXTRA

	MUL16_32_SIGNED
	ADD RES4, IR0					    	//movw res5:res4, ir1:ir0		 mov r18, r0
	ADC RES5, IR1

	MOVW R23:R22, COEFFML:COEFFL			//uns low* unsigned low
	MOV R21, R25							
	MOV R20, R24
	MUL16_32_UNSIGNED
	ADD RES0, IR0				//movw res1:res0, ir1:ir0	 mov r16, r0
	ADC RES1, IR1
	ADC RES2, IR2				//movw res3:res2, ir3:ir2	 mov r17, r1
	ADC RES3, IR3
	
	MOVW R23:R22, COEFFH:COEFFMH		//signed coeff high* unsigned data low
	MUL16_32_SIGNED_X_UNSIGNED
	add res2, ir0				//add r17, r0			
	adc res3, ir1
	adc res4, ir2				//adc r18, r1
	adc res5, ir3

	MOV R23, DATA_EXTRA			//	 //signed data high* unsigned coeff low
	MOV R22, DATA_EXTRA
	MOVW R21:R20, COEFFML:COEFFL
	MUL16_32_SIGNED_X_UNSIGNED
	add res2, ir0				//add r17, r0			
	adc res3, ir1
	adc res4, ir2				//adc r18, r1
	adc res5, ir3
.endm

//////////////////SET UP POINTERS AND CLEAN DATA/////////////////

;memory arranged as high-low-high-low
//CLEAN MEMORY LOCATIONS FOR INPUT DATA
	LDI XL,LOW(I0)		;make X point to x[n-2] High byte
	LDI XH,HIGH(I0)

	ST X+, EMPTY		;CLEAR X[N-2]
	ST X+, EMPTY
	ST X+, EMPTY
	ST X+, EMPTY
	ST X+, EMPTY		;CLEAR X[N-1]
	ST X+, EMPTY		
	ST X+, EMPTY
	ST X+, EMPTY		;X NOW POINTS TO HIGH BYTE X[N]

//CLEAN MEMORY LOCATIONS FOR OUTPUT
	LDI YL,LOW(I1)		;make Y point to y[n-2] High byte	
	LDI YH,HIGH(I1)

	ST Y+, EMPTY		;CLEAR Y[N-2]
	ST Y+, EMPTY
	ST Y+, EMPTY
	ST Y+, EMPTY
	ST Y+, EMPTY		;CLEAR Y[N-1] 
	ST Y+, EMPTY
	ST Y+, EMPTY
	ST Y+, EMPTY		;Y NOW POINTS TO HIGH BYTE Y[N]

/////////// DEFINE COEFFICIENTS VIA POLE/ZERO PLACEMENT//////////

/*
//LOW PASS CUT-OFF 1500HZ///
// a[1] = -1.8  a[2] = 0.81
// b[1] = 0.0736238464	b[2] = 0.1725312

// scaled coefficients
// a[1] = -1932735283  a[2] = 869730877
// b[1] = 79053003   b[2] = 185254020

// CHANGE SIGN OF A[1] AND B[2]
// a[1] = 1932735283  a[2] = 869730877
// b[1] = 79053003   b[2] = -185254020
*/
	.equ A1 = 1932735283			
	.equ A2 = 869730877
	.equ B1 = 79053003
	.equ B2 = -185254020

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

	LDI tempr, (1<<ADPS2) | (0<<ADPS1) |(1<<ADPS0)		//set prescaler (110 for clk/64)
	STS ADCSRA, tempr

	LDI tempr, (1<<REFS0) | (0<<ADLAR)					//set reference voltage, select channel and justified
	STS ADMUX, tempr

	LDS tempr, ADCSRA 
	ORI tempr, (1<<ADATE) 					// enable autotriggering
	STS ADCSRA, tempr

	LDS tempr, ADCSRA						//enable ADC
	ORI tempr, (1<<ADEN)
	STS ADCSRA, tempr

	LDS tempr, ADCSRA					//enable trigger for ADC conversion complete
	ORI tempr, (1<<ADIE)
	STS ADCSRA, tempr

	LDI tempr, (0<<ADTS2) | (1<<ADTS1) | (1 << ADTS0)	//set source that decides when an new ADC conversiion starts  (011 for timer)  (000 free running)
	STS ADCSRB, tempr 

;Init timer 0 and compA

	ldi tempr, 0x00			//clear contents of timer 0
	OUT TCNT0, tempr

	ldi tempr, 79			//set required value to output compare register
	OUT OCR0A,tempr

	ldi tempr, (1<<WGM01)	//set timer for ctc (clear timer when matches compare) mode (WGM01 = 1)
	OUT TCCR0A,tempr	

;START_TIMER0

	LDI tempr, (0<<CS02) | (1<<CS01) | (0<<CS00)	//select prescalar 
	OUT TCCR0B, tempr

;START FIRST CONVERSION ADC

	LDS tempr, ADCSRA									
	ORI tempr, (1<<ADSC)
	STS ADCSRA, tempr

;enable global interrupts

	SEI

;MAIN loop while all samples are taken
LDI XL, 0XFF	;set value to check first conversion
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

	LD R15, Y+				//load value pointed by y and send it out
	STS UDR0, R15
	 
	INC R16				//Increment the "number of results sent" counter
	rjmp start_TX

finish_TX:

;SEND DISTINGUISH BITS via USART

	LDI tempr, 0xAB
	mov R15, tempr
	LDI tempr, 10
	mov R9, tempr
	loopdist:
		send_c
		DEC R9
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

	LD R15, X+				//load value pointed by x and send it out
	STS UDR0, R15
	 
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

  CPI XL, 0xFF			
  BREQ first_conversion
  rjmp second_conversion_and_highier
	
first_conversion:
	LDS DATAL, ADCL
	LDS DATAH, ADCH		;dummy load
	LDI XL, low(I0_XN)
	rjmp return_ISR

second_conversion_and_highier:
  LDS DATAL, ADCL
  LDS DATAH, ADCH		;load high and low adc bytes

  ldi tempr, low(512)
  SUB DATAL, tempr
  ldi tempr, high(512)
  SBC DATAH, tempr

;set value of 2 upper bytes data_extra (0xff or 0x00)
	MOV tempr, DATAH			
	ANDI tempr, 0x80
	SBRS tempr, 7					
	rjmp its_positive	
		LDI tempr, 0XFF				//if set then it's negative so choose 0xFF
		MOV DATA_EXTRA, TEMPR
		rjmp continue				//skip its_positive part	
	its_positive:
		LDI tempr, 0X00				//if positive choose 0x00
		MOV  DATA_EXTRA,TEMPR
	continue:

;store new data value into x[n]. X now points to x[n+1] High byte
  ST X+, DATA_EXTRA
  ST X+, DATA_EXTRA
  ST X+, DATAH		
  ST X+, DATAL

;for each new data value we calculate a single new value of y[n]
;CLEAR 48 BIT ACCUMULATOR
	CLR RES0
	CLR RES1
	CLR RES2			 
	CLR RES3
	CLR RES4
	CLR RES5

;difference equation is given as:
; y[n] = A0*x[n] - A1*x[n-1] + A2*x[n-2] + B1*y[n-1] - B2*y[n-2]

;we get x[n]

	LD DATAL, -X			
	LD DATAH, -X		;DATA now holds x[n] x is now pointing to x[n] high byte
	LD DATA_EXTRA, -X
	LD DATA_EXTRA, -X

;we get x[n-1]

	LD DATAL, -X			
	LD DATAH, -X		;DATA now holds x[n-1] x is now pointing to x[n-1] HIGH byte
	LD DATA_EXTRA, -X
	LD DATA_EXTRA, -X

;get a[1]

	LDI tempr, 0x73
		MOV COEFFH, tempr
	LDI tempr, 0x33
		MOV COEFFMH, tempr
	LDI tempr, HIGH(A1)
		MOV COEFFML, tempr
	LDI tempr, LOW(A1)
		MOV COEFFL, tempr

	MUL_SIGNED_32X32_48  //USE SUBSTRACT BECOUSE A1 SHOULD BE NEGATIVE

;get x[n-2]	

	LD DATAL, -X			
	LD DATAH, -X		;DATA now holds x[n-2] x is now pointing to x[n-2] HIGH byte
	LD DATA_EXTRA, -X
	LD DATA_EXTRA, -X

;get a[2]
	LDI tempr, 0x33
		MOV COEFFH, tempr
	LDI tempr, 0xD7
		MOV COEFFMH, tempr
	LDI tempr, HIGH(A2)
		MOV COEFFML, tempr
	LDI tempr, LOW(A2)
		MOV COEFFL, tempr

	MUL_SIGNED_32X32_48

;get y[n-1]
	LD DATAL, -Y			
	LD DATAH, -Y		
	LD DATA_EXTRA, -Y	;2 upper bytes of Y will be the same so they can share variable;
	LD DATA_EXTRA, -Y	;DATA now holds Y[n-1] Y is now pointing to Y[n-1] HIGH byte

;get b[1]
	LDI tempr, 0x04
		MOV COEFFH, tempr
	LDI tempr, 0xBC
		MOV COEFFMH, tempr
	LDI tempr, HIGH(B1)
		MOV COEFFML, tempr
	LDI tempr, LOW(B1)
		MOV COEFFL, tempr

	MUL_SIGNED_32X32_48

;get y[n-2]
	LD DATAL, -Y			
	LD DATAH, -Y		
	LD DATA_EXTRA, -Y
	LD DATA_EXTRA, -Y	;;DATA now holds Y[n-2] Y is now pointing to Y[n-2] HIGH byte

;get b[2]
	LDI tempr, 0xF4
		MOV COEFFH, tempr
	LDI tempr, 0xF5
		MOV COEFFMH, tempr
	LDI tempr, HIGH(B2)
		MOV COEFFML, tempr
	LDI tempr, LOW(B2)
		MOV COEFFL, tempr

	MUL_SIGNED_32X32_48		//USE SUBSTRACT

//DOWNSCALE RESULT
	;2^30 CASE
	LDI tempr, 6
	Downslp:
		LSR RES5
		ROR RES4
		ROR RES3
	dec tempr
	BRNE Downslp
	
;now that we downsample we can add x[n]	

;MAKE x POINT TO CURRENT X[n] high byte
	LDI tempr, 2*data_size_bytes
	ADD XL, tempr
	ADC XH, EMPTY			//x now points to crrent x[n] high byte

;load the 4 bytes of current x[n]
   LD DATA_EXTRA, X+
   LD DATA_EXTRA, X+
   LD DATAH, X+
   LD DATAL, X+				//X now points x[n+1] which will be x[n] on next interation
   
;add result with current x[n]
	ADD RES3, DATAL
	ADC RES4, DATAH

//divide to avoid implicit gain

LDI tempr, 2
loop_div:
	ASR RES4
	ROR RES3
	dec tempr
BRNE loop_div

;make Y point to y[n] again
	ldi tempr, 2*data_size_bytes
	ADD YL, tempr
	ADC YH, EMPTY

;store value of Y
	MOV tempr, RES4			
	ANDI tempr, 0x80
	SBRS tempr, 7					
	rjmp its_positive2	
		LDI tempr, 0XFF				//if set then it's negative so choose 0xFF
		MOV DATA_EXTRA, TEMPR
		rjmp continue2				//skip its_positive part	
	its_positive2:
		LDI tempr, 0X00				//if positive choose 0x00
		MOV  DATA_EXTRA,TEMPR
	continue2:

	ST Y+, DATA_EXTRA
	ST Y+, DATA_EXTRA
	st Y+, RES4			;store high part first
	st Y+, RES3			;store in y[n], Y now points to y[n+1] High byte which will be y[n] on next loop

  
  dec looping
return_ISR:
	CP looping, EMPTY
reti