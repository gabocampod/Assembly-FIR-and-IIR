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
.DEF IR3 = R19	;highiest byte of intermediate results
.DEF DATAL = R20  ;LOW BYTE OF DATA FROM ADC
.DEF DATAH = R21  ;HIGH BYTE OF DATA
.DEF COEFFL = R22 ; LOW BYTE OF COEFFICIENT
.DEF COEFFH = R23 ; HIGH BYTE OF COEFFICIENT
.DEF EMPTY = R2	  ; CLEARED REGISTER
.DEF LOOPING = R8 
.DEF OUT_VAL = R15
.DEF FIRST_CHECK = R25

.def tempr = r24						;tempr register for intermediate values
.equ number_of_samples = 100
.equ N_Coefficients = 41
.equ coefficient_size_bytes = 2
.equ buffer_size_bytes = (N_Coefficients * coefficient_size_bytes)

.equ I0 = 0x0100						;I0 is adress of first coefficient
.equ I1 = 0x0200						;I1 is adress of first data point
.equ IRES = 0x0300						;IRES is adress where we store results
.equ I1max = I1 + buffer_size_bytes		;this adress is outside data buffer by 1 position		
.equ I1last = I1max - coefficient_size_bytes		;last adress on data buffer
.equ number_of_tx = number_of_samples*2

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
	wait:
		LDS tempr, UCSR0A			//load Uart status REG
		SBRS tempr, UDRE0			//Check empty bit
		rjmp wait    				//if not empty keep waiting

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

//////////////////SET UP COEFFICIENTS AND POINTERS/////////////////

;SET COEFFICIENTS

	call set_coefficients_FIR

;CLEAN THE DATA BUFFER

	LDI YL, LOW(I1)	; set inderect register Y (29:28) to adress of 1st data point
	LDI YH, HIGH(I1)

	ldi tempr, N_Coefficients
	cleaning_data:
		ST Y+, EMPTY
		ST Y+, EMPTY
		dec tempr
	BRNE cleaning_data

;SET ALL POINTERS TO CORRECT ADRESSES

	LDI XL,LOW(I0)		;set inderect register X (27:26) to adress of 1st coefficient 
	LDI XH,HIGH(I0)

	LDI YL, LOW(I1)		; set inderect register Y (29:28) to adress of 1st data point
	LDI YH, HIGH(I1)

	LDI ZL, LOW(IRES)	; set inderect register Z (31:30) to memory locations that will store result
	LDI ZH, HIGH(IRES)


//////////////SET UP UART, ADC AND TIMER SECTION/////////////////

;INIT UART
	LDI R25, HIGH(bps)
	LDI R24, LOW(bps)

	OUTREG UBRR0H, R25		// Set Baud rate
	OUTREG UBRR0L, R24		
	
	ldi r24, (1<< TXEN0) | 	(1<< RXEN0)		  // Enable transmitter and receivier 
	OUTREG UCSR0B,r24

	ldi r24, (0<<USBS0) | (1<<UCSZ01)	|	(1<<UCSZ00)	  // frame format: 8data, 1 stop bit
	OUTREG UCSR0C,r24

;turn off digital buffer at adc pin
	LDI tempr,(1<<ADC0D)
	STS DIDR0, tempr

;SET_UP_ADC_W_TRIGGER

	LDI tempr, (1<<ADPS2) | (1<<ADPS1) |(0<<ADPS0)		//set prescaler (110 for clk/64)
	STS ADCSRA, tempr

	LDI tempr, (1<<REFS0) | (0<<ADLAR)					//set reference voltage, select channel and right justify
	STS ADMUX, tempr

	LDS tempr, ADCSRA 
	ORI tempr, (1<<ADATE) 								// enable autotriggering
	STS ADCSRA, tempr

	LDS tempr, ADCSRA									//enable ADC
	ORI tempr, (1<<ADEN)
	STS ADCSRA, tempr

	LDS tempr, ADCSRA									//enable trigger for ADC conversion complete
	ORI tempr, (1<<ADIE)
	STS ADCSRA, tempr

	LDI tempr, (0<<ADTS2) | (1<<ADTS1) | (1 << ADTS0)	//set source that decides when an new ADC conversion starts  (011 for timer)  (000 free running)
	STS ADCSRB, tempr 

;Init timer 0 and compA

	ldi tempr, 0x00			//clear contents of timer 0
	OUT TCNT0, tempr

	ldi tempr, 99			//set required value to output compare register
	OUT OCR0A,tempr

	ldi tempr, (1<<WGM01)	//set timer for ctc (clear timer when matches compare) mode (WGM01 = 1)
	OUT TCCR0A,tempr	

;START_TIMER0

	LDI tempr, (0<<CS02) | (1<<CS01) | (0<<CS00)	//select prescalar using bits 2,1,0 of TCCR0B and start timer.   (011 FOR CLK/64) (010 for clk/8)
	OUT TCCR0B, tempr

;START FIRST CONVERSION ADC
	LDS tempr, ADCSRA									
	ORI tempr, (1<<ADSC)
	STS ADCSRA, tempr

;enable global interrupts
	SEI


LDI FIRST_CHECK, 0X01
LDI tempr, number_of_samples
MOV looping, TEMPR
main:							//LOOP UNTIL YOU TAKE ALL SAMPLES
	
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

	LDI ZL, LOW(IRES)		//make Z point to first result again 
	LDI ZH, HIGH(IRES)

	LDI R16, 0				//R16 holds how many results we have sent
	LDI R17, number_of_tx	//R17 holds total number of results taken

start_TX:

	wait:
		CP	R16, R17
		BREQ finish_TX
		LDS tempr, UCSR0A			//load Uart status REG
		SBRS tempr, UDRE0			//Check empty bit
		rjmp wait    				//if not empty keep waiting

	LD OUT_VAL, Z+				//load value pointed by Z and send it out
	STS UDR0, OUT_VAL
	 
	INC R16				//Increment the "number of results sent" counter
	rjmp start_TX

finish_TX:

end_all:		//infinite loop. end of program

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
  LDS DATAL, ADCL					;store ADC low value on r20
  LDS DATAH, ADCH					;Store high value on r21  

  SUBI DATAL, low(512)
  SBCI DATAH, HIGH(512)

;Store result in DATA BUFFER, store it as newest value
	ST Y+, DATAH			
	ST Y+, DATAL

 ;if Y> max_adress_data then set Y to first adress data
	MOVW R17:R16, YH:YL    ; R17:16 = R29:28  which is the adress of data pointed by Y
	SUBI R16, LOW(I1MAX)	 ; check if current adress is smaller than max
	SBCI R17, HIGH(I1MAX)
	BRPL GO_BACK_TO_FIRST_ADRESS  ; if current adress is larger result will be positive and branch will be taken 
	rjmp Keep_going
	GO_BACK_TO_FIRST_ADRESS:
		LDI YH, HIGH(I1)
		LDI YL, LOW(I1)
	Keep_going:					  ;return from branch

 ;Load first coefficient
	LD	COEFFH, X+		;HOLD FIRST COEFFCIENT
	LD	COEFFL, X+	
		
 ;CLEAR ACCUMULATOR
	CLR AC0			
	CLR AC1
	CLR AC2
	CLR AC3		

;set looping counter for number of macs-1
	LDI tempr, (N_Coefficients-1)		;looping counter
	MOV r7, tempr

 LC0:
 ;multiplication: multiply r23:r22*r21:r20 --> results stored in r19:r18:r17:r16
 ;Multiply coefficient with data
		MUL16_32_SIGNED
 ;Addition: we add the new result into our 32bit accumulator AC3 TO AC0
		ADD AC0, IR0
		ADC AC1, IR1
		ADC AC2, IR2
		ADC AC3, IR3

 ;LOAD THE NEXT DATA VALUE
		LD DATAH, Y+			
		LD DATAL, Y+

  ;LOAD THE NEXT COEFFICIENT	
		LD	COEFFH, X+			 				 
		LD	COEFFL, X+

  ;if Y> max_adress_data then set Y to first adress data
		MOVW R17:R16, YH:YL    ; R17:16 = R29:28  which is the adress of data pointed by Y
		SUBI R16, LOW(I1MAX)	 ; check if current adress is smaller than max
		SBCI R17, HIGH(I1MAX)
		BRPL GO_BACK_TO_FIRST_ADRESS_BUFFER	  ; if current adress is larger result will be positive and branch will be taken 
		Keep_going_buffer:					;return from branch

  	DEC r7
 BRNE LC0
 	
 ;LAST MULTIPLICATION AND ADDITION IS DONE OUTSIDE LOOP
		MUL16_32_SIGNED

 ;addition: we add the new result into our 32bit accumulator AC3 TO AC0
		ADD AC0, IR0
		ADC AC1, IR1
		ADC AC2, IR2
		ADC AC3, IR3

 ;Shift bytes to downscale result
		ROL AC1
		ROL AC2
		ROL AC3

 ;store 16bit result in memory
		ST Z+, AC3		
		ST Z+, AC2

;AGAIN set inderect register X (27:26) to adress of 1st coefficient 
		LDI XL,LOW(I0)			
		LDI XH,HIGH(I0)	

;Make Y point to nbytes less to overwrite oldest value
		SUBI YL, coefficient_size_bytes			
		SBC  YH, EMPTY

; Check if current adress is smaller than first adress on buffer
		MOVW R17:R16, YH:YL    ; R17:16 <-- R29:28  which is the adress of data pointed by Y
		SUBI R16, LOW(I1)		
		SBCI R17, HIGH(I1)
		BRMI GO_LAST_ADRESS_BUFFER			;if current adress is smaller result will be negative and branch will be taken 
		Keep_going_buffer_last:				;return from branch	


  dec looping	//Finished taking sample so decrease global counter
return_ISR:
reti


/////////BRANCHES ///////////////

;CODE TO SET A POINTER TO FIRST ADRESS ON DATA BUFFER
GO_BACK_TO_FIRST_ADRESS_BUFFER:
LDI YH, HIGH(I1)
LDI YL, LOW(I1)
rjmp Keep_going_buffer

;code to go to last adress on data buffer
GO_LAST_ADRESS_BUFFER:
	LDI YH, HIGH(I1last)
	LDI YL, LOW(I1last)
rjmp Keep_going_buffer_last


/////////FUNCTION DEFINITIONS//////////
set_coefficients_FIR:


	LDI XL,LOW(I0)		
	LDI XH,HIGH(I0)

	LDI R21, high(-5)	;COEFFICIENT
	LDI R20, low(-5)	
	
	ST X+, R21				;SET TO MEMORY
	ST X+, R20	  

	LDI R21, high(-44)	;COEFFICIENT
	LDI R20, low(-44)	
	
	ST X+, R21				;SET TO MEMORY
	ST X+, R20	  

	// REPEAT PROCESS FOR AS MANY //
	// COEFFICIENTS AS REQUIRED  //
  
ret

