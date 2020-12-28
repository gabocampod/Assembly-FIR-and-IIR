.include "./m328Pdef.inc"		;assembly inst set for 328p
.equ F_CPU = 16000000			;speed of processor (16M)
.equ	baud	= 9600			; baudrate
.equ	bps	= (F_CPU/16/baud) - 1	; baud prescale

///////REGISTER DEFINITIONS/////////////

.DEF EMPTY = R2	   ; CLEARED REGISTER
.DEF LOOPING_INNER = R9   ; FOR COUNTING FIR LOOP
.DEF LOOPING_OUTTER = R15 ;FOR COUNTING # OF SAMPLES

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
.DEF AL = R20	;these registers store values that will be multiplied
.DEF AH = R21
.DEF DATA_EXTRA = R14
.DEF tempr = r24						;tempr register for intermediate values

.equ number_of_samples = 50
.equ N_Coefficients = 	25
.equ coefficient_size_bytes = 4
.equ buffer_size_bytes = (N_Coefficients * coefficient_size_bytes)

.equ I0 = 0x0100						;I0 is adress of first coefficient
.equ I1 = 0x0200						;I1 is adress of first data point
.equ IRES = 0x0300						;IRES is adress where we store results
.equ I1max = I1 + buffer_size_bytes		;this adress is outside data buffer by 1 position		
.equ I1last = I1max - coefficient_size_bytes		;last adress on data buffer
.equ number_of_tx = number_of_samples*coefficient_size_bytes

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
		rjmp wait3   				//if not empty keep waiting

	OUTREG UDR0, R15
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
	mul	r23, r21		
	movw	r19:r18, r1:r0
	mul	r22, r20		
	movw	r17:r16, r1:r0
	mul	r23, r20		
	add	r17, r0
	adc	r18, r1
	adc	r19, r2
	mul	r21, r22		
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

//////////////////SET UP COEFFICIENTS AND POINTERS/////////////////

;SET COEFFICIENTS

	call set_coefficients_FIR_32

;CLEAN THE DATA BUFFER

	LDI YL, LOW(I1)	; set inderect register Y (29:28) to adress of 1st data point
	LDI YH, HIGH(I1)

	ldi tempr, N_Coefficients
	cleaning_data:
		ST Y+, EMPTY
		ST Y+, EMPTY
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

	LDI tempr, (1<<ADPS2) | (0<<ADPS1) |(1<<ADPS0)		//set prescaler (110 for clk/64) (101 FOR /32)
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

	ldi tempr, 49			//set required value to output compare register
	OUT OCR0A,tempr

	ldi tempr, (1<<WGM01)	//set timer for ctc (clear timer when matches compare) mode (WGM01 = 1)
	OUT TCCR0A,tempr	

;START_TIMER0

	LDI tempr, (0<<CS02) | (1<<CS01) | (1<<CS00)	//select prescalar using bits 2,1,0 of TCCR0B and start timer.   (011 FOR CLK/64) (010 for clk/8)
	OUT TCCR0B, tempr

;START FIRST CONVERSION ADC
	LDS tempr, ADCSRA									
	ORI tempr, (1<<ADSC)
	STS ADCSRA, tempr

;enable global interrupts
	SEI

LDI XL, 0XFF			//load wrong value to XL to check for first conversion
LDI tempr, number_of_samples
MOV looping_outter, TEMPR
main:							//LOOP UNTIL YOU TAKE ALL SAMPLES
	
	CP looping_outter, EMPTY

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

	LD R15, Z+				//load value pointed by Z and send it out
	STS UDR0, R15
	 
	INC R16				//Increment the "number of results sent" counter
	rjmp start_TX

finish_TX:

//infinite loop. end of program
	end_all:		

	jmp end_all

 //////////ISR DEFINITIONS/////////////

ISR_ADC_COMPLETE:

  LDI tempr, (1<<OCF0A)		//clear timer interrupt flag
  OUTREG TIFR0, tempr 

  CPI XL, 0xFF			//check if it's first conversion
  BREQ first_conversion
  rjmp second_conversion_and_highier
	
first_conversion:
	LDS DATAL, ADCL
	LDS DATAH, ADCH		;dummy load
	LDI XL, LOW(I0)		;set correct value of XL
	rjmp return_ISR

second_conversion_and_highier:
	LDS DATAL, ADCL					;store ADC low value on r20
	LDS DATAH, ADCH					;Store high value on r21  

	SUBI DATAL, low(512)
	SBCI DATAH, HIGH(512)

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

;Store read value in DATA BUFFER as newest value
	ST Y+, DATA_EXTRA
	ST Y+, DATA_EXTRA	
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

;LOAD FIRST COEFFICIENT (4 BYTES)
	LD COEFFH, X+	 
	LD COEFFMH, X+
	LD COEFFML, X+	
	LD COEFFL, X+	

;CLEAR 48 BIT ACCUMULATOR
	CLR RES0
	CLR RES1
	CLR RES2			 
	CLR RES3
	CLR RES4
	CLR RES5

;START FIR MAC LOOP
	LDI tempr, (N_Coefficients-1)		;looping counter for FIR
	MOV LOOPING_INNER, tempr

LC0:
	;MAC
		MUL_SIGNED_32X32_48
	
	;LOAD THE NEXT DATA VALUE
		LD DATA_EXTRA, Y+
		LD DATA_EXTRA, Y+
		LD DATAH, Y+			
		LD DATAL, Y+

	;LOAD THE NEXT COEFFICIENT						
		LD	COEFFH, X+			 				 
		LD	COEFFMH, X+
		LD  COEFFML, X+	
		LD  COEFFL, X+	

	;if Y> max_adress_data then set Y to first adress data
		MOVW R17:R16, R29:R28    ; R17:16 = R29:28  which is the adress of data pointed by Y
		SUBI R16, LOW(I1MAX)	 ; check if current adress is smaller than max
		SBCI R17, HIGH(I1MAX)
		BRMI keep_going_buffer	  ; if current address is smaller than max, result is negative and we skip the jump
			rjmp GO_BACK_TO_FIRST_ADRESS_BUFFER ; if current adress is larger result will be positive and we take the jump
		Keep_going_buffer:						;return from branch
				
	DEC LOOPING_INNER
BREQ last_mac		;loop until looping = 0 
RJMP LC0

last_mac:

;LAS MAC DONE OUTSIDE LOOP
	MUL_SIGNED_32X32_48

;DOWNSCALE RSULT
	ROL RES3
	ROL RES4
	ROL RES5

;CHECK SIGN OF RESULT AND ADD IT TO LAST 16 BITS
	MOV tempr, RES5			
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

;STORE USEFUL 16 bits OF RESULTS (sign extension already stored)
	ST Z+, DATA_EXTRA	
	ST Z+, DATA_EXTRA
	ST Z+, RES5		
	ST Z+, RES4

;AGAIN set inderect register X (27:26) to adress of 1st coefficient 
	LDI XL,LOW(I0)			
	LDI XH,HIGH(I0)	

;check if current adress is less than 1st location in data buffer

	SUBI R28, coefficient_size_bytes			;Make Y point to nbytes less to overwrite oldest value
		MOVW R17:R16, R29:R28    ; R17:16 <-- R29:28  which is the adress of data pointed by Y
		SUBI R16, LOW(I1)		; check if current adress is smaller than first adress on buffer
		SBCI R17, HIGH(I1)
		BRMI GO_LAST_ADRESS_BUFFER			;if current adress is smaller result will be negative and branch will be taken 
	Keep_going_buffer_last:				;return from branch
	
  dec looping_OUTTER	//Finished taking sample so decrease global counter
return_ISR:
CP looping_outter, EMPTY
reti

/////////BRANCHES ///////////////

;CODE TO SET A POINTER TO FIRST ADRESS ON DATA BUFFER
GO_BACK_TO_FIRST_ADRESS_BUFFER:
LDI R29, HIGH(I1)
LDI R28, LOW(I1)
rjmp Keep_going_buffer

;code to go to last adress on data buffer
GO_LAST_ADRESS_BUFFER:
	LDI R29, HIGH(I1last)
	LDI R28, LOW(I1last)
rjmp Keep_going_buffer_last


/////////FUNCTION DEFINITIONS//////////
set_coefficients_FIR_32:
	LDI XL,LOW(I0)		
	LDI XH,HIGH(I0)

	LDI R23, 0xFF				;COEFFICIENT
	LDI R22, 0xF5
    LDI R21, high(-715243)	
	LDI R20, low(-715243)	

	ST X+, R23					;set to memory
	ST X+, R22
	ST X+, R21					
	ST X+, R20

	LDI R23, 0xff				;COEFFICIENT
	LDI R22, 0x96
    LDI R21, high(-6912262)	
	LDI R20, low(-6912262)	

	ST X+, R23					;set to memory
	ST X+, R22
	ST X+, R21					
	ST X+, R20

	// REPEAT PROCESS FOR AS MANY //
	// COEFFICIENTS AS REQUIRED  //

ret

