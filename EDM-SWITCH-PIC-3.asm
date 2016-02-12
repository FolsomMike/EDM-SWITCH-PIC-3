;--------------------------------------------------------------------------------------------------
; Project:  EDM Notch Cutter -- SWITCH PIC software Model 3
; Date:     02/10/16
; Revision: 1.0
;
; This code is for the switch and button relay PIC on the EDM Notch Cutter User Interface Board.
;
; Overview:
;
; This program listens for state changes of switches and buttons. The button and switch states are
; stored locally on this device and repeatedly transmitted to the Main PIC as serial data.
;
; If a switch or button is pressed, the change will be stored in a temporary byte until the
; previous states of the switches or buttons have been properly sent to the Main PIC by the 
; interrupt routine. Then the temporary byte will be copied into the buffer to be sent. This ensures
; that a switch press does not override the previous presses before it is sent to the Main Pic.
;
;
;--------------------------------------------------------------------------------------------------
; Notes on PCLATH
;
; The program counter (PC) is 13 bits. The lower 8 bits can be read and written as register PCL.
; The upper bits cannot be directly read or written.
;
; When a goto is executed, 11 bits embedded into the goto instruction are loaded into the lower
; 11 bits of PC<10:0> while bits PCLATH<4:3> are copied to bits PC<12:11>
;
; Changing PCLATH does NOT instantly change the PC register. The PCLATH will be used the next time
; a goto is executed (or similar opcode) or the PCL register is written to. Thus, to jump farther
; than the 11 bits (2047 bytes) in the goto opcode will allow, the PCLATH register is adjusted
; first and then the goto executed.
;
;--------------------------------------------------------------------------------------------------
; Serial Data from Main PIC
;
; These notes are nearly identical for "OPT EDM LCD PIC.ASM" and "EDM MAIN PIC.ASM". Changes to
; either should be copied to the other. The "LCD" version uses internal 4Mhz clock for Fosc
; while "MAIN" uses external 16Mhz clock. Thus the timings are different for using Timer0 as
; an interrupt source.
;
; wip mks -- need to change this program to use 16Mhz like the Main PIC. Will have to adjust all
; timings to make that work.
;
; The "Main" PIC sends data to the "LCD" PIC via PortA,0 (RA0). A word is sent for each 
; command/data value:
;
; Data Format --
;
; the first byte is a control byte and specifies either an LCD command or an LCD character
; 	0 = data; 1 = command; other may be a command for a different PIC listening on the same line
;
; the second byte is either an LCD command or an LCD character
;
; PIC16 Timing --
;
; These notes were verified using an oscilloscope.
;
; The "LCD" PIC is configured to use the internal oscillator of 4Mhz (FOSC). The instruction cycle
; frequency is Fosc/4. All instructions except branches take 1 cycle.
;
; Timer0 increment frequency is Fosc/4 without prescaler -- in the "LCD" program the prescaler
; is assigned to the WatchDog, thus Timer0 freq = Fosc/4 (same as the cycle frequency)
;
; Serial Timing --
;
; each serial bit from the Main PIC is 64 uS wide 
;
; a single instruction cycle (a nop) on the "LCD" PIC is 1uS wide
; a goto instruction is 2uS
; a simple decfsz loop can be used for bit to bit timing
;
; Loop Code Timing
;
;	loop1: 	decfsz	counter,F
;    		goto    loop1
;
; if you are using a bsf/bcf pair to create a pulse train for verifying this
; 	measurement, the one cycle of the bcf will be included in the wait time and must be
;	accounted for by subtracting 1 cycle from the resulting delay times; for the board used
;	for this test, one cycle = 1 us
;
; delay for different starting values of counter:
;
;  2 -> 5 uS
;  3 -> 8 uS (each additional count -> 3 uS)
; 21 -> 64 uS  ~ ((64 - 5) / 3) + 2
; 
; in actual use: 19 gives the closest result taking all loop cycles into account
;
; Version 1.0 of the LCD PIC code read the word all at one time and then transmitted to the display
; between words without using an interrupt. The time between words is significantly larger
; allowing for the time for the display update. For version 2.0, which constantly refreshes the
; display from a local buffer, the time required for display access code is more than that allowed
; between incoming serial words. Thus, an interrupt is used to monitor for start bits.
;
; For reasonably accurate detection of a start bit, checking the input every 1/3 bit width (21 uS)
; is desired -- worst case should be detecting the start bit 2/3 after the down transition
; this should be close enough to successfully detect the remaining 8 bits, allowing for 21 uS
; of timing slop...
;
; ...HOWEVER...
;
; ...the interrupt takes up 15 cycles at its quickest and the program would spend all of its
; time in the interrupt routine. So the input will actually be checked every 32 uS (32 cycles)
; or one/half the time between bits. This still leaves only 17 cycles or so for the main thread
; to execute for every call to the interrupt, but the main thread is not time critical.
;
; When the interrupt detects a start bit, the main thread will not run again until an entire
; word is read. This could probably be changed to return to the main thread while waiting for the
; next bit, but that would require another mode flag check, more lost cycles, and a lot more
; complexity. The main thread will get 12 cycle blocks when between words and when the "Main" PIC
; is not sending data.
;
; NOTE: Because the main thread does not execute while the interrupt code is receiving a word,
; it must have time between words to perform its worst case processing so it can retrieve each
; new word before the interrupt overwrites it. Thus the "Main" PIC must have a significant
; delay between words.
;
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Configurations, etc. for the Assembler Tools and the PIC

    LIST p = PIC16F1459	;select the processor

    errorlevel  -306 ; Suppresses Message[306] Crossing page boundary -- ensure page bits are set.

    errorLevel  -302 ; Suppresses Message[302] Register in operand not in bank 0.

    errorLevel	-202 ; Suppresses Message[205] Argument out of range. Least significant bits used.
                     ;	(this is displayed when a RAM address above bank 1 is used -- it is
                     ;	 expected that the lower bits will be used as the lower address bits)


#INCLUDE <p16f1459.inc> 		; Microchip Device Header File

; Specify Device Configuration Bits

; CONFIG1
; __config 0xF9E4
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _WRT_ALL & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_4x & _PLLEN_DISABLED & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

; _FOSC_INTOSC -> internal oscillator, I/O function on CLKIN pin
; _WDTE_OFF -> watch dog timer disabled
; _PWRTE_OFF -> Power Up Timer disabled
; _MCLRE_OFF -> MCLR/VPP pin is digital input
; _CP_OFF -> Flash Program Memory Code Protection off
; _BOREN_OFF -> Power Brown-out Reset off
; _CLKOUTEN_OFF -> CLKOUT function off, I/O or oscillator function on CLKOUT pin
; _IESO_OFF -> Internal/External Oscillator Switchover off
;   (not used for this application since there is no external clock)
; _FCMEN_OFF -> Fail-Safe Clock Monitor off
;   (not used for this application since there is no external clock)
; _WRT_ALL -> Flash Memory Self-Write Protection on -- no writing to flash
;
; _CPUDIV_NOCLKDIV -> CPU clock not divided
; _USBLSCLK_48MHz -> only used for USB operation
; _PLLMULT_4x -> sets PLL (if enabled) multiplier -- 4x allows software override
; _PLLEN_DISABLED -> the clock frequency multiplier is not used
;
; _STVREN_ON -> Stack Overflow/Underflow Reset on
; _BORV_LO -> Brown-out Reset Voltage Selection -- low trip point
; _LPBOR_OFF -> Low-Power Brown-out Reset Off
; _LVP_OFF -> Low Voltage Programming off
;
;for improved reliability, Watch Dog code can be added and the Watch Dog Timer turned on - _WDT_ON
;turn on code protection to keep others from reading the code from the chip - _CP_ON
;
; end of configurations
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Constants
;
; The compiler replaces all occurences of the constants defined below with their proper values.
;
 
; Constants for serial data

SERIAL_IN_P             EQU     PORTA   ; PORTA - used to receive serial data from main
SERIAL_IN               EQU     RA0     ; RA0   - used to receive serial data from main
           
SERIAL_OUT_L            EQU     LATB    ; LATB - used to send serial data to main
SERIAL_IN               EQU     RB7     ; RB7   - used to send serial data to main

; end of Constants for serial data

; Switches

PORT_SWITCHES           EQU     PORTC   ; PORTC - port used to listen to all switches

SWITCH_SELECT           EQU		RC7     ; RC7 pin listens to the Select switch
SWITCH_SELECT_STATE     EQU     0       ; Select switch state is bit 0 of switchStates in ram

; end of Switches
     
; Bits in xmtBufferFlags

xmtBusy         EQU     0x00    ; bit 0: 0 = buffer not busy        1 = buffer busy
startBit        EQU     0x01    ; bit 1: 0 = start bit not due      1 = transmit start bit next
stopBit         EQU     0x02    ; bit 2: 0 = stop bit not due       1 = transmit stop bit next
endBuffer       EQU     0x03    ; bit 3: 0 = not buffer end         1 = buffer end reached
inDelay         EQU     0x04    ; bit 4: 0 = not delaying           1 = delaying

; end of Bits in xmtBufferFlags

; end of Constants
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Variables in RAM
;
; Note that you cannot use a lot of the data definition directives for RAM space (such as DB)
; unless you are compiling object files and using a linker command file.  The cblock directive is
; commonly used to reserve RAM space or Code space when producing "absolute" code, as is done here.
;  

;----------------------
; Bank 0 -- 80 bytes

    cblock 0x20                 ; starting address

    flags                   ; bit 0: unused
                            ; bit 1: 0 = char at cursor is off : 1 = char at cursor is on
                            ; bit 2:
                            ; bit 3:
                            ; bit 4:
                            ; bit 5:
							; bit 6:
							; bit 7:
                            
    switchPresses           ; All bits set to 1s if no buttons pressed (11111111)
                            ; bit 0: 0 = Select/Reset/Zero/Enter switch pressed
                            ; bit 1:
                            ; bit 2:
                            ; bit 3:
                            ; bit 4:
                            ; bit 5:
							; bit 6:
							; bit 7:

	smallDelayCnt			; used to count down for small delay
	bigDelayCnt				; used to count down for big delay

	; next variables ONLY written to by interrupt code

	intScratch0				; scratch pad variable for exclusive use by interrupt code
	bitCount				; used to count number of bits received
	newSerialByte			; each serial data byte is stored here upon being received
	newControlByte			; new control bytes stored here by interrupt routine
							; (this one is reset by main thread)

	; end of variables ONLY written to by interrupt code

    endc 
    
; end of Bank 0
;----------------------
    
;----------------------
; Bank 1 -- 80 bytes

 cblock 0xa0                ; starting address

    xmtFlags                ; bit 0: 0 = buffer not busy        1 = buffer busy
                            ; bit 1: 0 = start bit not due      1 = transmit start bit next
                            ; bit 2: 0 = stop bit not due       1 = transmit stop bit next
                            ; bit 3: 0 = not buffer end         1 = buffer end reached
                            ; bit 4: 0 = not delaying           1 = delaying

    xmtScratch0             ; scratch pad variable

    xmtBitCount             ; tracks bits of byte being transmitted to LCD
    xmtBufferCnt            ; number of characters in the buffer
    
    xmtBufferPtrH           ; points to next byte in buffer to be transmitted to LCD 
    xmtBufferPtrL
    
    xmtDelay1               ; delay counter for providing necessary time delay between characters
    xmtDelay0

    xmtBuffer0              ; xmt buffer - holds presses being transmitted to the Main PIC
    xmtBuffer1
    xmtBuffer2
    xmtBuffer3
    xmtBuffer4
    xmtBuffer5              ; xmtBuffer1:5 not currently used

 endc

; end of Bank 1
;----------------------

;-----------------
; Define variables in the memory which is mirrored in all 4 RAM banks.  This area is usually used
; by the interrupt routine for saving register states because there is no need to worry about
; which bank is current when the interrupt is invoked.
; On the PIC16F628A, 0x70 thru 0x7f is mirrored in all 4 RAM banks.

; NOTE:
; This block cannot be used in ANY bank other than by the interrupt routine.
; The mirrored sections:
;
;	Bank 0		Bank 1		Bank 2		Bank3
;	70h-7fh		f0h-ffh		170h-17fh	1f0h-1ffh
;

    cblock	0x70

    endc

;-----------------

; end of Variables in RAM
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Reset & Interrupt Vectors
;
; Note:
;
;   handleInterrupt not used. In most programs, handleInterrupt is called to determine the type of 
;   interrupt. Since only the Timer0 interrupt is used, it is assumed that any interrupt is a Timer0
;   interrupt.
;

    org 0x00                    ; Start of Program Memory

	goto start              ; jump to main code section
	nop			            ; Pad out so interrupt
	nop			            ; service routine gets
	nop			            ; put at address 0x0004.

; interrupt vector at 0x0004
; NOTE: You must clear PCLATH before jumping to the interrupt routine - if PCLATH has bits set it
; will cause a jump into an unexpected program memory bank.

	bcf 	INTCON,T0IF             ; clear the Timer0 overflow interrupt flag

    clrf    PCLATH                  ; set to bank 0 where the interrupt routine is located
    goto	handleTimer0Interrupt   ; all interrupts are from Timer0

; end of Reset & Interrupt Vectors
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setup
;
; Presets variables and configures hardware.
;

setup:

    call    setupClock      ; set system clock source and frequency

	; make sure both bank selection bits are set to Bank 0

    banksel INTCON

    clrf    INTCON          ; disable all interrupts

    movlw   0xff
    movwf   PORTA           ; 0xff -> Port A

 	movlw	0x00			                                
 	movwf	PORTB			; set Port B outputs low

    banksel TRISA

	movlw 	0x01                              
 	movwf 	TRISA			; 0x01 -> TRISA = PortA I/O 0000 0001 b (1=input, 0=output)
    						;	RA0 - Input : receives serial input data
							;	RA1 - Output: E strobe to initiate LCD R/W
							;	RA2 - Output: Register Select for LCD
							;	RA3 - Output: unused (pulled high for some reason)
							;	RA4 - Output: unused (pulled high for some reason)
							;	RA5 - Vpp for PIC programming, unused otherwise
							;	RA6 - unused (unconnected)
							;	RA7 - unused (unconnected)
 	movlw 	0x00
 	movwf	TRISB			; 0x00 -> TRISB = PortB I/O 0000 0000 b (1=input, 0=output)
							;	port B outputs data to the LCD display
							; 	RB6 is also used for programming the PIC
							; 	RB7 is also used for programming the PIC

	movlw	0x51			; set options 0101 0001 b
	movwf	OPTION_REG		; bit 7 = 0: PORTB pull-ups are enabled by individual port latch values
     						; bit 6 = 1: RBO/INT interrupt on rising edge
							; bit 5 = 0: TOCS ~ Timer 0 run by internal instruction cycle clock (CLKOUT ~ Fosc/4)
							; bit 4 = 1: TOSE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin (not used here)
							; bit 3 = 0: PSA ~ Prescaler enabled for Timer 0
                            ; bit 2 = 0 : Bits 2:0 control prescaler:
                            ; bit 1 = 0 :    001 = 1:4 scaling for Timer0
                            ; bit 0 = 1 :

    banksel TMR0
	
	movlw	TIMER0_RELOAD_START_BIT_SEARCH_Q
	movwf	TMR0

 	bcf		PORTA,LCD_E     ; set LCD E strobe low (inactive)
	bcf		PORTA,LCD_RS	; set LCD Register Select low (chooses instruction register)
	bsf		PORTA,UNUSED1	; set high to match pullup (unused)
	bcf		PORTA,UNUSED2   ; set high to match pullup (unused)

; enable the interrupts


	bsf	    INTCON,PEIE	    ; enable peripheral interrupts (Timer0 is a peripheral)
    bsf     INTCON,T0IE     ; enabe TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

	; make sure both bank selection bits are set to Bank 0

    banksel flags

	movlw	.0
	movwf	flags
	
	movlw	CURSOR_BLINK_RATE		; preset blink rate timer
	movwf	charBlinkRate

	movlw	LCD_COLUMN0_START
	movwf	currentCursorLocation

	movlw	DISP_ON_CURSOR_OFF_BLINK_OFF_CMD
	movwf	currentLCDOnOffState

	return

; end of setup
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupClock
;
; Sets up the system clock source and frequency (Fosc).
;
; Instruction cycle rate is Fosc/4.
;
; Assumes clock related configuration bits are set as follows:
;
;   _FOSC_INTOSC,  _CPUDIV_NOCLKDIV, _PLLMULT_4x, _PLLEN_DISABLED
;
; Assumes all programmable clock related options are at Reset default values.
;
; NOTE: Adjust I2C baud rate generator value when Fosc is changed.
;

setupClock:

    ; choose internal clock frequency of 16 Mhz ~ IRCF<3:0> = 1111

    banksel OSCCON

    bsf     OSCCON, IRCF3
    bsf     OSCCON, IRCF2
    bsf     OSCCON, IRCF1
    bsf     OSCCON, IRCF0

    return

; end of setupClock
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Main Code
;
; Sets up the PIC, the LCD, displays a greeting, then monitors the serial data input line from
; the main PIC for data and instructions to be passed on to the LCD display.
;

start:

	call	setup			; set up main variables and hardware

   	call    bigDelay		; should wait more than 15ms after Vcc = 4.5V
    
	call    initLCD

    banksel flags

	call	setUpLCDCharacterBuffer

	call	clearLCDLocalBuffer

    banksel flags

	call	displayGreeting

    banksel flags

	movlw	0xff			; 0xff in newControlByte means no new serial data
	movwf	newControlByte	; it is changed when a new value is received

	movlw	.30				; save 255 values
	movwf	debugCounter

	movlw	db0
	movwf	debugPointer


; begin monitoring the serial data input line from the main PIC for data and instructions
; to be passed on to the LCD display

; in between each check for incoming data on the serial line, write one character from the local
; LCD buffer to the LCD display

mainLoop:

    banksel flags

	movf	newControlByte,W	; if newControlByte is 0xff, then no new data to process
 	sublw	0xff
 	btfss	STATUS,Z
	call	handleDataFromMainPIC

    banksel flags

	call	writeNextCharInBufferToLCD ;write one character in the buffer to the LCD

    goto    mainLoop

; end of Main Code
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleDataFromMainPIC
;
; Processes the values in controlByte and lcdData.
;
; NOTE: This function must be called faster than the interrupt routine is reading consecutive
; words. The words have a delay between them which allows for more time between calls to this
; function.
;
; Control codes other than address changes and clear screen commands are written to the LCD
; display immediately. Address changes and data values are used to address and store in the local
; LCD character buffer.
;
; Data bank 0 should be selected on entry.
;

handleDataFromMainPIC:
	
	;copy the values to working variables so interrupt routine can use new variables
	
	movf	newControlByte,W
	movwf	controlByte
	movf	newLCDData,W
	movwf	lcdData

	movlw	0xff					; signal that the new data has been collected
	movwf	newControlByte

	; call to debug will cause communication to be corrupted with Main PIC when it comes time to
	; write to the eeprom as that disables interrupts for a while
	;call	debug	;debug mks -- store the controlByte and lcdData in eeprom

	movf	controlByte,W			; if the control byte is 0, then the second byte is data for the LCD
 	sublw	0
 	btfsc	STATUS,Z
    goto    writeToLCDBuffer		; store byte in the local LCD character buffer

    banksel flags
	movf	controlByte,W			; if the control byte is 1, then the second byte is an instruction for the LCD
 	sublw	0x1
 	btfsc	STATUS,Z
    goto    handleLCDInstruction

	return

; end of handleDataFromMainPIC
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleLCDInstruction
;
; Handles LCD instruction codes received from the master PIC. If the control code is an address
; change or clear screen code, the command is directed to the local LCD character buffer. The LCD
; display itself is not changed -- that is handled by the code which transmits the buffer contents
; to the display.
;
; All other control codes are transmitted directly to the LCD display.
;
; Data bank 0 should be selected on entry.
;

handleLCDInstruction:

	; catch clear screen command

	movf	lcdData,W
 	sublw	CLEAR_SCREEN_CMD
 	btfss	STATUS,Z
	goto	notClearScreenCmd

	goto	clearLCDLocalBuffer

notClearScreenCmd:

	; check for address change instruction

    btfss   lcdData,ADDRESS_SET_BIT	
	goto	notAddressChangeCmd
	
	goto	setLCDBufferWriteAddress

notAddressChangeCmd:

	; check for display on/off instruction

	; mask lower bits; leave only type selection bits to compare with commmand

	movlw	DISPLAY_ON_OFF_CMD_MASK
	andwf	lcdData,W
	sublw	DISPLAY_ON_OFF_CMD
 	btfss	STATUS,Z
	goto	notDisplayCursorBlinkCmd

	goto	setLCDOnOffAndCursorAndBlink

notDisplayCursorBlinkCmd:

	; transmit all other control codes straight to the display

	goto    writeLCDInstruction

; end of handleLCDInstruction
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeNextCharInBufferToLCD
;
; Writes the next character in the current line to the LCD display. If the end of the line is
; reached, the line pointer is incremented to the next line.
;
; Bank selection not important on entry.
; STATUS:IRP is modified
;

writeNextCharInBufferToLCD:

    banksel lcdBufOutPtrH

    movf    lcdBufOutPtrH,W  ; get pointer to next character to be written to LCD
    movwf   FSR0H
    movf    lcdBufOutPtrL,W  ; get pointer to next character to be written to LCD
    movwf   FSR0L

	movf	INDF0,W			; load the character

    banksel lcdData
	movwf	lcdData         ; store for use by writeLCDData function

	; check if cursor is blinking and is in the "off" state

	btfsc	currentLCDOnOffState,BLINK_ON_OFF_CMD_FLAG
	btfsc	flags,CHAR_AT_CURSOR_STATE
	goto	noHideCharacter

	; character should be off

    banksel lcdBufOutPtrL
	movf	lcdBufOutPtrL,W  ; only uses LSB -- buffer should not cross 256 byte boundary
    banksel currentCursorBufPosL
	subwf	currentCursorBufPosL,W
	btfss	STATUS,Z
	goto	noHideCharacter

	movlw	' '
	movwf	lcdData         ; store for use by writeLCDData function	

noHideCharacter:
	
    call    writeLCDData

    banksel lcdBufOutPtrH
	call	incrementLCDOutBufferPointers

	return

; end of writeNextCharInBufferToLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; incrementLCDOutBufferPointers
;
; Increments the pointers used to track the next character to be written to the LCD from the
; character buffer -- the buffer location, line number, and column number are all incremented.
;
; When the last column is reached, the line number is incremented while the column rolls back to 0;
; when the last line is reached the line number rolls back to 0.
;
; Data bank 1 should be selected on entry.
;

incrementLCDOutBufferPointers:

	incf	lcdBufOutPtrL,F	; point to next character in buffer
    btfsc   STATUS,Z		
    incf    lcdBufOutPtrH,F

	incf	lcdOutColumn,F	; track column number
	movf	lcdOutColumn,W	; check if highest column number reached
 	sublw	PAST_MAX_COLUMN
 	btfss	STATUS,Z
    goto	noRollOver

	clrf	lcdOutColumn	; start over at column 0

	incf	lcdOutLine,F	; track line number
	movf	lcdOutLine,W	; check if highest line number reached
 	sublw	PAST_MAX_LINE
 	btfss	STATUS,Z
	goto	setLCDVariablesPerLineNumber    ; highest not reached

	; highest line number reached -- handle end of refresh tasks

	call	handleEndOfRefreshTasks

    banksel lcdOutLine
	clrf	lcdOutLine						; start over at line 0
	call	setLCDVariablesPerLineNumber

noRollOver:

	return

; end of incrementLCDOutBufferPointers
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleEndOfRefreshTasks
;
; Performs tasks required at the end of a refresh such as setting cursor and blink on/off states.
; 
; Note: The cursor and blink functions of the display are not used -- the PIC code handles those
; functions. See notes "Cursor and Blinking" in this file.
;
; Bank selection not important on entry.
;

handleEndOfRefreshTasks:

    banksel flags

	; refreshing the buffer moves the cursor location with each character sent
	; set the cursor location to the last location specified by the "Main" PIC

	movf	currentCursorLocation,W
	movwf	lcdData         	; store for use by writeLCDData function
    call    writeLCDInstruction

	; the cursor and blink are turned off at the start of each buffer refresh to eliminate
	; flicker as they follow each character sent; at the end of each refresh, set them
	; to the last state specified by the "Main" PIC

	; check if "Main" PIC has turned blinking on

	btfss	currentLCDOnOffState,BLINK_ON_OFF_CMD_FLAG
	goto	blinkIsOffHERT

	; blinking is on -- check if blink rate counter has timed out

	decfsz	charBlinkRate,F
	goto	blinkIsOffHERT

	movlw	CURSOR_BLINK_RATE		; reset blink rate timer
	movwf	charBlinkRate

	; timed out -- switch on/off states of character at cursor location

	movf	flags,W					; flip the character at cursor on/off state flag
	xorlw	CHAR_AT_CURSOR_STATE_XOR_MASK
	movwf	flags

blinkIsOffHERT:

	return

; end of handleEndOfRefreshTasks
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setLCDVariablesPerLineNumber
;
; Sets the lcdBufOutPtr and the write address currently stored in the LCD display appropriate to
; the current buffer line number being written.
;
; Data bank 1 should be selected on entry.
;

setLCDVariablesPerLineNumber:

	movf	lcdOutLine,W	; handle line 0
 	sublw	0
 	btfss	STATUS,Z
    goto	notLine0

	movlw   high lcd0		; start of line 0
    movwf   lcdBufOutPtrH
	movlw   low lcd0
    movwf   lcdBufOutPtrL

	movlw	LCD_COLUMN0_START
	goto	writeLCDInstructionAndExit

notLine0:

	movf	lcdOutLine,W	; handle line 1
 	sublw	1
 	btfss	STATUS,Z
    goto	notLine1

	movlw   high lcd20		; start of line 1
    movwf   lcdBufOutPtrH
	movlw   low lcd20
    movwf   lcdBufOutPtrL

	movlw	LCD_COLUMN1_START
	goto	writeLCDInstructionAndExit

notLine1:

	movf	lcdOutLine,W	; handle line 2
 	sublw	2
 	btfss	STATUS,Z
    goto	notLine2

	movlw   high lcd40		; start of line 2
    movwf   lcdBufOutPtrH
	movlw   low lcd40
    movwf   lcdBufOutPtrL

	movlw	LCD_COLUMN2_START
	goto	writeLCDInstructionAndExit

notLine2:

	; don't check if line 3 -- any number not caught above is either 3 or illegal; if illegal then default
	; to line 3 to get things back on track

	movlw   high lcd60		; start of line 3
    movwf   lcdBufOutPtrH
	movlw   low lcd60
    movwf   lcdBufOutPtrL

	movlw	LCD_COLUMN3_START

writeLCDInstructionAndExit:

    banksel lcdData
	movwf	lcdData					; save set address instruction code for writing
    call    writeLCDInstruction		

	return

; end of setLCDVariablesPerLineNumber
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearLCDLocalBuffer
;
; Sets all data in the local LCD character buffer to spaces. The LCD display will be cleared
; when the local buffer is next transmitted to the display.
;
; Bank selection not important on entry.
; STATUS:IRP is modified     bsf     STATUS,IRP      ; use upper half of memory for indirect addressing of LCD buffer
;

clearLCDLocalBuffer:

    banksel lcdScratch0      ; select data bank 1 to access LCD buffer variables

	movlw	LCD_BUFFER_SIZE	; set up loop counter
	movwf	lcdScratch0

	movlw	high lcd0		; point indirect register FSR at buffer start
    movwf   FSR0H
	movlw	low lcd0
    movwf   FSR0L

	movlw	' '				; fill with spaces

clearLCDLoop:
	
	movwf	INDF0			; store to each buffer location
	incf	FSR0,F
	decfsz	lcdScratch0,F
	goto	clearLCDLoop
	
	call	setUpLCDCharacterBuffer

	return

; end of clearLCDLocalBuffer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setUpLCDCharacterBuffer
;
; Prepares the LCD character buffer for use.
;
; Bank selection not important on entry.
;

setUpLCDCharacterBuffer:

   	banksel lcdBufInPtrH     ; select data bank 1 to access LCD buffer variables

	movlw   high lcd0		; set write to buffer pointer from master PIC to line 0 column 0
    movwf   lcdBufInPtrH
	movlw   low lcd0
    movwf   lcdBufInPtrL

    clrf    lcdOutLine    	; start at line 0 for writing buffer to LCD
	clrf	lcdOutColumn	; start a column 0 for writing buffer to LCD

	call	setLCDVariablesPerLineNumber	; set up buffer out to LCD variables

	return

; end of setUpLCDCharacterBuffer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setLCDBufferWriteAddress
;
; Sets the LCD buffer write pointer according to the address in lcdData. This value is the
; control code that would be written to the LCD display to set an address. 
;
; see "LCD ADDRESSING NOTE" in header notes at top of page for addressing explanation
;
; Bank 0 should be selected on entry.
;

setLCDBufferWriteAddress:

	movf	lcdData,W		; load address control code from bank 0

	movwf	currentCursorLocation	; store as the cursor location for later use when the
									; display is refreshed

   	banksel lcdScratch0     ; select data bank 1 to access LCD buffer variables

	movwf	lcdScratch0		; store address control code in bank 1 for easy access

	call	getLCDLineContainingAddress	; find which line contains the specified address

	movf	lcdBufInPtrH,W          ; store as the cursor location for later use in making
    banksel currentCursorBufPosH    ; the character at that location blink
	movwf	currentCursorBufPosH        									
   	banksel lcdScratch0
    movf	lcdBufInPtrL,W
    banksel currentCursorBufPosL
	movwf	currentCursorBufPosL        									

	return

; end of setLCDBufferWriteAddress
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setLCDOnOffAndCursorAndBlink
;
; Stores the state of the display on/off, cursor on/off and blink on/off. This command is not
; immediately transmitted to the display -- it will be applied the next time the display is
; refreshed.
;
; The "Main" PIC sets the cursor location and then turns blink on when it wants to
; highlight a character. This causes problems because the latest "LCD" PIC code refreshes
; the display by redrawing the entire screen -- when blink is activated glitches can be seen
; zipping across the screen as each character tries to blink as it is written.
;
; To solve this, the last cursor location and blink status from the "Main" PIC are stored.
; Blink is turned off during a refresh; at the end of each refresh cycle, the cursor is
; briefly positioned at the last location specified by the "Main" PIC and if blink has been
; set to "on" then it is turned back on to briefly highlight the selected location before
; the next refresh.
;
; Bank 0 should be selected on entry.
;

setLCDOnOffAndCursorAndBlink:

	movf	lcdData,W		; load address control code from bank 0

	movwf	currentLCDOnOffState	; store as the display on/off, cursor on/off, blink on/off
									; for use next time the display is refreshed

	return

; end of setLCDOnOffAndCursorAndBlink
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeToLCDBuffer
;
; Writes the byte in lcdData to the local LCD character buffer at memory location stored in
; lcdBufInPtr. Pointer lcdBufInPtr is then incremented.
;
; The number of characters written to each line is tracked via lcdInColumn. If the maximum
; number of characters has been stored for a line, all further attempts to write will be ignored
; until the address is reset.
;
; Bank 0 should be selected on entry.
; STATUS:IRP is modified
;

writeToLCDBuffer:

	movf	lcdData,W		; get the byte to be stored

   	banksel lcdScratch0     ; select data bank 1 to access LCD buffer variables
	movwf	lcdScratch0		; store byte in bank 1 for easy access

	movf	lcdInColumn,W	; bail out if already one past the max column number
 	sublw	PAST_MAX_COLUMN	
 	btfsc	STATUS,Z	
	return

	incf	lcdInColumn,f	; track number of bytes written to the line
	
    movf    lcdBufInPtrH,W  ; get pointer to next memory location to be used
    movwf   FSR0H           ; point FSR at the character    
    movf    lcdBufInPtrL,W  ; get pointer to next memory location to be used
    movwf   FSR0L           ; point FSR at the character    

	movf	lcdScratch0,W	; retrieve the byte and store it in the buffer
    movwf	INDF0

    incf    lcdBufInPtrL,F	; increment the pointer
    btfsc   STATUS,Z		
    incf    lcdBufInPtrH,F

	return

; end of writeToLCDBuffer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; getLCDLineContainingAddress
;
; Returns in the W register the line containing the address specified by the control code in
; lcdScratch0. The control code is the value which would be sent to the LCD display to set the
; address.
;
; The lcdBufInPtr will be set to the proper buffer location for storing at the spot which mirrors
; the specified address in the LCD buffer. The LCD buffer has a non-contiguous addressing while
; the buffer in the PIC is contiguous, so some translation is required.
;
; NOTE: this function only manipulates the lower byte of the lcdBufInPtr. It is assumed that the
; buffer does not cross a 256 byte boundary and the pointer high byte does not change.
;
; An illegal address outside the range of any line defaults to line 3.
;
; see "LCD ADDRESSING NOTE" in header notes at top of page for addressing explanation
;
; Bank 1 should be selected on entry.
;
; REMEMBER: Borrow flag is inverse: 0 = borrow, 1 = no borrow
;

getLCDLineContainingAddress:

	; check for address any where on line 0 (between *_START and *_END

	movlw	LCD_COLUMN0_START	; compare address with *_START
    subwf	lcdScratch0,W		; address >= *_START?
    btfss   STATUS,C			; c = 0 = borrow = address<*_START
    goto	notLine0_GL

	movf	lcdScratch0,W		; compare address	
	sublw	LCD_COLUMN0_END		; address <= *_END?
    btfss   STATUS,C			; c = 0 = borrow = address>*_END
    goto	notLine0_GL

	movlw	LCD_COLUMN0_START	; calculate the buffer index for the address
	subwf	lcdScratch0,W		; by finding the column number first by
								; subtracting the line's start address
	movwf	lcdInColumn			; store the column
	addlw	lcd0				; add column to the line start's memory location
	movwf	lcdBufInPtrL		; to get the address's memory location

	movlw	0					; the address is in line 0
	return

notLine0_GL:

	movlw	LCD_COLUMN1_START	; compare address with *_START
    subwf	lcdScratch0,W		; address >= *_START?
    btfss   STATUS,C			; c = 0 = borrow = address<*_START
    goto	notLine1_GL

	movf	lcdScratch0,W		; compare address	
	sublw	LCD_COLUMN1_END		; address <= *_END?
    btfss   STATUS,C			; c = 0 = borrow = address>*_END
    goto	notLine1_GL

	movlw	LCD_COLUMN1_START	; calculate the buffer index for the address
	subwf	lcdScratch0,W		; by finding the column number first by
								; subtracting the line's start address
	movwf	lcdInColumn			; store the column
	addlw	lcd20				; add column to the line start's memory location
	movwf	lcdBufInPtrL		; to get the address's memory location

	movlw	1					; the address is in line 1
	return

notLine1_GL:

	movlw	LCD_COLUMN2_START	; compare address with *_START
    subwf	lcdScratch0,W		; address >= *_START?
    btfss   STATUS,C			; c = 0 = borrow = address<*_START
    goto	notLine2_GL

	movf	lcdScratch0,W		; compare address	
	sublw	LCD_COLUMN2_END		; address <= *_END?
    btfss   STATUS,C			; c = 0 = borrow = address>*_END
    goto	notLine2_GL

	movlw	LCD_COLUMN2_START	; calculate the buffer index for the address
	subwf	lcdScratch0,W		; by finding the column number first by
								; subtracting the line's start address
	movwf	lcdInColumn			; store the column
	addlw	lcd40				; add column to the line start's memory location
	movwf	lcdBufInPtrL		; to get the address's memory location

	movlw	2					; the address is in line 2
	return

notLine2_GL:

	; all addresses not caught so far returned as line 3
	; illegal addresses end up here as well and default to line 3

	movlw	LCD_COLUMN3_START	; calculate the buffer index for the address
	subwf	lcdScratch0,W		; by finding the column number first by
								; subtracting the line's start address
	movwf	lcdInColumn			; store the column
	addlw	lcd60				; add column to the line start's memory location
	movwf	lcdBufInPtrL		; to get the address's memory location

	movlw	3					; the address is in line 3
	return

; end of getLCDLineContainingAddress
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displayGreeting
;
; Displays a greeting string, version info, etc.
;
; The text is written to the local LCD character buffer so it will be transmitted to the display.
;
; Bank 0 should be selected on entry.
;
; wip mks -- convert this to the write string method used in "OPT EDM Main PIC.asm"
;

displayGreeting:

	movlw	0x80			; move cursor to line 1 column 1 (address 0x00 / code 0x80)
	movwf	lcdData         ;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
    banksel flags

	movlw	'O'				; display "OPT EDM" on the first line
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'P'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'T'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	' '
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'E'                             
	movwf 	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'D'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'M'                     
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	0xc1			; move cursor to line 2 column 2 (address 41h)
	movwf	lcdData         ;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
    banksel flags

	movlw	'N'				; display "Notcher" on the second line
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'o'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	't'                            
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'c'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'h'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'e'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'r'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	0x96			; move cursor to line 3 column 7 (address 16h)
	movwf	lcdData			;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
    banksel flags

	movlw	'b'				; display "by CMP" on the third line
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'y'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	' '  
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'M'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'K'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'S'
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	0xd7			; move cursor to line 4 column 8 (address 57h)
	movwf	lcdData			;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
    banksel flags

	movlw	'R'				; display "Rev 2.7" on the fourth line
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'e'                            
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'v'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	' '                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'2'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

 	movlw	'.'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	movlw	'1'
	movwf	lcdData                             
    call    writeToLCDBuffer
    banksel flags

	return

; end of displayGreeting
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; initLCD
;
; Initialize the LCD display.
;
; See Dmcman_full-user manual.pdf from www.optrex.com for details.
;

initLCD:

	bcf		LCD_CTRL,LCD_E		; LCD E strobe low                          
	bcf		LCD_CTRL, LCD_RS    ; LCD RS low (instruction register selected)                       
    call    smallDelay			; wait a bit

	movlw	0x30				; 1st send of Function Set Command: (8-Bit interface)(BF cannot be checked before this command.)
	movwf	LCD_DATA			; prepare to write                   
    call    strobeE				; write to LCD
    call    bigDelay			; should wait more than 4.1ms

	movlw	0x30				; 2nd send of Function Set Command: (8-Bit interface)(BF cannot be checked before this command.)
	movwf	LCD_DATA			; prepare to write                   
    call    strobeE				; write to LCD
    call    smallDelay			; should wait more than 100us

	movlw	0x30				; 3rd send of Function Set Command: (8-Bit interface)(BF can be checked after this command)
	movwf	LCD_DATA			; prepare to write	(BF busy flag cannot be checked on this board because R/W line is tied low)
    call    strobeE				; write to LCD
    call    smallDelay			; wait for a bit because BF (busy flag) cannot be checked on this board

	movlw	0x38				; write 0011 1000 Function Set Command ~ multi line display with 5x7 dot font
	movwf	LCD_DATA			;  0011 in upper nibble specifies Function Set Command
    call    strobeE				;  bit 3: 0 = 1 line display, 1 = multi-line display
								;  bit 2: 0 = 5x7 dot font, 1 = 5 x 10 dot font
	call    smallDelay			; wait for a bit because BF (busy flag) cannot be checked on this board

	movlw	0x0c				; write 0000 1000 ~ Display Off
	movwf	LCD_DATA			;  bit 3: specifies display on/off command
    call    strobeE				;  bit 2: 0 = display off, 1 = display on
	call    smallDelay			; wait for a bit because BF (busy flag) cannot be checked on this board
								; NOTE: LCD user manual instructs to turn off display here with 0x08
								;  but this did NOT work. Unknown why.

	movlw	0x01				; write 0000 0001 ~ Clear Display
	movwf	LCD_DATA
    call    strobeE
	call    smallDelay			; wait for a bit because BF (busy flag) cannot be checked on this board
								; NOTE: clear display added by MKS to match suggested setup in LCD user manual

	movlw	0x06				; write 0000 0110 ~ Entry Mode Set, increment mode, no display shift
	movwf	LCD_DATA			; bits 3:2 = 0:1 : specifies Entry Mode Set
    call    strobeE				; bit 1: 0 = no increment, 1 = increment mode; bit 0: 0 = no shift, 1 = shift display
	call    smallDelay			; wait for a bit because BF (busy flag) cannot be checked on this board
								; NOTE: Entry Mode Set was being done after display on -- moved by MKS to match
								;		suggested setup in LCD user manual.
								; NOTE2: See above regarding not working when display turned off before this --
								; does display need to be on when this command given regardless of LCD manual
								; suggestions?

	movlw	0x0c				; write 0000 1100 ~ Display On, cursor off, blink off
	movwf	LCD_DATA			;  bit 3: specifies display on/off command
    call    strobeE				;  bit 2: 0 = display off, 1 = display on
								;  bit 1: 0 = cursor off, 1 = cursor on
								;  bit 0: 0 = blink off, 1 = blink on

; Note: BF should be checked before each of the instructions starting with Display OFF.
;   Since this board design does not allow BF (LCD busy flag) to be checked, a delay is inserted after each
;	instruction to allow time for completion.

    call    bigDelay

	return

; end of initLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeLCDData
;
; Writes a data byte from variable lcdData to the LCD.  The data is a character to be displayed.
;
; Data bank 0 should be selected on entry.
;
                                 
writeLCDData:

	bcf		LCD_CTRL,LCD_E			; init E to low
	bsf		LCD_CTRL,LCD_RS			; select data register in LCD

;debug mks
;    call    smallDelay
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	movf	lcdData,W				; place data on output port
	movwf	LCD_DATA                          
    call    strobeE					; write the data

;debug mks
;    call    smallDelay
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	return

; end of writeLCDData
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeLCDInstruction
;
; Writes an instruction byte from variable lcdData to the LCD.  An instruction is one of several
; codes recognized by the LCD display for clearing the screen, moving the cursor, etc.
;
; Data bank 0 should be selected on entry.
;
                                 
writeLCDInstruction:

	bcf 	LCD_CTRL,LCD_E			; init E to low  
	bcf 	LCD_CTRL,LCD_RS			; select instruction register in LCD

;debug mks
;    call    smallDelay
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	movf 	lcdData,W				; place instruction on output port
	movwf	LCD_DATA                         
    call    strobeE					; write the instruction
	bsf		LCD_CTRL,LCD_RS			; set the instruction/data register select back to data register

;debug mks
;    call    smallDelay
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	return

; end of writeLCDInstruction
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; smallDelay
;
; Creates a small delay.
;
                                 
smallDelay:
	movlw	0x2a                             
	movwf	smallDelayCnt
                             
L8b:
	decfsz	smallDelayCnt,F                         
    goto    L8b
	return

; end of smallDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; bigDelay
;
; Creates a big delay.
;
                                 
bigDelay:

	movlw	0x28                             
	movwf	bigDelayCnt                             
    call	smallDelay

L9b:
	decfsz	bigDelayCnt,F                         
    goto    L9b
	return

; end of bigDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; strobeE
;
; Strobes LCD E line to read/write data to the LCD and then delays for a bit.
;

strobeE:

	bsf		LCD_CTRL,LCD_E
	nop                                    
	nop
	nop
	nop
	nop
	bcf		LCD_CTRL,LCD_E

;debug mks    
;    call    smallDelay
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	return

; end of strobeE
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleTimer0Interrupt
;
; This function is transmits the switch presses to the Main PIC. 
;
; This function will only be called if the xmtBusy flag is set. 
; only sends one bit inside the xmtBuffer and then exits. The next time around, it sends the next
; bit in the buffer. Values in the xmtBuffer should not be changed until the entire buffer is sent.
; When the whole buffer has been sent, 
;
; This function is called when the Timer0 register overflows.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 8 deep and
; it is very bad for the interrupt routine to use it.
;

handleTimer0Interrupt:

	bcf 	INTCON,T0IF         ; clear the Timer0 overflow interrupt flag

    banksel xmtFlags            ; select data bank 1 to access LCD buffer variables

    btfss   xmtFlags,xmtBusy    
    goto    endISR              ; if nothing in buffer, exit

    btfss   xmtFlags,inDelay    ; if in delay phase, waste time until counter is zero
    goto    startBitCheck       ;  (this delay is necessary between words, but is used
                                ;    between every byte for simplicity)

    decfsz  xmtDelay0,F         ; decrement (actually decrements upper byte when lower byte reaches 
                                ; 0 instead of just past 0, but accurate enough for this purpose)
    decfsz  xmtDelay1,F
    goto    endISR

    bcf     xmtFlags,inDelay    ; delay ended

    bsf     xmtFlags,startBit   ; transmit start bit on next interrupt
    
    btfss   xmtFlags,endBuffer  ; buffer empty?
    goto    endISR

    clrf    xmtFlags            ; if end of buffer reached, set LCD not busy
    movlw   high xmtBuffer0     ; reset pointer to beginning of the buffer
    movwf   xmtBufferPtrH
    movlw   low xmtBuffer0
    movwf   xmtBufferPtrL    
        
    clrf    xmtBufferCnt        ; no characters in the buffer

    goto    endISR

startBitCheck:

    btfss   xmtFlags,startBit   ; if set, initiate a startbit and exit    
    goto    stopBitCheck

    banksel PORT_LOCAL_TO_MAIN

    bcf     PORT_LOCAL_TO_MAIN,LOCAL_TO_MAIN ; transmit start bit (low)

    banksel xmtFlags

    bcf     xmtFlags,startBit   ; start bit done
    movlw   .8
    movwf   xmtBitCount         ; prepare to send 8 bits starting with next interrupt
    goto    endISR    

stopBitCheck:

    btfss   xmtFlags,stopBit    ; if set, initiate a stopbit and exit
    goto    transmitByteT0I

    banksel PORT_LOCAL_TO_MAIN

    bsf     SERIAL_OUT_P, SERIAL_OUT           ; transmit stop bit (high)

    banksel LCDFlags
    
    bcf     LCDFlags,stopBit    ; stop bit done
    
    movlw   0x30                ; don't use less than 1 here - will count from 0xff
    movwf   LCDDelay0
    movlw   0x01                ; don't use less than 1 here - will count from 0xff
    movwf   LCDDelay1           ; setup delay
    bsf     LCDFlags,inDelay    ; start delay on next interrupt
    
    goto    endISR    

transmitByteT0I:

    movf    LCDBufferPtrH,W     ; get pointer to next character to be transmitted
    movwf   FSR0H
    movf    LCDBufferPtrL,W
    movwf   FSR0L
    rlf     INDF0,F             ; get the first bit to transmit

    banksel SERIAL_OUT_P

    bcf     SERIAL_OUT_P,SERIAL_OUT ; set data line low first (brief low if bit is to be a one will
                                    ; be ignored by receiver)
    btfsc   STATUS,C
    bsf     SERIAL_OUT_P,SERIAL_OUT ; set high if bit was a 1

    banksel LCDFlags

endOfByteCheck:

    decfsz  LCDBitCount,F       ; count number of bits to transmit
    goto    endISR

    bsf     LCDFlags,stopBit    ; signal to transmit stop bit on next interrupt

    incf    LCDBufferPtrL,F     ; point to next character in buffer
    btfsc   STATUS,Z
    incf    LCDBufferPtrH,F
    
    decfsz  LCDBufferCnt,F      ; buffer empty?
    goto    endISR

    bsf     LCDFlags,endBuffer  ; signal to stop transmitting after next stop bit
    
endISR:
    
    retfie                  	; Return and enable interrupts

; end of handleTimer0Interrupt
;--------------------------------------------------------------------------------------------------
 
    END
