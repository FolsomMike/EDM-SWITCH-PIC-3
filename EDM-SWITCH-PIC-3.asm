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
; previous states of the switches or buttons have been sent to the Main PIC by the interrupt
; routine. Then the temporary byte will be copied into the buffer to be sent. This ensures that a
; switch press is never missed and the previous presses are safely sent to the Main Pic.
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
;
; Serial Data Timing
;
; This PIC sends serial data 4 times slower than the Main PIC sends its data to the LCD PIC. This
; allows the Main PIC to use the same Timer interrupt rate it uses to send data to also receive
; data. The Timer rate must be faster than the incoming data rate to make sure that start bits
; are detected at or before their midpoints so that the timing for the rest of the byte is proper.
;
; Note that TMR0, used for serial data sending interrupts, is never reloaded -- thus it wraps
; around and does a full count for each interrupt. This is identical to the operation in
; EDM-Main-Pic-2.asm.
;
; The Timer clock rate is slowed by enabling the pre-scaler and choosing ratio of 1:4.
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
SERIAL_OUT               EQU     RB7    ; RB7   - used to send serial data to main

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
                            
    switchPresses           ; All bits set if no buttons pressed (11111111b)
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

    xmtBuffer0              ; xmt buffer - holds data being transmitted to the Main PIC
    xmtBuffer1
    xmtBuffer2
    xmtBuffer3
    xmtBuffer4
    xmtBuffer5

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

    clrf    INTCON          ; disable all interrupts

    call    setupClock      ; set system clock source and frequency

    call    setupPortA

;    banksel LCD_RW_RS_L
;    bcf		LCD_RW_RS_L,LCD_RW  ; set LCD R/W low to select write to LCD mode
;    bcf		LCD_RW_RS_L,LCD_RS	; set LCD Register Select low (chooses instruction register)

    call    setupPortB

;    banksel LCD_E_L
; 	bcf		LCD_E_L,LCD_E       ; set LCD E strobe low (inactive)

    call    setupPortC

    banksel OPTION_REG

	movlw	0x51			; set options 0101 0001 b
	movwf	OPTION_REG		; bit 7 = 0: PORTB pull-ups are enabled by individual port latch values
     						; bit 6 = 1: RBO/INT interrupt on rising edge
							; bit 5 = 0: TOCS ~ Timer 0 run by internal instruction cycle clock (CLKOUT ~ Fosc/4)
							; bit 4 = 1: TOSE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin (not used here)
							; bit 3 = 0: PSA ~ Prescaler enabled for Timer 0
                            ; bit 2 = 0 : Bits 2:0 control prescaler:
                            ; bit 1 = 0 :    001 = 1:4 scaling for Timer0
                            ; bit 0 = 1 :

    banksel flags

	clrf   	flags

    banksel xmtFlags

    clrf    xmtFlags        
    movlw   high xmtBuffer0
    movwf   xmtBufferPtrH
    movlw   low xmtBuffer0
    movwf   xmtBufferPtrL

    clrf    xmtBufferCnt    ; no characters in the buffer

; enable the interrupts

	bsf	    INTCON,PEIE	    ; enable peripheral interrupts (Timer0 is a peripheral)
    bsf     INTCON,T0IE     ; enabe TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

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
; setupPortA
;
; Sets up Port A for I/O operation.
;
; NOTE: Writing to PORTA is same as writing to LATA for PIC16f1459. The code example from the
; data manual writes to both on initialization -- probably to be compatible with other PIC chips.
;
; NOTE: RA0, RA1 and RA3 can only be inputs on the PIC16f1459 device.
;       RA2, RA6, RA7 are not implemented.
;

setupPortA:

    banksel WPUA
    movlw   b'00000000'                 ; disable weak pull-ups
    movwf   WPUA

    banksel PORTA
    clrf    PORTA                       ; init port value

    banksel LATA                        ; init port data latch
    clrf    LATA

    banksel ANSELA
    clrf    ANSELA                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISA
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISA

    ; set direction for each pin used

;debug mks -- change these
;    bsf     TRISA, SERIAL_IN            ; input
;    bcf     TRISA, LCD_RW               ; output
;    bcf     TRISA, LCD_RS               ; output
;    bsf     TRISA, RA1                  ; input - unused - pulled up via external resistor

    return

; end of setupPortA
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortB
;
; Sets up Port B for I/O operation.
;
; NOTE: Writing to PORTB is same as writing to LATB for PIC16f1459. The code example from the
; data manual writes to both on initialization -- probably to be compatible with other PIC chips.
;
; NOTE: RB0, RB1, RB2, RB3 are not implemented on the PIC16f1459 device.
;

setupPortB:

    banksel WPUB
    movlw   b'00000000'                 ; disable weak pull-ups
    movwf   WPUB

    banksel PORTB
    clrf    PORTB                       ; init port value

    banksel LATB                        ; init port data latch
    clrf    LATB

    banksel SERIAL_OUT_L
    bsf     SERIAL_OUT_L,SERIAL_OUT     ; initialize SERIAL_OUT high before changing pin to output
                                        ; so a start bit won't be transmitted
    banksel ANSELB
    clrf    ANSELB                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISB
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISB

    ; set direction for each pin used

; debug mks -- change these
;    bcf     TRISB, SERIAL_OUT           ; output
;    bcf     TRISB, LCD_E                ; output

    return

; end of setupPortB
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortC
;
; Sets up Port C for I/O operation.
;
; NOTE: Writing to PORTC is same as writing to LATC for PIC16f1459. The code example from the
; data manual writes to both on initialization -- probably to be compatible with other PIC chips.
;
; For this program, PortC is the LCD data bus.
;

setupPortC:

    ; Port C does not have a weak pull-up register

    banksel PORTC
    clrf    PORTC                       ; init port value

    banksel LATC
    clrf    LATC                        ; init port data latch

    banksel ANSELC
    clrf    ANSELC                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISC
    movlw   b'00000000'                 ; set all to outputs
    movwf   TRISC

    return

; end of setupPortC
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Main Code
;
; Sets up the PIC, the LCD, displays a greeting, then monitors the serial data input line from
; the main PIC for data and instructions to be passed on to the LCD display.
;

start:

	call	setup			; set up main variables and hardware

mainLoop:

    banksel flags

    goto    mainLoop

; end of Main Code
;--------------------------------------------------------------------------------------------------









;--------------------------------------------------------------------------------------------------
; handleTimer0Interrupt
;
; This function transmits data in the xmt buffer to the Main PIC. 
;
; This function is called when the Timer0 register overflows.
;
; This function will only be called if the xmtBusy flag is set. 
; It only sends one bit on each invocation.
; Values in the xmtBuffer should not be changed until the entire buffer is sent.
;
; Note that TMR0, used for serial data sending interrupts, is never reloaded -- thus it wraps
; around and does a full count for each interrupt.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
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

    banksel SERIAL_OUT_L

    bcf     SERIAL_OUT_L,SERIAL_OUT ; transmit start bit (low)

    banksel xmtFlags

    bcf     xmtFlags,startBit   ; start bit done
    movlw   .8
    movwf   xmtBitCount         ; prepare to send 8 bits starting with next interrupt
    goto    endISR    

stopBitCheck:

    btfss   xmtFlags,stopBit    ; if set, initiate a stopbit and exit
    goto    transmitByteT0I

    banksel SERIAL_OUT_L

    bsf     SERIAL_OUT_L, SERIAL_OUT           ; transmit stop bit (high)

    banksel xmtFlags
    
    bcf     xmtFlags,stopBit    ; stop bit done
    
    movlw   0x30                ; don't use less than 1 here - will count from 0xff
    movwf   xmtDelay0
    movlw   0x01                ; don't use less than 1 here - will count from 0xff
    movwf   xmtDelay1           ; setup delay
    bsf     xmtFlags,inDelay    ; start delay on next interrupt
    
    goto    endISR    

transmitByteT0I:

    movf    xmtBufferPtrH,W     ; get pointer to next character to be transmitted
    movwf   FSR0H
    movf    xmtBufferPtrL,W
    movwf   FSR0L
    rlf     INDF0,F             ; get the first bit to transmit

    banksel SERIAL_OUT_L

    bcf     SERIAL_OUT_L,SERIAL_OUT ; set data line low first (brief low if bit is to be a one will
                                    ; be ignored by receiver)
    btfsc   STATUS,C
    bsf     SERIAL_OUT_L,SERIAL_OUT ; set high if bit was a 1

    banksel xmtFlags

endOfByteCheck:

    decfsz  xmtBitCount,F       ; count number of bits to transmit
    goto    endISR

    bsf     xmtFlags,stopBit    ; signal to transmit stop bit on next interrupt

    incf    xmtBufferPtrL,F     ; point to next character in buffer
    btfsc   STATUS,Z
    incf    xmtBufferPtrH,F
    
    decfsz  xmtBufferCnt,F      ; buffer empty?
    goto    endISR

    bsf     xmtFlags,endBuffer  ; signal to stop transmitting after next stop bit
    
endISR:
    
    retfie                  	; Return and enable interrupts

; end of handleTimer0Interrupt
;--------------------------------------------------------------------------------------------------
 
    END
