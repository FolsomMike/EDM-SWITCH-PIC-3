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
;--------------------------------------------------------------------------------------------------
; Operational Notes
;
;
;
;--------------------------------------------------------------------------------------------------
; Hardware Control Description
;
; Function by Pin
;
; Port A        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RA0   I/*,IOC,USB-D+                  ~ In ~ Jog Down switch input
; RA1   I/*,IOC,USB-D-                  ~ In ~ unused -- pulled high externally
; RA2   not implemented in PIC16f1459   ~ 
; RA3   I/*,IOC,T1G,MSSP-SS,Vpp,MCLR    ~ Vpp
; RA4   I/O,IOC,T1G,CLKOUT,CLKR, AN3    ~ Out ~ unused -- no connection
; RA5   I/O,IOC,T1CKI,CLKIN             ~ Out ~ unused -- no connection
; RA6   not implemented in PIC16f1459
; RA7   not implemented in PIC16f1459
;
; On version 1.0, RA0 is connected to Serial_Data_To_Local_PICs and RB5 is connected to the
; Jog Down switch. Those boards are modified with jumpers to switch RA0 and RB5 so that the
; EUSART RX on RB5 can be used to read serial data. From version 1.1 forward, the boards are
; redesigned and do not need modification.
;
; Port B        Pin/Options/Selected Option/Description  (only the most common optins are listed)
;
; RB0   not implemented in PIC16f1459
; RB1   not implemented in PIC16f1459
; RB2   not implemented in PIC16f1459
; RB3   not implemented in PIC16f1459
; RB4   I/O,IOC,MSSP-SDA/SDI,AN10       ~ In ~ I2CSDA, I2C bus data line
; RB5   I/O,IOC,EUSART-RX/DX,AN11       ~ In ~ EUSART-RX, serial port data in
; RB6   I/O,IOC,MSSP-SCL/SCK            ~ In ~ I2CSCL, I2C bus clock line
; RB7   I/O,IOC,EUSART-TX/CK            ~ Out ~ EUSART-TX, serial port data out
;
; Port C        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RC0   I/O,AN4,C1/2IN+,ICSPDAT,Vref    ~ Out ~ ICSPDAT ~ in circuit programming data, AC OK LED 
; RC1   I/O,AN5,C1/2IN1-,ICSPCLK,INT    ~ In ~ ICSPCLK ~ in circuit programming clock, Mode Switch
; RC2   I/O,AN6,C1/2IN2-,DACOUT1        ~ In ~ Jog Up Switch
; RC3   I/O,AN7,C1/2IN3-,DACOUT2,CLKR   ~ Out ~ Buzzer 
; RC4   I/O,C1/2OUT                     ~ In ~ Electrode Power Switch 
; RC5   I/O,T0CKI,PWM1                  ~ Out ~ unused -- no connection
; RC6   I/O,AN8,PWM2,MSSP-SS            ~ Out ~ Short Condition LED
; RC7   I/O,AN9,MSSP-SDO                ~ In ~ Select switch
;
;end of Hardware Control Description
;--------------------------------------------------------------------------------------------------
;
; User Inputs
;
; See Hardware Control Description above.
;
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Defines
;
    
; COMMENT OUT "#define DEBUG_MODE" line before using code in system.
; Defining DEBUG_MODE will insert code which simplifies simulation by skipping code which waits on
; stimulus and performing various other actions which make the simulation run properly.
; Search for "DEBUG_MODE" to find all examples of such code.

;#define DEBUG_MODE 1     ; set DEBUG_MODE testing "on" ;//debug mks -- comment this out later

JOG_DEGLITCH_CNT0 EQU    .255

; end of Defines
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

; Indicator Outputs

INDICATORS_OUT_L        EQU     LATC

AC_OK_LED               EQU     RC0
BUZZER                  EQU     RC3
SHORT_LED               EQU     RC6

; Switches

MODE_JOGUP_SEL_EPWR_P   EQU     PORTC
JOGDWN_P                EQU     PORTA

MODE_SW                 EQU     RC1
JOG_UP_SW               EQU     RC2
SELECT_SW               EQU		RC7
ELECTRODE_PWR_SW        EQU     RC4
JOG_DOWN_SW             EQU     RA0

;bits in switchStates variable

MODE_SW_FLAG            EQU     0
JOG_UP_SW_FLAG          EQU     1
JOG_DOWN_SW_FLAG        EQU     2          
SELECT_SW_FLAG          EQU     3
ELECTRODE_PWR_SW_FLAG   EQU     4
AC_OK_FLAG              EQU     5

;bits in outputStates variable

AC_OK_LED_FLAG          EQU     0
BUZZER_FLAG             EQU     1
SHORT_LED_FLAG          EQU     2

; end of Constants
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Software Definitions
;

; bits in flags2 variable

HEADER_BYTE_1_RCVD  EQU 0
HEADER_BYTE_2_RCVD  EQU 1
LENGTH_BYTE_VALID   EQU 2
SERIAL_PACKET_READY EQU 3

; bits in statusFlags variable

SERIAL_COM_ERROR    EQU 0
I2C_COM_ERROR       EQU 1

SERIAL_RCV_BUF_LEN  EQU .10

SERIAL_XMT_BUF_LEN  EQU .10

; Serial Port Packet Commands

NO_ACTION_CMD               EQU .0
ACK_CMD                     EQU .1
SET_OUTPUTS_CMD             EQU .2
SWITCH_STATES_CMD           EQU .3
LCD_DATA_CMD                EQU .4
LCD_INSTRUCTION_CMD         EQU .5
LCD_BLOCK_CMD               EQU .6

; end of Software Definitions
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
                            ; bit 1:
                            ; bit 2:
                            ; bit 3:
                            ; bit 4:
                            ; bit 5:
							; bit 6:
							; bit 7:

    flags2                  ; bit 0: 1 = first serial port header byte received
                            ; bit 1: 1 = second serial port header byte received
                            ; bit 2: 1 = serial port packet length byte received and validated
                            ; bit 3: 1 = data packet ready for processing
                            ; bit 4: 0 =
                            ; bit 5: 0 =
							; bit 6: 0 =
							; bit 7: 0 =

    statusFlags             ; bit 0: 0 = one or more com errors from serial have occurred
                            ; bit 1: 0 = one or more com errors from I2C have occurred
                            ; bit 2: 0 =
                            ; bit 3: 0 =
                            ; bit 4: 0 =
                            ; bit 5: 0 =
							; bit 6: 0 =
							; bit 7: 0 =
                        
    switchStates            ; all bits set if no buttons pressed (1111 1111b)
                            ; bit 0: 0 = Mode switch active
                            ; bit 1: 0 = Jog Up switch active
                            ; bit 2: 0 = Select switch active
                            ; bit 3: 0 = Electrode Power switch active
                            ; bit 4: 0 = Jog Down switch active
                            ; bit 5:
							; bit 6:
							; bit 7:

    outputStates            ; all bits set if no buttons pressed (1111 1111b)
                            ; bit 0: 0 = AC OK LED lit
                            ; bit 1: 0 = Buzzer on
                            ; bit 2: 0 = Short Condition LED lit
                            ; bit 3: 0 = 
                            ; bit 4: 0 = 
                            ; bit 5:
							; bit 6:
							; bit 7:

    deGlitchCntr0           ; counter used to ignore noise spikes on switch lines

	smallDelayCnt			; used to count down for small delay
	bigDelayCnt				; used to count down for big delay

    serialPortErrorCnt      ; number of com errors from Rabbit via serial port
    slaveI2CErrorCnt        ; number of com errors from Slave PICs via I2C bus

    serialRcvPktLenMain     ; used by main to process completed packets
    serialRcvPktCntMain     ; used by main to process completed packets
    
    usartScratch0
    usartScratch1
    serialIntScratch0
 
    ; used by serial receive interrupt    
    
    serialRcvPktLen
    serialRcvPktCnt
    serialRcvBufPtrH
    serialRcvBufPtrL
    serialRcvBufLen
    
    ; used by serial transmit interrupt    
    
    serialXmtBufNumBytes
    serialXmtBufPtrH
    serialXmtBufPtrL
    serialXmtBufLen

    ; variables for general use
    
    scratch0
    scratch1
    scratch2
    
    endc 
    
; end of Bank 0
;----------------------
    
;----------------------
; Assign variables in RAM - Bank 1
; Bank 1 has 80 bytes of free space
 
; WARNING: These buffers may be large enough to overrun the following banks. Linear indirect
; addressing is used to access them. They may be moved to a higher bank if necessary to make room
; for variables in this bank.

 cblock 0xa0                ; starting address

    serialRcvBuf:SERIAL_RCV_BUF_LEN

    serialXmtBuf:SERIAL_XMT_BUF_LEN
 
    endc

; Compute address of serialRcvBuf in linear data memory for use as a large buffer
RCV_BUF_OFFSET EQU (serialRcvBuf & 0x7f) - 0x20
SERIAL_RCV_BUF_LINEAR_ADDRESS   EQU ((serialRcvBuf/.128)*.80)+0x2000+RCV_BUF_OFFSET
SERIAL_RCV_BUF_LINEAR_LOC_H     EQU high SERIAL_RCV_BUF_LINEAR_ADDRESS
SERIAL_RCV_BUF_LINEAR_LOC_L     EQU low SERIAL_RCV_BUF_LINEAR_ADDRESS
    
; Compute address of serialXmtBuf in linear data memory for use as a large buffer
XMT_BUF_OFFSET EQU (serialXmtBuf & 0x7f) - 0x20
SERIAL_XMT_BUF_LINEAR_ADDRESS   EQU ((serialXmtBuf/.128)*.80)+0x2000+XMT_BUF_OFFSET
SERIAL_XMT_BUF_LINEAR_LOC_H     EQU high SERIAL_XMT_BUF_LINEAR_ADDRESS
SERIAL_XMT_BUF_LINEAR_LOC_L     EQU low SERIAL_XMT_BUF_LINEAR_ADDRESS

; end of Bank 1
;----------------------

;-----------------
; Define variables in the memory which is mirrored in all RAM banks.
;
; On older PICs, this section was used to store context registers during an interrupt as the
; current bank was unknown upon entering the interrupt. Now, the section can be used for any
; purpose as the more powerful PICs automatically save the context on interrupt.
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

	org 0x00                ; Start of Program Memory

	goto start              ; jump to main code section
	nop			            ; Pad out so interrupt
	nop			            ; service routine gets
	nop			            ; put at address 0x0004.


; interrupt vector at 0x0004

; NOTE: You must set PCLATH before jumping to the interrupt routine - if PCLATH is wrong the
; jump will fail.
 
    movlp   high handleInterrupt
    goto    handleInterrupt	; points to interrupt service routine
    
; end of Power On and Reset Vectors
;--------------------------------------------------------------------------------------------------

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

    call    setupPortB

    call    setupPortC

    call    setupSerialPort

    banksel OPTION_REG

    movlw   0x58
    movwf   OPTION_REG      ; Option Register = 0x58   0101 1000 b
                            ; bit 7 = 0 : weak pull-ups are enabled by individual port latch values
                            ; bit 6 = 1 : interrupt on rising edge
                            ; bit 5 = 0 : TOCS ~ Timer 0 run by internal instruction cycle clock (CLKOUT ~ Fosc/4)
                            ; bit 4 = 1 : TOSE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin (not used here)
							; bit 3 = 1 : PSA ~ Prescaler disabled; Timer0 will be 1:1 with Fosc/4
                            ; bit 2 = 0 : Bits 2:0 control prescaler:
                            ; bit 1 = 0 :    000 = 1:2 scaling for Timer0
                            ; bit 0 = 0 :

    banksel flags
	clrf   	flags

    banksel switchStates
    movlw   0xff            ; initialize switchStates -> no inputs active
    movwf   switchStates

    banksel outputStates
    movlw   0xff            ; initialize outputStates -> no outputs active
    movwf   outputStates

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

    bsf     TRISA, JOG_DOWN_SW          ; input

    bcf     TRISA, RA4                  ; set unconnected pin to output to avoid floating
    bcf     TRISA, RA5                  ; set unconnected pin to output to avoid floating

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
                                        ; so a start bit won't be transmitted
    banksel ANSELB
    clrf    ANSELB                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISB
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISB

    ; set direction for each pin used
    
    ; (no changes required)

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
    movlw   b'11111111'                 ; set all to inputs
    movwf   TRISC

    ;preset outputs before enabling

    banksel INDICATORS_OUT_L

    bsf     INDICATORS_OUT_L,AC_OK_LED  ; turn off LED
    bsf     INDICATORS_OUT_L,BUZZER     ; turn off buzzer
    bsf     INDICATORS_OUT_L,SHORT_LED  ; turn off LED

    ; set direction for each pin used

    banksel TRISC

    bsf     TRISC, MODE_SW              ; input
    bsf     TRISC, JOG_UP_SW            ; input
    bsf     TRISC, SELECT_SW            ; input
    bsf     TRISC, ELECTRODE_PWR_SW     ; input

    bcf     TRISC, AC_OK_LED            ; output
    bcf     TRISC, BUZZER               ; output
    bcf     TRISC, SHORT_LED            ; output

    bcf     TRISC, RC5                  ; set unconnected pin to output to avoid floating

    return

; end of setupPortC
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Main Code
;

start:

	call	setup			; set up main variables and hardware

mainLoop:

    call    trapSwitchInputs    ; check each switch input and store flag for any which are active

    call    sendDataIfReady     ; send the switch inputs if serial transmit buffer is empty

    movlp   high handleReceivedDataIfPresent
    call    handleReceivedDataIfPresent ; process received packet if available
    movlp   high mainLoop
    
    goto    mainLoop

; end of Main Code
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendDataIfReady
;
; Sends data when the serial transmit buffer is empty.
;

sendDataIfReady:

    banksel serialXmtBufNumBytes    ; when the xmt buffer is empty, send the switch states again
    movf    serialXmtBufNumBytes,W
    btfsc   STATUS,Z
    goto    sendSwitchStates

    return

; end of sendDataIfReady
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; trapSwitchInputs
;
; Checks each switch input and sets the associated flag for each if it is active.
;

trapSwitchInputs:


    banksel MODE_JOGUP_SEL_EPWR_P

    btfss   MODE_JOGUP_SEL_EPWR_P,MODE_SW
    bcf     switchStates,MODE_SW_FLAG

    call    trapJogUp

    call    trapJogDown

    btfss   MODE_JOGUP_SEL_EPWR_P,SELECT_SW
    bcf     switchStates,SELECT_SW_FLAG

    btfss   MODE_JOGUP_SEL_EPWR_P,ELECTRODE_PWR_SW
    bcf     switchStates,ELECTRODE_PWR_SW_FLAG

    return

; end of trapSwitchInputs
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; trapJogUp
;
; Checks Jog Up switch input and sets the associated flag if it is active.
; Uses a deglitch timer to ignore noise spikes.
;

trapJogUp:

    btfsc   MODE_JOGUP_SEL_EPWR_P,JOG_UP_SW     ; do nothing if switch not active
    return

    movlw   JOG_DEGLITCH_CNT0                   ; load deglitch counter
    movwf   deGlitchCntr0

tJU1:

    btfsc   MODE_JOGUP_SEL_EPWR_P,JOG_UP_SW     ; do nothing if switch not active
    return

    decfsz  deGlitchCntr0,F
    goto    tJU1

    btfss   MODE_JOGUP_SEL_EPWR_P,JOG_UP_SW
    bcf     switchStates,JOG_UP_SW_FLAG

    return

; end of trapJogUp
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; trapJogDown
;
; Checks Jog Down switch input and sets the associated flag if it is active.
; Uses a deglitch timer to ignore noise spikes.
;

trapJogDown:

    btfsc   JOGDWN_P,JOG_DOWN_SW                ; do nothing if switch not active
    return

    movlw   JOG_DEGLITCH_CNT0                   ; load deglitch counter
    movwf   deGlitchCntr0

tJD1:

    btfsc   JOGDWN_P,JOG_DOWN_SW                ; do nothing if switch not active
    return

    decfsz  deGlitchCntr0,F
    goto    tJD1

    btfss   JOGDWN_P,JOG_DOWN_SW
    bcf     switchStates,JOG_DOWN_SW_FLAG

    return

; end of trapJogDown
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setOutputs
;
; Sets outputs to the state specified in the serial receive buffer.
;
; On Entry:
;
;   FSR1 points to serialRcvBuf
; 

setOutputs:

    moviw   1[FSR1]                     ; get the output state value from the packet

    banksel outputStates
    movwf   outputStates                ; store the output states

    ; test each output flag and set the output pin correspondingly

    btfsc  outputStates,AC_OK_LED_FLAG
    goto    turnACOKLEDOff

    banksel INDICATORS_OUT_L
    bcf     INDICATORS_OUT_L,AC_OK_LED  ; turn on
    goto    soSkip1    

turnACOKLEDOff:

    banksel INDICATORS_OUT_L
    bsf     INDICATORS_OUT_L,AC_OK_LED  ; turn off

soSkip1:

    banksel outputStates
    btfsc   outputStates,BUZZER_FLAG
    goto    buzzerOff

    banksel INDICATORS_OUT_L
    bcf     INDICATORS_OUT_L,BUZZER     ; turn on
    goto    soSkip2

buzzerOff:

    banksel INDICATORS_OUT_L
    bsf     INDICATORS_OUT_L,BUZZER     ; turn off

soSkip2:

    banksel outputStates
    btfsc   outputStates,SHORT_LED_FLAG
    goto    turnShortLEDOff

    banksel INDICATORS_OUT_L
    bcf     INDICATORS_OUT_L,SHORT_LED     ; turn on
    goto    soExit

turnShortLEDOff:

    banksel INDICATORS_OUT_L
    bsf     INDICATORS_OUT_L,SHORT_LED     ; turn off

soExit:

    return

; end of setOutputs
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendSwitchStates
;
; Sends the switchStates variable via the serial port and sets switchStates to 0xff to ready it
; for trapping new input states.
;
; On Entry:
;
; On Exit:
;

sendSwitchStates:

    call    setupSwitchStatesPacket    
    
    banksel switchStates
    movf    switchStates,W
    
    call    writeByteToSerialXmtBuf    

    banksel switchStates
    movlw   0xff                        ; reset switchStates -> no inputs active
    movwf   switchStates
    
    call    startSerialPortTransmit

    return

; end of sendSwitchStates
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupSwitchStatesPacket
;
; Prepares the serial transmit buffer with header, length byte of 3, and command SWITCH_STATES_CMD.
; The data to be sent can then be added to the packet.
;
; Before the packet is transmitted, the length byte should be replaced with the actual number of
; bytes which have been added to the packet.
;
; On Entry:
;
; On Exit:
;
; packet is stuffed with header, length value of 3, and command byte
; FSR0 and serialXmtBufPtrH:serialXmtBufPtrL will point to the location for the next data byte

setupSwitchStatesPacket:

    movlw   SWITCH_STATES_CMD           ; packet command byte
    
    movlp   high setupSerialXmtPkt
    call    setupSerialXmtPkt
    movlp   high setupSwitchStatesPacket

    banksel flags

    return

; end of setupSwitchStatesPacket
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; parseCommandFromSerialPacket
;
; Parses the command byte in a serial packet and performs the appropriate action.
;
; On Entry:
;
; On Exit:
;
; FSR1 points to serialRcvBuf
;

parseCommandFromSerialPacket:

    movlw   SERIAL_RCV_BUF_LINEAR_LOC_H         ; point FSR0 at start of receive buffer
    movwf   FSR1H
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_L
    movwf   FSR1L

; parse the command byte by comparing with each command

    movf    INDF1, W
    sublw   SET_OUTPUTS_CMD
    btfsc   STATUS,Z
    goto    setOutputs

;    movf    INDF1, W
;    sublw   ???_CMD
;    btfsc   STATUS,Z
;    goto    handleSetPotRbtCmd

    return

; end of parseCommandFromSerialPacket
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupSerialXmtPkt
;
; Prepares the serial transmit buffer with header, a length byte of 3 and the command byte. The
; data to be sent can then be added to the packet.
;
; The default length of 3 is useful for many LCD commands which require 2 data bytes and a
; checksum. The length can be adjusted before transmission.
;
; On Entry:
;
; W contains the packet command.
;
; On Exit:
;
; packet is stuffed with header, command, and length value of 3.
; FSR0 and serialXmtBufPtrH:serialXmtBufPtrL will point to the location for the next data byte
;

setupSerialXmtPkt:

    banksel usartScratch0

    movwf   usartScratch1       ; store the command byte
    
    movlw   .3
    movwf   usartScratch0       ; default number of data bytes plus checksum byte
                                ;       can be changed before transmission

    goto    setUpSerialXmtBuf

; end of setupSerialXmtPkt
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; writeByteToSerialXmtBuf
;
; This subroutine writes the byte in W to the serial transmit buffer.
;
; On entry:
; 
; W contains byte to write
;
; NOTE: The data is placed in the buffer but is not submitted to be sent.  After using this
; function, call startSerialPortTransmit or printString to initiate transmission.
;

writeByteToSerialXmtBuf:

    banksel scratch2
    movwf   scratch2                        ; store character

    banksel serialXmtBufPtrH                ; load FSR0 with buffer pointer
    movf    serialXmtBufPtrH, W
    movwf   FSR0H
    movf    serialXmtBufPtrL, W
    movwf   FSR0L

    movf    scratch2,W                      ; retrieve character

    movwf   INDF0                           ; store character in buffer

    banksel serialXmtBufNumBytes            ; increment packet byte count
    incf    serialXmtBufNumBytes,f

    banksel serialXmtBufPtrH
    incf    serialXmtBufPtrL,F              ; point to next buffer position
    btfsc   STATUS,Z
    incf    serialXmtBufPtrH,F
    
    banksel flags

    return    
 
; end of writeByteToSerialXmtBuf
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
;--------------------------------------------------------------------------------------------------
;   EUSART Serial Port Core Functions
;
; Copy this block of code for all basic functions required for serial transmit. Build code, then
; copy all variables and defines which are shown to be missing.
;
;--------------------------------------------------------------------------------------------------
; setUpSerialXmtBuf
;
; Adds the header bytes, length byte, command byte, and various values from this Master PIC to the
; start of the serial port transmit buffer and sets serialXmtBufPtrH:L ready to add data bytes.
;
; Notes on packet length:
;
;   Example with 1 data bytes...
;
;   2 bytes (command byte + data byte)
;   ---
;   2 total (value passed to calcAndStoreCheckSumSerPrtXmtBuf; number bytes checksummed)
;
;   ADD (to determine length byte to insert into packet)
;
;   +1 checksum byte for the overall packet
;   3 total (value passed to setUpSerialXmtBuffer (this function) for packet length)
;
;   ADD (to determine actual number of bytes to send)
;
;   +2 header bytes
;   +1 length byte
;   ---
;   6 total (value passed to startSerialPortTransmit)
;
; On Entry:
;
; usartScratch0 should contain the number of data bytes plus one for the checksum byte in the packet
; usartScratch1 should contain the command byte
;
; On Exit:
;
; FSR0 and serialXmtBufPtrH:serialXmtBufPtrL will point to the location for the next data byte
; serialXmtBufNumBytes will be zeroed
;

setUpSerialXmtBuf:

    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H                   ; set FSR0 to start of transmit buffer
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   FSR0L

    banksel usartScratch0

    movlw   0xaa
    movwi   FSR0++                          ; store first header byte

    movlw   0x55
    movwi   FSR0++                          ; store first header byte

    movf    usartScratch0,W                 ; store length byte
    movwi   FSR0++

    movf    usartScratch1,W                 ; store command byte
    movwi   FSR0++

    banksel serialXmtBufPtrH                ; point serialXmtBufPtrH:L at next buffer position
    movf    FSR0H,W
    movwf   serialXmtBufPtrH
    movf    FSR0L,W
    movwf   serialXmtBufPtrL

    clrf   serialXmtBufNumBytes             ; tracks number of bytes added -- must be adjusted
                                            ; later to include the header bytes, length, command

    return

; end of setUpSerialXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleReceivedDataIfPresent
;
; Processes data in the serial receive buffer if a packet has been received.
;

handleReceivedDataIfPresent:

    banksel flags2                          ; handle packet in serial receive buffer if ready
    btfsc   flags2, SERIAL_PACKET_READY
    goto    handleSerialPacket

    return

; end of handleReceivedDataIfPresent
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; handleSerialPacket
;
; Processes a packet in the serial receive buffer.
;

handleSerialPacket:

    banksel serialRcvPktLen

    movf    serialRcvPktLen,W           ; store the packet length variable so the receive interrupt
    movwf   serialRcvPktLenMain         ; can overwrite it if a new packet arrives
        
    call    resetSerialPortRcvBuf       ; allow the serial receive interrupt to start a new packet
                                        ; see "Serial Data Timing" notes at the top of this page
    
    ;verify the checksum

    banksel serialRcvPktLenMain
  
    movf    serialRcvPktLenMain, W      ; copy number of bytes to variable for counting
    movwf   serialRcvPktCntMain

    movlw   SERIAL_RCV_BUF_LINEAR_LOC_H ; point FSR0 at start of receive buffer
    movwf   FSR0H
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_L
    movwf   FSR0L

    clrw                                ; preload W with zero

hspSumLoop:

    addwf   INDF0, W                    ; sum each data byte and the checksum byte at the end
    addfsr  FSR0,1
    decfsz  serialRcvPktCntMain, F
    goto    hspSumLoop

    movf    WREG, F                         ; test for zero
    btfsc   STATUS, Z                       ; error if not zero
    goto    parseCommandFromSerialPacket    ; checksum good so handle command

hspError:

    incf    serialPortErrorCnt, F           ; track errors
    bsf     statusFlags,SERIAL_COM_ERROR

    return

; end of handleSerialPacket
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; startSerialPortTransmit
;
; Initiates sending of the bytes in the transmit buffer. The transmission will be performed by an
; interrupt routine.
;
; The command byte and all following data bytes are used to compute the checksum which is inserted
; at the end.
;
; On Entry:
;
; serialXmtBufNumBytes should contain the number of bytes to send.
; The bytes to be sent should be in the serial port transmit buffer serialXmtBuf.
;

startSerialPortTransmit:

    ; get number of bytes stored in buffer, add one for the command byte, calculate checksum

    banksel serialXmtBufNumBytes
    movf    serialXmtBufNumBytes,W
    addlw   .1
    movwf   serialXmtBufNumBytes
    movwf   usartScratch0

    call    calcAndStoreCheckSumSerPrtXmtBuf

    banksel serialXmtBufPtrH                ; set FSR0 and pointer to start of transmit buffer
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H
    movwf   serialXmtBufPtrH
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   serialXmtBufPtrL
    movwf   FSR0L

    ; add 1 to length to account for checksum byte, store in packet length byte
    ;  packet length = command byte + data bytes + checksum

    banksel serialXmtBufNumBytes
    movf    serialXmtBufNumBytes,W
    addlw   .1
    movwi   2[FSR0]

    ; add 3 to length to account for two header bytes and length byte, store for xmt routine
    ; this is the total number of bytes to transmit

    addlw   .3
    movwf   serialXmtBufNumBytes

    banksel PIE1                            ; enable transmit interrupts
    bsf     PIE1, TXIE                      ; interrupt will trigger when transmit buffers empty

    return

; end of startSerialPortTransmit
;--------------------------------------------------------------------------------------------------
 
;--------------------------------------------------------------------------------------------------
; clearSerialPortXmtBuf
;
; Sets all bytes up to 255 in the Serial Port transmit buffer to zero. If the buffer is larger
; than 255 bytes, only the first 255 will be zeroed.
;

clearSerialPortXmtBuf:

    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H                   ; set FSR0 to start of transmit buffer
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   FSR0L

    banksel usartScratch0                       ; get buffer size to count number of bytes zeroed
    movlw   SERIAL_XMT_BUF_LEN
    movwf   usartScratch0

    movlw   0x00

cSPXBLoop:

    movwi   FSR0++
    decfsz  usartScratch0,F
    goto    cSPXBLoop

    return

; end of clearSerialPortXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupSerialPort
;
; Sets up the serial port for communication.
; Also prepares the receive and transmit buffers for use.
;

setupSerialPort:

    banksel serialRcvBufLen     ;store buffer length constants in variables for easier maths

    movlw   SERIAL_RCV_BUF_LEN
    movwf   serialRcvBufLen
    movlw   SERIAL_XMT_BUF_LEN
    movwf   serialXmtBufLen

    clrf    serialPortErrorCnt
    bcf     statusFlags,SERIAL_COM_ERROR

    ;to set the baud rate to 57,600 (will actually be 57.97K with 0.64% error)
    ;for Fosc of 16 Mhz: SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 68

    ;to set the baud rate to 19,200 (will actually be 19.23K with 0.16% error)
    ;for Fosc of 16 Mhz: SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 207

    ;to set the baud rate to 9,600 (will actually be 9592 with 0.08% error)
    ;for Fosc of 16 Mhz: SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 416 (0x1a0)
    
    ;to set the baud rate to 2,400 (will actually be 2399.5 with 0.02% error)
    ;for Fosc of 16 Mhz: SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 1666 (0x682)
    
    banksel TXSTA
    bsf     TXSTA, BRGH
    banksel BAUDCON
    bsf     BAUDCON, BRG16
    banksel SPBRGH
    movlw   0x01
    movwf   SPBRGH
    banksel SPBRGL
    movlw   0xa0
    movwf   SPBRGL

    ;set UART mode and enable receiver and transmitter

    banksel ANSELB          ; RB5/RB7 digital I/O for use as RX/TX
    bcf     ANSELB,RB5
    bcf     ANSELB,RB7

    banksel TRISB
    bsf     TRISB, TRISB5   ; set RB5/RX to input
    bcf     TRISB, TRISB7   ; set RB7/TX to output

    banksel TXSTA
    bcf     TXSTA, SYNC     ; clear bit for asynchronous mode
    bsf     TXSTA, TXEN     ; enable the transmitter
    bsf     RCSTA, CREN     ; enable the receiver
    bsf     RCSTA, SPEN     ; enable EUSART, configure TX/CK I/O pin as an output

    call    resetSerialPortRcvBuf
    call    resetSerialPortXmtBuf

    ; enable the receive interrupt; the transmit interrupt (PIE1/TXIE) is not enabled until data is
    ; ready to be sent
    ; for interrupts to occur, INTCON/PEIE and INTCON/GIE must be enabled also

    banksel PIE1
    bsf     PIE1, RCIE      ; enable receive interrupts
    bcf     PIE1, TXIE      ; disable transmit interrupts (re-enabled when data is ready to xmt)

    return

; end of setupSerialPort
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForTXIFHigh
;
; Waits in a loop for TXIF bit in register PIR1 to go high. This signals that the EUSART serial
; port transmit buffer is empty and a new byte can be sent.
;

waitForTXIFHigh:

    ifdef DEBUG_MODE  ; if debugging, don't wait for interrupt to be set high as the MSSP is not
    return            ; simulated by the IDE
    endif

    banksel PIR1

wfth1:
    btfss   PIR1, TXIF
    goto    wfth1

    return

; end of waitForTXIFHigh
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; resetSerialPortRcvBuf
;
; Resets all flags and variables associated with the serial port receive buffer.
;

resetSerialPortRcvBuf:

    banksel flags2

    clrf    serialRcvPktLen
    clrf    serialRcvPktCnt
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_H
    movwf   serialRcvBufPtrH
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_L
    movwf   serialRcvBufPtrL

    banksel RCSTA           ; check for overrun error - must be cleared to receive more data
    btfss   RCSTA, OERR
    goto    RSPRBnoOERRError

    bcf     RCSTA, CREN     ; clear error by disabling/enabling receiver
    bsf     RCSTA, CREN
        
RSPRBnoOERRError:

    banksel RCREG           ; clear any pending interrupt by clearing both bytes of the buffer
    movf    RCREG, W
    movf    RCREG, W
    
    banksel flags2

    bcf     flags2, HEADER_BYTE_1_RCVD
    bcf     flags2, HEADER_BYTE_2_RCVD
    bcf     flags2, LENGTH_BYTE_VALID
    bcf     flags2, SERIAL_PACKET_READY
    
    return

; end of resetSerialPortRcvBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; resetSerialPortXmtBuf
;
; Resets all flags and variables associated with the serial port transmit buffer.
;

resetSerialPortXmtBuf:

    banksel flags2

    clrf    serialXmtBufNumBytes
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H
    movwf   serialXmtBufPtrH
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   serialXmtBufPtrL

    return

; end of resetSerialPortXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; calcAndStoreCheckSumSerPrtXmtBuf
;
; Calculates the checksum for a series of bytes in the serial port transmit buffer. The two
; header bytes and the length byte are not included in the checksum.
;
; On Entry:
;
; usartScratch0 contains number of bytes in series, not including the 2 header bytes and 1 length
; byte
;
; On Exit:
;
; The checksum will be stored at the end of the series.
; FSR0 points to the location after the checksum.
;

calcAndStoreCheckSumSerPrtXmtBuf:

    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H                   ; set FSR0 to start of transmit buffer
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   FSR0L

    addfsr  FSR0,.3                             ; skip 2 header bytes and 1 length byte
                                                ; command byte is part of checksum

    goto    calculateAndStoreCheckSum

; end calcAndStoreCheckSumSerPrtXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; calculateAndStoreCheckSum
;
; Calculates the checksum for a series of bytes.
;
; On Entry:
;
; usartScratch0 contains number of bytes in series
; FSR0 points to first byte in series.
;
; On Exit:
;
; The checksum will be stored at the end of the series.
; FSR0 points to the location after the checksum.
;

calculateAndStoreCheckSum:

    call    sumSeries                       ; add all bytes in the buffer

    comf    WREG,W                          ; use two's complement to get checksum value
    addlw   .1

    movwi   FSR0++                          ; store the checksum at the end of the summed series

    return

; end calculateAndStoreCheckSum
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sumSeries
;
; Calculates the sum of a series of bytes. Only the least significant byte of the sum is retained.
;
; On Entry:
;
; usartScratch0 contains number of bytes in series.
; FSR0 points to first byte in series.
;
; On Exit:
;
; The least significant byte of the sum will be returned in WREG.
; Z flag will be set if the LSB of the sum is zero.
; FSR0 points to the location after the last byte summed.
;

sumSeries:

    banksel usartScratch0

    clrf    WREG

sumSLoop:                       ; sum the series

    addwf   INDF0,W
    addfsr  INDF0,1

    decfsz  usartScratch0,F
    goto    sumSLoop

    return

; end sumSeries
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleInterrupt
;
; All interrupts call this function.  The interrupt flags must be polled to determine which
; interrupts actually need servicing.
;
; Note that after each interrupt type is handled, the interrupt handler returns without checking
; for other types.  If another type has been set, then it will immediately force a new call
; to the interrupt handler so that it will be handled.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleInterrupt:

                                    ; INTCON is a core register, no need to banksel
	btfsc 	INTCON, T0IF     		; Timer0 overflow interrupt?
	call 	handleTimer0Int         ; call so the serial port interrupts will get checked
                                    ;  if not, the timer interrupt can block them totally

    banksel PIR1
    btfsc   PIR1, RCIF              ; serial port receive interrupt
    goto    handleSerialPortReceiveInt

    banksel PIE1                    ; only handle UART xmt interrupt if enabled
    btfss   PIE1, TXIE              ;  the TXIF flag is always set whenever the buffer is empty
    retfie                          ;  and should be ignored unless the interrupt is enabled
    
    banksel PIR1
    btfsc   PIR1, TXIF              ; serial port transmit interrupt
    goto    handleSerialPortTransmitInt


; Not used at this time to make interrupt handler as small as possible.
;	btfsc 	INTCON, RBIF      		; NO, Change on PORTB interrupt?
;	goto 	portB_interrupt       	; YES, Do PortB Change thing

INT_ERROR_LP1:		        		; NO, do error recovery
	;GOTO INT_ERROR_LP1      		; This is the trap if you enter the ISR
                               		; but there were no expected interrupts

endISR:

	retfie                  	; Return and enable interrupts

; end of handleInterrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleTimer0Int
;
; This function is called when the Timer0 register overflows.
;
; TMR0 is never reloaded -- thus it wraps around and does a full count for each interrupt.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleTimer0Int:

	bcf 	INTCON,T0IF     ; clear the Timer0 overflow interrupt flag

    ; do stuff here
    
    return

; end of handleTimer0Int
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPortReceiveInt
;
; This function is called when a byte(s) has been received by the serial port. The byte(s) will be
; checked to see if it is a header byte, a packet length byte, or a data byte. Data bytes will be
; stored in a buffer. If an error occurs in receiving a packet, the function will ignore data
; received before the error and begin watching for the next packet signature. Upon receiving a
; complete packet, a flag will be set to notify the main loop.
;
; The receive register is a two byte fifo, so two bytes could be ready. This function will process
; all bytes available.
;
; The RCIF flag is cleared by reading all data from the two byte receive FIFO.
;
; This code check each byte sequence to see if it starts with a header prefix (0xaa,0x55) followed
; by a valid length byte. If these are found, the bytes after the length byte are stored in a
; buffer. If the sequence is not matched or the supposed length byte is larger than the buffer,
; all flags are reset and the search for the first header byte starts over.
;
; Packet format:
;   0xaa, 0x55, length, data1, data2, data3,...checksum.
;
; This interrupt function does not verify the checksum; the main loop should do that if required.
; Once a packet has been received, a flag is set to alert the main loop that it is ready for
; processing. All further data will be ignored until the main loop clears that flag. If an error
; occurs, the data received to that point will be discarded and the search for the next packet
; begun anew.
;
; The packet length byte is the number of data bytes plus one for the checksum byte. It does not
; include the two header bytes or the length byte itself. If the length byte value is 0 or is
; greater than the buffer size, the packet will be ignored. If the length byte value is greater
; than the actual number of bytes sent (but still less than the buffer size), the current packet
; AND the next packet(s) will be discarded as the interrupt routine will wait until enough bytes
; are received from subsequent packets to equal the erroneously large length byte value.
;
; Thus, only one packet at a time can be handled. The processing required is typically minimal, so
; the main loop should be able to process each packet before another is received. Some care should
; be taken by the receiver to not flood the line with packets.
;
; The main loop does all the actual processing in order to minimize the overhead of the interrupt.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleSerialPortReceiveInt:

    ; if the packet ready flag is set, ignore all data until main loop clears it

    banksel flags2
    btfss   flags2, SERIAL_PACKET_READY
    goto    readSerialLoop

    ; packet ready flag set means last packet still being processed, read byte to clear interrupt
    ; or it will result in an endless interrupt loop, byte is tossed and a resync will occur

    banksel RCREG
    movf    RCREG, W
    goto    rslExit

    ;RCREG is a two byte FIFO and may contain two bytes; read until RCIF flag is clear

readSerialLoop:

    banksel RCREG
    movf    RCREG, W        ; get byte from receive fifo

    banksel flags2

    btfsc   flags2, HEADER_BYTE_1_RCVD      ; header byte 1 already received?
    goto    rsl1                            ; if so, check for header byte 2

    bsf     flags2, HEADER_BYTE_1_RCVD      ; preset the flag, will be cleared on fail

    sublw   0xaa                            ; check for first header byte of 0xaa
    btfsc   STATUS, Z                       ; equal?
    goto    rsllp                           ; continue on, leaving flag set

    goto    rslError                        ; not header byte 1, reset all to restart search

rsl1:
    btfsc   flags2, HEADER_BYTE_2_RCVD      ; header byte 2 already received?
    goto    rsl2                            ; if so, check for length byte

    bsf     flags2, HEADER_BYTE_2_RCVD      ; preset the flag, will be cleared on fail

    sublw   0x55                            ; check for second header byte of 0x55
    btfsc   STATUS, Z                       ; equal?
    goto    rsllp                           ; continue on, leaving flag set

    goto    rslError                        ; not header byte 2, reset all to restart search

rsl2:
    btfsc   flags2, LENGTH_BYTE_VALID       ; packet length byte already received and validated?
    goto    rsl3                            ; if so, jump to store data byte

    movwf   serialRcvPktLen                 ; store the packet length
    movwf   serialRcvPktCnt                 ; store it again to count down number of bytes stored

    bsf     flags2, LENGTH_BYTE_VALID       ; preset the flag, will be cleared on fail

    movf    serialRcvPktLen, F              ; check for invalid packet size of 0
    btfsc   STATUS, Z
    goto    rslError

    subwf   serialRcvBufLen, W              ; check if packet length < buffer length
    btfsc   STATUS, C                       ; carry cleared if borrow was required
    goto    rsllp                           ; continue on, leaving flag set
                                            ; if invalid length, reset all to restart search

rslError:

    incf    serialPortErrorCnt, F           ; track errors
    bsf     statusFlags,SERIAL_COM_ERROR
    call    resetSerialPortRcvBuf    
    goto    rsllp

rsl3:

    movwf   serialIntScratch0               ; store the new character

    movf    serialRcvBufPtrH, W             ; load FSR0 with buffer pointer
    movwf   FSR0H
    movf    serialRcvBufPtrL, W
    movwf   FSR0L

    movf    serialIntScratch0, W            ; retrieve the new character
    movwi   INDF0++                         ; store in buffer

    movf    FSR0H, W                        ; save adjusted pointer
    movwf   serialRcvBufPtrH
    movf    FSR0L, W
    movwf   serialRcvBufPtrL

    decfsz  serialRcvPktCnt, F              ; count down number of bytes stored
    goto    rsllp                           ; continue collecting until counter reaches 0

rsl4:

    bsf     flags2, SERIAL_PACKET_READY     ; flag main loop that a data packet is ready
    goto    rslExit

rsllp:

    banksel PIR1                            ; loop until receive fifo is empty
    btfsc   PIR1, RCIF
    goto    readSerialLoop

rslExit:

    banksel RCSTA           ; check for overrun error - must be cleared to receive more data
    btfss   RCSTA, OERR
    goto    noOERRError

    bcf     RCSTA, CREN     ; clear error by disabling/enabling receiver
    bsf     RCSTA, CREN

noOERRError:

    goto    endISR

; end of handleSerialPortReceiveInt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPortTransmitInt
;
; This function is called when a byte is to be transmitted to the host via serial port. After
; data is placed in the transmit buffer, the TXIE flag is enabled so this routine gets called
; as an interrupt whenever the transmit buffer is empty. After all bytes in the buffer have been
; transmitted, this routine clears the TXIE flag to disable further interrupts.
;
; Before the TXIE flag is set to start the process, serialXmtBufNumBytes should be set to value
; > 0, i.e. the number of valid bytes in the transmit buffer.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;
; The TXIF flag is cleared in the second instruction cycle after writing data to TXREG.
;

handleSerialPortTransmitInt:

    banksel serialXmtBufPtrH                ; load FSR0 with buffer pointer
    movf    serialXmtBufPtrH, W
    movwf   FSR0H
    movf    serialXmtBufPtrL, W
    movwf   FSR0L

    moviw   FSR0++                          ; send next byte in buffer
    banksel TXREG
    movwf   TXREG

    banksel serialXmtBufPtrH                ; store updated FSR0 in buffer pointer
    movf    FSR0H, W
    movwf   serialXmtBufPtrH
    movf    FSR0L, W
    movwf   serialXmtBufPtrL

    decfsz  serialXmtBufNumBytes, F
    goto    endISR                          ; more data to send, exit with interrupt still enabled

    banksel PIE1                            ; no more data, disable further transmit interrupts
    bcf     PIE1, TXIE

    goto    endISR

; end of handleSerialPortTransmitInt
;--------------------------------------------------------------------------------------------------
;
;   End of EUSART Serial Port Core Functions
;
;--------------------------------------------------------------------------------------------------
;--------------------------------------------------------------------------------------------------

    END
