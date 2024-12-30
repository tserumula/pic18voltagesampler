/*
 * File:   main.c
 * Author: thabang
 *
 */

// PIC18F14K50 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection bits (No CPU System Clock divide)
#pragma config USBDIV = OFF     // USB Clock Selection bit (USB clock comes directly from the OSC1/OSC2 oscillator block; no divide)

// CONFIG1H
#pragma config FOSC = IRC       // Oscillator Selection bits (Internal RC oscillator)
#pragma config PLLEN = OFF      // 4 X PLL Enable bit (PLL is under software control)
#pragma config PCLKEN = ON      // Primary Clock Enable bit (Primary clock enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 19        // Brown-out Reset Voltage bits (VBOR set to 1.9 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up bit (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RA3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = OFF      // Boot Block Size Select bit (1kW boot block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF       // Table Write Protection bit (Block 1 not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic18f14k50.h>
#include <stdio.h>

#define _XTAL_FREQ 2000000  // 2 MHz
#define EEPROM_ADDRESS 0b10100000 //0xA0
#define TIMER_OVERFLOW_POSTSCALER 57 //Results in period of 59.76 seconds

#define SHIFT_CLK PORTCbits.RC6   // Shift Register clock (SRCLK)
#define SHIFT_LATCH PORTCbits.RC7   // Latch Pin Clock (RCLK)

#define SHIFT0_DATA PORTCbits.RC5   // Data pin (SER)
#define SHIFT1_DATA PORTCbits.RC4   // Data pin (SER)
#define SHIFT2_DATA PORTCbits.RC3   // Data pin (SER)

#define EEPROM_SEPARATOR_CHAR 254 

//_____ Global variables ______

uint16_t adcResult = 0;

uint8_t overflowCount = 0;
uint8_t sampling = 0;

uint8_t eeprom_data_lsb = 0;
uint8_t eeprom_data_msb = 0;
uint8_t eeprom_data_ctl = 0;
uint16_t eeprom_data = 0;

uint8_t received_data = 0;

uint8_t address_low_byte = 0; //LSB
uint8_t address_high_byte = 0; //MSB
uint16_t address_index = 0; //2 Byte address

uint8_t segPattern0;
uint8_t segPattern1;
uint8_t segPattern2;

uint8_t displayDigit0;
uint8_t displayDigit1;
uint8_t displayDigit2;

uint8_t turnOnDisplay = 0;
uint8_t displayAddress = 0;
uint8_t displaySample = 0;

uint8_t fsm_state = 0;

double adc_step_size = 0.0048875855; //vref/2^n -1  , vref = 5
double adc_voltage = 0;

unsigned int c_target = 0; 
unsigned int c_remainder = 0;
unsigned int c_number = 0;

char uart_buffer[10]; 
unsigned int button_counter = 0;

//________


uint8_t getLowByte(uint16_t number){ // Extract the lower 8 bits
    return ( number & 0xFF);  
}

uint8_t getHighByte(uint16_t number){ // Extract the upper 8 bits
    return ((number >> 8) & 0xFF); 
}

void adcSampling(){

    sampling = 1;
    
    // Start ADC Conversion       
    ADCON0bits.GO = 1;  // Start ADC conversion
        
    while (ADCON0bits.GO);  // Wait for conversion to complete
        
    adcResult = ((uint16_t)ADRESH << 8) | ADRESL;  

}


// ______ Function prototypes ___

void setupADC(void);
void setupTimer1(void);
void setupI2C(void);

void setButtonInterrupt(void);
uint8_t get7SegmentPattern(uint8_t);
void displayDigits( uint8_t, uint8_t, uint8_t );
void show7Segment(void);
void setDisplayDigits(void);

void I2CStart(void);
void I2CStop(void);
void I2CRestart(void);
void I2CWrite( uint8_t );
uint8_t I2CRead(uint8_t );
void EEPROMWrite(uint16_t , uint8_t );
uint8_t EEPROMRead(uint16_t );

void setupEUSART(void);
void EUSART_TransmitChar(char);
void EUSART_TransmitString( char *);

void __interrupt() ISR(void) {

    if ( PIR1bits.TMR1IF) {  // Timer1 interrupt flag 
        PIR1bits.TMR1IF = 0;  // Clear Timer1 interrupt flag
        
        TMR1 = 0; 

        if (fsm_state == 1 || fsm_state == 2 ) {

            overflowCount++;

            if (overflowCount >= TIMER_OVERFLOW_POSTSCALER) { // Number of overflows for 60 seconds
                overflowCount = 0;

                // Start ADC Conversion            
                adcSampling();

            }
        }        
        else if( fsm_state == 7 ){ //Clear memory        
            LATAbits.LA5 ^= 1; //flash                    
        }        
    }
    
    
    if ( INTCON3bits.INT1IF || INTCON3bits.INT2IF ) { //Button External Interrupt
        
        button_counter++;                

        if( INTCON3bits.INT2IF && button_counter > 800 ){ //Button B

            INTCON3bits.INT2IF = 0;    
            
            displayAddress = 1;
            //displaySample = 0;
            displaySample = 1;  
            button_counter = 0;
        
            switch(fsm_state){
                case 0 : { //Init state
                    fsm_state = 1; //Move to Sampling Task
                    address_index = 0; //start from first index
                    if( turnOnDisplay == 0 ){
                        adcSampling(); //Take initial sample
                    }
                    break;
                }
                case 1 : { //Sampling task
                    fsm_state = 2; //Sample send
                    address_index = 0; //start from first index
                    break;
                }
                case 2 : { //Sample Send
                    fsm_state = 3; //Move to Copy data
                    address_index = 0; //start from first index
                    EUSART_TransmitString("START \r\n");
                    break;
                }
                case 3 : { //Copy data
                    fsm_state = 4; //Move to standby
                    break;
                }
                case 4 : { //Stand By
                    fsm_state = 0; //Move to Init state
                    break;
                }
                case 5 : { //Stand By 1
                    fsm_state = 0; //Move to Init state
                    break;
                }                
                case 6 : { //Stand By 2
                    fsm_state = 0; //Move to Init state
                    break;
                }   
                case 7 : { //CLR
                    fsm_state = 0; //Move to Init state
                    break;
                }        
                default : break;
            }            
            
        }else if( INTCON3bits.INT1IF && button_counter > 800 ){ //Button A            
            
            INTCON3bits.INT1IF = 0; 
            
            displayAddress = 0;
            displaySample = 1;    
            button_counter = 0;
            
            
            switch(fsm_state){            
                case 4 : { //Stand By 0
                    fsm_state = 5; //Move to StandBy 1
                    break;
                }
                case 5 : { //Stand By 1
                    fsm_state = 6; //Move to StandBy 2
                    break;
                }                  
                case 6 : { //StandBy 2
                    fsm_state = 7; //Move to Clear Memory
                    break;
                }             
                
                default : break;
            }            
        }
        
        if( turnOnDisplay == 1 && button_counter < 10 ){ //User pressed while display is ON       
            setDisplayDigits();
            show7Segment(); //update displayed content        
        }else{
            turnOnDisplay = 1;
        }
    }   
}


void main(void) {
   
    //Set main Clock (Internal oscillator block))
    OSCCONbits.SCS0 = 1;
    OSCCONbits.SCS1 = 1;

    OSCCONbits.OSTS = 0; //Device is running from internal oscillator
    
    //Set internal clock frequency to 2MHz
    OSCCONbits.IRCF0 = 0;
    OSCCONbits.IRCF1 = 0;
    OSCCONbits.IRCF2 = 1;

    __delay_ms(500);
    
    //SET UP ADC
    ANSEL = 0b00001000;   // Set RA4 as analog input (AN3)
    TRISA = 0b00010000;   // Set RA4 as input
    
    TRISCbits.RC0 = 0;
    ANSELbits.ANS4 = 0; // RC0 is used as digital output, disable Analog
    
    setupADC();
    
    __delay_us(100);
    
    //Set up Timer 1 for 1 second delay
    setupTimer1();
    
    setupI2C(); // Initialize I2C module
            
    //Set RC5, RC4, RC3, RC6, RC7 as output pins [ Display Shift Register pins]
     
    TRISCbits.TRISC5 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 0;
    
    LATCbits.LATC5 = 0;
    LATCbits.LATC4 = 0;
    LATCbits.LATC3 = 0;
    LATCbits.LATC6 = 0;
    LATCbits.LATC7 = 0; 
    
    //Setup button interrupt
    TRISCbits.RC1 = 1; // input
    TRISCbits.RC2 = 1; // input
    setButtonInterrupt();
    
    //Setup UART peripheral
    setupEUSART();
    
    INTCONbits.PEIE = 1;  // Enable peripheral interrupts
    INTCONbits.GIE = 1;   //Enable Global interrupts 
           
    adcSampling(); //Take first sample
            
    while(1){       
        
        if( sampling == 1 && fsm_state == 1 ){
            
            LATAbits.LA5 = 1;
            
            adc_voltage = adc_step_size * adcResult;            
            
            //Save data to EEPROM next address
            
            address_index++;
            EEPROMWrite(address_index, getLowByte( adcResult) );
            address_index++;
            EEPROMWrite(address_index, getHighByte( adcResult) );
            address_index++;
            EEPROMWrite(address_index, EEPROM_SEPARATOR_CHAR );                                                 
            
            if( address_index >= 7900 ){ //8K bytes EEPROM
                //Don't write data beyond 7200 address number
                //Indicate EEPROM memory full?
                //stop timer 
                
                fsm_state = 0;               
                address_index = 0;            
            }
            
            __delay_ms(1000);
            
            sampling = 0;
            LATAbits.LA5 = 0;
            
        }else if( sampling == 1 && fsm_state == 2 ){ //Sample Send
            LATAbits.LA5 = 1;            
            adc_voltage = adc_step_size * adcResult;              
          
            
            // Convert double to string
            sprintf(uart_buffer, "%.2f", adc_voltage);

            EUSART_TransmitString(uart_buffer);
            EUSART_TransmitString("\r\n");
            
            __delay_ms(1000);
            
            sampling = 0;
            LATAbits.LA5 = 0;
            
        }            
        
        if( fsm_state == 3 ){ //Copy Data               
            
            LATAbits.LA5 = 1; //Keep LED ON while copying data            
                        
            if( address_index == 0){
                EUSART_TransmitString("START \r\n");
                address_index++;
                
            }else if(address_index  >= 7900 ){
                EUSART_TransmitString("END \r\n");               
                address_index = 0; //cycle back
            
            }else {
                eeprom_data_lsb = EEPROMRead(address_index);
                address_index++;
                eeprom_data_msb = EEPROMRead(address_index);
                address_index++;
                eeprom_data_ctl = EEPROMRead(address_index);
                address_index++;
                
                if( eeprom_data_ctl == EEPROM_SEPARATOR_CHAR ){ //Data valid
                
                    // Combine high and low bits into a 16-bit integer
                    eeprom_data = ((uint16_t) eeprom_data_msb << 8) | eeprom_data_lsb;

                    adc_voltage = adc_step_size * eeprom_data;

                    // Convert double to string
                    sprintf(uart_buffer, "%.2f", adc_voltage);

                    EUSART_TransmitString(uart_buffer);
                    EUSART_TransmitString("\r\n");
                }else{
                    address_index = 7900; // will cycle 
                }                            
            }        
        }else{
            
            if( fsm_state < 7){
                LATAbits.LA5 = 0; //Keep off
            }        
        }        
        
        if( turnOnDisplay == 1 ){     
        
            PORTCbits.RC0 = 0; //Enable Shift registers latch output                                      
            
            setDisplayDigits();            
            show7Segment();
                   
            __delay_ms(10000); //10 seconds
           
            turnOnDisplay = 0;           
      
        }else{
         
            PORTCbits.RC0 = 1; //Disable Shift registers latch output            
        
             __delay_ms(1000);      
        }        
    }
    
    return;
}


void setupADC() {
    ADCON1 = 0b00000000;  // Set reference voltages to VDD and VSS
    ADCON0 = 0b00001101;  // Select AN3 (RA4), enable ADC module
    ADCON2 = 0b10101111; // FRC Clock source (600KHz), 12 TAD, ADFM = 1 (Right Justified)
}

void setupTimer1(void) {
    T1CON = 0b00110101;   // Timer1 ON, pre-scaler 1:8
    TMR1 = 0;
    
    // Clear Timer1 register
    PIR1bits.TMR1IF = 0;           // Clear Timer1 interrupt flag
    PIE1bits.TMR1IE = 1;           // Enable Timer1 interrupt

    // Timer1 period equates to 1.048576 seconds:
    // Timer1 clock = FOSC/4 = 2M/4
    // Timer1 Prescaler = 1/8
    
    TMR1 = 0;
}

void setupI2C() {
    TRISBbits.RB4 = 1;
    TRISBbits.RB6 = 1;
    SSPCON1 = 0b00101000; //I2C Master Mode, Serial Port on, no overflow, no collision detect
    SSPADD = ((_XTAL_FREQ / 4) / 100000) - 1;  // Set clock speed to 100kHz
    SSPSTAT = 0x00;  // Slew rate disabled for standard speed mode
    
    SSPCON2 = 0b00000000;
}


void setButtonInterrupt(){
    
    INTCON2bits.INTEDG1 = 1; // External Interrupt 1 on rising edge
    INTCON2bits.INTEDG2 = 1; // External Interrupt 2 on rising edge
    
    INTCON3bits.INT1IE = 1; //Enable INT1 Interrupts
    INTCON3bits.INT2IE = 1; //Enable INT2 Interrupts
    
    INTCON3bits.INT1IP = 0; //Low priority
    INTCON3bits.INT2IP = 0; //Low priority
    
    
    INTCON3bits.INT1IF = 0; // Clear external Interrupt flag
    INTCON3bits.INT2IF = 0; // Clear external Interrupt flag
            
}

uint8_t get7SegmentPattern(uint8_t digit) {
    //Bit pattern for common-cathode 7 segment display
    
    uint8_t pattern;
    
    switch (digit) {
        case 0: pattern = 0b00111111; break; // 0: a, b, c, d, e, f
        case 1: pattern = 0b00000110; break; // 1: b, c
        case 2: pattern = 0b01011011; break; // 2: a, b, d, e, g
        case 3: pattern = 0b01001111; break; // 3: a, b, c, d, g
        case 4: pattern = 0b01100110; break; // 4: b, c, f, g
        case 5: pattern = 0b01101101; break; // 5: a, c, d, f, g
        case 6: pattern = 0b01111101; break; // 6: a, c, d, e, f, g
        case 7: pattern = 0b00000111; break; // 7: a, b, c
        case 8: pattern = 0b01111111; break; // 8: a, b, c, d, e, f, g
        case 9: pattern = 0b01101111; break; // 9: a, b, c, d, f, g
        case 89: pattern = 0b01111001; break; // error code 
        case 'A': pattern = 0b01110111; break;
        case 'C': pattern = 0b00111001; break;
        case 'S': pattern = 0b01101101; break;
        case 'L': pattern = 0b00111000; break;
        case 'r': pattern = 0b00110001; break;
        default: pattern = 0b00000000; break; // Turn off all segments if invalid digit
    }
    
    return pattern;
}

void displayDigits( uint8_t digit0, uint8_t digit1, uint8_t digit2 ){
    int i;
    for ( i = 0 ; i < 8 ; i++ ){
        
        SHIFT0_DATA = (digit0 >> i) & (0x01);
        SHIFT1_DATA = (digit1 >> i) & (0x01);
        SHIFT2_DATA = (digit2 >> i) & (0x01);
        
        //Pulse the Shift register clock pin 
        SHIFT_CLK = 1; 
        __delay_us(500);
        SHIFT_CLK = 0;
        __delay_us(500);
    }
    
    //All bits shifted in. Now we pulse the Latch pin
    SHIFT_LATCH = 1;
    __delay_us(500);
    SHIFT_LATCH = 0;
    
}

void I2CStart() {
    SSPCON2bits.SEN = 1;   // Initiate Start condition (start pulse))
    while (SSPCON2bits.SEN); // Wait for start to complete
}

void I2CStop() {
    SSPCON2bits.PEN = 1;   // Initiate Stop condition
    while (SSPCON2bits.PEN); // Wait for stop to complete
}

void I2CRestart() {
    SSPCON2bits.RSEN = 1;  // Initiate Restart condition
    while (SSPCON2bits.RSEN); // Wait for restart to complete
}

void I2CWrite( uint8_t data) {
    SSPBUF = data;         // Write data to buffer
    while (SSPSTATbits.BF); // Wait until transmission is complete
    while (SSPCON2bits.ACKSTAT); // Wait for ACK from the slave
}

uint8_t I2CRead(uint8_t ack) {
    SSPCON2bits.RCEN = 1;  // Enable receive mode
    while (!SSPSTATbits.BF); // Wait for data reception1`   
    received_data = SSPBUF; // Read received data
    
    SSPCON2bits.ACKDT = ack;  // Acknowledge or Not Acknowledge
    SSPCON2bits.ACKEN = 1;    // Send Acknowledge bit
    while (SSPCON2bits.ACKEN); // Wait for ACK/NACK to complete
    return received_data;
}

void EEPROMWrite(uint16_t address, uint8_t data) {
    I2CStart();
    I2CWrite(EEPROM_ADDRESS);  // Send EEPROM device address + Write
    
    address_low_byte = getLowByte(address);
    address_high_byte = getHighByte(address);
    
    I2CWrite(address_low_byte);         // Send memory address
    I2CWrite(address_high_byte);         // 2ND word byte
    I2CWrite(data);            // Send data to be written
    I2CStop();
    
    __delay_ms(5);              // EEPROM write cycle time
}

uint8_t EEPROMRead(uint16_t address ) {
    uint8_t data;
    
    address_low_byte = getLowByte(address);
    address_high_byte = getHighByte(address);
    
    I2CStart();
    I2CWrite(EEPROM_ADDRESS);  // Send EEPROM device address + Write
    I2CWrite(address_low_byte); // Send memory address
    I2CWrite(address_high_byte); // 2ND word byte
    I2CRestart();
    I2CWrite(EEPROM_ADDRESS | 1); // Send EEPROM device address + Read [0b10100000| 0b00000001 = 0b10100001 ]
    data = I2CRead(1);         // Read data with NACK
    I2CStop();
    return data;
}

void setupEUSART() {
    //UART Mode = 8N1, asynchronous
    //Desired baud rate = 2400

    BAUDCONbits.BRG16 = 0; //Use 8-bit mode
    BAUDCONbits.ABDEN = 0; //Auto-baud detect is off
    BAUDCONbits.CKTXP = 0;  //Polarity - High to Low
    BAUDCONbits.ABDOVF = 0; 
    
    
    SPBRG = 12; // (_XTAL_FREQ / (64 * baud_rate)) - 1
   
    TXSTAbits.BRGH = 0;  // Low-speed baud rate

    TXSTAbits.SYNC = 0;  // Asynchronous mode
    TXSTAbits.TX9 = 0; // 8-bit transmit mode
    RCSTAbits.SPEN = 1;  // Enable serial port
    TXSTAbits.TXEN = 1;  // Enable transmitter
    RCSTAbits.CREN = 0;  // Disable receiver

    TXSTAbits.SENDB = 0;
    
    // Configure TX and RX pins as digital
    TRISBbits.TRISB7 = 1; // TX pin
    TRISBbits.TRISB5 = 1; // RX pin
}


void EUSART_TransmitChar(char data) {
    while (!TXSTAbits.TRMT); // Wait until the transmit buffer is empty
    TXREG = data; 
}

void EUSART_TransmitString( char *a){
    int i;
    for( i= 0; a[i] !='\0'; i++ ){
       EUSART_TransmitChar(a[i]);
    }
}

void setDisplayDigits() {
    if (fsm_state == 0) { //Init state

        displayDigit0 = 11; //Will turn off
        displayDigit1 = 'A';
        displayDigit2 = 11;

    } else if (fsm_state == 1) { //Sampling state

        c_target = (unsigned int) (adc_voltage * 100);
        c_remainder = c_target % 100;
        c_number = c_target / 100;

        displayDigit0 = (uint8_t) c_number;
        displayDigit1 = (uint8_t) (c_remainder / 10);
        displayDigit2 = c_remainder % 10;

    } else if (fsm_state == 2) { //Sample Send

        c_target = (unsigned int) (adc_voltage * 100);
        c_remainder = c_target % 100;
        c_number = c_target / 100;

        displayDigit0 = (uint8_t) c_number;
        displayDigit1 = (uint8_t) (c_remainder / 10);
        displayDigit2 = 'C';

    } else if (fsm_state == 3) { //Copy data                

        displayDigit0 = 11;
        displayDigit1 = 'C';
        displayDigit2 = 11;

    } else if (fsm_state == 4) { //Standby 1

        displayDigit0 = 'S';
        displayDigit1 = 0;
        displayDigit2 = 11;

    } else if (fsm_state == 5) { //Standby 1

        displayDigit0 = 'S';
        displayDigit1 = 1;
        displayDigit2 = 11;

    } else if (fsm_state == 6) { //Standby 2

        displayDigit0 = 'S';
        displayDigit1 = 2;
        displayDigit2 = 11;

    } else if (fsm_state == 7) { //Clear Memory

        displayDigit0 = 'C';
        displayDigit1 = 'L';
        displayDigit2 = 'r';

    }
    else {
        NOP();
    }
}

void show7Segment(){

    segPattern0 = get7SegmentPattern( displayDigit0 ); // 7 segment bit pattern 
            
    if( displaySample == 1 && ( fsm_state == 1 || fsm_state == 2 ) ){
        segPattern0 = segPattern0 | 0b10000000; // Add decimal point
    }
            
    segPattern1 = get7SegmentPattern( displayDigit1 ); // 7 segment bit pattern
    segPattern2 = get7SegmentPattern( displayDigit2 ); // 7 segment bit pattern
      
    displayDigits( segPattern0, segPattern1, segPattern2 );
}
