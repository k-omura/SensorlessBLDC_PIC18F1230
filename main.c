/*
 * Created on August 28, 2017
 * August 30: ADC/PWM test
 * September 10: Open-loop
 * September 12: Closed-loop
 * 
 * MPLAB X(XC8)
 * 
 * PWM0: U Nch (Low side)
 * PWM1: U Pch (High side)
 * PWM2: V Nch (Low side)
 * PWM3: V Pch (High side)
 * PWM4: W Nch (Low side)
 * PWM5: W Pch (High side)
 * 
 * ADC0: U
 * ADC1: V
 * ADC2: Supply voltage for motor drive (Vref source -> Vref+ pin)
 * ADC3: W
 */

// PIC18F1230 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = INTIO2     // Oscillator (Internal oscillator, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3L
#pragma config PWMPIN = OFF     // PWM Output Pins Reset State Control bit (PWM outputs disabled upon Reset)
#pragma config LPOL = HIGH      // Low-Side Transistors Polarity bit (Even PWM Output Polarity Control bit) (PWM0, PWM2 and PWM4 are active-high (default))
#pragma config HPOL = HIGH      // High Side Transistors Polarity bit (Odd PWM Output Polarity Control bit) (PWM1, PWM3 and PWM5 are active-high (default))

// CONFIG3H
#pragma config FLTAMX = RA5     // FLTA Mux bit (FLTA input is muxed onto RA5)
#pragma config T1OSCMX = LOW    // T1OSO/T1CKI MUX bit (T1OSO/T1CKI pin resides on RB2)
#pragma config MCLRE = OFF      // Master Clear Enable bit (RA5 input pin enabled, MCLR pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable bit (Reset on stack overflow/underflow disabled)
#pragma config BBSIZ = BB256    // Boot Block Size Select bits (256 Words (512 Bytes) Boot Block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit Block 0 (000400-0007FF) (Block 0 is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit Block 1 (000800-000FFF) (Block 1 is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Code Protection bit (Boot Block Memory Area) (Boot Block is not code-protected)
#pragma config CPD = OFF        // Code Protection bit (Data EEPROM) (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit Block 0 (000400-0007FF) (Block 0 is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit Block 1 (000800-000FFF) (Block 1 is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Write Protection bit (Configuration Registers) (Configuration registers are not write-protected)
#pragma config WRTB = OFF       // Write Protection bit (Boot Block Memory Area) (Boot Block is not write-protected)
#pragma config WRTD = OFF       // Write Protection bit (Data EEPROM) (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit Block 0 (000400-0007FF) (Block 0 is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit Block 1 (000800-000FFF) (Block 1 is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protection bit (Boot Block Memory Area) (Boot Block is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 32000000
#define motorADCThreshold 0x80 // (Vmotor / 2) Adjust the value by looking at the oscilloscope, if necessary.

//functions
void BLDCPosition(int);
void setDuty(unsigned int, unsigned int);
void nextState(unsigned char);

//var
unsigned char ADCPortNum = 0;
unsigned char ADCON[3] = {0};
unsigned char BEMFValue;
unsigned char motorPosition;
unsigned char direction;
unsigned char CLEnable;
unsigned char reachO2CSpeed = 0;

//const
const unsigned char ADPortCHS[3] = {0b00, 0b01, 0b11};
const unsigned char feedBack[2][6] = {
    {
        0b00000110,
        0b00000100,
        0b00000101,
        0b00000001,
        0b00000011,
        0b00000010
    },
    {
        0b00000010,
        0b00000110,
        0b00000100,
        0b00000101,
        0b00000001,
        0b00000011
    }
};

void interrupt isr() {
    if (PIR1bits.ADIF) {
        ADCON[ADCPortNum] = ADRESH;

        ADCPortNum = (ADCPortNum >= 2) ? 0 : ADCPortNum + 1;

        if (ADCPortNum == 0) {
            //BEMF
            BEMFValue = 0b00000000;
            BEMFValue += (ADCON[0] > motorADCThreshold) ? 0b00000001 : 0;
            BEMFValue += (ADCON[1] > motorADCThreshold) ? 0b00000010 : 0;
            BEMFValue += (ADCON[2] > motorADCThreshold) ? 0b00000100 : 0;

            if ((feedBack[direction][motorPosition] == BEMFValue) && reachO2CSpeed) {
                //if (feedBack[0][0] == sensorLessFeedBack) { //debug

                //change motor position
                nextState(direction);
                CLEnable = 1;
            }
        }

        ADCON0bits.CHS = ADPortCHS[ADCPortNum];
        PIR1bits.ADIF = 0;
    } else if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
    }
}

void main(void) {
    OSCCON = 0b01110000;
    OSCTUNE = 0b01000000;

    TRISA = 0b01010011;
    TRISB = 0b00000000;
    PORTA = 0b00000000;
    PORTB = 0b00000000;

    //interrupt
    INTCON = 0b11000000;
    INTCON2 = 0b00000000;
    INTCON3 = 0b00000000;

    //configure timer 0
    T0CON = 0b01000000;

    //configure ADC
    ADCON0 = 0b10000000;
    ADCON1 = 0b00010000; // Vref source -> Vref+ pin
    ADCON2 = 0b00001000;

    //configure PWM
    PTCON0 = 0b00000010; //Postscale 1:1, Fosc/4 Prescale 1:1
    PTCON1 = 0b10000000; //Time base in ON, Time base counts up
    PWMCON0 = 0b01000111; //All PWM I/O pins enabled for PWM output, Independent mode for all pins
    PWMCON1 = 0b00000001; //Postscale 1:1, special event on count downwards, updates from Duty Cycle and Period Buffers enabled
    OVDCOND = 0b00000000;
    OVDCONS = 0b00000000;
    //Period control for all PWM output
    PTPERH = 0x00;
    PTPERL = 0x3f; //8bit PWM
    DTCON = 0b00000110;
    SEVTCMPH = 0;
    SEVTCMPL = 1;
    //FLTCONFIG = 0b00000010;

    PDC0H = 0; //Duty Cycle control for PWM0/1
    PDC0L = 0; //Duty Cycle control for PWM0/1

    PDC1H = 0; //Duty Cycle control for PWM2/3
    PDC1L = 0; //Duty Cycle control for PWM2/3

    PDC2H = 0; //Duty Cycle control for PWM4/5
    PDC2L = 0; //Duty Cycle control for PWM4/5
    setDuty(0, 10);

    //Blink for start confirmation
    __delay_ms(500);
    PORTBbits.RB3 = 1;
    __delay_ms(500);
    PORTBbits.RB3 = 0;

    //start timer 0
    T0CONbits.TMR0ON = 0;
    INTCONbits.TMR0IE = 0;

    //start ADC
    ADCON0bits.CHS = 0;
    ADCON0bits.ADON = 1;
    __delay_ms(50);
    ADCON0bits.GO_nDONE = 1;
    PIE1bits.ADIE = 1;

    //var for open-loop
    unsigned int OLCurrent, OLInitialSpeed, openToLoopSpeed, OLaccelerate;
    unsigned int OLSpeedCount, OLAccelerateCount;

    //var for closed-loop
    unsigned int current, CLaccelerate;

    //settings
    direction = 0; //rotate direction 0/1/others
    OLCurrent = 0x1e; //Open-loop current
    OLInitialSpeed = 200; //Open-loop initial speed
    openToLoopSpeed = 50; //Open to close speed (Open-loop max speed)
    OLaccelerate = 2; //Open-loop "OLInitialSpeed" to "openToLoopSpeed" acceleration
    CLaccelerate = 5000; //Closed-loop acceleration
    //settings end

    //initialize
    current = 0x00;
    motorPosition = 0; //Motor initial position
    CLEnable = 0; //Closed-loop enable flag
    OLAccelerateCount = OLInitialSpeed;
    //initialize end

    //main motor control part
    while (1) {
        setDuty(current, CLaccelerate);

        //start motor with open-loop
        //transition to closed-loop
        if (!CLEnable) {
            current = ((current + 1) < OLCurrent) ? current + 1 : OLCurrent;
            OLAccelerateCount = ((OLAccelerateCount - OLaccelerate) > openToLoopSpeed) ? (OLAccelerateCount - OLaccelerate) : openToLoopSpeed;

            for (OLSpeedCount = 0; OLSpeedCount < OLAccelerateCount; OLSpeedCount++) {
                __delay_us(10);
            }

            if (OLAccelerateCount == openToLoopSpeed) {
                reachO2CSpeed = 1;
            }

            //change motor position
            nextState(direction);
        } else {
            current = 0xff;
            OLAccelerateCount = 0;
        }
    }

    return;
}

//PWM on/off setting

void BLDCPosition(int state) {
    switch (state) {
        case 0:
            OVDCOND = 0b00001001;
            break;
        case 1:
            OVDCOND = 0b00100001;
            break;
        case 2:
            OVDCOND = 0b00100100;
            break;
        case 3:
            OVDCOND = 0b00000110;
            break;
        case 4:
            OVDCOND = 0b00010010;
            break;
        case 5:
            OVDCOND = 0b00011000;
            break;
        default:
            OVDCOND = 0b00000000;
            break;
    }
    return;
}

//PWM duty ratio = Current supply to motor = speed of motor(closed-loop))

void setDuty(unsigned int newDuty, unsigned int acceleration) {
    static unsigned int prevDuty = 0;
    int i, accelerateCount;
    
    if (newDuty > 0xff) {
        newDuty = 0xff;
    }

    if(newDuty == prevDuty){
        return;
    } else if (newDuty > prevDuty) {
        for (i = prevDuty; i <= newDuty; i++) {
            PDC0L = i;
            PDC1L = i;
            PDC2L = i;
            for (accelerateCount = 0; accelerateCount < acceleration; accelerateCount++) {
                __delay_us(1);
            }
        }
    } else {
        for (i = prevDuty; i >= newDuty; i--) {
            PDC0L = i;
            PDC1L = i;
            PDC2L = i;
            for (accelerateCount = 0; accelerateCount < acceleration; accelerateCount++) {
                __delay_us(1);
            }
        }
    }

    prevDuty = newDuty;
    
    return;
}

void nextState(unsigned char directionSet) {
    if (directionSet == 0) {
        motorPosition = (motorPosition == 5) ? 0 : motorPosition + 1;
    } else if (directionSet == 1) {
        motorPosition = (motorPosition == 0) ? 5 : motorPosition - 1;
    } else {
        //stop
        motorPosition = 0;
        setDuty(0, 10);
    }
    BLDCPosition(motorPosition);
}
