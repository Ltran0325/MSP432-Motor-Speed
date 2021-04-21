/*******************************************************************************
*                           MSP432 Motor Speed                                 *
*                                                                              *
* Author:  Long Tran                                                           *
* Device:  MSP432P401R LaunchPad                                               *
* Program: Display encoder turn count on 7-segment display                     *
*                                                                              *
* Demo: https://youtu.be/3tQvZHvHI0I                                           *
* refer to: https://github.com/Ltran0325/MSP432-Motor-Speed                    *
*******************************************************************************/

#include "msp.h"
#include "stdlib.h"

// Configuration prototypes
void init_NVIC(void);
void init_clock(void);
void init_encoder(void);
void init_pot(void);
void init_motor(void);
void init_ADC(void);
void init_display(void);
void init_Timer_A0(void);
void init_Timer_A1(void);

// display prototypes
void sseg_modulo( uint16_t speed); // divide speed into digits using modulo operator
void sseg_display(void);             // display counter digits on 7-segment display
void wait(uint32_t t);

// ADC prototypes
uint32_t get_ADC(void);

const uint8_t look_up[10] = { // 7-segment display look up table
0b11000000,  // 0
0b11111001,  // 1
0b10100100,  // 2
0b10110000,  // 3
0b10011001,  // 4
0b10010010,  // 5
0b10000010,  // 6
0b11111000,  // 7
0b10000000,  // 8
0b10010000,  // 9
};

float x0 = 0, y0 = 0, x1 = 0, y1 = 0, temp = 0;
volatile int16_t counter = 0;   // encoder angle counter
volatile int16_t speed = 0;
uint8_t display[4] = {0,0,0,0}; // 7-seg display array

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    init_NVIC();
    init_clock();
    init_encoder();
    init_pot();
    init_motor();
    init_ADC();
    init_display();
    init_Timer_A0();
    init_Timer_A1();

    uint16_t delay = 0;

    while(1){

        if(delay == 1000){
            delay = 0;
            temp = get_ADC();   //map 14-bit ADC to 75
            temp *= 75;
            temp /= 16384;

            if (temp > 74){ temp = 74;} // motor stops if temp = 0 or 75
            if(temp < 1){ temp = 1;}
            TIMER_A0->CCR[1] = temp;
            TIMER_A0->CCR[2] = 75-temp;
        }
        delay++;

        sseg_modulo(abs(speed));
        sseg_display();
    }

}

//-- Functions
void init_NVIC(void){
    //-- Configure NVIC
    NVIC->ISER[0] = 1 << ((TA1_0_IRQn) & 31); //enable TA1_0 interrupt
    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31); //enable P3 interrupt
}

void init_clock(void){

    //-- Configure DCO clock to 48MHz
    while((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;  // AM_LDO_VCORE1
    while((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 &= 0x0000FFFF;    // lock PCM

    // Flash read wait state number change
    FLCTL->BANK0_RDCTL &= ~(BIT(12) | BIT(13) | BIT(14) | BIT(15) ); // reset bits
    FLCTL->BANK0_RDCTL |=   BIT(12);                                 // set 1 wait state
    FLCTL->BANK1_RDCTL &= ~(BIT(12) | BIT(13) | BIT(14) | BIT(15) );
    FLCTL->BANK1_RDCTL |=   BIT(12);                                 // set 1 wait state

    // Enable DCO, set frequency to 48 MHz
    CS->KEY = 0x0000695A;           // unlock clock system registers (MSP432 Ref. Manual, pg.394)
    CS->CTL0 |= BIT(16)| BIT(18);   // set DCO frequency range to 48 MHz
    CS->CTL0 |= BIT(23);            // enable DCO oscillator

    // Select DCO as the source for MCLK
    CS->CTL1 |= BIT0 | BIT1;        // MCLK source: DCOCLK
    CS->CLKEN |= BIT1;              // enable MCLK

    // Set SMCLK to DCO/128 = 375kHz
    CS->CTL1 |= CS_CTL1_SELS_3;
    CS->CTL1 |= CS_CTL1_DIVS__128;

    // lock CS registers
    CS->KEY =0x0;

}

void init_encoder(void){

    P3->DIR &= ~BIT6;   // phaseA input
    P3->IE  |= BIT6;    // enable P3.6 interrupt
    P3->IES &= ~BIT6;   // rising edge

    P5->DIR &= ~BIT3;   // phaseB input

}

void init_pot(void){

    P9->DIR |= BIT4;        // set P9.4 as OUTPUT to drive voltage to potentiometer
    P9->OUT |= BIT4;        // set potentiometer voltage high

}
void init_motor(void){

    P2->OUT &= ~(BIT4 | BIT5);
    P2->DIR |= BIT4 | BIT5;     // PWM output to motor
    P2->SEL0 |= BIT4 | BIT5;    // TA0.1 and TA0.2

}

void init_ADC(void){

    P5->DIR &= ~BIT1;   // ADC input from POT
    P5->SELC |= BIT1;   // analog input A4

    ADC14->CTL0 &= !ADC14_CTL0_ENC;

    ADC14->CTL0 |= ADC14_CTL0_PDIV__4    | // predivider to 4
                   ADC14_CTL0_SHS_0      | // sample-and-hold source select to ADC14SC bit
                   ADC14_CTL0_SHP        | // SAMPCON signal source to sampling timer
                   ADC14_CTL0_DIV__8     | // divider to 8
                   ADC14_CTL0_SSEL__MCLK | // MCLK as clock source
                   ADC14_CTL0_SHT0__32   | // sampling period to 32 ADC14CLK cycles for MEM0-7 and MEM14-31
                   ADC14_CTL0_ON;          // turn ADC on

    ADC14->CTL1 |= ADC14_CTL1_RES__14BIT;  // resolution to 14 bit

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_4; // input channel A4

    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

void init_display(void){

    P4->DIR = 0xFF;  // P4 is 7-segment LED output
    P8->DIR = 0xFF;  // P8 is display output
    P5->DIR |= BIT0; // P5.0 is red LED angle polarity indicator

}

void init_Timer_A0(void){

    // Set TA0CCR0 = 375kHz/5KHz = 75, PWM period
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | // SMCLK as clock source
                     TIMER_A_CTL_MC__UP;       // up mode
    TIMER_A0->CCR[0] = 75;                     // CC register value

    // Set TA0CCR1-2 as PWM output
    TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_2;
    TIMER_A0->CCTL[2] |= TIMER_A_CCTLN_OUTMOD_2;
}

void init_Timer_A1(void){

    TIMER_A1->CTL |= TIMER_A_CTL_SSEL__SMCLK | // SMCLK as clock source
                     TIMER_A_CTL_MC__UP;       // up mode

    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; // clear CC interrupt flag
    TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE;   // enable CC interrupt
    TIMER_A1->CCR[0] = 1875;                     // CC register value

}
void sseg_modulo( uint16_t speed){

    display[0] = speed/1000;
    display[1] = (speed/100)%10;
    display[2] = (speed/10)%10;
    display[3] = speed%10;
}

void sseg_display(void){

    static uint8_t k = 0;

    // Display digit-k
    P4->OUT = 0xFF;                      // blank 7-seg display
    P8->OUT = 0xFF & ~(BIT5 >> k);       // enable k-th digit in 7-seg display
    P4->OUT = look_up[display[k]];       // display k-th digit in 7-seg display

    // increment k index
    k++;
    if (k >= 4){k = 0;}

    if(speed >= 0){    /* Error?: Encoder is backwards*/
        P5->OUT &= ~BIT0;   // red LED off, positive angle (CW)
    }else{
        P5->OUT |= BIT0;    // red LED on, negative angle (CCW)
    }

    // reduce flickering
    wait(100);
}

void wait(uint32_t t){
    while(t > 0){t--;}
}

uint32_t get_ADC(void){

    // BIT 0-1 ADC14SC and ADC14ENC
    ADC14->CTL0 |= ( BIT0|BIT1 );   // start ADC14 conversion by setting both registers

    while(ADC14->CTL0 & BIT(16));   // ADC14BUSY, wait while ADC14 is busy

    return ADC14->MEM[0];     // get ADC14 result
}

//-- Interrupts
void PORT3_IRQHandler(void){
    if(P3->IV & 0x0E){      // if phaseA is interrupt source (rising edge)
        if(P5->IN & BIT3){  // if phaseB is high
            counter--;      // decrement counter (CCW)
        }else{              // else
            counter++;      // increment counter (CW)
        }
    }
}

void TA1_0_IRQHandler(void){    // sampling freq = 200Hz

    // speed in RPM = (n*f*60)/N
    speed = counter*200*60/400;
    counter = 0;

    // low pass algorithm,
    x0 = speed;
    y0 = 0.9691*y1 +0.03093*x0;

    speed = y0;
    y1 = y0;
    x1 = x0;

    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  // clear interrupt flag
}

