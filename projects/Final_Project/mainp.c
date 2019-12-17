/*
 * ---------------------------------------------------------------------
 * Author:      Tomas Fryza
 *              Dept. of Radio Electronics, Brno Univ. of Technology
 * Created:     2018-10-23
 * Last update: 2019-11-01
 * Platform:    ATmega328P, 16 MHz, AVR 8-bit Toolchain 3.6.2
 * ---------------------------------------------------------------------
 * Description:
 *    Analog-to-digital conversion with displaying result on LCD and 
 *    transmitting via UART.
 * 
 * Note:
 *    Peter Fleury's UART library.
 */

/* Includes ----------------------------------------------------------*/
#include <stdlib.h>             // itoa() function
#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
#include "lcd.h"
#include "uart.h"
#include "stdio.h"

/* Typedef -----------------------------------------------------------*/
/* Define ------------------------------------------------------------*/
#define UART_BAUD_RATE 9600

/* Variables ---------------------------------------------------------*/
/* Function prototypes -----------------------------------------------*/

/* Functions ---------------------------------------------------------*/
/**
 *  Brief:  Main program. Read ADC result and transmit via UART.
 *  Input:  None
 *  Return: None
 */
int main(void)
{
    // LCD display
    lcd_init(LCD_DISP_ON);

    /* ADC
     * TODO: Configure ADC reference, clock source, enable ADC module, 
     *       and enable conversion complete interrupt */
        ADMUX |=  _BV(REFS0);
        ADMUX &= ~_BV(REFS1);

      /*  ADMUX |=  _BV(MUX0);
        ADMUX &= ~_BV(MUX1);
        ADMUX &= ~_BV(MUX2);
        ADMUX &= ~_BV(MUX3);*/

        ADCSRA |= _BV(ADPS0);
        ADCSRA |= _BV(ADPS1);
        ADCSRA |= _BV(ADPS2);
        
        ADCSRA |= _BV(ADEN);

        ADCSRA |= _BV(ADIE);
    
     
      /*GPIO_config_output(&ADMUX,AVCC);*/
       
    /* Timer1
     * TODO: Configure Timer1 clock source and enable overflow 
     *       interrupt */
    TIM_config_prescaler(TIM1, TIM_PRESC_64);
    TIM_config_interrupt(TIM1, TIM_OVERFLOW_ENABLE);

    // UART: asynchronous, 8-bit data, no parity, 1-bit stop
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Put string to ringbuffer for transmitting via UART.
    //uart_puts("UART testing\r\n");

    // Infinite loop
    for (;;) {
    

    }

    // Will never reach this
    return (0);
}

/**
 *  Brief: Timer1 overflow interrupt routine. Start ADC conversion.
 */
ISR(TIMER1_OVF_vect)
{
    // TODO: Start ADC conversion

     ADCSRA |= _BV(ADSC);
}

/**
 *  Brief: ADC complete interrupt routine. Update LCD and UART 
 *         transmiter.
 */

//Voltmeter
void Voltmeter(void)
{
    float V_out = 0;
    float V_in ;
    uint16_t R1 = 470;
    uint16_t R2 = 470;
    char uart_string[4];

	// Read 10-bit ACD value and converting it to voltage (ADC*5/1023)
     V_out = ADC*0.004888;

    // Calculating the input voltage
     V_in = (V_out*(R1+R2))/R2;

    // TODO: Update LCD and UART transmiter
       
    uart_puts("Voltage = ");
    dtostrf(V_in, 5, 3, uart_string);
    //itoa(R2, uart_string, 10);  
    uart_puts(uart_string); 
    uart_puts(" V\r\n");

/*  lcd_puts("Voltmeter");
    lcd_gotoxy(0,1);*/
    lcd_puts(uart_string); 
    lcd_gotoxy(5,0);
    lcd_puts("V ");
}

//Amperemeter
void Ammeter(void)
{
    float V_out = 0;
    float I_in=0;
    float V_nominal = 2.500;
    float constant = 0.185;
    char uart_string[10];

	// Read 10-bit ACD value and converting it to voltage (ADC*5/1023)
    V_out = ADC*0.004888;

    // Calculating the input voltage
    I_in = ((V_out-V_nominal)/constant);
    
    // TODO: Update LCD and UART transmiter

    uart_puts("Current = ");
    dtostrf(I_in, 5, 3, uart_string);
    //itoa(I_in, uart_string, 10);
    uart_puts(uart_string); 
    uart_puts("A\r\n");

  //lcd_puts("Amp meter");
    lcd_gotoxy(7,0)
    lcd_puts(uart_string); 
    lcd_gotoxy(12,0);
    lcd_puts("A");

}


//Ohmmeter
void Ohmmeter(void)
{
     float V_out = 0;
     float V_in = 5;
     float R1 = 10000;
     float R2;
     char uart_string[10];

	// Read 10-bit ACD value and converting it to voltage (ADC*5/1023)
    V_out = ADC*0.004888;

    // Calculating the input voltage
    R2 = (R1*V_out)/(V_in-V_out);

    // TODO: Update LCD and UART transmiter
    if(R2<1000){
    uart_puts("Ohmmeter");
    dtostrf(R2, 5, 3, uart_string);
    //itoa(R2, uart_string, 10);  
    uart_puts(uart_string); 
    uart_puts(" Ohms\r\n");

    lcd_puts("Ohmmeter");
    lcd_gotoxy(0,1);
    lcd_puts(uart_string); 
    lcd_gotoxy(7,1);
    lcd_puts("Ohms       ");
    }
    else{
    R2 = R2/1000;
    uart_puts("Ohmmeter\n\r");
    dtostrf(R2, 5, 3, uart_string);
    //itoa(R2, uart_string, 10);  
    uart_puts(uart_string); 
    uart_puts(" Kilo Ohms\r\n");

    lcd_puts("Ohmmeter");
    lcd_gotoxy(0,1);
    lcd_puts(uart_string); 
    lcd_gotoxy(7,1);
    lcd_puts("Kilo Ohms       ");
    }
    //itoa(R2, uart_string, 10);
}


//Luxmeter
void Luxmeter(void)
{
    float V_out = 0;
    float V_in = 5;
    uint16_t R1 = 10000;
    float ldr;
    //float lux;
    char uart_string[4];

	// Read 10-bit ACD value and converting it to voltage (ADC*5/1023)
     V_out = ADC*0.004888;

    // Calculating the input voltage
     ldr = (R1*V_out)/(V_in-V_out);
     //lux = 500/ldr;

    // TODO: Update LCD and UART transmiter
       
     if(ldr<1000){
    uart_puts("");
    dtostrf(ldr, 5, 3, uart_string);
    //itoa(R2, uart_string, 10);  
    uart_puts(uart_string); 
    uart_puts(" Ohms\r\n");

    lcd_puts("LDR Resistance");
    lcd_gotoxy(0,1);
    lcd_puts(uart_string); 
    lcd_gotoxy(7,1);
    lcd_puts("Ohms       ");
    }
    else{
    ldr = ldr/1000;
    uart_puts("");
    dtostrf(ldr, 5, 3, uart_string);
    //itoa(R2, uart_string, 10);  
    uart_puts(uart_string); 
    uart_puts(" Kilo Ohms\r\n");

    lcd_puts("LDR Resistance");
    lcd_gotoxy(0,1);
    lcd_puts(uart_string); 
    lcd_gotoxy(7,1);
    lcd_puts("Kilo Ohms       ");
    }
}


ISR(ADC_vect)
{
    switch (ADMUX)
    {
    case 0x80:
        Voltmeter();
        ADMUX = 0x81;
        break;
    case 0x81:
        Ammeter();
        ADMUX = 0x82;
        break;    
    case 0x82:
        Ohmmeter();
        ADMUX = 0x83;
        break;    
    case 0x83:
        Luxmeter();
        ADMUX = 0x80;
        break;
    default:
        break;
    }

}