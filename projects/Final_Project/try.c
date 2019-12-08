#include 

#define setbit(port,bit)  ((port) |=  (1<<bit)) // set bit
#define clrbit(port,bit)  ((port) &= ~(1<<bit)) // clear bit

#define REF_AVCC (1<<REFS0)  // reference = AVCC = 5 V
#define REF_INT  (1<<REFS0)|(1<<REFS1) // internal reference 2.56 V 

// global variables
uint16_t adc_value0, adc_value1;

//------------------------------------------------------
uint16_t adc_read(uint8_t channel)
{
   ADMUX = REF_AVCC | channel;  // set reference and channel
   ADCSRA |= (1<<ADSC);         // start conversion  
   loop_until_bit_is_clear(ADCSRA,ADSC); // wait for conversion complete  
   return ADC;
}

uint16_t temp;

//======================================================

int main(void)
{
   DDRB = 0xff; // portB all outputs
   ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
   
   while(1)
   {
      adc_value0 = adc_read(0); // read ADC0 (pin PA0)
      adc_value1 = adc_read(1); // read ADC1
      
      if(adc_value0 > 512)  // >2.5 V
         setbit(PORTB,0);
      else
         clrbit(PORTB,0); 
      
      if(adc_value1 > 512)
         setbit(PORTB,1);
      else
         clrbit(PORTB,1); 
   }
}