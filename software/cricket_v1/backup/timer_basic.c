// Define clock speed for delay.h:
// (8MHz internal crystal; /8 prescaler): 

#define LED1 PB0 
#define KNOB PB2 
#define F_CPU 1000000L 

#include <avr/io.h> 
#include <util/delay.h> 
#include <avr/interrupt.h>

#define FLAG_NONE 0
#define FLAG_TOGGLE_LED 1

void setFlag(int);
int getFlag();
volatile int flag = FLAG_NONE;

ISR(TIM1_OVF_vect)
{
    TCNT1 = 150; //startcount from 200 instead from 0
    setFlag(FLAG_TOGGLE_LED);
}

int main(void)
{
    // make PB0 an output (LED), make PB2 an input (Button)
    DDRB |= (1<<DDB0);
    // turn off LED
    PORTB &= ~(1<<PB0);
    //comparator B output mode pg 89
    TCCR1 &= ~( (1<<COM1A1)|(1<<COM1A0) ); //COM1A1 = 0, COMA10 = 0
    //Comparator b output mode pg 90
    TCCR1 &= ~((1<<COM1B1)|(1<<COM1B0));
    // Timer/Counter1 Prescale Select pg 89
    //TCCR1 |= (1<<CS13)|(1<<CS11);
    TCCR1 |= (1<<CS12)|(1<<CS11);
    TCCR1 &= ~((1<<CS13)|(1<<CS10));
    //TCR1 &= ~((1<<CS11)|1<<CS10));
    //Enable interrupt
    TIMSK |= (1<<TOIE1);
    sei();

    TCNT1 = 0; 

    while(1)
    {
        if(getFlag() == FLAG_TOGGLE_LED)
        {
            PORTB ^= (1<<PB0);
            setFlag(FLAG_NONE);
        }
    }
}

void setFlag(int f)
{
    flag = f;
}

int getFlag()
{
    return flag;
}

int analog_read(int read_pin){ 
    //Read ADC2L:
    if(read_pin != 1 && read_pin != 2 && read_pin != 3){
        //if read_pin is not a valid number, break. 
        return 0;
    }
    ADMUX |= read_pin; //set the pin that the adc reads to ADC2 (PB4) 
    ADCSRA |= (1 << ADEN); // Analog-Digital enable bit 
    ADCSRA |= (1 << ADSC); // Discard first conversion 
    while (ADCSRA & (1 << ADSC)); // wait until conversion is done 
    ADCSRA |= (1 << ADSC); // start single conversion 
    while (ADCSRA & (1 << ADSC)) // wait until conversion is done
    ADCSRA &= ~(1<<ADEN); // shut down the ADC
    int low_val = ADCL;
    int high_val = ADCH;
    int val = (high_val<<8)|low_val;
    //Clear bottom 3 bits of ADMUX:
    ADMUX &= ~(0x111);
    return val;
}

void delay_microseconds(int us){
    int i;
    for (i = 0; i < us; i++){
        _delay_us(1);
    }
}

void delay(int ms){
    int i;
    for (i=0; i<ms; i++) {
        _delay_ms(1);
    }
}

void toggle(int pin){
    PORTB ^= (1<<pin);
}

void set_high(int pin){
    PORTB |= (1<<pin);
}

void set_low(int pin){
    PORTB &= ~(1<<pin);
}

/*
int main(void){

    // Setup :
    //Set LED as output:
    DDRB |= (1<<LED1);
    set_high(LED1);
    //default is input, 1 is output.
    // Main:
    int knob_pos;
    while (1){
        knob_pos = analog_read(2);
        delay(knob_pos);
        toggle(LED1);
    }
    return 0;
    *
} 
*/

