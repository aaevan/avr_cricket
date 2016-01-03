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

#define CHIRP_LENGTH 1250 //1250
#define CHIRP_PITCH 163 //163 base pitch
#define SUBCHIRP_LENGTH 15 //15

void setFlag(int);
int getFlag();
volatile int flag = FLAG_NONE;

ISR(TIM1_OVF_vect)
{
    TCNT1 = CHIRP_PITCH; //startcount from 200 instead from 0
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
    TCCR1 |= (1<<CS10);
    TCCR1 &= ~((1<<CS13)|(1<<CS12)|(1<<CS11));
    //TCR1 &= ~((1<<CS11)|1<<CS10));
    //Enable interrupt
    TIMSK |= (1<<TOIE1);
    sei();

    TCNT1 = 0; 

    int i = 0;
    int j = 0;
    int temp_val = analog_read(PB3);
    int temp = 0;
    float temp_c = 0;
    float n_8 = 0;
    float chirps_per_second = 0;
    float delay_time = 0;
    float period = 0;
    
    while(1)
    {
        //TCCR1 &= ~((1<<CS10)|(1<<CS13)|(1<<CS12)|(1<<CS11));
        //TCCR1 |= (1<<CS10);
        //TCCR1 &= ~((1<<CS13)|(1<<CS12)|(1<<CS11));
        if(temp_c > 12.8){
        for(j=0; j<=2; j++)
            {
                for(i=0; i<=CHIRP_LENGTH; i++)
                {
                    if(getFlag() == FLAG_TOGGLE_LED)
                    {
                        PORTB ^= (1<<PB0);
                        setFlag(FLAG_NONE);
                    }
                }
                PORTB &= ~(1<<PB0);
                delay(SUBCHIRP_LENGTH);
            }
        }
        temp_val = analog_read(PB4); //we did have pb3 here before...

        //temp_c = 0.509 * (float) temp_val + 297.57;
        //temp_c = temp_val * 0.0048 * 100 - 273.15;
        temp_c = 24;
        n_8 = temp_c - 5;
        chirps_per_second = n_8 / 8;
        //chirps_per_second = 2.5;
        period = 1.0/chirps_per_second;
        delay_time = 1000.0 * period - 122;

        //delay(delay_time);//800
        delay((int) delay_time);//800
        //40 + N15
        //2.66 + N1 = T(f) - 2.66
        /*
        int temp = analog_read(PB3);
        delay(temp);
        //delay(analog_read(PB4));
        PORTB ^= (1<<PB0);
        */
    }
}


//void tone_delay

void setFlag(int f) {
    flag = f;
}

int getFlag() {
    return flag;
}

int analog_read(int read_pin) { 
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

