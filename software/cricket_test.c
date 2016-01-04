// Define clock speed for delay.h:
// (8MHz internal crystal; /8 prescaler): 

#define LED1 PB0 
#define KNOB PB2 
#define F_CPU 1000000L 

#include <avr/io.h> 
#include <util/delay.h> 
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define FLAG_NONE 0
#define FLAG_TOGGLE_LED 1

#define CHIRP_LENGTH 1250 //1250
#define CHIRP_PITCH 163 //163 base pitch
#define SUBCHIRP_LENGTH 15 //15

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void setFlag(int);
int getFlag();
volatile int flag = FLAG_NONE;

volatile int f_wdt = 1;

ISR(TIM1_OVF_vect)
{
    TCNT1 = CHIRP_PITCH; //startcount from CHIRP_PITCH instead from 0
    setFlag(FLAG_TOGGLE_LED);
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
    if (read_pin != 1 && read_pin != 2 && read_pin != 3) {
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

void delay_microseconds (int us) {
    int i;
    for (i = 0; i < us; i++) {
        _delay_us(1);
    }
}

void delay(int ms) {
    int i;
    for (i=0; i<ms; i++) {
        _delay_ms(1);
    }
}

void toggle(int pin) {
    PORTB ^= (1<<pin);
}

void set_high(int pin) {
    PORTB |= (1<<pin);
}

void set_low(int pin) {
    PORTB &= ~(1<<pin);
}

void watchdog_setup(void) {
    cli(); //disable interrupts
    // If a reset was caused by the Watchdog Timer...
    if(MCUSR & _BV(WDRF)){            
        MCUSR &= ~_BV(WDRF);               // Clear the WDT reset flag
        WDTCR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
        WDTCR = 0x00;                      // Disable the WDT
    }
    wdt_reset(); //reset watchdog
    WDTCR = (1<<WDCE)|(1<<WDE); //set up WDT interrupt
    //Start watchdog timer with 1 second prescaler:
    //WDTCR = (1<<WDIE)|(1<<WDE)|(1<<WDP2)|(1<<WDP1); 
    WDTCR = (1<<WDIE)|(1<<WDE)|(1<<WDP2)|(1<<WDP1); 
    sei(); //Enable global interrupts
}

ISR(WDT_vect) {
    f_wdt=1;  // set global flag
    tester(400);
    tester(200);
    //tester(); //DEBUG
    //PASS???
}

void timer_setup() {
    //TODO: make this more legible. I don't know what half these things do.
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
}

void setup() {
    // TODO: fix this line's comment to be relevant
    // make PB0 an output (LED), make PB2 an input (Button)
    DDRB |= (1<<DDB0);
    // turn off speaker pin
    PORTB &= ~(1<<PB0);
    timer_setup();
    watchdog_setup(); //does it matter the order of these? TODO
}

void system_sleep() {
    cbi(ADCSRA,ADEN); // switch Analog to Digitalconverter OFF
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
    sleep_enable();
    sleep_mode();     // System sleeps here
    // System continues execution here when watchdog timed out
    sleep_disable();
    sbi(ADCSRA,ADEN); // switch Analog to Digitalconverter ON
}
    
void chirp_loop() {
    //turn on interrupts for the length of the chirp.
    wdt_disable(); // this should fix the WDT interrupting chirping...
    sei();
    int i = 0;
    int j = 0;
    for(j=0; j<=2; j++) {
        for(i=0; i<=CHIRP_LENGTH; i++) {
            if(getFlag() == FLAG_TOGGLE_LED) {
                PORTB ^= (1<<PB0);
                setFlag(FLAG_NONE);
            }
        }
        PORTB &= ~(1<<PB0); //toggle pin low
        delay(SUBCHIRP_LENGTH);
    }
    //when we're done chirping, turn off interrupts.
    cli();
    wdt_enable(WDTO_250MS);
}

//DEBUG LAND-----------------------------------------
void tester(int pitch) {
    //turn on interrupts for the length of the chirp.
    wdt_disable(); // this should fix the WDT interrupting chirping...
    sei();
    delay(100);
    int i = 0;
    int j = 0;
    for(j=0; j<=2; j++) {
        for(i=0; i<=pitch; i++) {
            if(getFlag() == FLAG_TOGGLE_LED) {
                PORTB ^= (1<<PB0);
                setFlag(FLAG_NONE);
            }
        }
        PORTB &= ~(1<<PB0); //toggle pin low
        delay(SUBCHIRP_LENGTH);
    }
    //when we're done chirping, turn off interrupts.
    cli();
    wdt_enable(WDTO_250MS);
}
//DEBUG LAND (END) ----------------------------------

int main(void) {
    setup();
    // Set up Watch Dog Timer for Inactivity
    // WDTCR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
    // WDTCR =   _BV(WDIE) |              // Enable WDT Interrupt
    //_BV(WDP2) | _BV(WDP1);   // Set Timeout to ~1 seconds
    int temp_val = 500;
    int light_level = analog_read(PB2);
    int light_thresh = 850;
    for (;;) {
        if (f_wdt==1) {
            f_wdt = 0;
            // do sensor readings here
            light_level = analog_read(PB2);
            temp_val = analog_read(PB3);
            //if we pass the temperature and light level checks...
            if(light_level > light_thresh) { //TODO: Also, we need to test for temperature too.
                delay(temp_val);//800
                chirp_loop();
            }
            //otherwise, go to sleep!
            else {
                system_sleep();
            }
        }
    }
}
