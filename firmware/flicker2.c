/*****************************************************
Flickering LED candle for the AVR ATtiny13
by Roy M. Silvernail, with help from the net.

This code is released into the public domain with no warranty.  
Use it, abuse it, pass it on.  But if it breaks, you get to keep the
pieces.

Note that the delay_ms function, while theoretically correct, doesn't
seem to line up with actual milliseconds.  Since the absolute timing for
this project isn't critical, I just eyeballed the timing values.

Sept. 2, 2008

Update Oct. 19, 2008
There are now two flicker patterns.  The original one maintains a lower brightness
and flickers to full brightness.  The new pattern maintains full brightness and
flickers to a lower level.  This is closer in concept to the original behavior of the
LED candle, but looks better than a brief turn-off period.  The code will alternate 
between the two patterns on each power-up cycle.

The watchdog timer is now activated to reset the chip if the main loop should freeze.

Thanks to the TV-B-Gone crew on LadyAda's forums for the inspiration to store some
state in the EEPROM, and to Instructables member cheeze69 for the idea to use the watchdog.
*****************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
static inline void _delay_loop_2(uint16_t __count) __attribute__((always_inline));

#define delay_us(us)   _delay_loop_2((unsigned int)(((float)F_CPU*us)/4000000L))
#define nop()         asm("nop")

//Global variables and definition
#define BRIGHT_MIN 40
#define BRIGHT_ALT_MIN 20
#define BRIGHT_MAX 255
#define BRIGHT_RATE 60
#define BRIGHT_HOLD 150
#define INTER_FLASH 128

uint16_t EEMEM eeprom_initialized = 0x6502;
uint8_t EEMEM eeprom_alt_pattern;

void delay_ms(uint16_t x)
	{
	while(x--)
		{
		delay_us(1000);
		}
	}
	

/*
 * pseudorandom
 * return the next pseudo-random number (PRN) using a standard maximum
 * length xor-feedback 16-bit shift register.
 * This returns the number from 1 to 65535 in a fixed but apparently
 * random sequence.  No one number repeats.
 */
 
uint16_t randreg = 10;

static uint16_t pseudorandom16 (void)
{
    uint16_t newbit = 0;

    if (randreg == 0) {
        randreg = 1;
    }
    if (randreg & 0x8000) newbit = 1;
    if (randreg & 0x4000) newbit ^= 1;
    if (randreg & 0x1000) newbit ^= 1;
    if (randreg & 0x0008) newbit ^= 1;
    randreg = (randreg << 1) + newbit;
    return randreg;
} 

void pwm_start(void){
        OCR0A = BRIGHT_MIN;   	// Initial pulse width
        DDRB |= 1;            	// Set pin 5 as output
        TCCR0A = 0x81;          //8-bit, Non-Inverted PWM
        TCCR0B = 1;             //Start PWM
}


int main(void)
	{
	int i;
	
	MCUSR = 0;                     // clear watchdog flag
	WDTCR = _BV(WDCE) | _BV(WDE);  // enable WDT disable
	WDTCR = 0;                     // disable WDT
	
	if(eeprom_read_word(&eeprom_initialized)!=0x6502)	//Make sure eeprom is initialized to default values.
		{
		eeprom_write_word(&eeprom_initialized,0x6502);
		eeprom_write_byte(&eeprom_alt_pattern,0);
		}
	uint8_t alt_pattern = eeprom_read_byte(&eeprom_alt_pattern);  //Flag to disable blink.
	if(alt_pattern)
		{
   		eeprom_write_byte(&eeprom_alt_pattern,0);
   		}
   	else
   		{
   		eeprom_write_byte(&eeprom_alt_pattern,1);
   		}

	pwm_start();
	// turn on watchdog timer immediately, this protects against
	// a 'stuck' system by resetting it
	wdt_enable(WDTO_8S); // 1 second long timeout
	while(1)
		{
		if(alt_pattern)
			{
			for(i=BRIGHT_MAX;i>=BRIGHT_ALT_MIN;i--)
				{
				OCR0A = i;
				delay_us(BRIGHT_RATE);
				}
			delay_us(BRIGHT_HOLD);
			for(;i<BRIGHT_MAX;i++)
				{
				OCR0A = i;
				delay_us(BRIGHT_RATE);
				}
			}
		else
			{
			for(i=BRIGHT_MIN;i<256;i++)
				{
				OCR0A = i;
				delay_us(BRIGHT_RATE);
				}
			delay_us(BRIGHT_HOLD);
			for(;i>=BRIGHT_MIN;i--)
				{
				OCR0A = i;
				delay_us(BRIGHT_RATE);
				}
			}
		wdt_reset();	//To keep Watchdog from resetting in middle of code.			
		delay_ms(pseudorandom16() % INTER_FLASH);
		}
    }

