#include <avr/io.h>
#include <util/delay.h>

#if 0
void my_delay_us(uint16_t delay)
{
	// the step size '5' was determined
	// using simple experiments.
	while (delay > 5)
		delay -= 5;
}
#else
void my_delay_us(uint16_t delay);
__asm__(
// this is what the compiler generated for
// my_delay_us(). we `materialized' it to
// be imune to changes in the compiler..
"my_delay_us:\n"
"        rjmp .mdelus_L2\n"
".mdelus_L1:\n"
"        sbiw r24,5\n"
".mdelus_L2:\n"
"        cpi r24,6\n"
"        cpc r25,__zero_reg__\n"
"        brsh .mdelus_L1\n"
"        ret\n");
#endif

void my_delay_ms(uint16_t delay)
{
	while (delay != 0) {
		// experiments have shown that 1ms is in fact 970us
		// when using my_delay_us() (at least with the test
		// chip -- RC oszillators aren't very stable...)
		my_delay_us(970);
		delay--;
	}
}

void setvervo(uint16_t pos)
{
	PORTB |= 1;
	my_delay_us(550+pos*6);
	PORTB &= ~1;
	my_delay_ms(20);
}

int main(void)
{
	DDRB |= 1;
	PORTB &= ~1;

#if 0
	while (1)
	{
		setvervo(0);
		my_delay_ms(980);
		setvervo(0);
		my_delay_ms(980);
		for (uint8_t i = 0; i < 50; i++)
			setvervo(0);

		setvervo(255);
		my_delay_ms(980);
		setvervo(255);
		my_delay_ms(980);
		for (uint8_t i = 0; i < 50; i++)
			setvervo(255);
	}
#else
	while (1) {
		for (uint16_t i = 0; i < 64; i++)
			setvervo(0);
		for (uint16_t i = 0; i < 255; i++)
			setvervo(i);
		for (uint16_t i = 0; i < 64; i++)
			setvervo(255);
		for (uint16_t i = 0; i < 255; i++)
			setvervo(255-i);
	}
#endif

	return 0;
}

