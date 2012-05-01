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

// -------------------- <SOFT-UART> --------------------

// 9600 Baud on pin B0
#define SOFTUART_PORT PORTB
#define SOFTUART_BITV _BV(0)
#define SOFTUART_DELAY my_delay_us(80);

static inline void softuart_send_bit(uint8_t bit)
{
	SOFTUART_PORT = bit ? (SOFTUART_PORT | SOFTUART_BITV) :
			(SOFTUART_PORT & ~SOFTUART_BITV);
	SOFTUART_DELAY
}

void softuart_send_byte(uint8_t byte)
{
	// start bit
	softuart_send_bit(0);

	// send LSB first
	for (uint8_t i = 0; i < 8; i++) {
		softuart_send_bit(byte & 1);
		byte = byte >> 1;
	}

	// two stop bits
	softuart_send_bit(1);
	softuart_send_bit(1);
}

void softuart_setup()
{
	// convert PORTx to DDRx and mark pin as output
	*(uint8_t*)(_SFR_IO_ADDR(SOFTUART_PORT) - 1) |= SOFTUART_BITV;
	SOFTUART_PORT |= SOFTUART_BITV;
}

// -------------------- </SOFT-UART> -------------------

int main(void)
{
	DDRB |= 1;
	PORTB |= 1;
	while (1) {
		softuart_send_byte('H');
		softuart_send_byte('e');
		softuart_send_byte('l');
		softuart_send_byte('l');
		softuart_send_byte('o');
		softuart_send_byte(' ');
		softuart_send_byte('W');
		softuart_send_byte('o');
		softuart_send_byte('r');
		softuart_send_byte('l');
		softuart_send_byte('d');
		softuart_send_byte('!');
		softuart_send_byte('\r');
		softuart_send_byte('\n');
		my_delay_ms(1000);
	}
	return 0;
}

