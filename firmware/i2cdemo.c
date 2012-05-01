#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"

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

/*********************************************************/
/*                      <SOFT-UART>                      */

// 9600 Baud on pin B0
#define SOFTUART_DDR  DDRB
#define SOFTUART_PORT PORTB
#define SOFTUART_BITV _BV(2)
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

void softuart_send_hex_nibble(uint8_t nibble)
{
	if (nibble < 10)
		softuart_send_byte('0' + nibble);
	else
		softuart_send_byte('A' + nibble - 10);
}

void softuart_send_hex_byte(uint8_t byte)
{
	softuart_send_hex_nibble(byte >> 4);
	softuart_send_hex_nibble(byte & 0x0f);
}

void softuart_setup()
{
	// convert PORTx to DDRx and mark pin as output
	SOFTUART_DDR |= SOFTUART_BITV;
	SOFTUART_PORT |= SOFTUART_BITV;
}

/*                      </SOFT-UART>                     */
/*********************************************************/

#define MMA7660FC_ADDR 0x98

uint8_t read_accel_reg(uint8_t addr)
{
	uint8_t ret;
	i2c_start_wait(MMA7660FC_ADDR + I2C_WRITE);
	i2c_write(addr);
	i2c_rep_start(MMA7660FC_ADDR + I2C_READ);
	ret = i2c_readNak();
	i2c_stop();
	return ret;
}

int main(void)
{
	i2c_init();
	softuart_setup();

	while (1) {
		softuart_send_byte('-');
		softuart_send_byte('-');
		softuart_send_byte('-');
		softuart_send_byte('\r');
		softuart_send_byte('\n');
		for (uint8_t i = 0; i < 11; i++) {
			softuart_send_hex_byte(i);
			softuart_send_byte(' ');
			softuart_send_hex_byte(read_accel_reg(i));
			softuart_send_byte('\r');
			softuart_send_byte('\n');
		}
		my_delay_ms(1000);
	}
	return 0;
}

