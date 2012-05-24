#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"

/*********************************************************/
/*                    Delay functions                    */
/*********************************************************/

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
/*                       SOFT-UART                       */
/*********************************************************/

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

void softuart_setup(void)
{
	// convert PORTx to DDRx and mark pin as output
	SOFTUART_DDR |= SOFTUART_BITV;
	SOFTUART_PORT |= SOFTUART_BITV;
}

/*********************************************************/
/*                   I2C Accelerometer                   */
/*********************************************************/

#define MMA7660FC_ADDR 0x98

uint8_t accel_read_reg(uint8_t addr)
{
	uint8_t ret;
	i2c_start_wait(MMA7660FC_ADDR + I2C_WRITE);
	i2c_write(addr);
	i2c_rep_start(MMA7660FC_ADDR + I2C_READ);
	ret = i2c_readNak();
	i2c_stop();
	return ret;
}

int8_t accel_read_sreg(uint8_t addr)
{
	int8_t ret = accel_read_reg(addr) & 0x3f;
	if (ret & 0x20)
		ret |= 0xc0;
	return ret;
}

void accel_write_reg(uint8_t addr, uint8_t data)
{
	i2c_start_wait(MMA7660FC_ADDR + I2C_WRITE);
	i2c_write(addr);
	i2c_write(data);
	i2c_stop();
}

void accel_setup(void)
{
	accel_write_reg(0x07, 0x00);

	accel_write_reg(0x05, 0x00);
	accel_write_reg(0x06, 0x00);
	accel_write_reg(0x08, 0x03);
	accel_write_reg(0x09, 0x00);
	accel_write_reg(0xA0, 0x00);

	accel_write_reg(0x07, 0x01);
}

/*********************************************************/
/*                        Servos                         */
/*********************************************************/

// num is 1 or 2, pos is approx. in range 0..256
void servo_pulse(uint8_t num, uint16_t pos)
{
	PORTB &= ~num;
	my_delay_us(550+pos*6);
	PORTB |= num;
}

void servo_setup(void)
{
	DDRB |= 3;
	PORTB |= 3;
}

/*********************************************************/
/*                         Main                          */
/*********************************************************/

int main(void)
{
	i2c_init();
	accel_setup();
	softuart_setup();
	servo_setup();

	uint8_t count = 0;
	while (1)
	{
		uint8_t xout = accel_read_sreg(0x00) + 0x20; // range: 0 .. 63
		uint8_t yout = accel_read_sreg(0x01) + 0x20;

		servo_pulse(1, xout * 4);
		servo_pulse(2, yout * 4);
		my_delay_ms(20);

		if (count++ % 64 == 0) {
			softuart_send_hex_byte(xout);
			softuart_send_byte(' ');
			softuart_send_hex_byte(yout);
			softuart_send_byte('\r');
			softuart_send_byte('\n');
		}
	}
	return 0;
}

