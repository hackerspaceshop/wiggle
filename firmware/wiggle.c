/*

State: TESTING / ALPHA

wiggle 0.1 firmware from overflo
 
based on example code from clifford wolf

License to be decided yet.

*/

#undef DEBUG_RS232

#define SORRYPOS   1
#define HOMEPOS   10
#define ACTIVEPOS 40
#define DANCE_LR_COUNT 2

#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"

/*********************************************************/
/*                    Delay functions                    */
/*********************************************************/

#if 0
static void my_delay_us(uint16_t delay)
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

static void my_delay_ms(uint16_t delay)
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

#ifdef DEBUG_RS232

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

static void softuart_send_byte(uint8_t byte)
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

static void softuart_send_hex_nibble(uint8_t nibble)
{
	if (nibble < 10)
		softuart_send_byte('0' + nibble);
	else
		softuart_send_byte('A' + nibble - 10);
}

static void softuart_send_hex_byte(uint8_t byte)
{
	softuart_send_hex_nibble(byte >> 4);
	softuart_send_hex_nibble(byte & 0x0f);
}

static void softuart_setup(void)
{
	// convert PORTx to DDRx and mark pin as output
	SOFTUART_DDR |= SOFTUART_BITV;
	SOFTUART_PORT |= SOFTUART_BITV;
}

#endif

/*********************************************************/
/*                   I2C Accelerometer                   */
/*********************************************************/

#define MMA7660FC_ADDR 0x98

static uint8_t accel_read_reg(uint8_t addr)
{
	uint8_t ret;
	i2c_start_wait(MMA7660FC_ADDR + I2C_WRITE);
	i2c_write(addr);
	i2c_rep_start(MMA7660FC_ADDR + I2C_READ);
	ret = i2c_readNak();
	i2c_stop();
	return ret;
}

static int8_t accel_read_sreg(uint8_t addr)
{
	int8_t ret = accel_read_reg(addr) & 0x3f;
	if (ret & 0x20)
		ret |= 0xc0;
	return ret;
}

static void accel_write_reg(uint8_t addr, uint8_t data)
{
	i2c_start_wait(MMA7660FC_ADDR + I2C_WRITE);
	i2c_write(addr);
	i2c_write(data);
	i2c_stop();
}

static void accel_setup(void)
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
static void servo_pulse(uint8_t num, uint16_t pos)
{
	PORTB &= ~num;
	my_delay_us(550+pos*6);
	PORTB |= num;
}

static void servo_setup(void)
{
	DDRB |= 3;
	PORTB |= 3;
}

static void move_ear(uint8_t servo, uint8_t degree)
{
	// send servo pulse to servo1 or servo2 10 times
	for (uint8_t i = 0; i < 10; i++) {
		servo_pulse(servo, degree * 4);
		my_delay_ms(20);
	}
}

// LFSR polynomial coefficients: 0001001
// we iterate over the 127 bit long max. lenght sequency in 8 bit steps
static uint8_t rng() {
	static uint8_t state = 1;
	for (uint8_t i = 0; i < 8; i++)
		state = (state >> 1) | ((!(state&8) != !(state&1)) << 6);
	return state;
}

static void eardance()
{
	for (uint8_t i = 0; i < 10; i++) {
		uint8_t v = rng() & 63;
		move_ear(1, v);
		move_ear(2, 64-v);
	}
	my_delay_ms(200);
}

static void wiggle(uint8_t servo)
{
	move_ear(servo, HOMEPOS);

	move_ear(servo, ACTIVEPOS);
	my_delay_ms(50);

	move_ear(servo, HOMEPOS);
}

// both servos reset to zero.
static void reset_ears()
{
	move_ear(1, HOMEPOS);
	move_ear(2, HOMEPOS);
}

// ohren hängen lassen :(
static void sorry_ears()
{
	move_ear(1, SORRYPOS);
	move_ear(2, SORRYPOS);
}


/*********************************************************/
/*                         Main                          */
/*********************************************************/

int main(void)
{
	// LED ON
	DDRB |= _BV(2);
	PORTB |= _BV(2);
	my_delay_ms(500);

	// LED OFF
	DDRB &= ~_BV(2);
	PORTB &= ~_BV(2);
	my_delay_ms(500);

	i2c_init();
	accel_setup();
	servo_setup();
	reset_ears();

#ifdef DEBUG_RS232
	uint8_t cycle_count = 0;
	softuart_setup();
	softuart_send_byte('-');
	softuart_send_byte('-');
	softuart_send_byte('-');
	softuart_send_byte('\r');
	softuart_send_byte('\n');
#endif

	// this can be inside one byte.. 
	int8_t last_wiggle = 0;
	uint8_t was_sorry = 0;
	uint8_t head_forward = 0;
	uint8_t head_backward = 0;
	uint8_t head_left = 0;
	uint8_t head_right = 0;
	uint8_t left_right_count = 0;

	while (1)
	{
		int8_t xout = accel_read_sreg(0x00); // range: -32 .. 31
		int8_t yout = accel_read_sreg(0x01);

		head_forward  = (xout > +10 && head_forward  < 200) ? head_forward+1  : 0;
		head_backward = (xout < -10 && head_backward < 200) ? head_backward+1 : 0;
		head_left     = (yout > +10 && head_left     < 200) ? head_left+1     : 0;
		head_right    = (yout < -10 && head_right    < 200) ? head_right+1    : 0;

#ifndef DEBUG_RS232
		if (head_forward > 50) {
			was_sorry = 1;
			sorry_ears();
			last_wiggle = 0;
		} else {
			if (head_backward > 10) {
				reset_ears();
				last_wiggle = 0;
			} else if (head_left > 10) {
				if (last_wiggle < 0 && ++left_right_count >= DANCE_LR_COUNT)
					eardance();
				else
					wiggle(1);
				last_wiggle = +100;
			} else if (head_right > 10) {
				if (last_wiggle > 0 && ++left_right_count >= DANCE_LR_COUNT)
					eardance();
				else
					wiggle(2);
				last_wiggle = -100;
			} else {
				if (was_sorry)
					reset_ears();
				my_delay_ms(20);
				was_sorry = 0;
			}
		}

		if (last_wiggle != 0)
			last_wiggle += last_wiggle > 0 ? -1 : +1;
		else
			left_right_count = 0;
#else
		if (cycle_count++ % 64 == 0) {
			softuart_send_hex_byte(xout);
			softuart_send_byte(' ');
			softuart_send_hex_byte(yout);
			softuart_send_byte(' ');
			softuart_send_hex_byte(head_forward);
			softuart_send_byte(' ');
			softuart_send_hex_byte(head_backward);
			softuart_send_byte(' ');
			softuart_send_hex_byte(head_left);
			softuart_send_byte(' ');
			softuart_send_hex_byte(head_right);
			softuart_send_byte('\r');
			softuart_send_byte('\n');
		}
#endif
	}

	return 0;
}
