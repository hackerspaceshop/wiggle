// simple test for the rng:
// clang -o rngtest rngtest.c && ./rngtest | sort -nu | wc -l

#include <stdio.h>
#include <stdint.h>

// LFSR polynomial coefficients: 0001001
// we iterate over the 127 bit long max. lenght sequency in 8 bit steps
uint8_t rng() {
	static uint8_t state = 1;
	for (uint8_t i = 0; i < 8; i++)
		state = (state >> 1) | ((!(state&8) != !(state&1)) << 6);
	return state;
}

int main()
{
	for (int i = 0; i < 127; i++)
		printf("%d\n", rng());
	return 0;
}
