#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile unsigned int Tick;   // 100KHz pulse
volatile unsigned int sPulse;   // Servo pulse variable
volatile unsigned int Tick_20ms;   // Servo frame variable

int main (void)
{

   sei(); //  Enable global interrupts
   DDRB |= (1<<PB0) | (1<<PB1); // PB0 and PB1 as outputs
   TCCR0A |= (1<<WGM01); // Configure timer 1 for CTC mode
   TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
   OCR0A = 95; // Set CTC compare value
   TCCR0B |= (1<<CS00); // No prescaler
   Tick = 0;
   sPulse = 100;
   Tick_20ms = 0;

   while(1)
   {
      for(int i=0; i<=100; i++)   // Servo teste cycling
      {
         sPulse = sPulse + 1;
         _delay_loop_2(10000);
      }

      sPulse = 200;

      for(int i=0; i<=100; i++)
      {
         sPulse = sPulse - 1;
         _delay_loop_2(10000);
      }
      sPulse = 100;
   }
}

ISR(TIM0_COMPA_vect)   // 100 KHz interrupt frequency
{
   if(Tick >= 2000)   // One servo frame (20ms) completed
      {
         Tick = 0;
        Tick_20ms = Tick_20ms + 1;
      }

   Tick = Tick + 1;
   if(Tick <= sPulse)   // Generate servo pulse
   {
      PORTB |= (1<<PB0);   // Servo pulse high
   }
   else
   {
      PORTB &= ~(1<<PB0);   // Servo pulse low
   }
} 
