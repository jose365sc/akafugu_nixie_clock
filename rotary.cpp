/*
 * The Akafugu Nixie Clock
 * (C) 2012-13 Akafugu Corporation
 *
 * This program is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 */

#include "global.h"

#include "rotary.h"
#include <avr/interrupt.h>

// rotary encoder
// NOTE: PB6 and PB7 are used for the oscillator on normal Arduino boards, and are not mapped to
//       I/O pin numbers, so we will use direct pin access
#define ROTARY_DDR  PORTB
#define ROTARY_PORT PORTB
#define ROTARY_1 PORTB6
#define ROTARY_2 PORTB7
#define ROTARY_1_PIN PINB6
#define ROTARY_2_PIN PINB7

////// interrupt handler for rotary encoder
//// increment/decrement s_rotary_raw_pos regarding pins state at each interrupt

// hold last state of pins
uint8_t s_encoder_state;
// Raw position of rotary encoder (4 ticks per click)
volatile int32_t s_rotary_raw_pos = 0;

#if defined(BOARD_STANDARD) || defined(BOARD_MK2) || defined(BOARD_MODULAR)
#  define POSITION_INCREMENT(v, n) (v) += (n)
#  define POSITION_DECREMENT(v, n) (v) -= (n)
#elif defined(BOARD_DIET)
#  define POSITION_INCREMENT(v, n) (v) -= (n)
#  define POSITION_DECREMENT(v, n) (v) += (n)
#endif // board type

// fixme: set flag to see if rotary encoder is moving or not
// Rotary encoder interrupt
// Based on code in this library http://www.pjrc.com/teensy/td_libs_Encoder.html
ISR( PCINT0_vect )
{
  uint8_t s = s_encoder_state & 3;
  if (PINB & _BV(ROTARY_1_PIN)) s |= 4;
  if (PINB & _BV(ROTARY_2_PIN)) s |= 8;

  switch (s) {
    case 0: case 5: case 10: case 15:
      break;
    case 1: case 7: case 8: case 14:
      POSITION_INCREMENT(s_rotary_raw_pos, 1); break;
    case 2: case 4: case 11: case 13:
      POSITION_DECREMENT(s_rotary_raw_pos, 1); break;
    case 3: case 12:
      POSITION_INCREMENT(s_rotary_raw_pos, 2); break;
    default:
      POSITION_DECREMENT(s_rotary_raw_pos, 2); break;
    }

  s_encoder_state = (s >> 2);
}

  static int Rotary::s_from = 0;
  static int Rotary::s_to = 1;
  static int Rotary::s_value_base = 0;
  static int Rotary::s_divider = 1;
  static int Rotary::s_saved_from = 0;
  static int Rotary::s_saved_to = 1;
  static int Rotary::s_saved_value = 0;
  static int Rotary::s_saved_divider = 1;

void Rotary::begin()
{
  // rotary encoder
  ROTARY_DDR &= ~(_BV(ROTARY_1));
  ROTARY_DDR &= ~(_BV(ROTARY_2));

  // enable pullups for all rotary encoder pins
  ROTARY_PORT |= _BV(ROTARY_1) | _BV(ROTARY_2); // enable pullup  
  
  // set up interrupt for rotary encoder pins
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT6);
  PCMSK0 |= (1 << PCINT7);

  // Initialize rotary encoder
  uint8_t s = 0;
  if (PINB & _BV(ROTARY_1_PIN)) s |= 1;
  if (PINB & _BV(ROTARY_2_PIN)) s |= 2;
  s_encoder_state = s;
}

bool Rotary::init(int from, int to, int current_value, int divider)
{
  if (from < to) { s_from = from; s_to = to; }
  else if (from > to) { s_from = to; s_to = from; }
  else { s_from = from; s_to = to + 1; }

  s_divider = divider;
  s_value_base = current_value;
  s_rotary_raw_pos = 0;
  
  return !(from == to || divider <= 0 || s_value_base < s_from || s_to <= s_value_base);
}

int Rotary::getValue()
{
  return ((s_value_base + (s_rotary_raw_pos + s_divider / 2) / s_divider) - s_from) % (s_to - s_from) + s_from;
}

void Rotary::incrementValue()
{
  s_rotary_raw_pos += s_divider;
}

void Rotary::decrementValue()
{
  s_rotary_raw_pos -= s_divider;
}

void Rotary::save()
{
  s_saved_from = s_from;
  s_saved_to = s_to;
  s_saved_value = getValue();
  s_saved_divider = s_divider;
}

void Rotary::restore()
{
  init(s_saved_from, s_saved_to, s_saved_value, s_saved_divider);
}