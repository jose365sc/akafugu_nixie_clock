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
#ifndef ROTARY_H_
#define ROTARY_H_

#include <stdbool.h>
#include <avr/io.h>

class Rotary {
public:
  static void begin();
  static bool init(int from, int to, int current, int divider);
  static int getValue();
  static void incrementValue();
  static void decrementValue();
  static const int ticksPerRotation = 4 * 12;
  static void save();
  static void restore();
  static bool isMoved();
  static void clearMoved();
private:
  static int s_from;
  static int s_to;
  static int s_value_base;
  static int s_divider;
  static int s_saved_from;
  static int s_saved_to;
  static int s_saved_value;
  static int s_saved_divider;
};

#endif // ROTARY_H_

