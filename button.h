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

#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdbool.h>

struct BUTTON_STATE
{
	bool b1_keydown : 1;
	bool b1_keyup : 1;
	bool b1_repeat : 1;
	bool b2_keydown : 1;
	bool b2_keyup : 1;
	bool b2_repeat : 1;
	bool b3_keydown : 1;
	bool b3_keyup : 1;
	bool b3_repeat : 1;
	bool both_held : 1;
	bool none_held : 1;
};

void initialize_button(int8_t pin1, int8_t pin2);
void get_button_state(struct BUTTON_STATE* buttons);
void button_timer(void);

#endif
