/*
File : timer_clock.h
*/
#ifndef TIMER_CLOCK_H
#define TIMER_CLOCK_H

#include "armv10_std.h" // For the LCD Functions
#include <stdint.h>
#include <time.h>

uint64_t get_tick(void);
void set_tick(uint64_t);

void init_clock(void);

void lcd_put_tick(void);

int8_t is_editing(void);

#define read_period 50

#endif
