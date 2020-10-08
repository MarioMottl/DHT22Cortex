/*
File : dht22.h
*/
#ifndef DHT22_H
#define DHT22_H

#include "armv10_std.h" // For the LCD Functions
#include <stdint.h>
#include <time.h>
#include "timer_clock.h"
#include "uart.h"


/*
Define for easier usage
*/

#define GPIOB_IDR GPIOB_BASE + 2*sizeof(uint32_t)
#define SDA *((volatile unsigned long *)(BITBAND_PERI(GPIOB_ODR,0))) //PB0
#define SDA_IN *((volatile unsigned long *)(BITBAND_PERI(GPIOB_IDR,0))) //PB0 - Input

/*
Struct for the data received from the sensor
*/
typedef struct{
    float temperature;
    float humidity;
}dht22;

/*
Global variables
*/
static uint32_t cycles[80];
static uint8_t data[5];
static dht22 sensor;

/*
Prototypes of the functions
*/
void comm_init(void);
void comm_start(void);
int comm_read(void);
uint32_t excpectpulses(int v);
void init(void);

#endif
