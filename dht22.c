/*
File : dht22.c
*/
#include "dht22.h"

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "misc.h"

/*
Initial initialization of the Port for the one wire transmission
PUSH PULL
Futhermore the pin is supplied with a clock
*/
void comm_init(void){
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	GPIOB->CRL &= 0xFFFFFFF0;
	GPIOB->CRL |= 0x00000005;
	int i;
	//data storage gets cleared for initial usage
	for(i=0; i<sizeof(data)/sizeof(data[0]); i++)
	{
		data[i] = 0;
	}
	SDA = 1;
}

/*
Timing critical code!!
*/
void comm_start(void){
    SDA = 0;
	for (int i = 0;i<=80;i++){wait_10us();}
	SDA = 1;
}

/*

Timing critical code!!

DATA = 0
LOW 50us
High 26us
DATA = 1
LOW 50us
HIGH 70us

Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
microsecond low pulse followed by a variable length high pulse.  If the
high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
then it's a 1.  We measure the cycle count of the initial 50us low pulse
and use that to compare to the cycle count of the high pulse to determine
if the bit is a 0 (high state cycle count < low state cycle count), or a
1 (high state cycle count > low state cycle count). Note that for speed
all the pulses are read into a array and then examined in a later step.

*/
uint32_t excpectpulses(int v){
	uint32_t i = 0;
	//while(SDA_IN == v)
	while(SDA_IN == v)
	{
		i++;
	}
	return i;
}

int comm_read(void){
	// First expect a low signal for ~80 microseconds followed by a high signal
	// for ~80 microseconds again.
	if (excpectpulses(0) == 0){
		//DHT Timeout evtl DEBUG MESSAGE
		//usart1_send_string("DHT TIMEOUT LOW PHASE");
		//return 1;
	}
	if (excpectpulses(1) == 0){
		//DHT Timeout evtl DEBUG MESSAGE
		//usart1_send_string("DHT TIMEOUT HIGH PHASE");
		//return 1;
	}
	int i;
	for (i = 0; i < 80; i += 2){
		cycles[i] = excpectpulses(0);
		cycles[i+1] = excpectpulses(1);
	}
	// Timing critical code is now complete
    for (i = 0; i < 40; ++i) {
        uint32_t lowCycles = cycles[2 * i];
        uint32_t highCycles = cycles[2 * i + 1];
        data[i / 8] <<= 1;
		data[i / 8] &= ~0x01; // Clear last bit
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
        // High cycles are greater than 50us low cycle count, must be a 1.
        data[i / 8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }
	 if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)){
        //Checksummen fehler evtl DEBUG MESSAGE ?
		//usart1_send_string("DHT CHECKSUMME FEHLER");
		//return 1;
  }
	sensor.temperature = ((int)(data[2] & 0x7F)) << 8 | data[3];
	sensor.temperature *= 0.1;
	if (data[2] & 0x7F) {
		sensor.temperature *= -1;
	}
	sensor.humidity = ((int)(data[0])) << 8 | data[1];
    sensor.humidity *= 0.1;
	data[0] = data[1] = data[2] = data[3] = data[4] = 0;
	return 0;
}

void init(void){
	init_clock();
	usart1_init(9600);
	lcd_init();
    lcd_clear();
    comm_init();
}
