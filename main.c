/*
File : main.c
*/
#include "dht22.h"

#include <stdlib.h>

uint64_t last_sensor_output = 0;

int main(void)
{
	char buf[256];
	init();

	SystemCoreClockUpdate(); // Rechne Systemclock neu aus
	sprintf(buf, "Sysclock: %d\r\n", SystemCoreClock); // Gib systemclock auf USART1 aus
	usart1_send_string(buf); // Gib systemclock auf USART1 aus

	usart1_send_string("ready to work\r\n"); // Initialisierung beendet
	//sprintf(buffer,"Temperature : %2.2f, Humidity  : %2.2f \r\n",sensor.temperature,sensor.humidity);
	while(1)
	{
		lcd_put_tick();
		if((get_tick() >= last_sensor_output + read_period)) // == 1 for offset after reading sensor data
		{
			last_sensor_output = get_tick();
			sprintf(buf,"T:%2.2f, H:%2.2f \r\n",sensor.temperature,sensor.humidity);
			lcd_set_cursor(1,0);
			lcd_put_string(buf);
			usart1_send_string(buf);
		}
	}
}
