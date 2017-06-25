#include "tsl2561.h"
#include <iostream>
#include <string>

int main()
{
	TSL2561::LightSensor i2c("/dev/i2c-1", TSL2561::FLOAT);
	i2c.init();
	
	i2c.readID();
	
	i2c.setTiming("low", false, "long");
	i2c.setInterrupt("level", 2, 1000, 2000);

	std::cout << "Attempting to read ADC0" << std::endl;
	unsigned int data = i2c.readADC(0);

	std::cout << "ADC0: " << std::to_string(data) << std::endl;
	return 0;
}
