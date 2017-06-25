#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <iostream>
#include <algorithm>
#include "tsl2561.h"

namespace TSL2561
{
    /*  Constructor
    *   device: file name of I2C device in filesystem.
    *   slave_addr: the slave address of the sensor (TSL2561::FLOAT, TSL2561::GND, TSL2561::VDD)
    */
    LightSensor::LightSensor(std::string device_name, unsigned char slave_addr)
    {
        this->device_name = device_name;
        this->slave_addr = slave_addr;
	this->device = 0;
    }

    /*  Destructor
    *   Closes file connection.
    */
    LightSensor::~LightSensor()
    {
        if(this->device)
        {
            close(this->device);
        }
    }

    /*  Initialize
    *   Should be called before anything else. Opens a connection to the device
    *   file in the OS, and powers on the ADCs in the sensor.
    */
    void LightSensor::init()
    {
        unsigned char buffer[2];
        unsigned char sumw = 0;

        this->device = open(this->device_name.c_str(), O_RDWR);
        if(this->device < 0)
        {
            std::cerr << "Failed to open the I2C bus!" << std::endl;
            exit(1);
        }

        if(ioctl(this->device, I2C_SLAVE, this->slave_addr) < 0)
        {
            std::cerr << "Failed to acquire the I2C bus to talk to slave device" << std::endl;
        }

        buffer[0] = CMD | CONTROL;
        buffer[1] = PWR_UP;

        sumw += write(this->device, buffer, 1);
        sumw += write(this->device, &buffer[1], 1);
        if(sumw < 2)
        {
            std::cerr << "Failed to write to the I2C bus!" << std::endl;
            exit(1);
        }

        std::cout << "TSL2561 has powered up!" << std::endl;
    }

    /*  Read an ADC channel
    *   ch: The channel number to read (0 or 1)
    *   Reads the data from the ADCs. Can be used later to convert to lux if needed.
    */
    unsigned int LightSensor::readADC(unsigned char ch)
    {
        unsigned char buffer[3];
        unsigned char sumw = 0;
        unsigned char sumr = 0;
        unsigned char data[2];

        if(ch > 1 || ch < 0)
        {
            std::cerr << "Invalid channel. Select CH0 or CH1" << std::endl;
            exit(1);
        }

        buffer[0] = CMD | CONTROL;
        buffer[1] = (ch == 0) ? (CMD | DATA0L):(CMD | DATA1L);
        buffer[2] = (ch == 0) ? (CMD | DATA0H):(CMD | DATA1H);
        sumw += write(this->device, &buffer[0], 1);
        sumw += write(this->device, &buffer[1], 1);
        sumr += read(this->device, data, 1);
        sumw += write(this->device, &buffer[2], 1);
        sumr += read(this->device, &data[1], 1);

        if(sumw < 3)
        {
            std::cerr << "Failed to write to the I2C bus!" << std::endl;
            exit(1);
        }

        if(sumr < 2)
        {
            std::cerr << "Failed to read from the I2C bus!" << std::endl;
        }

        std::cout << "Successfully read from ADC" << ch << "." << std::endl;
        return (data[1] << 8) | data[0];
    }

    /*  Set Timing register
    *   gain: Low gain (1x) or high gain (16x)
    *   manual: enable/disable manual Timing
    *   integration: sets the integration time
    *       short = 13.7 ms
    *       medium = 101 ms
    *       long = 402 ms
    *   Sets the timing register for the ADCs. Longer integration time means
    *   more accurate reading from the sensor. Higher gain will allow lower
    *   light levels to be detected, but higher levels will be clipped.
    */
    void LightSensor::setTiming(std::string gain, bool manual, std::string integration)
    {
        unsigned char buffer[2];
        unsigned char sumw = 0;

        std::transform(gain.begin(), gain.end(), gain.begin(), tolower);
        if(gain.compare("low") == 0)
        {
            buffer[1] &= ~GAIN;
        }
        else if(gain.compare("high") == 0)
        {
            buffer[1] |= GAIN;
        }
        else
        {
            std::cerr << "Invalid gain setting. Select LOW or HIGH" << std::endl;
            exit(1);
        }

        buffer[0] = CMD | TIMING;
        if(manual)
        {
            buffer[1] |= MANUAL;
        }
        else
        {
            buffer[1] &= ~MANUAL;
        }

        std::transform(integration.begin(), integration.end(), integration.begin(), tolower);
        if(integration.compare("short") == 0)
        {
            buffer[1] |= INTEG0;
        }
        else if(integration.compare("medium") == 0)
        {
            buffer[1] |= INTEG1;
        }
        else if(integration.compare("long") == 0)
        {
            buffer[1] |= INTEG2;
        }
        else
        {
            std::cerr << "Invalid integration time! Use SHORT, MEDIUM, LONG" << std::endl;
            exit(1);
        }

        sumw += write(this->device, buffer, 1);
        sumw += write(this->device, &buffer[1], 1);
        if(sumw < 2)
        {
            std::cerr << "Failed to write to I2C bus!" << std::endl;
            exit(1);
        }

        std::cout << "Wrote to timing register!" << std::endl;
    }

    /*  Set Interrupt control and threshold registers.
    *   mode: interrupt mode
    *       disable - turn off interrupts
    *       level - threshold level interrupts
    *       smb - SMB Alert compatible threshold level interrupts
    *       test - sets interrupt immediately and functions as level mode after
    */
    void LightSensor::setInterrupt(std::string mode, unsigned char persist, unsigned short threshL, unsigned short threshH)
    {
        unsigned char buffer[5];
	unsigned char sumw = 0;

        buffer[10] = CMD | TIMING;

        std::transform(mode.begin(), mode.end(), mode.begin(), tolower);
        if(mode.compare("disable") == 0)
        {
            buffer[1] |= DISABLE;
        }
        else if(mode.compare("level") == 0)
        {
            buffer[1] |= LEVEL;
        }
        else if(mode.compare("alert") == 0)
        {
            buffer[1] |= SMBALRT;
        }
        else if(mode.compare("test") == 0)
        {
            buffer[1] |= TEST;
        }
        else
        {
            std::cerr << "Invalid interrupt mode! Use DIABLE, LEVEL, SMB, TEST" << std::endl;
        }

        if(!(persist > 0 && persist < 0x0f))
        {
            std::cerr << "Invalid persistance setting! Use an integer between 0-15" << std::endl;
            exit(1);
        }

        buffer[1] |= persist;
        sumw += write(this->device, buffer, 1);
        sumw += write(this->device, &buffer[1], 1);

        if(sumw < 2)
        {
            std::cerr << "Failed to write to interrupt control register!" << std::endl;
            exit(1);
        }

        buffer[2] = CMD | THRESH0;
        buffer[3] = threshL & 0xff;
        buffer[4] = CMD | THRESH1;
        buffer[5] = (threshL >> 8) & 0xff;
        buffer[6] = CMD | THRESH2;
        buffer[7] = threshH & 0xff;
        buffer[8] = CMD | THRESH3;
        buffer[9] = (threshH >> 8) & 0xff;

        sumw += write(this->device, &buffer[2], 1);
        sumw += write(this->device, &buffer[3], 1);
        sumw += write(this->device, &buffer[4], 1);
        sumw += write(this->device, &buffer[5], 1);
        sumw += write(this->device, &buffer[6], 1);
        sumw += write(this->device, &buffer[7], 1);
        sumw += write(this->device, &buffer[8], 1);
        sumw += write(this->device, &buffer[9], 1);

        if(sumw < 10)
        {
            std::cerr << "Failed to write to threshold registers!" << std::endl;
            exit(1);
        }

        std::cout << "Successfully set the interrupt thresholds and persistance" << std::endl;
    }

    /*  Enable ADCs
    *   Applies power to the ADCs and begins integration.
    */
    void LightSensor::enableSensor()
    {
        unsigned char buffer[2];
        unsigned char sumw = 0;

        buffer[0] = CMD | CONTROL;
        buffer[1] = PWR_UP;

        sumw += write(this->device, buffer, 1);
        sumw += write(this->device, &buffer[1], 1);

        if(sumw < 2)
        {
            std::cerr << "Failed to write the control register!" << std::endl;
            exit(1);
        }

        std::cout << "Device power re-enabled" << std::endl;
    }

    /*  Disables ADCs
    *   Stops integration and removes power from the ADCs
    */
    void LightSensor::disableSensor()
    {
        unsigned char buffer[2];
        unsigned char sumw = 0;

        buffer[0] = CMD | CONTROL;
        buffer[1] = PWR_DWN;

        sumw += write(this->device, buffer, 1);
        sumw += write(this->device, &buffer[1], 1);

        if(sumw < 2)
        {
            std::cerr << "Failed to write the control register!" << std::endl;
            exit(1);
        }

        std::cout << "Device power disabled" << std::endl;
    }

    /*  Read device ID
    *   Reads the partnumber and device ID for testing.
    */
    std::string LightSensor::readID()
    {
        unsigned char buffer;
        unsigned char sumw = 0;
        unsigned char sumr = 0;
        unsigned char data;
        std::string rev;
        std::string part;

        buffer = CMD | ID;
        sumw += write(this->device, &buffer, 1);
        sumr += read(this->device, &data, 1);

        if(sumr < 1)
        {
            std::cerr << "Failed to read from ID register" << std::endl;
        }

        if(sumw < 1)
        {
            std::cerr << "Failed to write the ID register!" << std::endl;
            exit(1);
        }

        if((data & 0xf0) == 0)
        {
            part = "TSL2560";
        }
        else if((data & 0xf0) == 1)
        {
            part = "TLS2561";
        }

        rev = std::to_string(data & 0x0f);

        std::cout << "Part number: " << part << " rev: " << rev << std::endl;
    }
}
