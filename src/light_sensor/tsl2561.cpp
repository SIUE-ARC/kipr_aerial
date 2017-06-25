#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <iostream>
#include "tsl2561.h"

namespace TSL2561
{
    /*  Constructor
    *   device: file name of I2C device in filesystem.
    *   slave_addr: the slave address of the sensor (0x39, 0x29, 0x49)
    *   device: a file handle for the I2C device. Initialize to null
    *   note - please do not initialize device file handle, use init() instead.
    */
    LightSensor::LightSensor(std::string device_name, unsigned char slave_addr)
    {
        this.device_name = device_name;
        this.slave_addr = slave_addr;
        this.device = null;
    }

    /*  Destructor
    *   Closes file connection.
    */
    LightSensor::~LightSensor()
    {
        if(this.device)
        {
            close(this.device);
        }
    }

    /*  Initialize
    *   Should be called before anything else. Opens a connection to the device
    *   file in the OS, and powers on the ADCs in the sensor.
    */
    void LightSensor::init()
    {
        unsigned char byte[2];
        unsigned char sumw = 0;

        this.device = open(this.device_name, O_RDWR);
        if(this.device < 0)
        {
            cerr << "Failed to open the I2C bus!" << endl;
            return;
        }

        if(ioctl(this.device, I2C_SLAVE, this.slave_addr) < 0)
        {
            cerr << "Failed to acquire the I2C bus to talk to slave device" << endl;
        }

        buffer[0] = CMD | CONTROL;
        buffer[1] = PWR_UP;

        sumw += write(this.device, buffer, 1);
        sumw += write(this.device, &buffer[1], 1);
        if(sumw < 2)
        {
            cerr << "Failed to write to the I2C bus!" << endl;
            return;
        }

        cout << "TSL2561 has powered up!" << endl;
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
            cerr << "Invalid channel. Select CH0 or CH1" << endl;
            return;
        }

        buffer[0] = CMD | CONTROL;
        buffer[1] = (ch == 0) ? (CMD | DATA0L):(CMD | DATA1L);
        buffer[2] = (ch == 0) ? (CMD | DATA0H):(CMD | DATA1H);
        sumw += write(this.device, &buffer[0], 1);
        sumw += write(this.device, &buffer[1], 1);
        sumr += read(this.device, data, 1);
        sumw += write(this.device, &buffer[2], 1);
        sumr += read(this.device, &data[1], 1)

        if(sumw < 3)
        {
            cerr << "Failed to write to the I2C bus!" << endl;
            return;
        }

        if(sumr < 2)
        {
            cerr << "Failed to read from the I2C bus!" << endl;
        }

        cout << "Successfully read from ADC" << ch << "." << endl;
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

        std::transform(gain.begin(), gain.end(), gain.begin()::tolower);
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
            cerr << "Invalid gain setting. Select LOW or HIGH" << endl;
            return;
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

        std::transform(integration.begin(), integration.end(), integration.begin()::tolower);
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
            cerr << "Invalid integration time! Use SHORT, MEDIUM, LONG" << endl;
            return;
        }

        sumw += write(this.device, buffer, 1);
        sumw += write(this.device, &buffer[1], 1);
        if(sumw < 2)
        {
            cerr << "Failed to write to I2C bus!" << endl;
            return;
        }

        cout << "Wrote to timing register!" << endl;
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

        buffer[10] = CMD | TIMING;

        std::transform(mode.begin(), mode.end(), mode.begin()::tolower);
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
            cerr << "Invalid interrupt mode! Use DIABLE, LEVEL, SMB, TEST" << endl;
        }

        if(!(persist > 0 && persist < 0x0f))
        {
            cerr << "Invalid persistance setting! Use an integer between 0-15" << endl;
            return;
        }

        buffer[1] |= persist;
        sumw += write(this.device, buffer, 1);
        sumw += write(this.device, &buffer[1], 1);

        if(sumw < 2)
        {
            cerr << "Failed to write to interrupt control register!" << endl;
            return;
        }

        buffer[2] = CMD | THRESH0;
        buffer[3] = threshL & 0xff;
        buffer[4] = CMD | THRESH1;
        buffer[5] = (threshL >> 8) & 0xff;
        buffer[6] = CMD | THRESH2;
        buffer[7] = threshH & 0xff;
        buffer[8] = CMD | THRESH3;
        buffer[9] = (threshH >> 8) & 0xff;

        sumw += write(this.device, &buffer[2], 1);
        sumw += write(this.device, &buffer[3], 1);
        sumw += write(this.device, &buffer[4], 1);
        sumw += write(this.device, &buffer[5], 1);
        sumw += write(this.device, &buffer[6], 1);
        sumw += write(this.device, &buffer[7], 1);
        sumw += write(this.device, &buffer[8], 1);
        sumw += write(this.device, &buffer[9], 1);

        if(sumw < 10)
        {
            cerr << "Failed to write to threshold registers!" << endl;
            return;
        }

        cout << "Successfully set the interrupt thresholds and persistance" << endl;
    }

    /*  Enable ADCs
    *   Applies power to the ADCs and begins integration.
    */
    void LightSensor::enable()
    {
        unsigned char buffer[2];
        unsigned char sumw = 0;

        buffer[0] = CMD | CONTROL;
        buffer[1] = PWR_UP;

        sumw += write(this.device, buffer, 1);
        sumw += write(this.device, &buffer[1], 1);

        if(sumw < 2)
        {
            cerr << "Failed to write the control register!" << endl;
            return;
        }

        cout << "Device power re-enabled" << endl;
    }

    /*  Disables ADCs
    *   Stops integration and removes power from the ADCs
    */
    void LightSensor::disable()
    {
        unsigned char buffer[2];
        unsigned char sumw = 0;

        buffer[0] = CMD | CONTROL;
        buffer[1] = PWR_DWN;

        sumw += write(this.device, buffer, 1);
        sumw += write(this.device, &buffer[1], 1);

        if(sumw < 2)
        {
            cerr << "Failed to write the control register!" << endl;
            return;
        }

        cout << "Device power disabled" << endl;
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
        sumw += write(this.device, &buffer, 1);
        sumr += read(this.device, &data, 1);

        if(sumr < 1)
        {
            cerr << "Failed to read from ID register" << endl;
        }

        if(sumw < 1)
        {
            cerr << "Failed to write the ID register!" << endl;
            return;
        }

        if((data & 0xf0) == 0)
        {
            part = "TSL2560";
        }
        else if((data & 0xf0) == 1)
        {
            part = "TLS2561"
        }

        rev = std::to_string(data & 0f);

        cout << "Part number: " << part << " rev: " << rev << endl;
    }
}
