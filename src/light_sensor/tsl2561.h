#ifndef TSL2561_H
#define TSL2561_H

#include <string>

namespace TSL2561
{
    /*      Slave Addresses */
    const unsigned char     FLOAT       =	0x39;
    const unsigned char     GND         =	0x29;
    const unsigned char     VDD         =	0x49;

    /*      Register Addresses  */
    const unsigned char     CONTROL     =	0x00;
    const unsigned char     TIMING      =	0x01;
    const unsigned char     THRESH0     =	0x02;
    const unsigned char     THRESH1     =	0x03;
    const unsigned char     THRESH2     =	0x04;
    const unsigned char     THRESH3     =	0x05;
    const unsigned char     INTERRUPT   =	0x06;
    const unsigned char     ID          =	0x0A;
    const unsigned char     DATA0L      =	0x0C;
    const unsigned char     DATA0H      =	0x0D;
    const unsigned char     DATA1L      =	0x0E;
    const unsigned char     DATA1H      =	0x0F;

    /*      CMD Reg bits    */
    const unsigned char     CMD         =	0x80;
    const unsigned char     CLR_INT     =	0x40;
    const unsigned char     SMB_WRD     =	0x20;
    const unsigned char     BLK_R_WR    =	0x10;

    /*      Control Reg bits    */
    const unsigned char     PWR_UP      =	0x03;
    const unsigned char     PWR_DWN     =	0x00;

    /*      Timing Reg bits     */
    const unsigned char     GAIN        =	0x10;
    const unsigned char     MANUAL      =	0x08;
    const unsigned char     INTEG0      =	0x00;
    const unsigned char     INTEG1      =	0x01;
    const unsigned char     INTEG2      =	0x02;

    /*      Interrupt Control Register  bits    */
    const unsigned char     DISABLE     =	0x00;
    const unsigned char     LEVEL       =	0x10;
    const unsigned char     SMBALRT     =	0x20;
    const unsigned char     TEST        =	0x30;

    class LightSensor
    {
        private:
            int device;
            std::string device_name;
            unsigned char slave_addr;
        public:
            LightSensor(std::string device_name, unsigned char slave_addr = FLOAT);
            ~LightSensor();
            void init();
            unsigned int readADC(unsigned char ch);
            void setTiming(std::string gain, bool manual, std::string integration);
            void setInterrupt(std::string mode, unsigned char persist, unsigned short threshL, unsigned short threshH);
            void enableSensor();
            void disableSensor();
            std::string readID();
    };
}
#endif
