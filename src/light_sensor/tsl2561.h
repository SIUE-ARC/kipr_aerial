#ifndef TSL2561_H
#define TSL2561_H

#include <string>

namespace TSL2561
{
    /*      Operation bits  */
    #define     START       0x00
    #define     STOP        0xFF
    #define     WR          0x00
    #define     RD          0x01

    /*      Slave Addresses */
    #define     FLOAT       0x39
    #define     GND         0x29
    #define     VDD         0x49

    /*      Register Addresses  */
    #define     CONTROL     0x00
    #define     TIMING      0x01
    #define     THRESH0     0x02
    #define     THRESH1     0x03
    #define     THRESH2     0x04
    #define     THRESH3     0x05
    #define     INTERRUPT   0x06
    #define     CRC         0x08
    #define     ID          0x0A
    #define     DATA0L      0x0C
    #define     DATA0H      0x0D
    #define     DATA1L      0x0E
    #define     DATA1H      0x0F

    /*      CMD Reg bits    */
    #define     CMD         0x80
    #define     CLR_INT     0x40
    #define     SMB_WRD     0x20
    #define     BLK_R_WR    0x10

    /*      Control Reg bits    */
    #define     PWR_UP      0x03
    #define     PWR_DWN     0x00

    /*      Timing Reg bits     */
    #define     GAIN        0x10
    #define     MANUAL      0x08
    #define     INTEG0      0x00
    #define     INTEG1      0x01
    #define     INTEG2      0x02

    /*      Interrupt Control Register  bits    */
    #define     DISABLE     0x00
    #define     LEVEL       0x10
    #define     SMBALRT     0x20
    #define     TEST        0x30

    class LightSensor
    {
        private:
            FILE* device;
            string device_name;
            unsigned char slave_addr;
        public:
            LightSensor(string device_name, unsigned char slave_addr = FLOAT);
            ~LightSensor();
            void init();
            unsigned int readADC(unsigned char ch);
            void setTiming(std::string gain, bool manual, std::string integration);
            void setInterrupt(std:string mode, unsigned char persist, unsigned short threshL, unsigned short threshH)
            void enableSensor();
            void disableSensor();
            std::string readID();
    }
}
#endif
