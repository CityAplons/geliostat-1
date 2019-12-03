#ifndef SKMS5611_h
#define SKMS5611_h

#include "Arduino.h"
#include <Wire.h>

// typical I2C-Address of chip
#define I2C_ADR 0x77

// I2C commands of chip
#define SKMS5611_CMD_RESET	0x1E    // perform reset
#define SKMS5611_CMD_ADC_READ 0x00    // initiate read sequence
#define SKMS5611_CMD_ADC_CONV 0x40    // start conversion
#define SKMS5611_CMD_ADC_D1   0x00    // read ADC 1
#define SKMS5611_CMD_ADC_D2   0x10    // read ADC 2
#define SKMS5611_CMD_ADC_256  0x00    // set ADC oversampling ratio to 256
#define SKMS5611_CMD_ADC_512  0x02    // set ADC oversampling ratio to 512
#define SKMS5611_CMD_ADC_1024 0x04    // set ADC oversampling ratio to 1024
#define SKMS5611_CMD_ADC_2048 0x06    // set ADC oversampling ratio to 2048
#define SKMS5611_CMD_ADC_4096 0x08    // set ADC oversampling ratio to 4096
#define SKMS5611_CMD_PROM_RD  0xA0    // initiate readout of PROM registers

class SKMS5611
{
  protected:
	uint16_t C[8];
	long P;
	long TEMP;
	char i2caddr;
	TwoWire *_Wire;
	
	unsigned char send_cmd(unsigned char aCMD);
	int32_t read_adc(unsigned char aCMD);
	
  public:
    SKMS5611(TwoWire *aWire);
    void setI2Caddr(char aAddr);
    byte connect();
    
    void ReadProm();
    void Readout();

    unsigned int Calc_CRC4(unsigned char poly=0x30);
    unsigned int Read_CRC4();
    unsigned char CRCcodeTest();

    unsigned int Read_C(unsigned int index);
    
    long GetTemp();
    long GetPres();
};

#endif
