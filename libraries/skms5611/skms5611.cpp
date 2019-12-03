#include "SKMS5611.h"

SKMS5611::SKMS5611(TwoWire *aWire) : i2caddr(I2C_ADR) {
	_Wire=aWire;
}

void SKMS5611::setI2Caddr(char aAddr) {
	i2caddr=aAddr;
}

byte SKMS5611::send_cmd(byte aCMD)
{
  _Wire->beginTransmission(i2caddr);
  _Wire->write(aCMD);
  uint8_t ret=_Wire->endTransmission(false);
  return ret;
}

uint8_t SKMS5611::connect() {
	_Wire->begin();
	_Wire->beginTransmission(i2caddr);
	uint8_t ret=_Wire->endTransmission(true);
	return ret;
}

void SKMS5611::ReadProm() {
	send_cmd(SKMS5611_CMD_RESET);
	delay(3);
	  
	for(uint8_t i=0;i<8;i++) 
	{
	    C[i]=0x0000;
	    send_cmd(SKMS5611_CMD_PROM_RD+2*i);
	    _Wire->requestFrom(i2caddr, 2);

	    C[i] = _Wire->read() << 8 | _Wire->read();
		
	}
	
}

unsigned int SKMS5611::Calc_CRC4(unsigned char poly)
{
    int cnt;                   		// simple counter
    unsigned int n_rem;                 // CRC remainder
    unsigned int crc_read;              // original value of the CRC
    unsigned int l_pol = poly;
    unsigned char n_bit;

    l_pol = ( l_pol << 8 ) & 0xf000;	// shift bits and apply mask
    n_rem = 0x0000;

    crc_read = C[ 7 ];                  // save read RCR
    C[ 7 ] = ( 0xFF00 & ( C[ 7 ] ) );   // CRC byte is replaced by 0
    for ( cnt = 0; cnt < 16; cnt++ )    // operation is performed on bytes
    {// choose LSB or MSB
        if ( cnt % 2 == 1 ) n_rem ^= ( unsigned short ) ( ( C[ cnt >> 1 ] ) & 0x00FF );
        else n_rem ^= ( unsigned short ) ( ( C[ cnt >> 1 ] >> 8) & 0x00FF );

        for ( n_bit = 8; n_bit > 0; n_bit-- )
        {
            if ( n_rem & ( 0x8000 ) )
            {
            	n_rem = ( n_rem << 1 ) ^ l_pol;
            }
            else
            {
                n_rem = ( n_rem << 1 );
            }
        }
    }
    C[ 7 ] = crc_read;
    n_rem = (0x000F & (n_rem >> 12)); // final 4-bit remainder is CRC code
    return n_rem;
}

unsigned int SKMS5611::Read_CRC4()
{

    unsigned int crc_read = ( 0x000F & ( C[ 7 ] ) );
    return ( crc_read );
}

unsigned int SKMS5611::Read_C( unsigned int index)
{
    unsigned int retval = 0;
    if ( ( index >= 0) && ( index <= 7 ) )
        retval = C[ index ];
    return retval;
}

int32_t SKMS5611::read_adc(unsigned char aCMD)
{
  uint16_t value=0;
  
  send_cmd(SKMS5611_CMD_ADC_CONV+aCMD); // start DAQ and conversion of ADC data
  switch (aCMD & 0x0f)
  {
    case SKMS5611_CMD_ADC_256 : delayMicroseconds(900);
    break;
    case SKMS5611_CMD_ADC_512 : delay(3);
    break;
    case SKMS5611_CMD_ADC_1024: delay(4);
    break;
    case SKMS5611_CMD_ADC_2048: delay(6);
    break;
    case SKMS5611_CMD_ADC_4096: delay(10);
    break;
  }
  send_cmd(SKMS5611_CMD_ADC_READ); // read out values
  _Wire->requestFrom(i2caddr, 3);
  
  value = (_Wire->read() << 16) | (_Wire->read() << 8) | _Wire->read();
 
  return value;
}

void SKMS5611::Readout() {
	uint32_t D1=0, D2=0;
	
	int32_t dT;
	int64_t OFF;
	int64_t SENS;

	D2=read_adc(SKMS5611_CMD_ADC_D2+SKMS5611_CMD_ADC_4096);
	D1=read_adc(SKMS5611_CMD_ADC_D1+SKMS5611_CMD_ADC_4096);

	// calculate 1st order pressure and temperature (MS5607 1st order algorithm)
	dT=D2-C[5]*256;
	TEMP=(2000+(dT*C[6])/8388608);
	OFF=C[2]*65536+dT*C[4]/128;
	SENS=C[1]*32768+dT*C[3]/256;
	P=((D1*SENS)/2097152-OFF)/32768;
	 
	//perform higher order corrections
	int32_t T2=0.;
	int64_t OFF2, SENS2;
	OFF2 = 0;
	SENS2 = 0;
	if(TEMP<2000) {
	  T2=dT*dT/2147483648;
	  OFF2=5*(TEMP-2000)*(TEMP-2000)/2;
	  SENS2=5*(TEMP-2000)*(TEMP-2000)/4;
	  if(TEMP<-1500) {
	    OFF2+=7*(TEMP+1500)*(TEMP+1500);
	    SENS2+=11*(TEMP+1500)*(TEMP+1500)/2;
	  }
	}
	  
	TEMP-=T2;
	OFF-=OFF2;
	SENS-=SENS2;
	P=(((D1*SENS)/2097152-OFF)/32768);
		
}

long SKMS5611::GetTemp() {
	return TEMP;
}

long SKMS5611::GetPres() {
	return P;
}

unsigned char SKMS5611::CRCcodeTest(){
	unsigned int nprom[] = {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x4500}; //expected output is 0xB
	for(uint8_t i=0;i<8;i++) {
		C[i] = nprom[i];
	}
	unsigned char crc = Calc_CRC4(); //expected output is 0xB
	ReadProm();
	return crc;
}

