#include "pm5003.h"

//Check routine taken from
//http://web.mit.edu/6.115/www/miscfiles/amulet/amulet-help/xmodem.htm
unsigned int pm5003::calcrc(unsigned char *ptr, char count)
{
    unsigned int  crc;
    char i;
    crc = 0;
    while (--count >= 0)
    {
        crc = crc ^ (unsigned int) *ptr++ << 8;
        i = 8;
        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        } while(--i);
    }
    return (crc);
}

char pm5003::checkValue(unsigned char *thebuf, char leng)
{  
  char receiveflag=0;
  int receiveSum=0;

  for(int i=0; i<(leng-2); i++){
  receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;
 
  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))  //check the serial data 
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

int pm5003::transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  //PM01Val=((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
  PM01Val=((thebuf[9]<<8) + thebuf[10]); //count PM1.0 value of the air detector module
  return PM01Val;
}

//transmit PM Value to PC
int pm5003::transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  //PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
  PM2_5Val=((thebuf[11]<<8) + thebuf[12]);
  return PM2_5Val;
  }

//transmit PM Value to PC
int pm5003::transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  //PM10Val=((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module  
  PM10Val=((thebuf[13]<<8) + thebuf[14]);
  return PM10Val;
}

void pm5003::printAll(unsigned char *thebuf, char leng) {
	uint8_t count = 1;
	int val;
	for(int i=1; i<leng; i+=2) {
		Serial.print(count);
		Serial.print(" ");
		val = ((thebuf[i]<<8) + thebuf[i+1]);
		Serial.println(val);
		count++;
	}

	
}