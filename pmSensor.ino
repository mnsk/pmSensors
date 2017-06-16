//pmSensor.ino
#include <AltSoftSerial.h>
#include <BME280_I2C.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "pm5003.h"

#define DEBUG_ON

#define roundInt(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
#define roundChar(x) ((x)>=0?(char)((x)+0.5):(char)((x)-0.5))

#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
#define OUTPUT_LENG 40

BME280_I2C bme;
AltSoftSerial altSerial;
pm5003 pm;

union flt
{
  float f;
  unsigned char bytes[4];
}p,e;

void floatToBytes (unsigned char *buf, byte &pos, float data) {

	p.f = data;

	buf[pos++] = p.bytes[3];
	buf[pos++] = p.bytes[2];
	buf[pos++] = p.bytes[1];
	buf[pos++] = p.bytes[0];

	#ifdef DEBUG_ON
	for (int i=0; i<4; i++)
	{
	 Serial.print(p.bytes[i], HEX); // Print the hex representation of the float
	 Serial.print(' ');
	}
	Serial.println();

  	e.bytes[0] = buf[pos-1];
  	e.bytes[1] = buf[pos-2];
  	e.bytes[2] = buf[pos-3];
  	e.bytes[3] = buf[pos-4];

  	Serial.print("Value: ");
	Serial.println(e.f);
	#endif
}

#ifdef DEBUG_ON
void printSerialData(unsigned char *buf) {
	byte count = 0;
	Serial.print(" ");
	Serial.println(buf[0],HEX);
	Serial.print(count++);
	Serial.print(" ");
	Serial.println(buf[1],DEC);
	unsigned int val;
	for(int i=2; i<25; i+=2) {
		Serial.print(count);
		Serial.print(" ");
		val = (unsigned int)((buf[i]<<8) + buf[i+1]);
		Serial.println(val);
		count++;
	}
	Serial.print(count++);
	Serial.print(" ");
	e.bytes[3] = buf[26];
  	e.bytes[2] = buf[27];
  	e.bytes[1] = buf[28];
  	e.bytes[0] = buf[29];
	Serial.println(e.f);
	Serial.print(count++);
	Serial.print(" ");
	e.bytes[3] = buf[30];
  	e.bytes[2] = buf[31];
  	e.bytes[1] = buf[32];
  	e.bytes[0] = buf[33];
	Serial.println(e.f);
	Serial.print(count++);
	Serial.print(" ");
	e.bytes[3] = buf[34];
  	e.bytes[2] = buf[35];
  	e.bytes[1] = buf[36];
  	e.bytes[0] = buf[37];
	Serial.println(e.f);
	Serial.print(count++);
	Serial.print(" ");
	Serial.println((unsigned int)((buf[38]<<8) + buf[39]),HEX);

}
#endif

bool serialWrite(unsigned char *pmBuf, float temp, float hum, float press) {
	unsigned char buf[OUTPUT_LENG];
	byte i=0;

	buf[i++] = (unsigned char)pmBuf[0]; 	//0 byte = start character
	buf[i++] = (unsigned char)((pmBuf[1]<<8) + pmBuf[2]);	//1 byte = frame length

	//2-25 byte = Data 1 to Data 12
	for(i; i<=25; i++)
		buf[i] = pmBuf[i+1];

	//26-37 bytes
	floatToBytes(buf,i,temp);
	floatToBytes(buf,i,hum);
	floatToBytes(buf,i,press);

	//crc16 of 2-37 data bytes
	unsigned int crc = pm.calcrc(buf+2, 36);

	buf[i++] = (crc >> 8);
	buf[i++] = crc;

	if(i == OUTPUT_LENG) {
		buf[1] += 10;
		Serial.write(buf, OUTPUT_LENG);
		#ifdef DEBUG_ON
		Serial.println();
		Serial.print("CRC: ");
		Serial.println(crc,HEX);
		printSerialData(buf);
		#endif
		return true;
	}
	else
		return false;
}

void setup() {
	wdt_disable();

	Serial.begin(115200);
	while(!Serial) {} // Wait  

  	altSerial.begin(9600); //Serial for PM sensor  
  	altSerial.setTimeout(1500); //set the Timeout to 1500ms, longer than the data transmission periodic time of the sensor
  	#ifdef DEBUG_ON
	Serial.println(F("Initializing..."));
	#endif
 	
 	wdt_enable(WDTO_8S);

 	while(!bme.begin()){
    	#ifdef DEBUG_ON
    	Serial.println(F("Could not find BME280I2C sensor!"));
    	#endif
    	delay(100);
  	}
}

void loop() {

	float temp=0.0, press=0.0, hum=0.0;
	int pm1=0, pm2_5=0, pm10=0;
	unsigned char buf[LENG];

	//read from BME280
	bme.readData();
	temp = (float)bme.calibration_T(bme.temp_raw) / 100.0;
	press = (float)bme.calibration_P(bme.pres_raw) / 100.0;
	hum = (float)bme.calibration_H(bme.hum_raw) / 1024.0;

	//read from PMS5003
	if(altSerial.find(0x42)){    //start to read when detect 0x42
	    altSerial.readBytes(buf,LENG);

	    if(buf[0] == 0x4d){
			if(pm.checkValue(buf,LENG)){
				pm1=pm.transmitPM01(buf); //count PM1.0 value of the air detector module
				pm2_5=pm.transmitPM2_5(buf);//count PM2.5 value of the air detector module
				pm10=pm.transmitPM10(buf); //count PM10 value of the air detector module
				#ifdef DEBUG_ON
				pm.printAll(buf,LENG);
				Serial.println();
				#endif
			}           
	    } 
  	}

  	static unsigned long wait=millis();

  	if (millis() - wait >= 2000) {

  		wait = millis();

  		bool check = serialWrite(buf,temp,hum,press);

  		#ifdef DEBUG_ON
  		if(!check)
  		Serial.println(F("Error in sending serial data"));

		Serial.print("TEMP : ");
		Serial.print(temp);
		Serial.print(" DegC  PRESS : ");
		Serial.print(press);
		Serial.print(" hPa  HUM : ");
		Serial.print(hum);
		Serial.println(" %");
		Serial.print("PM1.0: ");  
		Serial.print(pm1);
		Serial.println("  ug/m3");            

		Serial.print("PM2.5: ");  
		Serial.print(pm2_5);
		Serial.println("  ug/m3");     

		Serial.print("PM1 0: ");  
		Serial.print(pm10);
		Serial.println("  ug/m3");   
		Serial.println();
		#endif
	}

	wdt_reset();
}
