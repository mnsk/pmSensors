//pmSensor.ino
//#include <VoltageReference.h>
#include <AltSoftSerial.h>
#include <BME280_I2C.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <RF24.h>
#include <BTLE.h>

#include "pm5003.h"
#include "oled.h"

#define DEBUG_ON
#define DEBUG_SETUP

#define roundInt(x) ((x)>=0?(unsigned int)((x)+0.5):(unsigned int)((x)-0.5))
#define roundChar(x) ((x)>=0?(char)((x)+0.5):(char)((x)-0.5))

#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
#define OUTPUT_LENG 40
#define BLE_LENG 12
#define OLED_TIME 2000
#define START_TIME 9000
#define BLE_TIME 6000
#define SAMPLES 10
#define LOW_BATT_THRSH 3580

#define VBUS_PIN 2
#define BATT_PIN_VALUE 1234

uint8_t MAC[6] = {0xC0,0x2C,0xBE,0x2C,0x12,0xD8};

BME280_I2C bme;
AltSoftSerial altSerial;
pm5003 pm;
oled display;

//extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

RF24 radio(7,10);
BTLE btle(&radio);

union flt
{
  float f;
  unsigned char bytes[4];
}p,e;

bool debug=0, header=0, csv=0, raw=1;

void floatToBytes (unsigned char *buf, byte &pos, float data) {

	p.f = data;

	buf[pos++] = p.bytes[3];
	buf[pos++] = p.bytes[2];
	buf[pos++] = p.bytes[1];
	buf[pos++] = p.bytes[0];
}

#ifdef DEBUG_ON
void printSerialData(unsigned char *buf, bool flag=false) {
	byte count = 0;
	if(!flag) {
	Serial.println(F("SERIAL DATA: "));
	Serial.print(" ");
	Serial.println(buf[0],HEX);
	Serial.print(count++);
	Serial.print(" ");
	Serial.println(buf[1],DEC);
	}
	unsigned int val;
	for(int i=2; i<25; i+=2) {
		if(!flag) {
			Serial.print(count);
			Serial.print(" ");
			val = (unsigned int)((buf[i]<<8) + buf[i+1]);
			Serial.println(val);
			count++;
		}
		else {
			val = (unsigned int)((buf[i]<<8) + buf[i+1]);
			Serial.print(val);
			Serial.print(",");
		}
	}
	if(!flag) {
		Serial.print(count++);
		Serial.print(" ");
	}
	e.bytes[3] = buf[26];
  	e.bytes[2] = buf[27];
  	e.bytes[1] = buf[28];
  	e.bytes[0] = buf[29];
  	if(!flag){
		Serial.println(e.f);
		Serial.print(count++);
		Serial.print(" ");
	}
	else{
		Serial.print(e.f);
		Serial.print(",");
	}
	e.bytes[3] = buf[30];
  	e.bytes[2] = buf[31];
  	e.bytes[1] = buf[32];
  	e.bytes[0] = buf[33];
  	if(!flag) {
		Serial.println(e.f);
		Serial.print(count++);
		Serial.print(" ");
	}
	else {
		Serial.print(e.f);
		Serial.print(",");
	}
	e.bytes[3] = buf[34];
  	e.bytes[2] = buf[35];
  	e.bytes[1] = buf[36];
  	e.bytes[0] = buf[37];
  	if(!flag) {
		Serial.println(e.f);
		Serial.print(count++);
		Serial.print(" ");
		Serial.println((unsigned int)((buf[38]<<8) + buf[39]),HEX);
	}
	else {
		Serial.print(e.f);
		Serial.println();
	}

}

void printBleData(unsigned char *buf) {

	Serial.println(F("BLE DATA: "));

	byte count = 0;
	Serial.print(count++);
	Serial.print(" ");
	Serial.print((char)buf[0],DEC);
	Serial.print(", ");
	Serial.println((char)buf[0],HEX);
	Serial.print(count++);
	Serial.print(" ");
	Serial.print(buf[1],DEC);
	Serial.print(", ");
	Serial.println(buf[1],HEX);
	Serial.print(count++);
	Serial.print(" ");
	Serial.print((buf[2]<<8) + buf[3],DEC);
	Serial.print(", ");
	Serial.println((buf[2]<<8) + buf[3],HEX);
	Serial.print(count++);
	Serial.print(" ");
	Serial.print(((buf[4]<<8) + buf[5]));
	Serial.print(", ");
	Serial.println(((buf[4]<<8) + buf[5]),HEX);
	Serial.print(count++);
	Serial.print(" ");
	Serial.print(((buf[6]<<8) + buf[7]));
	Serial.print(", ");
	Serial.println(((buf[6]<<8) + buf[7]),HEX);
	Serial.print(count++);
	Serial.print(" ");
	Serial.print(((buf[8]<<8) + buf[9]));
	Serial.print(", ");
	Serial.println(((buf[8]<<8) + buf[9]),HEX);
	Serial.print(count++);
	Serial.print(" ");
	Serial.print(((buf[10]<<8) + buf[11]));
	Serial.print(", ");
	Serial.println(((buf[10]<<8) + buf[11]),HEX);

}
#endif

bool serialWrite(unsigned char *pmBuf, float temp, float hum, float press) {
	unsigned char buf[OUTPUT_LENG] = {0};
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
		if(raw)
			Serial.write(buf, OUTPUT_LENG);
		#ifdef DEBUG_ON
		if(debug) {
			Serial.println();
			Serial.print("CRC: ");
			Serial.println(crc,HEX);
			printSerialData(buf,false);
		}
		if(header) {
			Serial.println();
			Serial.println(F("PM1(CF=1),PM2.5(CF=1),PM10(CF=1),PM1,PM2.5,PM10,P>0.3,P>0.5,P>1.0,P>2.5,P>5.0,P>10,TEMP,HUM,PRESS"));
			header = false;
		}
		if(csv)
			printSerialData(buf,true);
		#endif

		return true;
	}
	else
		return false;
}

void bleAdvertise(float temp, float hum, float press, int pm1, int pm2_5, int pm10, int voltage, unsigned char *buf) {

	buf[0] = roundChar(temp);
	buf[1] = roundChar(hum);
	buf[2] = roundInt(press)>>8;
	buf[3] = roundInt(press);
	buf[4] = pm1>>8;
	buf[5] = pm1;
	buf[6] = pm2_5>>8;
	buf[7] = pm2_5;
	buf[8] = pm10>>8;
	buf[9] = pm10;
	buf[10] = voltage>>8;
	buf[11] = voltage;

	#ifdef DEBUG_ON
	if(debug)
		printBleData(buf);
	#endif
}

void setup() {
	wdt_disable();

	Serial.begin(115200);
	while(!Serial) {} // Wait  

	analogReference(INTERNAL);	//setting 1.1v internal reference
	analogRead(A0);

	pinMode(VBUS_PIN, INPUT);

  	altSerial.begin(9600); //Serial for PM sensor  
  	altSerial.setTimeout(1500); //set the Timeout to 1500ms, longer than the data transmission periodic time of the sensor
  	#ifdef DEBUG_SETUP
	Serial.println(F("Initializing..."));
	#endif

	btle.begin("creav");
	radio.powerDown();

	wdt_enable(WDTO_8S);

	display.oledInit();

	//draw creav logo
	display.u8g2.firstPage();
	do {
		display.drawLogo();
	} while ( display.u8g2.nextPage() );

 	while(!bme.begin()){
    	#ifdef DEBUG_SETUP
    	Serial.println(F("Could not find BME280I2C sensor!"));
    	#endif
    	delay(100);
  	}
}

void loop() {

	float temp=0.0, press=0.0, hum=0.0;
	int pm1=0, pm2_5=0, pm10=0, aqi=0;
	unsigned char buf[LENG];
	static uint16_t voltage;
	static byte analogCount, lowVoltCount;

	bool cableInserted = digitalRead(VBUS_PIN);

	#ifdef DEBUG_ON
	if (Serial.available()) {
		char mode = Serial.read();

		if(mode == 'D') {
			debug = true;
			raw = false;
			csv = false;
		}
		else if(mode == 'C') {
			debug = false;
			raw = false;
			if(!csv) {
				csv = true;
				header = true;
			}
		}
		else if(mode == 'R') {
			debug = false;
			raw = true;
			csv = false;
		}

	}
	#endif

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
				aqi=pm.calculateAQI(pm2_5);
				#ifdef DEBUG_ON
				if(debug) {
					pm.printAll(buf,LENG);
					Serial.println();
				}
				#endif
			}           
	    } 
  	}

  	static unsigned long waitOled=millis();

  	if (millis() - waitOled >= OLED_TIME) {
  		#ifdef DEBUG_ON
  		if(debug) {
	  		Serial.println(F("Updating oled..."));
	  		Serial.print("AQI: ");
	  		Serial.println(aqi);
	  	}
  		#endif
  		aqi = map(aqi,0,300,0,63);
  		#ifdef DEBUG_ON
  		if(debug){
	  		Serial.print("Mapped AQI: ");
	  		Serial.println(aqi);
	  	}
  		#endif
  		display.u8g2.firstPage();
  		do {
		  	display.drawScreen();
		  	display.setOledReading(temp,hum,press,pm1,pm2_5,pm10);
		    display.drawPointer(aqi);
		} while ( display.u8g2.nextPage() );
  		waitOled = millis();
  	}

  	static unsigned long wait=millis();

  	if (millis() - wait >= START_TIME){

  		wdt_reset();

  		voltage = ((voltage/10)*43)/analogCount;	//values of vresistor in voltage divider (R2=10K,R1+R2=43K)

  		if((voltage < LOW_BATT_THRSH) && (cableInserted == false))
  			lowVoltCount++;
  		else
  			lowVoltCount = 0;

  		if(lowVoltCount >= SAMPLES) {
  			display.u8g2.firstPage();
			do {
				display.drawBattLogo();
				#ifdef DEBUG_ON
				if(debug) {
					display.u8g2.setCursor(36,55);
					display.u8g2.print(voltage);
					display.u8g2.print(" V");
				}
				#endif
			} while ( display.u8g2.nextPage() );
			while(1){
			    wdt_reset();			//enters in infinite loop
			}
  		}

  		if(cableInserted)
  			voltage = BATT_PIN_VALUE; 

  		#ifdef DEBUG_ON
  		if(debug) {
			Serial.print("Temp: ");
			Serial.print(temp);
			Serial.print(" C  Press: ");
			Serial.print(press);
			Serial.print(" hPa  Hum: ");
			Serial.print(hum);
			Serial.println(" %");

			Serial.print("PM1.0: ");  
			Serial.print(pm1);
			Serial.print(" ug/m3");            
			Serial.print(" PM2.5: ");  
			Serial.print(pm2_5);
			Serial.print(" ug/m3");     
			Serial.print(" PM10: ");  
			Serial.print(pm10);
			Serial.println(" ug/m3");   

			Serial.print("Count: ");
			Serial.print(analogCount);
			Serial.print(" Voltage: ");
			Serial.println(voltage);

			display.u8g2.firstPage();
			do {
				display.showDebug(temp,hum,press,pm1,pm2_5,pm10,voltage,aqi,cableInserted);
			} while ( display.u8g2.nextPage() );
		}
		#endif

		bool checkSerial = serialWrite(buf,temp,hum,press);
		#ifdef DEBUG_ON
		if(!checkSerial)
			if(debug)
  				Serial.println(F("Error in sending serial data"));
  		#endif

  		unsigned char bleBuf[BLE_LENG] = {0};
  		bleAdvertise(temp,hum,press,pm1,pm2_5,pm10,voltage,bleBuf);
  		radio.powerUp();
		unsigned long bleWait = millis();
		while(millis() - bleWait <= BLE_TIME) {

			wdt_reset();

			bool checkBLE = btle.advertise(bleBuf,BLE_LENG,MAC);
			#ifdef DEBUG_ON
	  		if(!checkBLE)
	  			if(debug)
	  				Serial.println(F("Error in sending ble data"));
	  		#endif
	  		btle.hopChannel();
		}

		radio.powerDown();
		wait = millis();
		voltage = 0;
		analogCount = 0;

	}

	if(analogCount < SAMPLES) {
		voltage += analogRead(A0) * (1100 / 1024.0);
		analogCount++;
	}

	wdt_reset();
}
