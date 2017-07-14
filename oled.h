#include "Arduino.h"
#include <U8g2lib.h>
#include <Wire.h>

class oled {
	public:

		U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2;
		oled();
		void drawPointer(u8g2_uint_t y);	//takes the AQI reading mapped to 0-63
		void oledInit();
		void drawScreen();
		void setOledReading(float temp, float hum, float press, int pm1, int pm2_5, int pm10);
		void drawLogo();
		void drawBattLogo();
		void showDebug(float temp, float hum, float press, int pm1, int pm2_5, int pm10, int voltage, int aqi, bool cableInserted);
};