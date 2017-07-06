#include "Arduino.h"

class pm5003 {
	public:

		const float concentrationBoundaries[7][2] =
		{
		        {0.0f,12.0f},
		        {12.1f,35.4f},
		        {35.5f,55.4f},
		        {55.5f,150.4f},
		        {150.5f,250.4f},
		        {250.5f,350.4f},
		        {350.5f,500.4f}
		};

		const unsigned int indexBoundaries[7][2] =
		{
		        {0,50},
		        {51,100},
		        {101,150},
		        {151,200},
		        {201,300},
		        {301,400},
		        {401,500}
		};

		unsigned int calcrc(unsigned char *ptr, char count);

		char checkValue(unsigned char *thebuf, char leng);

		int transmitPM01(unsigned char *thebuf);

		int transmitPM2_5(unsigned char *thebuf);

		int transmitPM10(unsigned char *thebuf);

		void printAll(unsigned char *thebuf, char leng);

		int calculateAQI(float pm25);


};