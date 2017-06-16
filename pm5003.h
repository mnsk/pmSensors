#include "Arduino.h"

class pm5003 {
	public:
		unsigned int calcrc(unsigned char *ptr, char count);

		char checkValue(unsigned char *thebuf, char leng);

		int transmitPM01(unsigned char *thebuf);

		int transmitPM2_5(unsigned char *thebuf);

		int transmitPM10(unsigned char *thebuf);

		void printAll(unsigned char *thebuf, char leng);


};