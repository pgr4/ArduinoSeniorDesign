#include <Arduino.h>

class Parser {

	private:

	public:
		struct Header
		{
			uint sourceIP;
			uint destIP;
			int command;
			bool status;
		};

		struct TrackMessage
		{
			byte fByte;
			byte sByte;
		};

	   Parser();
	   
	   static void resetPointer();
	   static Header ParseHeader(char* message);
	   static TrackMessage ParseTrackMessage(char* message);
  
};