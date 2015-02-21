#include <Arduino.h>

class Parser {

	private:

	public:
		struct Header
		{
			int sourceIP;
			int destIP;
			int command;
			bool status;
		};

		struct TrackMessage
		{
			int location;
		};

	   Parser();

	   static Header ParseHeader(char* message);
	   static TrackMessage ParseTrackMessage(char* message);
  
};