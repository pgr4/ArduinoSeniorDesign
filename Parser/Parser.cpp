#include "Parser.h"

Parser::Parser()
{

}

int pointer = 0;

uint getIP(char* message)
{
	uint ret;
	
	ret += message[pointer++] << 24;
	ret += message[pointer++] << 16;
	ret += message[pointer++] << 8;
	ret += message[pointer++];

	return ret;
}

uint getCommand(char* message)
{
	return message[pointer++];
}

bool setStatus(char* message)
{
	for(int i = 0; i< 6;i++)
	{
		if(message[pointer + i] != 111)
		{
			return false;
		}
	}
	return true;
}

void resetPointer()
{
	pointer = 0;
}

Parser::Header Parser::ParseHeader(char* message)
{
	Serial.println("Parsing Header");
	Header ret;
	ret.sourceIP = getIP(message);
	ret.destIP = getIP(message);
	ret.command = getCommand(message);
	ret.status = setStatus(message);
	return ret;
}

Parser::TrackMessage Parser::ParseTrackMessage(char* message)
{
	Serial.println("Parsing Message");
	TrackMessage ret;
	ret.location = message[pointer++];
	resetPointer();
	return ret;
}