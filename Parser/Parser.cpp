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

int getCommand(char* message)
{
	return message[pointer++];
}

bool setStatus(char* message)
{
	for(int i = 0; i< 6;i++)
	{
		if(message[pointer++] != 111)
		{
			return false;
		}
	}
	return true;
}

void Parser::resetPointer()
{
	pointer = 0;
}

Parser::Header Parser::ParseHeader(char* message)
{
	Header ret;
	ret.sourceIP = getIP(message);
	ret.destIP = getIP(message);
	ret.command = getCommand(message);
	ret.status = setStatus(message);
	return ret;
}

Parser::TrackMessage Parser::ParseTrackMessage(char* message)
{
	TrackMessage ret;
	ret.fByte = message[pointer++];
	ret.sByte = message[pointer++];
	return ret;
}