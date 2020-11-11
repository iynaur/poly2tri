#pragma once

#ifdef linux
#define API
#else
#define API __declspec(dllexport)
#endif


extern "C"
{
	API	int readPCD(char* filename, int slen, char** index);
}