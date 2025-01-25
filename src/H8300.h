//test

#ifndef H8300_H
#define H8300_H



#include <array>
#include <string>
#include <memory>
#include <variant>

#include "types.h"


using namespace melonDS;
class H8300 {
public:
	H8300();
	~H8300();
	u8 handleSPI(u8& IRCmd, u8& val, u32&pos, bool& last);


	//TODO add options later
	int configureSerialPort();
	int debugSerialPort();
	int readSerialPort();
	int sendSerialPort(u8 len);

private:
    u8 IRMode = 1;


	u8 handleSPICompatibility(u8& IRCmd, u8& val, u32&pos, bool& last); //Mode 0
	u8 handleSPISerial(u8& IRCmd, u8& val, u32&pos, bool& last);        //Mode 1
	u8 handleSPIUDP(u8& IRCmd, u8& val, u32&pos, bool& last);           //Mode 2

    long long getTimeUS();


	//file where the TTY will be stores
	int fd = -1;

	u8 recvLen = 0;
	char buf[0xB8];
	char sendBuf[0xB8];
};

#endif

