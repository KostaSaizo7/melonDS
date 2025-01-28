/*
	This file is copyright Barret Klics and maybe melonDS team I guess?
   */


#include "H8300.h"
#include <string.h>
#include "types.h"
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <arpa/inet.h>


H8300::H8300(){
    printf("H8/300 has been created");
    //debugSerialPort();
    if (IRMode == 2) debugPipe();
}

H8300::~H8300(){
	close(fd);
	printf("H8/300 has been killed");
}

int H8300::debugSerialPort(){
	int option = 0;
	while(option != 9){
		printf("\n\nWelcome to the mini H8/300 Debugger. Please select an option.\n");
		printf("1. Read\n");
		printf("2. Write\n");
		printf("3. FD status\n");
		printf("4. Configure Serial Port\n");
		printf("5. Close serial port\n");
		printf("8. Exit without closing port\n");
		printf("9. Exit and close serial port\n");
		scanf("%d", &option);
		if (option ==1){
			printf("You have 5 seconds to send me some data!!!\n");
			sleep(5);

			memset(buf, 0, sizeof(buf));
			printf("reading");

			int len;
			len = read(fd, buf, sizeof(buf));

			if (len == -1){
				perror("Failed to read port");
			}
			printf("Rxed %d bytes\n", len);

			for (int i = 0; i < len; i++){
				printf("0x%02x ", (unsigned char)buf[i]);
			}
			printf("\n");
		}

		else if (option == 2){
			printf("writing not implemented\n");
		}
		else if (option == 3){

			printf("FD status: %d\n", fd);
		}

		else if (option == 4){
			configureSerialPort();
		}
		else if (option == 5){
			close(fd);
			printf("hopefully closed fd\n");
		}

		else if (option == 8){
			break;
		}

		else if (option == 9){
			close(fd);
			printf("Exiting.\n");
			break;
		}
	}
	return 0;
}


//Separate the different operating modes.
u8 H8300::handleSPI(u8& IRCmd, u8& val, u32&pos, bool& last){
    switch (IRMode){
        case 0:
            return handleSPICompatibility(IRCmd, val, pos, last);
        case 1:
            return handleSPISerial(IRCmd, val, pos, last);
        case 2:
            return handleSPIUDP(IRCmd, val, pos, last);
    }
    return handleSPICompatibility(IRCmd, val, pos, last);
}


//Exact copy of original handling in NDSCart.cpp
u8 H8300::handleSPICompatibility(u8& IRCmd, u8& val, u32&pos, bool& last){
    if (pos == 0)
    {
        IRCmd = val;
        return 0;
    }
    switch (IRCmd)
    {
    case 0x00: //Should not happen
        return 0xFF;
    case 0x08: // ID
        return 0xAA;
    }

    return 0;
}


//Serial Forwarding
u8 H8300::handleSPISerial(u8& IRCmd, u8& val, u32&pos, bool& last){
    if (fd == -1){
        configureSerialPort();


    }
    switch (IRCmd)
    {
    case 0x00: // pass-through. This should never occur and should (and is) intercepted by NDSCart.cpp
        return 0xFF;
    case 0x01:
	//This should never occur. Pos 0 is used to "load" the IRCmd in the first place.
	if (pos == 0){
		return 0x00;
	}
	if (pos == 1){
		memset(buf, 0, sizeof(buf)); //needed???
		recvLen = readSerialPort();
		return recvLen;
	}
	else{
		u8 data = (unsigned char)buf[pos-2];

		return data;
	}
	return 0x00;

    case 0x02:
	if (pos == 0){
	    recvLen = 0;
	}
	else{
	    sendBuf[pos-1] = (u8) val; //Load the spi packet into the buffer;
    }

	if (last == 1){
		u8 sendLen = pos;
		sendSerialPort(sendLen);
    }
	return 0x00;
    case 0x08: //Maybe there is a better place to handle this, but I do not really see any issue.
	if (fd <0){
		printf("Configuring port during IR init sequence\n");
		configureSerialPort();
	}
        return 0xAA; // Version check.
    }
    return 0x00;
}



//TODO Implement UDP mode. For now, just set it as compatibility mode.
u8 H8300::handleSPIUDP(u8& IRCmd, u8& val, u32&pos, bool& last){

    printf("IRCmd: %d   val: 0x%02x   pos: %ld   last %d\n", IRCmd, val ^ 0xaa, pos, last);
    switch (IRCmd)
    {
    case 0x00: // pass-through. This should never occur and should (and is) intercepted by NDSCart.cpp
        return 0xFF;
    case 0x01:
	//This should never occur. Pos 0 is used to "load" the IRCmd in the first place.
	if (pos == 0){
		return 0x00;
	}
	if (pos == 1){
		//memset(buf, 0, sizeof(buf)); //needed???
		recvLen = readUDP();
		return recvLen;
	}
	else{
		u8 data = (unsigned char)buf[pos-2];

        printf("ATE 0x%02x at POS %d\n", data, pos);
		return data;
	}
	return 0x00;

    case 0x02:
	if (pos == 0){
	    recvLen = 0;
	}
	else{
	    sendBuf[pos-1] = (u8) val; //Load the spi packet into the buffer;
    }

	if (last == 1){
		u8 sendLen = pos;
		sendUDP(sendLen);
    }
	return 0x00;
    case 0x08: //Maybe there is a better place to handle this, but I do not really see any issue.
	if (fd <0){
		//printf("Configuring port during IR init sequence\n");
		//configureSerialPort();
	}
        return 0xAA; // Version check.
    }
    return 0x00;
}


int H8300::readUDP(){
    char tempBuf[0xB8];
    socklen_t addrLen = sizeof(out_ip);

    int len = recvfrom(sock, tempBuf, sizeof(tempBuf), 0, (struct sockaddr*)&out_ip, &addrLen);
    if (len < 0) {
        perror("UDP receive error: ");
        return 0;
    }

    recvLen = len;
    memcpy(buf, tempBuf, len); // Copy the received data into buf

    printf("\nReceived %d Bytes via UDP:\n", recvLen);
    for (int i = 0; i < recvLen; i++) {
        printf("0x%02x ", (u8)buf[i] ^ 0xaa);
    }
    printf("\n");
    return recvLen;



}

int H8300::sendUDP(u8 len){
    printf("\nSending %d Bytes via UDP:\n", len);
    for (int i = 0; i < len; i++) {
        printf("0x%02x ", (u8)sendBuf[i] ^ 0xaa);
    }
    printf("\n");

    int sent = sendto(sock, sendBuf, len, 0, (struct sockaddr*)&out_addr, sizeof(out_addr));
    if (sent < 0) {
        perror("UDP send error: ");
        return -1;
    }
    return sent;

}



/////////////////////////////////////////////////////////////////////
int H8300::configureSerialPort(){
	memset(buf, 0, sizeof(buf));
	//This should be configured in options
	const char *portname = "/dev/ttyUSB0";    //sudo chmod 666 /dev/ttyUSB0
	fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1){
		printf("H8/300 failed to open the serial port\n");
		perror("Err:");
		return 1;
	}

	struct termios options;
	tcgetattr(fd, &options);
	if (tcgetattr(fd, &options) == -1){
		printf("H8/300 failed to configure the serial port\n");
        perror("Err:");
		close(fd);
		return 1;
	}

	//Meant to optimize these to the p-walker. May or may not be the same for all IR peripheral comms.
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag &= ~CRTSCTS;
	options.c_cflag |= CREAD | CLOCAL;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
	options.c_iflag &= ~(ICRNL | INLCR);
//	options.c_cc[VTIME] = 1; not needed in non-blocking mode
//	options.c_cc[VMIN] = 0;
	options.c_iflag = 0;
	options.c_oflag = 0;
	tcflush(fd, TCIOFLUSH);
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);
	return 0;
}



long long H8300::getTimeUS(){
    struct timeval tv;
	gettimeofday(&tv, NULL);
	long long t = (tv.tv_sec * 1000000LL) + tv.tv_usec;
    return t;
}


int H8300::readSerialPort(){
	char tempBuf[0xB8];
	int len = read(fd, tempBuf, sizeof(tempBuf));
	u8 pointer = 0;
    long long lastRxTime = getTimeUS();
	if (len > 0){
		lastRxTime = getTimeUS();
		for(int i = 0; i < len; i++){
			buf[pointer + i] = tempBuf[i];
		}
		pointer = pointer + len;
		while((getTimeUS() - lastRxTime) < 3500){ //Maybe this can be fine tuned
			len = read(fd, tempBuf, sizeof(tempBuf));
			if (len < 0){ continue;}
			else{
				lastRxTime = getTimeUS();
				for(int i = 0; i < len; i++){
					buf[pointer + i] = tempBuf[i];
				}
				pointer = pointer + len;
			}
		}
	}

	recvLen = pointer;
	if (recvLen == 0) return 0;

	printf("\nRecieved %d Bytes \n", recvLen);
	for (int i = 0; i < recvLen; i++){
		printf("0x%02x ", (u8)buf[i] ^ 0xaa);
	}
	printf("\n");
	return recvLen;
}


int H8300::sendSerialPort(u8 sendLen){
	printf("\n Sending %d Bytes: \n", sendLen);
	for (int i = 0; i < sendLen; i++){
		printf("0x%02x ", (u8)sendBuf[i] ^ 0xaa);
	}
	printf("\n");

	if ((u8)sendBuf[0] == 94){
        usleep(10000);  /*Necessary hack. Could be a slower time possibly, but is less than one frame length. This needs to halt
		                 the game card from piggybacking this '94/ 0xf4 immediate disconnect' onto is previous send*/
	}
	int written = write(fd, sendBuf, sendLen);
	if (written == -1){
		perror("err: ");
	}
	return 0;
}







void H8300::debugPipe(){
	int option = 0;
	while(option != 9){
		printf("\n\nWelcome to the H8300 UDP debugger.\n");
		printf("1. cfg1 \n");
		printf("2. cfg2\n");
		printf("3. send ping\n");
        printf("4. rx loop\n");
        printf("9. exit\n");
		scanf("%d", &option);
		if (option ==1){

            in_ip = "127.0.0.1";
            in_port = 12345;
            out_ip = "127.0.0.1";
            out_port = 12346;
            startUDP();
        }
		else if (option == 2){

            in_ip = "127.0.0.1";
            in_port = 12346;
            out_ip = "127.0.0.1";
            out_port = 12345;

            startUDP();
        }

        else if (option == 9){
            printf("continuing");
            break;
        }
    }
}

u8 H8300::startUDP(){
    char buffer[256];

    socklen_t addr_len = sizeof(in_addr);


    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket");
        exit(1);
    }


    struct timeval timeout;
    timeout.tv_sec = 3;
    timeout.tv_usec = 0;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        perror("setsockopt");
        exit(1);
    }



    //INCOMING
    in_addr.sin_family = AF_INET;
    in_addr.sin_port = htons(in_port);
    in_addr.sin_addr.s_addr = inet_addr(in_ip);
    if (bind(sock, (struct sockaddr *)&in_addr, sizeof(in_addr)) < 0){
        perror("Failed to bind");
        exit(1);
    }


    // OUTGOING
    out_addr.sin_family = AF_INET;
    out_addr.sin_port = htons(out_port);
    out_addr.sin_addr.s_addr = inet_addr(out_ip);


    printf("Program running on incoming port %d, sending messages to %s:%d\n", in_port, out_ip, out_port);


    /*
    while (1) {
        // Send "ping"
        strcpy(buffer, "ping");
        sendto(sock, buffer, strlen(buffer), 0, (struct sockaddr *)&out_addr, addr_len);

        // Receive response
        int received = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&in_addr, &addr_len);
        if (received < 0) {
            perror("recvfrom timeout");
            printf("Timeout: No response from the other program.\n");
        } else {
            buffer[received] = '\0'; // Null-terminate the received message
            printf("Received: %s\n", buffer);
        }

        sleep(1); // Wait before sending the next message
    }*/

   // close(sock);



    /*int flags = fcntl(sock, F_GETFL, 0);  // Get current flags
    if (flags == -1) return -1;             // Error checking

    return fcntl(sock, F_SETFL, flags | O_NONBLOCK);  // Set non-blocking*/
    return 0;


}
