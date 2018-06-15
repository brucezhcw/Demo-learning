#pragma once
#ifndef _DETECTOR
#define _DETECTOR
#include <iostream>
#include <WinSock2.h>
#include <stdio.h>
#include <cstringt.h>
#include <string>
#include <opencv.hpp>
#pragma comment(lib, "ws2_32.lib")

class Detector {
public:
	Detector() {
		port = 9999;
		addr = "127.0.0.1";
		WORD sockVersion = MAKEWORD(2, 2);
		WSADATA wsaData;
		WSAStartup(sockVersion, &wsaData);
	};
	~Detector() { WSACleanup(); };
	void detect(cv::Mat Img, int* Output) {
		SOCKET sclient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sclient == INVALID_SOCKET)
		{
			printf("invalid socket!");
			return;
		}
		sockaddr_in serAddr;
		serAddr.sin_family = AF_INET;
		serAddr.sin_port = htons(port);
		serAddr.sin_addr.S_un.S_addr = inet_addr(addr);
		if (connect(sclient, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
		{  //connection fail   
			printf("connect error !");
			closesocket(sclient);
			return;
		}
		std::string data = "";
		for (int i = 0; i < 48; ++i)
		for (int j = 0; j < 48; ++j)
		{
			data += std::to_string(Img.at<UINT16>(i,j));
			if (i != 48 - 1 || j != 48 - 1)
				data += " ";
		}

		const char * sendData;
		sendData = data.c_str();      
		send(sclient, sendData, strlen(sendData), 0);
		char recData[255];
		int ret = recv(sclient, recData, 255, 0);
		if (ret>0) {
			recData[ret] = 0x00;
			printf(recData);
			printf("\n");
		}
		sscanf_s(recData, "%d %d %d %d %d %d %d %d %d %d", Output, Output + 1, Output + 2, Output + 3, Output + 4, Output + 5, Output + 6, Output + 7, Output + 8, Output + 9);
		closesocket(sclient);
	};
private:
	int port;
	const char *addr;
};

#endif // !_DETECTOR
