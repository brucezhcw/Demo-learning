#pragma once
#ifndef _MOTOR_H
#define _MOTOR_H

#include "MOTOCOMES.h"
#include "sign.h"
#include "myVector.h"
class Motor
{
public:
	//构造函数与析构函数
	Motor(int);
	Motor(char*);
	~Motor();
	//功能函数
	void servoOpen();
	void servoClose();
	//数据函数
	char* getAddress();
	HANDLE getHandle();
	void Initial(int);
	void Home(int);
	void Move(int, double*);
	void getTheta(double*);
	friend void angle2pulse(int*, double*);
	friend void pulse2angle(int*, double*);
private:
	HANDLE handle;
	char *IPAddress;
	int _speed;
};

// Error Function
inline char* servoOpenError(LONG result);


#endif