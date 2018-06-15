#include "stdafx.h"
#include "Motor.h"
#include <iostream>

using namespace std;

Motor::Motor(char* address = "10.0.0.2"){
	IPAddress = address;
	handle = nullptr;
}

Motor::Motor(int speed){
	IPAddress = "10.0.0.2";
	handle = nullptr;
	_speed = speed;
}

Motor::~Motor(){

}

extern void angle2pulse(int *pulse, double *angle)
{
	for (int i = 0; i < 5; ++i)
		pulse[i] = int(angle[i] * 1149.16 / _PI * 180.0);
	pulse[5] = int(angle[5] * 1012.62 / _PI * 180.0);
	pulse[6] = int(angle[6] * 671.29 / _PI * 180.0);
	//S -90 ~ 270
	//L -110 ~ 110
	//E -170 ~ 170
	//U -90 ~ 115
	//R -180 ~ 180
	//B -110 ~ 110
	//T -180 ~ 180
}

extern void pulse2angle(int *pulse, double *angle)
{
	for (int i = 0; i < 5; ++i)
		angle[i] = pulse[i] / 1149.16 * _PI / 180.0;
	angle[5] = pulse[5] / 1012.62 * _PI / 180.0;
	angle[6] = pulse[6] / 671.29 * _PI / 180.0;
	//S -90 ~ 270
	//L -110 ~ 110
	//E -170 ~ 170
	//U -90 ~ 115
	//R -180 ~ 180
	//B -110 ~ 110
	//T -180 ~ 180
}

void Motor::servoOpen(){
	LONG result;
	result = ESOpen(2, IPAddress, &handle);
	ESSetTimeOut(&handle, TIMEOUT, RETRY);
	if (result)
		cout << servoOpenError(result) << endl;
	ESServo(handle, ON);
	ESClose(handle);
}

void Motor::servoClose(){
	ESOpen(2, IPAddress, &handle);
	ESSetTimeOut(&handle, TIMEOUT, RETRY);
	ESServo(handle, OFF);
	ESClose(handle);
}

char* Motor::getAddress(){
	return IPAddress;
}

HANDLE Motor::getHandle(){
	return handle;
}


inline char* servoOpenError(LONG result){
	switch (result)
	{
	case 0x0069: return "Hardware Lock Key is not found.";
	case 0x006A: return "Hardware Lock Key is found, but is not for MOTOCOM32.";
	case 0x9000: return "Connection Error.";
	case 0x9100: return "Parameter has error.";
	default: return "Other Error.";
	}
}

void Motor::Initial(int arm)
{
	ESOpen(2, IPAddress, &handle);
	ESSetTimeOut(handle, TIMEOUT, RETRY);

	ESPulseMoveData DmoveData;

	DmoveData.moveData.robotNo = arm;
	DmoveData.moveData.speed = _speed;   //速度大小
	DmoveData.moveData.speedType = 0; //速度类型
	DmoveData.moveData.stationNo = 0;

	DmoveData.robotPos.axis[0] = -45527; //S
	DmoveData.robotPos.axis[1] = -17302; //L
	DmoveData.robotPos.axis[2] = -55336; //U
	DmoveData.robotPos.axis[3] = -154060; //R
	DmoveData.robotPos.axis[4] = 14771; //B
	DmoveData.robotPos.axis[5] = 55193; //T
	DmoveData.robotPos.axis[6] = 153320; //E
	DmoveData.robotPos.axis[7] = 0;

	DmoveData.toolNo = 0;
	DmoveData.basePos.axis[0] = 0;
	DmoveData.basePos.axis[1] = 0;
	DmoveData.basePos.axis[2] = 0;
	DmoveData.stationPos.axis[0] = 0;
	DmoveData.stationPos.axis[1] = 0;
	DmoveData.stationPos.axis[2] = 0;
	DmoveData.stationPos.axis[3] = 0;
	DmoveData.stationPos.axis[4] = 0;
	DmoveData.stationPos.axis[5] = 0;

	ESPulseMove(handle, 1, DmoveData); //运动
	ESClose(handle);
}

void Motor::Home(int arm)
{
	ESOpen(2, IPAddress, &handle);
	ESSetTimeOut(handle, TIMEOUT, RETRY);

	ESPulseMoveData DmoveData;

	DmoveData.moveData.robotNo = arm;
	DmoveData.moveData.speed =_speed;   //速度大小
	DmoveData.moveData.speedType = 0; //速度类型
	DmoveData.moveData.stationNo = 0;

	DmoveData.robotPos.axis[0] = 0; //S
	DmoveData.robotPos.axis[1] = 0; //L
	DmoveData.robotPos.axis[2] = 0; //U
	DmoveData.robotPos.axis[3] = 0; //R
	DmoveData.robotPos.axis[4] = 0; //B
	DmoveData.robotPos.axis[5] = 0; //T
	DmoveData.robotPos.axis[6] = 0; //E
	DmoveData.robotPos.axis[7] = 0;

	DmoveData.toolNo = 0;
	DmoveData.basePos.axis[0] = 0;
	DmoveData.basePos.axis[1] = 0;
	DmoveData.basePos.axis[2] = 0;
	DmoveData.stationPos.axis[0] = 0;
	DmoveData.stationPos.axis[1] = 0;
	DmoveData.stationPos.axis[2] = 0;
	DmoveData.stationPos.axis[3] = 0;
	DmoveData.stationPos.axis[4] = 0;
	DmoveData.stationPos.axis[5] = 0;

	ESPulseMove(handle, 1, DmoveData); //运动
	ESClose(handle);
}

void Motor::Move(int arm, double *angle)
{
	ESOpen(2, IPAddress, &handle);
	ESSetTimeOut(handle, TIMEOUT, RETRY);

	ESPulseMoveData DmoveData;
	int pulse[7];
	angle2pulse(pulse, angle);

	DmoveData.moveData.robotNo = arm;
	DmoveData.moveData.speed = _speed;   //速度大小
	DmoveData.moveData.speedType = 0; //速度类型
	DmoveData.moveData.stationNo = 0;

	DmoveData.robotPos.axis[0] = pulse[0]; //S
	DmoveData.robotPos.axis[1] = pulse[1]; //L
	DmoveData.robotPos.axis[2] = pulse[3]; //U
	DmoveData.robotPos.axis[3] = pulse[4]; //R
	DmoveData.robotPos.axis[4] = pulse[5]; //B
	DmoveData.robotPos.axis[5] = pulse[6]; //T
	DmoveData.robotPos.axis[6] = pulse[2]; //E
	DmoveData.robotPos.axis[7] = 0;

	DmoveData.toolNo = 0;
	DmoveData.basePos.axis[0] = 0;
	DmoveData.basePos.axis[1] = 0;
	DmoveData.basePos.axis[2] = 0;
	DmoveData.stationPos.axis[0] = 0;
	DmoveData.stationPos.axis[1] = 0;
	DmoveData.stationPos.axis[2] = 0;
	DmoveData.stationPos.axis[3] = 0;
	DmoveData.stationPos.axis[4] = 0;
	DmoveData.stationPos.axis[5] = 0;

	ESPulseMove(handle, 1, DmoveData); //运动
	ESClose(handle);
}

void Motor::getTheta(double* theta)
{
	ESOpen(2, IPAddress, &handle);
	ESSetTimeOut(handle, TIMEOUT, RETRY);
	ESPositionData positionPulseData;
	ESGetPosition(handle, 1, &positionPulseData);
	int pulse[7];
	pulse[0] = positionPulseData.axesData.axis[0];
	pulse[1] = positionPulseData.axesData.axis[1]; 
	pulse[2] = positionPulseData.axesData.axis[6];
	pulse[3] = positionPulseData.axesData.axis[2];
	pulse[4] = positionPulseData.axesData.axis[3];
	pulse[5] = positionPulseData.axesData.axis[4];
	pulse[6] = positionPulseData.axesData.axis[5];
	pulse2angle(pulse, theta);
	ESClose(handle);
}