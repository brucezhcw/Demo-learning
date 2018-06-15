#pragma once
#ifndef _SIGN_H
#define _SIGN_H
const double pi = 3.1415926;

enum SWITH{
	ON = 1,
	OFF = 2
};

enum MoveType{
	MOVJ = 1, //Link absolute position operation
	MOVL = 2, //Straight absolute position operation
	IMOV = 3  //Straight increment value operation
};

enum SpeedType{
	Ratio = 1, //Unit %
	CartesianSpace = 2, //Unit mm/s
	JointSpace = 3 //Unit бу/s
};

enum Arm{
	Left = 1,
	Right = 2
};

#define _PI 3.1415926
#endif