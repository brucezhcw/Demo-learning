#ifndef _MYVECTOR_H
#define _MYVECTOR_H
#include <Kinect.h>
#include <iostream>
#include <cmath>

class Vector3D{
public:
	Vector3D();
	Vector3D(float);
	Vector3D(float, float, float);
	Vector3D(CameraSpacePoint);
	Vector3D operator-(Vector3D&);
	Vector3D operator+(Vector3D&);
	Vector3D operator*(Vector3D&);
	Vector3D operator/(Vector3D&);
	Vector3D operator-(float);
	Vector3D operator+(float);
	Vector3D operator*(float);
	Vector3D operator/(float);
	friend std::ostream& operator<<(std::ostream&, Vector3D&);
	friend inline Vector3D pow(Vector3D, double);
	friend inline double sum(Vector3D&);
	friend inline double length(Vector3D&);
	friend inline void norm(Vector3D&);
	friend inline Vector3D leftTrans(Vector3D&);
	friend inline Vector3D rightTrans(Vector3D&);
	friend inline Vector3D cross(Vector3D&, Vector3D&);
	friend inline void computeAngle(Vector3D&, Vector3D&, Vector3D&, Vector3D&);
	friend inline double angle(Vector3D a, Vector3D b);
public:
	float x, y, z;
};

#endif