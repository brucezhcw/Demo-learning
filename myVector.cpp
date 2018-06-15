#include "stdafx.h"
#include "myVector.h"

Vector3D::Vector3D(): x(0), y(0), z(0){};

Vector3D::Vector3D(float a) : x(a), y(a), z(a){};

Vector3D::Vector3D(float x, float y, float z) : x(x), y(y), z(z){};

Vector3D::Vector3D(CameraSpacePoint point) : x(point.X), y(point.Y), z(point.Z){ 
	//std::cout << *this; 
};

Vector3D Vector3D::operator-(Vector3D &b){
	return Vector3D(this->x - b.x, this->y - b.y, this->z - b.z);
}

Vector3D Vector3D::operator+(Vector3D &b){
	return Vector3D(this->x + b.x, this->y + b.y, this->z + b.z);
}

Vector3D Vector3D::operator*(Vector3D &b){
	return Vector3D(this->x * b.x, this->y * b.y, this->z * b.z);
}

Vector3D Vector3D::operator/(Vector3D &b){
	return Vector3D(this->x / b.x, this->y / b.y, this->z / b.z);
}

Vector3D Vector3D::operator-(float b){
	return *this - Vector3D(b);
}

Vector3D Vector3D::operator+(float b){
	return *this + Vector3D(b);
}

Vector3D Vector3D::operator*(float b){
	return *this * Vector3D(b);
}

Vector3D Vector3D::operator/(float b){
	return *this / Vector3D(b);
}

std::ostream& operator<<(std::ostream &out, Vector3D &b){
	out << b.x << "\t" << b.y << "\t" << b.z << std::endl;
	return out;
}

extern inline Vector3D pow(Vector3D a, double b){
	return Vector3D(pow(a.x, b), pow(a.y, b), pow(a.z, b));
}

extern inline double sum(Vector3D &a){
	return a.x + a.y + a.z;
}

extern inline double length(Vector3D &a){
	return sqrt(sum(pow(a,2)));
}

extern inline void norm(Vector3D &a){
	a = a / length(a);
}

extern inline Vector3D leftTrans(Vector3D &b){
	return Vector3D(-b.y, b.z, -b.x);
}

extern inline Vector3D rightTrans(Vector3D &b){
	return Vector3D(b.y, -b.z, b.x);
}

extern inline Vector3D cross(Vector3D &a, Vector3D &b){
	return Vector3D(
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x);
}

extern inline double angle(Vector3D a, Vector3D b){
	return acos((a.x * b.x + a.y * b.y + a.z * b.z) / length(a) / length(b));
}

extern inline void computeAngle(Vector3D& shoulder, Vector3D& elbow, Vector3D& wrist, Vector3D& hand){
	Vector3D vec1 = elbow - shoulder;
	norm(vec1);
	Vector3D vec2 = wrist - elbow;
	norm(vec2);
	Vector3D vec3 = cross(vec1, vec2);
}