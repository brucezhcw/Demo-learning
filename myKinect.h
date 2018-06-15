#pragma once
#include <Kinect.h>
#include <opencv.hpp>
using namespace cv;
// Define Realease Function
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class MyKinect {
private:
	bool colorEnable, depthEnable, bodyEnable;
	IKinectSensor *myKinectSensor;
	ICoordinateMapper *myCoordinateMapper;
	//Color Frame Variables
	IColorFrameSource *myColorFrameSource;
	IColorFrameReader *myColorFrameReader;
	IColorFrame *myColorFrame;
	Mat* img_rgb;
	//Depth Frame Variables
	IDepthFrameSource *myDepthFrameSource;
	IDepthFrameReader *myDepthFrameReader;
	IDepthFrame *myDepthFrame;
	Mat* img_depth;
	//Body Frame Variables
	IBodyFrameSource *myBodyFrameSource;
	IBodyFrameReader *myBodyFrameReader;
	IBodyFrame *myBodyFrame;
	CameraSpacePoint points[JointType_Count];
	ColorSpacePoint Cpoints[JointType_Count];
	DepthSpacePoint Dpoints[JointType_Count];
	Point Cdraw[JointType_Count];
	Point Ddraw[JointType_Count];
	void updateColor();
	void updateDepth();
	void updateBody();
public:
	MyKinect(bool colorenable, bool depthenable, bool bodyenable);
	~MyKinect();
	int open();
	bool close();
	bool update();
	Mat& getDepth();
	Mat& getColor();
	CameraSpacePoint* getBody();
	void depth2camera(DepthSpacePoint* depth, CameraSpacePoint* camera, UINT16* depthval, int size);
	void camera2depth(CameraSpacePoint* camera, DepthSpacePoint* depth, int size);
};