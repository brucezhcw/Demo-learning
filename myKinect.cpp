#include "stdafx.h"
#include "myKinect.h"
#include <time.h>

using namespace cv;

MyKinect::MyKinect(bool colorenable, bool depthenable, bool bodyenable) {
	colorEnable = colorenable;
	depthEnable = depthenable;
	bodyEnable = bodyenable;
	myKinectSensor = NULL;
	myCoordinateMapper = NULL;
	//Color Frame Variables
	myColorFrameSource = NULL;
	myColorFrameReader = NULL;
	myColorFrame = NULL;
	img_rgb = new Mat(1080, 1920, CV_8UC4);
	//Depth Frame Variables
	myDepthFrameSource = NULL;
	myDepthFrameReader = NULL;
	myDepthFrame = NULL;
	img_depth = new Mat(424, 512, CV_16UC1);
	//Body Frame Variables
	myBodyFrameSource = NULL;
	myBodyFrameReader = NULL;
	myBodyFrame = NULL;
}

int MyKinect::open() {
	HRESULT hr;
	// Open Kinect Sensor
	hr = GetDefaultKinectSensor(&myKinectSensor);
	if (SUCCEEDED(hr))
		hr = myKinectSensor->Open();
	if (FAILED(hr)) {
		return 1;
	}
	// Get CoordinateMapper
	hr = myKinectSensor->get_CoordinateMapper(&myCoordinateMapper);
	if (FAILED(hr)) {
		return 2;
	}
	if (colorEnable) {
		// Get Color Frame Source
		hr = myKinectSensor->get_ColorFrameSource(&myColorFrameSource);
		if (FAILED(hr)) {
			return 3;
		}
		// Open Color Frame Reader
		hr = myColorFrameSource->OpenReader(&myColorFrameReader);
		if (FAILED(hr)) {
			return 4;
		}
	}
	if (depthEnable) {
		// Get Depth Frame Source
		hr = myKinectSensor->get_DepthFrameSource(&myDepthFrameSource);
		if (FAILED(hr)) {
			return 5;
		}
		// Open Depth Frame Reader
		hr = myDepthFrameSource->OpenReader(&myDepthFrameReader);
		if (FAILED(hr)) {
			return 6;
		}
	}
	if (bodyEnable) {
		// Get Body Frame Source
		hr = myKinectSensor->get_BodyFrameSource(&myBodyFrameSource);
		if (FAILED(hr)) {
			return 7;
		}
		// Open Body Frame Reader
		hr = myBodyFrameSource->OpenReader(&myBodyFrameReader);
		if (FAILED(hr)) {
			return 8;
		}
	}
	BOOLEAN isAvailable = false;
	do {
		myKinectSensor->get_IsAvailable(&isAvailable);
	} while (!isAvailable);
	return 0;
}

MyKinect::~MyKinect() {
	close();
}

bool MyKinect::close() {
	SafeRelease(myCoordinateMapper);
	SafeRelease(myBodyFrame);
	SafeRelease(myDepthFrame);
	SafeRelease(myColorFrame);
	SafeRelease(myBodyFrameReader);
	SafeRelease(myDepthFrameReader);
	SafeRelease(myColorFrameReader);
	SafeRelease(myBodyFrameSource);
	SafeRelease(myDepthFrameSource);
	SafeRelease(myColorFrameSource);
	if (myKinectSensor != NULL)
		myKinectSensor->Close();
	SafeRelease(myKinectSensor);
	return true;
}

void MyKinect::updateColor() {
	const UINT buffer_size = 1920 * 1080 * 4;
	while (true)
	{
		if (myColorFrameReader->AcquireLatestFrame(&myColorFrame) == S_OK &&
			myColorFrame->CopyConvertedFrameDataToArray(buffer_size, reinterpret_cast<BYTE*>(img_rgb->data), ColorImageFormat::ColorImageFormat_Bgra) == S_OK)
		{
			SafeRelease(myColorFrame);
			break;
		}
		else
			Sleep(1);
	}
}

void MyKinect::updateDepth() {
	const UINT buffer_size = 424 * 512;
	while (true)
	{
		if (myDepthFrameReader->AcquireLatestFrame(&myDepthFrame) == S_OK &&
			myDepthFrame->CopyFrameDataToArray(buffer_size, reinterpret_cast<UINT16*>(img_depth->data)) == S_OK)
		{
			SafeRelease(myDepthFrame);
			break;
		}
		else
			Sleep(1);
	}
}

void MyKinect::updateBody() {
	IBody* Bodys[BODY_COUNT] = { 0 };
	while (true)
	{
		if (myBodyFrameReader->AcquireLatestFrame(&myBodyFrame) == S_OK)
		{
			myBodyFrame->GetAndRefreshBodyData(_countof(Bodys), Bodys);
			for (int i = 0; i < BODY_COUNT; ++i)
			{
				IBody *pBody = Bodys[i];
				if (pBody)
				{
					BOOLEAN isTracked = false;
					if (pBody->get_IsTracked(&isTracked) == S_OK && isTracked)
					{
						Joint joints[JointType_Count];
						if (pBody->GetJoints(JointType_Count, joints) == S_OK)
						{
							for (int j = 0; j < JointType_Count; ++j)
								points[j] = joints[j].Position;
						}
						//myCoordinateMapper->MapCameraPointsToColorSpace(JointType_Count, points, JointType_Count, Cpoints);
						//myCoordinateMapper->MapCameraPointsToDepthSpace(JointType_Count, points, JointType_Count, Dpoints);
						//for (int j = 0; j < JointType_Count; ++j)
						//{
						//	Cdraw[j].x = int(Cpoints[j].X);
						//	Cdraw[j].y = int(Cpoints[j].Y);
						//	Ddraw[j].x = int(Dpoints[j].X);
						//	Ddraw[j].y = int(Dpoints[j].Y);
						//}
					}
				}
			}
			SafeRelease(myBodyFrame);
			break;
		}
		else
			Sleep(1);
	}
}

bool MyKinect::update() {
	if (colorEnable)
		updateColor();
	if (depthEnable)
		updateDepth();
	if (bodyEnable)
		updateBody();
	return true;
}

Mat& MyKinect::getColor() {
	return *img_rgb;
}

Mat& MyKinect::getDepth() {
	return *img_depth;
}

CameraSpacePoint* MyKinect::getBody() {
	return points;
}

void MyKinect::depth2camera(DepthSpacePoint* depth, CameraSpacePoint* camera, UINT16* depthval, int size) {
	myCoordinateMapper->MapDepthPointsToCameraSpace(size, depth, size, depthval, size, camera);
}

void MyKinect::camera2depth(CameraSpacePoint* camera, DepthSpacePoint* depth, int size) {
	myCoordinateMapper->MapCameraPointsToDepthSpace(size, camera, size, depth);
}