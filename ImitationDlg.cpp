// ImitationDlg.cpp : 实现文件
//
#include "stdafx.h"
#include "Imitation.h"
#include "ImitationDlg.h"
#include "afxdialogex.h"
#include <opencv.hpp>
#include <Kinect.h>
#include <thread>
#include <time.h>
#include "Motor.h"
#include "myVector.h"
#include "myKinect.h"
#include "floodfill.h"
#include "Detector.h"
#include <io.h> 
#include <fcntl.h>
#include <conio.h>
#include <vector>
#include <fstream>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace std;
using namespace cv;

#pragma region Global Variables
const float Alpha[7] = { 0, -pi / 2, pi / 2, -pi / 2, -pi / 2, -pi / 2, pi / 2 };
const float Theta[7] = { 0, 0, 0, -pi / 2, -pi / 2, 0, 0 };
bool Run;
double theta[7];
double thetaTotal[7] = { 0 };
Motor *myMotor;
MyKinect* myKinect;
Detector* detector;
Mat img_color;
Mat img_depth;
CameraSpacePoint* points;
bool enableMotor = false;
bool openedMotor = false;
thread *MainThread;
vector<double*> thetaRobot;
vector<double*> thetaProcess;
vector<double*> thetaRaw;
vector<Vector3D*> elbowRaw;
vector<Vector3D*> wristRaw;
Vector3D elbowTotal(0);
Vector3D wristTotal(0);
int outputPeriod = 2;
int filtersize = 50;
#pragma endregion
#pragma region Global Function
Mat Rot(float alpha, float theta) {
	Mat result(3, 3, CV_32FC1);
	result.at<float>(0, 0) = cos(theta);
	result.at<float>(0, 1) = -sin(theta);
	result.at<float>(0, 2) = 0;
	result.at<float>(1, 0) = cos(alpha) * sin(theta);
	result.at<float>(1, 1) = cos(alpha) * cos(theta);
	result.at<float>(1, 2) = -sin(alpha);
	result.at<float>(2, 0) = sin(alpha) * sin(theta);
	result.at<float>(2, 1) = sin(alpha) * cos(theta);
	result.at<float>(2, 2) = cos(alpha);
	return result;
}
//Inverse Kinematics
void InverseKinematics(double *theta, Vector3D shoulder, Vector3D elbow, Vector3D wrist)
{
	Vector3D Vec1 = wrist - elbow;
	norm(Vec1);
	norm(elbow);

	theta[1] = acos(elbow.z);
	if (elbow.y < 0)
		theta[1] = -theta[1];

	theta[0] = acos(elbow.x / sin(theta[1]));

	theta[3] = angle(elbow, Vec1) - pi / 2;
	theta[3] = -theta[3]; // axis inverse

	Vector3D nVec = cross(elbow, Vec1);
	norm(nVec);
	theta[2] = asin(nVec.z / sin(theta[1]));
}
//// Function to Draw lines
//inline void DrawHelper(Mat &img, Point *&DrawPoints, const int *Index, int length)
//{
//	Point point2draw[10] = {};
//	for (int i = 0; i < length; ++i)
//		point2draw[i] = DrawPoints[Index[i]];
//	for (int i = 0; i < length - 1; ++i)
//		line(img, point2draw[i], point2draw[i + 1], Scalar(0, 255, 0), 2);
//}
//// Function to Draw Skeleton
//inline void DrawSkeleton(Mat &img, Point DrawPoints[25], int r = 20, const int length = JointType_Count)  // inline here may improve the speed
//{
//	//Index of the Skeleton data
//	const int LeftArm[] = { 20, 4, 5, 6, 7, 21 };
//	const int RightArm[] = { 20, 8, 9, 10, 11, 23 };
//	const int LeftThumb[] = { 6, 22 };
//	const int RightThumb[] = { 10, 24 };
//	const int LeftLeg[] = { 0, 12, 13, 14, 15 };
//	const int RightLeg[] = { 0, 16, 17, 18, 19 };
//	const int Spinal[] = { 3, 2, 20, 1, 0 };
//	//--------------------------------------------
//	for (int i = 0; i < length; ++i)
//		circle(img, DrawPoints[i], r, Scalar(0, 255, 0), -1);
//	DrawHelper(img, DrawPoints, LeftArm, 6);
//	DrawHelper(img, DrawPoints, RightArm, 6);
//	DrawHelper(img, DrawPoints, LeftThumb, 2);
//	DrawHelper(img, DrawPoints, RightThumb, 2);
//	DrawHelper(img, DrawPoints, LeftLeg, 5);
//	DrawHelper(img, DrawPoints, RightLeg, 5);
//	DrawHelper(img, DrawPoints, Spinal, 5);
//}
// Process Arm state
void processArmState(CameraSpacePoint points[25])
{
	Vector3D left_shoulder(points[4]);
	Vector3D left_elbow(points[5]);
	Vector3D left_wrist(points[6]);
	Vector3D left_hand(points[7]);

	Vector3D right_shoulder(points[8]);
	Vector3D right_elbow(points[9]);
	Vector3D right_wrist(points[10]);
	Vector3D right_hand(points[11]);

	left_shoulder = leftTrans(left_shoulder);
	left_elbow = leftTrans(left_elbow) - left_shoulder;
	left_wrist = leftTrans(left_wrist) - left_shoulder;
	left_hand = leftTrans(left_hand) - left_shoulder;
	left_shoulder = left_shoulder - left_shoulder;

	right_shoulder = rightTrans(right_shoulder);
	right_elbow = rightTrans(right_elbow) - right_shoulder;
	right_wrist = rightTrans(right_wrist) - right_shoulder;
	right_hand = rightTrans(right_hand) - right_shoulder;
	right_shoulder = right_shoulder - right_shoulder;

	int cnt = elbowRaw.size();
	if (cnt < filtersize)
	{
		elbowTotal = elbowTotal + right_elbow;
		wristTotal = wristTotal + right_wrist;
	}
	else
	{
		elbowTotal = elbowTotal + right_elbow - *elbowRaw[cnt - filtersize];
		wristTotal = wristTotal + right_wrist - *wristRaw[cnt - filtersize];
	}

	Vector3D* tem = new Vector3D(right_elbow);
	elbowRaw.push_back(tem);
	tem = new Vector3D(right_wrist);
	wristRaw.push_back(tem);
	InverseKinematics(theta, right_shoulder, right_elbow, right_wrist);
	//if (cnt < filtersize)
	//	InverseKinematics(theta, right_shoulder, elbowTotal / (cnt + 1), wristTotal / (cnt + 1));
	//else
	//	InverseKinematics(theta, right_shoulder, elbowTotal / filtersize, wristTotal / filtersize);
}
// main thread
void Tracking() {
	VideoWriter myWriter;
	myWriter.open("depth.avi", -1, 12, Size(512, 424), false);
	VideoWriter ColorWriter;
	ColorWriter.open("color.avi", CV_FOURCC('M', 'J', 'P', 'G'), 12, Size(1920, 1080), true);
	Sleep(100);
	while (Run) 
	{
		int lasttime = clock();
		myKinect->update();
		img_color = myKinect->getColor();
		img_depth = myKinect->getDepth();
		points = myKinect->getBody();
		DepthSpacePoint hand;
		myKinect->camera2depth(points + 11, &hand, 1);
		UINT16 hand_depth = img_depth.at<UINT16>(int(hand.Y), int(hand.X));
		UINT16 bbox = UINT16(47000 / (hand_depth + 1));
		Mat seg = img_depth(Rect(int(hand.X - bbox), int(hand.Y - bbox), 2 * bbox, 2 * bbox));
		Mat mask(seg.size().height, seg.size().width, CV_16UC1);
		for (int i = 0; i < mask.size().height; ++i)
			for (int j = 0; j < mask.size().width; ++j)
				mask.at<UINT16>(i, j) = 0;
		floodFillDepth(seg, mask, hand_depth, bbox, bbox, 15);
		Mat seg_ = seg.mul(mask);
		Mat resized(48, 48, CV_16UC1);
		resize(seg_, resized, Size(48, 48));
		if (openedMotor && enableMotor)
		{
			// solve first four angle
			processArmState(points);
			// finger detect
			int pos[10];
			detector->detect(resized, pos);
			DepthSpacePoint fingers_depth[5];
			UINT16 fingers_depthval[5];
			CameraSpacePoint fingers_camera[5];
			for (int i = 0; i < 5; ++i)
			{
				pos[2 * i] = int(float(pos[2 * i]) / 48 * seg.size().height + hand.Y - bbox);
				pos[2 * i + 1] = int(float(pos[2 * i + 1]) / 48 * seg.size().width + hand.X - bbox);
				fingers_depth[i].X = pos[2 * i + 1];
				fingers_depth[i].Y = pos[2 * i];
				fingers_depthval[i] = img_depth.at<UINT16>(fingers_depth[i].Y, fingers_depth[i].X);
			}

			const int R = 3;
			img_depth = img_depth * 200;
			circle(img_depth, Point(pos[1], pos[0]), R, Scalar(255, 0, 0), -1);
			circle(img_depth, Point(pos[3], pos[2]), R, Scalar(127, 127, 0), -1);
			circle(img_depth, Point(pos[5], pos[4]), R, Scalar(0, 255, 0), -1);
			circle(img_depth, Point(pos[7], pos[6]), R, Scalar(0, 127, 127), -1);
			circle(img_depth, Point(pos[9], pos[8]), R, Scalar(0, 0, 255), -1);
			cvNamedWindow("depth", 0);
			imshow("depth", img_depth);
			myWriter << img_depth;
			ColorWriter << img_color;
			cvWaitKey(1);

			myKinect->depth2camera(fingers_depth, fingers_camera, fingers_depthval, 5);
			Vector3D right_shoulder(points[8]);
			right_shoulder = rightTrans(right_shoulder);
			Vector3D palm(points[11]);
			palm = rightTrans(palm) - right_shoulder;
			Vector3D fingers[5];
			for (int i = 0; i < 5; ++i)
			{
				fingers[i] = Vector3D(fingers_camera[i]);
				fingers[i] = rightTrans(fingers[i]) - right_shoulder;
			}

			Mat R04 = Rot(Alpha[0], Theta[0] + theta[0])
				* Rot(Alpha[1], Theta[1] + theta[1])
				* Rot(Alpha[2], Theta[2] + theta[2])
				* Rot(Alpha[3], Theta[3] - theta[3]);

			Mat Q1(5, 3, CV_32FC1);
			Mat Z(5, 1, CV_32FC1);
			for (int i = 0; i < 5; ++i) 
			{
				Q1.at<float>(i, 0) = fingers[i].x;
				Q1.at<float>(i, 1) = fingers[i].y;
				Q1.at<float>(i, 2) = 1;
				Z.at<float>(i, 0) = fingers[i].z;
			}
			Mat invert1;
			Mat Q1T;
			transpose(Q1, Q1T);
			invert(Q1T * Q1, invert1);
			Mat THETA1 = invert1 * Q1T * Z;//(3,1)
			float a = THETA1.at<float>(0, 0);
			float b = THETA1.at<float>(1, 0);
			float c = THETA1.at<float>(2, 0);
			Vector3D avec(a, b, -1);
			if (sum((fingers[0] - palm) * avec) < 0)
				avec = avec * -1;
			norm(avec);

			Vector3D fingers_projection[5];
			for (int i = 0; i < 5; ++i) 
			{
				float t = (-a * fingers[i].x - b * fingers[i].y + fingers[i].z - c) / (a * a + b * b + 1);
				fingers_projection[i].x = a * t + fingers[i].x;
				fingers_projection[i].y = b * t + fingers[i].y;
				fingers_projection[i].z = -t + fingers[i].z;
			}

			Vector3D nvec = fingers_projection[1] - fingers_projection[0];
			norm(nvec);
			Vector3D ovec = cross(avec, nvec);
			norm(ovec);
			Mat R07_(3, 3, CV_32FC1);
			R07_.at<float>(0, 0) = nvec.x;
			R07_.at<float>(1, 0) = nvec.y;
			R07_.at<float>(2, 0) = nvec.z;
			R07_.at<float>(0, 1) = ovec.x;
			R07_.at<float>(1, 1) = ovec.y;
			R07_.at<float>(2, 1) = ovec.z;
			R07_.at<float>(0, 2) = avec.x;
			R07_.at<float>(1, 2) = avec.y;
			R07_.at<float>(2, 2) = avec.z;
			Mat Q2(4, 2, CV_32FC1);
			Mat Y(4, 1, CV_32FC1);
			for (int i = 0; i < 4; ++i) 
			{
				Q2.at<float>(i, 0) = fingers_projection[i + 1].x;
				Q2.at<float>(i, 1) = 1;
				Y.at<float>(i, 0) = fingers_projection[i + 1].y;
			}
			Mat Q2T;
			Mat invert2;
			transpose(Q2, Q2T);
			invert(Q2T * Q2, invert2);
			Mat THETA2 = invert2 * Q2T * Y;
			float k = THETA2.at<float>(0, 0);
			//float bb = THETA2.at<float>(0, 1);
			//float d = abs(bb / sqrt(k * k + 1)); // not use
			float alpha = atan(-1 / k) + pi / 2;
			Mat R07 = R07_ * Rot(0, alpha);
			Mat R04T;
			invert(R04, R04T);
			Mat R47 = R04T * R07;
			theta[4] = atan(R47.at<float>(0, 2) / R47.at<float>(2, 2));
			theta[5] = asin(R47.at<float>(0, 2) / sin(theta[4]));
			theta[6] = asin(R47.at<float>(1, 1) / sin(theta[5]));

			if (_isnan(theta[4]) || _isnan(theta[5]) || _isnan(theta[6]))
			{
				if (thetaProcess.size() > 0)
				{
					theta[4] = thetaProcess[thetaProcess.size() - 1][4];
					theta[5] = thetaProcess[thetaProcess.size() - 1][5];
					theta[6] = thetaProcess[thetaProcess.size() - 1][6];
				}
				else
				{
					theta[4] = 0;
					theta[5] = 0;
					theta[6] = 0;
				}
			}
			
			int cnt = thetaRaw.size();
			if (cnt < filtersize)
			{
				for (int i = 0; i < 7; ++i)
					thetaTotal[i] += theta[i];
			}
			else
			{
				for (int i = 0; i < 7; ++i)
					thetaTotal[i] = thetaTotal[i] + theta[i] - thetaRaw[cnt - filtersize][i];
			}

			double* tem = new double[7];
			for (auto i = 0; i < 7; ++i)
				tem[i] = theta[i];
			thetaRaw.push_back(tem);

			for (int i = 0; i < 7; ++i)
				theta[i] = thetaTotal[i] / (cnt < filtersize ? (cnt + 1) : filtersize);
			
			tem = new double[7];
			for (auto i = 0; i < 7; ++i)
				tem[i] = theta[i];
			thetaProcess.push_back(tem);

			double *rtheta = new double[7];
			myMotor->getTheta(rtheta);
			thetaRobot.push_back(rtheta);
			if (cnt % outputPeriod == 0)
			{
				_cprintf("Inverse Kinematics Result:");
				for (int i = 0; i < 7; ++i) 
				{
					_cprintf("theta%d: %.2f, ", (i+1), theta[i]);
				}
				_cprintf("\n");
				myMotor->Move(Left, theta);
			}
		}
		else
		{
			cvNamedWindow("segmentation", 0);
			imshow("segmentation", resized * 200);
			cvWaitKey(1);
		}
		_cprintf("time cost of last frame:\t%d\n", clock() - lasttime);
	}
	myWriter.release();
	ColorWriter.release();
}
void InitConsoleWindow()
{
	AllocConsole();
	HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);
	int hCrt = _open_osfhandle((long)handle, _O_TEXT);
	FILE * hf = _fdopen(hCrt, "w");
	*stdout = *hf;
}
#pragma endregion
//-----------------------------------------

// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CImitationDlg 对话框



CImitationDlg::CImitationDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_IMITATION_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CImitationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CImitationDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDOK, &CImitationDlg::OnBnClickedOk)
	ON_BN_CLICKED(openKinect, &CImitationDlg::OnBnClickedopenkinect)
	ON_BN_CLICKED(openMotor, &CImitationDlg::OnBnClickedopenmotor)
	ON_BN_CLICKED(releaseKinect, &CImitationDlg::OnBnClickedreleasekinect)
	ON_BN_CLICKED(startTracking, &CImitationDlg::OnBnClickedstarttracking)
	ON_BN_CLICKED(closeMotor, &CImitationDlg::OnBnClickedclosemotor)
	ON_BN_CLICKED(stopTracking, &CImitationDlg::OnBnClickedstoptracking)
	ON_BN_CLICKED(startImitation, &CImitationDlg::OnBnClickedstartimitation)
	ON_BN_CLICKED(stopImitation, &CImitationDlg::OnBnClickedstopimitation)
	ON_BN_CLICKED(motorHome, &CImitationDlg::OnBnClickedmotorhome)
	ON_BN_CLICKED(Photo, &CImitationDlg::OnBnClickedPhoto)
END_MESSAGE_MAP()


// CImitationDlg 消息处理程序

BOOL CImitationDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	elbowTotal.x = 0;
	elbowTotal.y = 0;
	elbowTotal.z = 0;
	wristTotal.x = 0;
	wristTotal.y = 0;
	wristTotal.z = 0;
	InitConsoleWindow();
	_cprintf("Open console OK\n\n");
	myKinect = new MyKinect(true, true, true);
	detector = new Detector();
	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CImitationDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CImitationDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CImitationDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CImitationDlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnOK();
}

void CImitationDlg::OnBnClickedopenkinect()
{
	// TODO: 在此添加控件通知处理程序代码
	int state = myKinect->open();
	switch (state)
	{
	case 0:
		_cprintf("Kinect sensor is ready!\n\n");
		break;
	case 1:
		_cprintf("Open Kinect Sensor Error!\n\n");
		break;
	case 2:
		_cprintf("Get Coordinate Mapper Error\n\n");
		break;
	case 3:
		_cprintf("Get Color Frame Source Error\n\n");
		break;
	case 4:
		_cprintf("Open Color Reader Error\n\n");
		break;
	case 5:
		_cprintf("Get Depth Frame Source Error\n\n");
		break;
	case 6:
		_cprintf("Open Depth Reader Error\n\n");
		break;
	case 7:
		_cprintf("Get Body Frame Source Error\n\n");
		break;
	case 8:
		_cprintf("Open Body Frame Reader Error\n\n");
		break;
	default:
		break;
	}
}


void CImitationDlg::OnBnClickedopenmotor()
{
	// TODO: 在此添加控件通知处理程序代码
	myMotor = new Motor(5);
	myMotor->servoOpen();
	openedMotor = true;
	_cprintf("Motor Opend!\n\n");
}


void CImitationDlg::OnBnClickedreleasekinect()
{
	// TODO: 在此添加控件通知处理程序代码
	myKinect->close();
	_cprintf("Kinect source released!\n\n");
}


void CImitationDlg::OnBnClickedstarttracking()
{
	// TODO: 在此添加控件通知处理程序代码
	Run = true;
	MainThread = new thread(Tracking);
	_cprintf("Is Tracking now....\n\n");
}


void CImitationDlg::OnBnClickedclosemotor()
{
	// TODO: 在此添加控件通知处理程序代码
	myMotor->servoClose();
	openedMotor = false;
	_cprintf("Motor is closed!\n\n");
}


void CImitationDlg::OnBnClickedstoptracking()
{
	// TODO: 在此添加控件通知处理程序代码
	Run = false;
	_cprintf("Tracking is stopped!\n");
	ofstream f1("thetaRobot.txt", ios::out);
	for (auto i = 0; i < thetaRobot.size(); ++i)
	{
		for (int j = 0; j < 7; ++j)
			f1 << thetaRobot[i][j] << " ";
		f1 << endl;
	}	
	f1.flush();
	f1.close();

	ofstream f2("thetaProcess.txt", ios::out);
	for (auto i = 0; i < thetaProcess.size(); ++i) {
		for (int j = 0; j < 7; ++j)
			f2 << thetaProcess[i][j] << " ";
		f2 << endl;
	}
	f2.flush();
	f2.close();

	ofstream f5("thetaRaw.txt", ios::out);
	for (auto i = 0; i < thetaRaw.size(); ++i) {
		for (int j = 0; j < 7; ++j)
			f5 << thetaRaw[i][j] << " ";
		f5 << endl;
	}
	f5.flush();
	f5.close();

	ofstream f3("elbowdata.txt", ios::out);
	for (auto i = 0; i < elbowRaw.size(); ++i)
		f3 << *elbowRaw[i];
	f3.flush();
	f3.close();

	ofstream f4("wristdata.txt", ios::out);
	for (auto i = 0; i < wristRaw.size(); ++i)
		f4 << *wristRaw[i];
	f4.flush();
	f4.close();

	_cprintf("File saved!\n\n");
}


void CImitationDlg::OnBnClickedstartimitation()
{
	// TODO: 在此添加控件通知处理程序代码
	Sleep(5000);
	enableMotor = true;
	_cprintf("Motor is imitating.....\n\n");
}


void CImitationDlg::OnBnClickedstopimitation()
{
	// TODO: 在此添加控件通知处理程序代码
	enableMotor = false;
	_cprintf("Motor is not imitating any more\n\n");
}


void CImitationDlg::OnBnClickedmotorhome()
{
	// TODO: 在此添加控件通知处理程序代码
	_cprintf("Motor is going to Home position....");
	myMotor->Home(Left);
	_cprintf("Motor is Home now\n\n");
}


void CImitationDlg::OnBnClickedPhoto()
{
	// TODO: 在此添加控件通知处理程序代码
	_cprintf("Smile!");
	int start = clock();
	while (clock() - start < 1000);
	_cprintf("Taking a photo....");
	imwrite("rgb.png", myKinect->getColor());
	imwrite("dep.png", myKinect->getDepth());
	_cprintf("Photo is OK\n\n");
}
