#pragma once
#include <iostream>
#include <WinSock2.h>
#include <stdio.h>
#include <opencv.hpp>
using namespace cv;
void floodFillDepth(Mat &Img, Mat &Mask, UINT16 last, int x, int y, int threshold) {
	int r = Img.size().height;
	int c = Img.size().width;
	if (x >= 0 && x < r && y >= 0 && y < c && Mask.at<UINT16>(x, y) == 0 && fabs((double)(Img.at<UINT16>(x, y) - last)) <= threshold) {
		Mask.at<UINT16>(x, y) = 1;
		floodFillDepth(Img, Mask, Img.at<UINT16>(x, y), x + 1, y, threshold);
		floodFillDepth(Img, Mask, Img.at<UINT16>(x, y), x - 1, y, threshold);
		floodFillDepth(Img, Mask, Img.at<UINT16>(x, y), x, y + 1, threshold);
		floodFillDepth(Img, Mask, Img.at<UINT16>(x, y), x, y - 1, threshold);
	}
}