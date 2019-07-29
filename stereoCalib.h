#pragma once
#include "singleCalib.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include<vector>
#include<string>
#include <algorithm>
#include <stdlib.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp> //用于摄像机标定的包

using namespace std;
using namespace cv;

void outputCameraParam(Mat cameraMatL, Mat distcoeffsL, Mat cameraMatR, Mat distcoeffsR,
	Mat R, Mat T, Mat Rl, Mat Rr, Mat Pl, Mat Pr, Mat Q);
void calRealPoint(int boardWidth, int boardHeight, int imgNumber, int squareSize, Mat zs, vector<vector<Point3f>>& obj);
void stereoCalib(Size imgSiz, Size boardSiz, Size squareSiz, int frameNum, Mat zs);