#pragma once
#include "pch.h"

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void getXyz(vector<string> &rgbImgLVec, vector<string> &rgbImgRVec, vector<Mat> &xyzs, vector<Mat> &newImgs);