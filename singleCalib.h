#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include<vector>
#include<string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp> //用于摄像机标定的包

using namespace std;
using namespace cv;

/*输入数据默认在imgPaths.txt和img中
输出数据在calibResults.txt
*/
void singleCalib(Size image, Size square, string base);
