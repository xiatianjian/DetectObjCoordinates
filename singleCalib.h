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
#include <opencv2/calib3d/calib3d.hpp> //����������궨�İ�

using namespace std;
using namespace cv;

/*��������Ĭ����imgPaths.txt��img��
���������calibResults.txt
*/
void singleCalib(Size image, Size square, string base);
