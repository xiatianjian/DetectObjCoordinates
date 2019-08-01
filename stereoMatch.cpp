#include "pch.h"

//#include <iostream>
//#include <vector>
//#include <string>
//
//#include <opencv2/opencv.hpp>
//
//using namespace std;
//using namespace cv;
//
///*START 定义一些要用到的数据-----------------------------------------------------------------*/
//	//图片大小
//int imgWidth = 1280;
//int imgHeight = 720;
//Size imgSize = Size(imgWidth, imgHeight);
//
//FileStorage fsin("intrinsics.yml", FileStorage::READ); //内参数
//FileStorage fsex("extrinsics.yml", FileStorage::READ); //外参数
//FileStorage fsconfig("config.yml", FileStorage::READ);
//
////左右相机的内参数矩阵和畸变系数
//Mat cameraMatL,
//cameraMatR,
//distcoeffsL,
//distcoeffsR;
////fsin["cameraMatL"] >> cameraMatL;
////fsin["cameraMatR"] >> cameraMatR;
////fsin["distcoeffsL"] >> distcoeffsL;
////fsin["distcoeffsR"] >> distcoeffsR;
//
////旋转向量，平移向量，旋转矩阵
//Mat R,
//T,
//RMat;
////fsex["R"] >> R;
////fsex["T"] >> T;
//
////图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
//Rect validROIL;
//Rect validROIR;
//
////校正旋转矩阵R，投影矩阵P 重投影矩阵Q
//Mat Rl, Rr,
//Pl, Pr,
//Q;
//
////映射表
//Mat mapLx, mapLy,
//mapRx, mapRy;
//
////暂存图片的矩阵
//Mat rgbImgL, grayImgL,
//rgbImgR, grayImgR,
//rectifyImgL,
//rectifyImgR;
//
////三维坐标
//Mat xyz;
//
////立体匹配相关参数
//int blockSize = 0, uniquenessRatio = 0, numDisparities = 0;
//FileStorage matchfs("stereoMatchPara.yml", FileStorage::WRITE);
//
//Ptr<StereoBM> bm = StereoBM::create(16, 9);
//
//Point origin;         //鼠标按下的起始点
//Rect selection;      //定义矩形选框
//bool selectObject = false;    //是否选择对象
//
///*END 定义一些要用到的数据-----------------------------------------------------------------*/
//
//void stereo_match(int, void*)
//{
//	//更新stereoMatchPara.yml配置文件
//	/*matchfs = FileStorage("stereoMatchPara.yml",FileStorage::WRITE);
//	matchfs << "blockSize" << blockSize;
//	matchfs << "uniquenessRatio" << uniquenessRatio;
//	matchfs << "numDisparities" << numDisparities;
//	matchfs.release();*/
//
//	bm->setBlockSize(2 * blockSize + 5);     //SAD窗口大小，5~21之间为宜
//	bm->setROI1(validROIL);
//	bm->setROI2(validROIR);
//	bm->setPreFilterCap(31);
//	bm->setMinDisparity(0);  //最小视差，默认值为0, 可以是负值，int型
//	bm->setNumDisparities(numDisparities * 16 + 16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
//	bm->setTextureThreshold(10);
//	bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
//	bm->setSpeckleWindowSize(100);
//	bm->setSpeckleRange(32);
//	bm->setDisp12MaxDiff(-1);
//	Mat disp, disp8;
//	bm->compute(rectifyImgL, rectifyImgR, disp);//输入图像必须为灰度图
//	disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式，转换为usighed8,方便展示
//
//	//计算得到的非正常值是否给值，如果为true则给值10000
//	reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
//	xyz = xyz * 16; //毫米级真实位置
//	imshow("disparity", disp8);
//	
//	/*将视差图保存到文件中*/
//	string path = string("disp\\");
//	string s1, s2;
//	stringstream stm;
//
//	s1 = string("disp_");
//	stm << blockSize;
//	stm >> s2;
//	s1 = s1 + s2;
//	path = path + s1;
//	stm.clear();
//
//	s1 = string("_");
//	stm << uniquenessRatio;
//	stm >> s2;
//	s1 = s1 + s2;
//	path = path + s1;
//	stm.clear();
//
//	s1 = string("_");
//	stm << numDisparities;
//	stm >> s2;
//	s1 = s1 + s2;
//	path = path + s1;
//	stm.clear();
//
//	path = path + ".jpg";
//
//	imwrite(path, disp8);
//}
//
//
///*
//算法功能：输入一张左相机图片和右相机图片，测试立体匹配的参数
//入口参数：
//	rgbImgLVec: 左相机的n张rgb图片组成的vector
//	rgbImgRVec: 右相机的n张rgb图片组成的vector
//*/
//void stereoMatch(vector<string> &rgbImgLVec, vector<string> &rgbImgRVec) {
//	 
//	imgWidth = fsconfig["imgWidth"];
//	imgHeight = fsconfig["imgHeight"];
//	fsin["cameraMatL"] >> cameraMatL;
//	fsin["cameraMatR"] >> cameraMatR;
//	fsin["distcoeffsL"] >> distcoeffsL;
//	fsin["distcoeffsR"] >> distcoeffsR;
//
//	//T = (Mat_<double>(3, 1) << -61.34485, 2.89570, -4.76870);//T平移向量,提前标定好的
//	//R = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);//旋转向量
//	//Rodrigues(R, RMat); //Rodrigues变换
//	//cout << RMat << endl;
//	fsex["R"] >> RMat;
//	fsex["T"] >> T;
//
//	//计算立体矫正的映射矩阵
//	//alpha=0(CALIB...后面的)表示输出是裁剪后的图像,然后再resize,并且ROI覆盖整个图像；alpha=1,输出图像和原图尺寸一样，但是可能会有黑色区域
//	stereoRectify(cameraMatL, distcoeffsL, cameraMatR, distcoeffsR, imgSize, RMat, T,
//		Rl, Rr, Pl, Pr, Q,
//		CALIB_ZERO_DISPARITY, 0,imgSize, &validROIL, &validROIR);
//
//	//得到左相机的x和y的映射表
//	initUndistortRectifyMap(cameraMatL, distcoeffsL, Rl, Pl, imgSize, CV_32FC1, mapLx, mapLy);
//
//	//得到右相机的x和y的映射表
//	initUndistortRectifyMap(cameraMatR, distcoeffsR, Rr, Pr, imgSize, CV_32FC1, mapRx, mapRy);
//
//	/*START 读取每一个图片（左右相机的），然后矫正，立体匹配*/
//	int imgNum = rgbImgLVec.size();
//	for (int i = 0; i < imgNum; ++i) {
//		/*START 矫正阶段--------------------------------------------------------------------------*/
//		//得到左右相机图片的路径
//		string rgbImgLPath = rgbImgLVec.at(i);
//		string rgbImgRPath = rgbImgRVec.at(i);
//
//		//得到左右相机的图片的Mat
//		rgbImgL = imread(rgbImgLPath, CV_LOAD_IMAGE_COLOR);
//		rgbImgR = imread(rgbImgRPath, CV_LOAD_IMAGE_COLOR);
//
//		//转换为灰度图
//		cvtColor(rgbImgL, grayImgL, CV_RGB2GRAY);
//		cvtColor(rgbImgR, grayImgR, CV_RGB2GRAY);
//
//		//显示灰度图
//		/*imshow("grayImgL before rectify", grayImgL);
//		imshow("grayImgR before rectify", grayImgR);*/
//
//		/*经过remap后，左右相机的图片已经共面并且行对准了*/
//		remap(grayImgL, rectifyImgL, mapLx, mapLy, INTER_LINEAR);
//		remap(grayImgR, rectifyImgR, mapRx, mapRy, INTER_LINEAR);
//
//		//显示矫正后的伪彩色图
//		Mat rgbRectifyImgL, rgbRectifyImgR;
//		cvtColor(rectifyImgL, rgbRectifyImgL, CV_GRAY2RGB);
//		cvtColor(rectifyImgR, rgbRectifyImgR, CV_GRAY2RGB);
//		imshow("rgbImgL after rectify", rgbRectifyImgL);
//		//imshow("rgbImgR after rectify", rgbRectifyImgR);
//
//		/*END 矫正阶段------------------------------------------------------------------------------------*/
//
//		/*START 立体匹配测试阶段,调节参数------------------------------------------------------------------------------*/
//		//stereo_match作为回调函数，参数只有两个，所以很多要用到的数据必须放到全局变量中。
//		namedWindow("disparity", CV_WINDOW_AUTOSIZE);
//		// 创建SAD窗口 Trackbar
//		createTrackbar("BlockSize:\n", "disparity", &blockSize, 8, stereo_match);
//		// 创建视差唯一性百分比窗口 Trackbar
//		createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match);
//		// 创建视差窗口 Trackbar
//		createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);
//		stereo_match(0, 0);
//		waitKey();
//
//		/*END 立体匹配测试阶段----------------------------------------------------------------------------------*/	
//	}
//}
//
//int main(int argc, char* argv[]) {
//	if (argc == 3) {
//		vector<string> rgbImgLVec;
//		vector<string> rgbImgRVec;
//
//		string rgbImgLPath = string(*(++argv));
//		string rgbImgRPath = string(*(++argv));
//		rgbImgLVec.push_back(rgbImgLPath);
//		rgbImgRVec.push_back(rgbImgRPath);
//
//		stereoMatch(rgbImgLVec, rgbImgRVec);
//	}
//	else {
//		cout << "no arguments!" << endl;
//	}
//}


/*
1. 百度上找不到任何关于立体匹配BM算法的详解
2. 为什么stereoRectify会出错
3. R和T到底是什么，为什么作者写的R和T是（3，1）的，但是在双目标定时算的是（3，3）和（3，3）的
   为什么我从双目标定得到的R用于stereoMatch中会出错
*/