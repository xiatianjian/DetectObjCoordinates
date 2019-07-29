#include "pch.h"
#include "getXyz.h"
#include "stereoCalib.h"

#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
#include<stdio.h>

#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/videoio.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/tracking.hpp>

using namespace std;
using namespace cv;

#include <vector>
#include <string>
#include <iostream>
using namespace std;

/*
算法功能：按一种字符，将一个字符串分割成多个字符串，放在vector中
*/
vector<string> splitStr(const string &s, const string &seperator) 
{

	vector<string> result;
	typedef string::size_type string_size;
	string_size i = 0;

	while (i != s.size()) {
		//找到字符串中首个不等于分隔符的字母；
		int flag = 0;
		while (i != s.size() && flag == 0) {
			flag = 1;
			for (string_size x = 0; x < seperator.size(); ++x)
				if (s[i] == seperator[x]) {
					++i;
					flag = 0;
					break;
				}
		}

		//找到又一个分隔符，将两个分隔符之间的字符串取出；
		flag = 0;
		string_size j = i;
		while (j != s.size() && flag == 0) {
			for (string_size x = 0; x < seperator.size(); ++x)
				if (s[j] == seperator[x]) {
					flag = 1;
					break;
				}
			if (flag == 0)
				++j;
		}
		if (i != j) {
			result.push_back(s.substr(i, j - i));
			i = j;
		}
	}

	return result;
}



/*
算法功能：取n张左相机图片和n张右相机图片，保存，得到矫正图片和xyzs，检测跟踪物体，得到每个物体的xyz的变化
在这个过程中，会调用到物体检测，物体追踪，计算深度的函数
入口参数：
	videoFil: 若不为空，表明要用视频流中的图片，否则用相机拍摄照片
	trackMode: 追踪算法的选择
出口参数：
	variety: 得到每个物体的xyz的变化。
			 这是二维vector，第一维表示物体，第二维表示图片
	objs: 记录所有检测到的物体
*/
int getVarietyOfCoordOfObjs(string videoFil, string trackMode,
	vector<vector<pair<int,Vec3f>>> &variety,
	vector<string> &objs) 
{

	FileStorage configfs("config.yml", FileStorage::READ);
	/*START 设置基本的数据--------------------------------------------*/
	string videoFile;
	VideoCapture cap;

	//设置捕获视频的宽和高(宽是左右相机图片宽的总和)
	int imgWidth = 2560; //最好是4的倍数
	int imgHeight = 960;
	imgWidth = configfs["imgWidth"];
	imgHeight = configfs["imgHeight"];

	//图片Mat
	Mat frame,
		frameL, frameR;
	
	//设置总共要读取的图片的帧数
	int frameNum = 10;
	frameNum = configfs["frameNum"];

	//设置每隔多少ms读取一帧
	int delay = 10; 
	delay = configfs["delay"];

	//保存图像文件的基本路径
	string leftBaseStr, rightBaseStr;
	leftBaseStr = configfs["leftBase"];
	rightBaseStr = configfs["rightBase"];
	const char* leftBase = leftBaseStr.c_str();
	const char* rightBase = rightBaseStr.c_str();
	
	
	//保存文件的路径
	char leftImgPath[100];
	char rightImgPath[100];

	//左右相机图片路径的集合
	vector<string> rgbImgLVec;
	vector<string> rgbImgRVec;

	//保存生成的世界坐标矩阵和新图片的Mat
	vector<Mat> xyzs;
	vector<Mat> newImgs;

	//读取物体检测生成的txt文件所需的数据
	ifstream objdetFin("position.txt");
	string objdetLine; //文件中的一段

	//第一个图片的物体的boxpoint,用长度为4的数组表示,和objs一一对应
	vector<Rect2d> boxpointOfFirst; 

	//跟踪器
	Ptr<Tracker> tracker;
	if(trackMode == string("MIL")){
		tracker = TrackerMIL::create();
	}
	else if (trackMode == string("KCF")) {
		tracker = TrackerKCF::create();
	}
	else if (trackMode == string("TLD")) {
		tracker = TrackerTLD::create();
	}
	else if(trackMode == string("MEDIANFLOW")){
		tracker = TrackerMedianFlow::create();
	}
	else if(trackMode == string("GOTURN")){
		tracker = TrackerGOTURN::create();
	}
	else if (trackMode == string("CSRT")) {
		tracker = TrackerCSRT::create();
	}
	else if (trackMode == string("Boosting")) {
		tracker = TrackerBoosting::create();
	}
	else if (trackMode == string("MOSSE")) {
		tracker = TrackerMOSSE::create();
	}
	else {
		tracker = TrackerKCF::create();
	}

	/*END 设置基本的数据----------------------------------------------------*/

	/*检查是否是第一次运行程序，若是，则运行双目标定函数*/
	FileStorage firstRunfs("firstRun.yml", FileStorage::READ);
	int firstRunFlag;
	firstRunFlag = (int)firstRunfs["firstRunFlag"];
	if (firstRunFlag == 1) { //如果是第一次运行
		int boardWidth, boardHeight, squareWidth, squareHeight, frameNumber;
		Mat zs;

		boardWidth = firstRunfs["boardWidth"];
		boardHeight = firstRunfs["boardHeight"];
		squareWidth = firstRunfs["squareWidth"];
		squareHeight = firstRunfs["squareHeight"];
		frameNumber = firstRunfs["frameNumber"];
 		firstRunfs["zs"] >> zs;

		stereoCalib(Size((imgWidth/2), imgHeight), Size(boardWidth, boardHeight), 
			Size(squareWidth, squareHeight), frameNumber, zs);
	}

	videoFile = videoFil;

	/*设置cap*/
	if (videoFile == "") {
		cap.open(0);

		cap.set(CV_CAP_PROP_FRAME_WIDTH, imgWidth); //设置捕获视频的宽度 
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgHeight);
	}
	else {
		cap.open(videoFile);
	}
	
	/*可能会出错，若出错，就返回-1*/
	if (!cap.isOpened()) {
		cout << "读取不到视频流" << endl;
		return -1;
	}

	//while(true){
	for (int i = 0; i < frameNum; ++i) {
		//读完了
		if (!cap.read(frame)) {
			cout << "no video frame" << endl;
			break;
		}

		/*把读到的视频帧进行处理并保存到文件中*/
		//分为左相机图片和右相机图片
		int width = int(imgWidth / 4);
		int height = imgHeight / 2;
		frameL = frame(Rect(0, 0, width, height));
		frameR = frame(Rect(width, 0, width, height));

		//展示视频
		/*namedWindow("video left", 1);
		imshow("video left", frameL);

		namedWindow("video right", 2);
		imshow("video right", frameR);*/

		//保存到文件中
		sprintf_s(leftImgPath, leftBase, i);
		sprintf_s(rightImgPath, rightBase, i);
		imwrite(leftImgPath, frameL);
		imwrite(rightImgPath, frameR);
		/*END */

		//图片路径保存到rgbImgVec中
		rgbImgLVec.push_back(leftImgPath);
		rgbImgRVec.push_back(rightImgPath);

		//延迟
		waitKey(delay);
	}

	//计算xyzs和矫正后的伪彩色图片
	getXyz(rgbImgLVec, rgbImgRVec, xyzs, newImgs);

	/*现在得到了照片上各像素点的实际坐标，那么就开始用矫正图片得到物体的boxpoint*/
	
	//.......
	//在这里调用物体检测代码，检测第一张图片，获得物体检测信息的txt文件position
	//......

	while (getline(objdetFin, objdetLine)) {
		
		//分割字符串，得到信息
		vector<string> splits = splitStr(objdetLine, " ");

		//提取出物体名字
		objs.push_back(splits.at(0));
		
		//提取出物体的boxpoint, 转为Rect2d类型
		vector<int> tempBox;
		Rect2d tempBox2;
		for (int i = 1; i < 5; ++i) {
			string s = splits.at(i);
			int temp = stoi(s); //str to int
			
			tempBox.push_back(temp);
		}
		tempBox2.x = tempBox.at(0);
		tempBox2.y = tempBox.at(1);
		tempBox2.width = tempBox.at(2);
		tempBox2.height = tempBox.at(3);

		boxpointOfFirst.push_back(tempBox2);
	}

	/*计算第一张图片上所有物体的世界坐标*/
	variety.resize(boxpointOfFirst.size()); //调整variety的大小为检测到的物体的大小
	for (int i = 0; i < boxpointOfFirst.size(); ++i) {
		Rect2d tempBox = boxpointOfFirst.at(i);
		int x, y, width, height;

		x = tempBox.x;
		y = tempBox.y;
		width = tempBox.width;
		height = tempBox.height;

		int centerX = (x + width) / 2;
		int centerY = (y + height) / 2;

		//该物体的世界坐标
		Vec3f vec3f;
		vec3f = xyzs.at(0).at<Vec3f>(Point(centerX,centerY));

		pair<int, Vec3f> p(i, vec3f);
		variety.at(i).push_back(p);
	}

	/*在这里测试上述代码的正确性，查看保存的图片，输出boxpointOfFirst,输出objs,输出第一个物体的坐标*/
	cout << "输出第一张图片检测到的物体的boxpoint" << endl;
	for (int i = 0; i < boxpointOfFirst.size(); ++i) {
		Rect temp = boxpointOfFirst.at(i);
		cout << temp.x << "  " << temp.y << "  " << temp.width << "  " << temp.height;
		cout << endl;
	}
	cout << "输出第一个物体的世界坐标XYZ(单位是mm)" << endl;
	cout << variety.at(0).at(0).second << endl;


	/*对后续的9张（或更少，n-1）图片进行物体追踪*/
	Mat firstFrame = newImgs.at(0);
	Mat frameToTrace;
	for (int i = 0; i < boxpointOfFirst.size(); ++i) { //遍历每一个物体
		Rect box = boxpointOfFirst.at(i);
		//每次追踪一个物体时要初始化tracker
		tracker->init(firstFrame, box); 

		for (int j = 1; j < newImgs.size(); ++j) {
			//读取拍摄的照片
			frameToTrace = newImgs.at(j);

			//存放物体追踪的结果,boxpoint
			Rect2d boxToTrace;
			tracker->update(frameToTrace, boxToTrace);
			
			//查询XYZ,保存到输出参数variety中;而且默认能够追踪到
			int x, y, width, height;
			x = boxToTrace.x;
			y = boxToTrace.y;
			width = boxToTrace.width;
			height = boxToTrace.height;

			int centerX = (x + width) / 2;
			int centerY = (y + height) / 2;

			//该物体的世界坐标
			Vec3f vec3f;
			vec3f = xyzs.at(j).at<Vec3f>(Point(centerX, centerY));
			pair<int, Vec3f> p(i, vec3f);
			variety.at(i).push_back(p);

			cout << "第"<<j<<"张图片的三维坐标" << p.second << endl;
		}
	}
	
}

int main() 
{
	
	vector<vector<pair<int, Vec3f>>> variety;
	vector<string> objs;
	getVarietyOfCoordOfObjs("", "KCF", variety, objs);

	for (int i = 0; i < variety.size(); ++i) {
		cout << "第" << i << "个物体" << endl;
		vector<pair<int, Vec3f>> vec1 = variety.at(i);
		for (int j = 0; j < vec1.size(); ++j) {
			cout << vec1.at(j).second << "  ";
		}
	}

	return 0;
}
/*
1. 注意cap.set(CV_CAP_PROP_FRAME_WIDTH, imgWidth)和cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgHeight)中width和height的设置
2. 为什么获得左右相机图片是把一个图片分两半，难道左右相机共用一个设备号？
3. 物体检测代码的输出为position.txt。 一行的格式为obj x y width height
4. 由于在通过boxpoint查询物体的世界坐标时，没有考虑值的有效性，导致可能出现很离谱的值。
5. tracker->update万一跟踪不到物体怎么办
*/