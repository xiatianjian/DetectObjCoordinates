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
�㷨���ܣ���һ���ַ�����һ���ַ����ָ�ɶ���ַ���������vector��
*/
vector<string> splitStr(const string &s, const string &seperator) 
{

	vector<string> result;
	typedef string::size_type string_size;
	string_size i = 0;

	while (i != s.size()) {
		//�ҵ��ַ������׸������ڷָ�������ĸ��
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

		//�ҵ���һ���ָ������������ָ���֮����ַ���ȡ����
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
�㷨���ܣ�ȡn�������ͼƬ��n�������ͼƬ�����棬�õ�����ͼƬ��xyzs�����������壬�õ�ÿ�������xyz�ı仯
����������У�����õ������⣬����׷�٣�������ȵĺ���
��ڲ�����
	videoFil: ����Ϊ�գ�����Ҫ����Ƶ���е�ͼƬ�����������������Ƭ
	trackMode: ׷���㷨��ѡ��
���ڲ�����
	variety: �õ�ÿ�������xyz�ı仯��
			 ���Ƕ�άvector����һά��ʾ���壬�ڶ�ά��ʾͼƬ
	objs: ��¼���м�⵽������
*/
int getVarietyOfCoordOfObjs(string videoFil, string trackMode,
	vector<vector<pair<int,Vec3f>>> &variety,
	vector<string> &objs) 
{

	FileStorage configfs("config.yml", FileStorage::READ);
	/*START ���û���������--------------------------------------------*/
	string videoFile;
	VideoCapture cap;

	//���ò�����Ƶ�Ŀ�͸�(�����������ͼƬ����ܺ�)
	int imgWidth = 2560; //�����4�ı���
	int imgHeight = 960;
	imgWidth = configfs["imgWidth"];
	imgHeight = configfs["imgHeight"];

	//ͼƬMat
	Mat frame,
		frameL, frameR;
	
	//�����ܹ�Ҫ��ȡ��ͼƬ��֡��
	int frameNum = 10;
	frameNum = configfs["frameNum"];

	//����ÿ������ms��ȡһ֡
	int delay = 10; 
	delay = configfs["delay"];

	//����ͼ���ļ��Ļ���·��
	string leftBaseStr, rightBaseStr;
	leftBaseStr = configfs["leftBase"];
	rightBaseStr = configfs["rightBase"];
	const char* leftBase = leftBaseStr.c_str();
	const char* rightBase = rightBaseStr.c_str();
	
	
	//�����ļ���·��
	char leftImgPath[100];
	char rightImgPath[100];

	//�������ͼƬ·���ļ���
	vector<string> rgbImgLVec;
	vector<string> rgbImgRVec;

	//�������ɵ���������������ͼƬ��Mat
	vector<Mat> xyzs;
	vector<Mat> newImgs;

	//��ȡ���������ɵ�txt�ļ����������
	ifstream objdetFin("position.txt");
	string objdetLine; //�ļ��е�һ��

	//��һ��ͼƬ�������boxpoint,�ó���Ϊ4�������ʾ,��objsһһ��Ӧ
	vector<Rect2d> boxpointOfFirst; 

	//������
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

	/*END ���û���������----------------------------------------------------*/

	/*����Ƿ��ǵ�һ�����г������ǣ�������˫Ŀ�궨����*/
	FileStorage firstRunfs("firstRun.yml", FileStorage::READ);
	int firstRunFlag;
	firstRunFlag = (int)firstRunfs["firstRunFlag"];
	if (firstRunFlag == 1) { //����ǵ�һ������
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

	/*����cap*/
	if (videoFile == "") {
		cap.open(0);

		cap.set(CV_CAP_PROP_FRAME_WIDTH, imgWidth); //���ò�����Ƶ�Ŀ�� 
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgHeight);
	}
	else {
		cap.open(videoFile);
	}
	
	/*���ܻ�����������ͷ���-1*/
	if (!cap.isOpened()) {
		cout << "��ȡ������Ƶ��" << endl;
		return -1;
	}

	//while(true){
	for (int i = 0; i < frameNum; ++i) {
		//������
		if (!cap.read(frame)) {
			cout << "no video frame" << endl;
			break;
		}

		/*�Ѷ�������Ƶ֡���д������浽�ļ���*/
		//��Ϊ�����ͼƬ�������ͼƬ
		int width = int(imgWidth / 4);
		int height = imgHeight / 2;
		frameL = frame(Rect(0, 0, width, height));
		frameR = frame(Rect(width, 0, width, height));

		//չʾ��Ƶ
		/*namedWindow("video left", 1);
		imshow("video left", frameL);

		namedWindow("video right", 2);
		imshow("video right", frameR);*/

		//���浽�ļ���
		sprintf_s(leftImgPath, leftBase, i);
		sprintf_s(rightImgPath, rightBase, i);
		imwrite(leftImgPath, frameL);
		imwrite(rightImgPath, frameR);
		/*END */

		//ͼƬ·�����浽rgbImgVec��
		rgbImgLVec.push_back(leftImgPath);
		rgbImgRVec.push_back(rightImgPath);

		//�ӳ�
		waitKey(delay);
	}

	//����xyzs�ͽ������α��ɫͼƬ
	getXyz(rgbImgLVec, rgbImgRVec, xyzs, newImgs);

	/*���ڵõ�����Ƭ�ϸ����ص��ʵ�����꣬��ô�Ϳ�ʼ�ý���ͼƬ�õ������boxpoint*/
	
	//.......
	//�����������������룬����һ��ͼƬ�������������Ϣ��txt�ļ�position
	//......

	while (getline(objdetFin, objdetLine)) {
		
		//�ָ��ַ������õ���Ϣ
		vector<string> splits = splitStr(objdetLine, " ");

		//��ȡ����������
		objs.push_back(splits.at(0));
		
		//��ȡ�������boxpoint, תΪRect2d����
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

	/*�����һ��ͼƬ�������������������*/
	variety.resize(boxpointOfFirst.size()); //����variety�Ĵ�СΪ��⵽������Ĵ�С
	for (int i = 0; i < boxpointOfFirst.size(); ++i) {
		Rect2d tempBox = boxpointOfFirst.at(i);
		int x, y, width, height;

		x = tempBox.x;
		y = tempBox.y;
		width = tempBox.width;
		height = tempBox.height;

		int centerX = (x + width) / 2;
		int centerY = (y + height) / 2;

		//���������������
		Vec3f vec3f;
		vec3f = xyzs.at(0).at<Vec3f>(Point(centerX,centerY));

		pair<int, Vec3f> p(i, vec3f);
		variety.at(i).push_back(p);
	}

	/*��������������������ȷ�ԣ��鿴�����ͼƬ�����boxpointOfFirst,���objs,�����һ�����������*/
	cout << "�����һ��ͼƬ��⵽�������boxpoint" << endl;
	for (int i = 0; i < boxpointOfFirst.size(); ++i) {
		Rect temp = boxpointOfFirst.at(i);
		cout << temp.x << "  " << temp.y << "  " << temp.width << "  " << temp.height;
		cout << endl;
	}
	cout << "�����һ���������������XYZ(��λ��mm)" << endl;
	cout << variety.at(0).at(0).second << endl;


	/*�Ժ�����9�ţ�����٣�n-1��ͼƬ��������׷��*/
	Mat firstFrame = newImgs.at(0);
	Mat frameToTrace;
	for (int i = 0; i < boxpointOfFirst.size(); ++i) { //����ÿһ������
		Rect box = boxpointOfFirst.at(i);
		//ÿ��׷��һ������ʱҪ��ʼ��tracker
		tracker->init(firstFrame, box); 

		for (int j = 1; j < newImgs.size(); ++j) {
			//��ȡ�������Ƭ
			frameToTrace = newImgs.at(j);

			//�������׷�ٵĽ��,boxpoint
			Rect2d boxToTrace;
			tracker->update(frameToTrace, boxToTrace);
			
			//��ѯXYZ,���浽�������variety��;����Ĭ���ܹ�׷�ٵ�
			int x, y, width, height;
			x = boxToTrace.x;
			y = boxToTrace.y;
			width = boxToTrace.width;
			height = boxToTrace.height;

			int centerX = (x + width) / 2;
			int centerY = (y + height) / 2;

			//���������������
			Vec3f vec3f;
			vec3f = xyzs.at(j).at<Vec3f>(Point(centerX, centerY));
			pair<int, Vec3f> p(i, vec3f);
			variety.at(i).push_back(p);

			cout << "��"<<j<<"��ͼƬ����ά����" << p.second << endl;
		}
	}
	
}

int main() 
{
	
	vector<vector<pair<int, Vec3f>>> variety;
	vector<string> objs;
	getVarietyOfCoordOfObjs("", "KCF", variety, objs);

	for (int i = 0; i < variety.size(); ++i) {
		cout << "��" << i << "������" << endl;
		vector<pair<int, Vec3f>> vec1 = variety.at(i);
		for (int j = 0; j < vec1.size(); ++j) {
			cout << vec1.at(j).second << "  ";
		}
	}

	return 0;
}
/*
1. ע��cap.set(CV_CAP_PROP_FRAME_WIDTH, imgWidth)��cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgHeight)��width��height������
2. Ϊʲô����������ͼƬ�ǰ�һ��ͼƬ�����룬�ѵ������������һ���豸�ţ�
3. �������������Ϊposition.txt�� һ�еĸ�ʽΪobj x y width height
4. ������ͨ��boxpoint��ѯ�������������ʱ��û�п���ֵ����Ч�ԣ����¿��ܳ��ֺ����׵�ֵ��
5. tracker->update��һ���ٲ���������ô��
*/