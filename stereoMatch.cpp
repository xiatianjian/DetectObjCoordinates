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
///*START ����һЩҪ�õ�������-----------------------------------------------------------------*/
//	//ͼƬ��С
//int imgWidth = 1280;
//int imgHeight = 720;
//Size imgSize = Size(imgWidth, imgHeight);
//
//FileStorage fsin("intrinsics.yml", FileStorage::READ); //�ڲ���
//FileStorage fsex("extrinsics.yml", FileStorage::READ); //�����
//FileStorage fsconfig("config.yml", FileStorage::READ);
//
////����������ڲ�������ͻ���ϵ��
//Mat cameraMatL,
//cameraMatR,
//distcoeffsL,
//distcoeffsR;
////fsin["cameraMatL"] >> cameraMatL;
////fsin["cameraMatR"] >> cameraMatR;
////fsin["distcoeffsL"] >> distcoeffsL;
////fsin["distcoeffsR"] >> distcoeffsR;
//
////��ת������ƽ����������ת����
//Mat R,
//T,
//RMat;
////fsex["R"] >> R;
////fsex["T"] >> T;
//
////ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������  
//Rect validROIL;
//Rect validROIR;
//
////У����ת����R��ͶӰ����P ��ͶӰ����Q
//Mat Rl, Rr,
//Pl, Pr,
//Q;
//
////ӳ���
//Mat mapLx, mapLy,
//mapRx, mapRy;
//
////�ݴ�ͼƬ�ľ���
//Mat rgbImgL, grayImgL,
//rgbImgR, grayImgR,
//rectifyImgL,
//rectifyImgR;
//
////��ά����
//Mat xyz;
//
////����ƥ����ز���
//int blockSize = 0, uniquenessRatio = 0, numDisparities = 0;
//FileStorage matchfs("stereoMatchPara.yml", FileStorage::WRITE);
//
//Ptr<StereoBM> bm = StereoBM::create(16, 9);
//
//Point origin;         //��갴�µ���ʼ��
//Rect selection;      //�������ѡ��
//bool selectObject = false;    //�Ƿ�ѡ�����
//
///*END ����һЩҪ�õ�������-----------------------------------------------------------------*/
//
//void stereo_match(int, void*)
//{
//	//����stereoMatchPara.yml�����ļ�
//	/*matchfs = FileStorage("stereoMatchPara.yml",FileStorage::WRITE);
//	matchfs << "blockSize" << blockSize;
//	matchfs << "uniquenessRatio" << uniquenessRatio;
//	matchfs << "numDisparities" << numDisparities;
//	matchfs.release();*/
//
//	bm->setBlockSize(2 * blockSize + 5);     //SAD���ڴ�С��5~21֮��Ϊ��
//	bm->setROI1(validROIL);
//	bm->setROI2(validROIR);
//	bm->setPreFilterCap(31);
//	bm->setMinDisparity(0);  //��С�ӲĬ��ֵΪ0, �����Ǹ�ֵ��int��
//	bm->setNumDisparities(numDisparities * 16 + 16);//�Ӳ�ڣ�������Ӳ�ֵ����С�Ӳ�ֵ֮��,���ڴ�С������16����������int��
//	bm->setTextureThreshold(10);
//	bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio��Ҫ���Է�ֹ��ƥ��
//	bm->setSpeckleWindowSize(100);
//	bm->setSpeckleRange(32);
//	bm->setDisp12MaxDiff(-1);
//	Mat disp, disp8;
//	bm->compute(rectifyImgL, rectifyImgR, disp);//����ͼ�����Ϊ�Ҷ�ͼ
//	disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//��������Ӳ���CV_16S��ʽ��ת��Ϊusighed8,����չʾ
//
//	//����õ��ķ�����ֵ�Ƿ��ֵ�����Ϊtrue���ֵ10000
//	reprojectImageTo3D(disp, xyz, Q, true); //��ʵ�������ʱ��ReprojectTo3D������X / W, Y / W, Z / W��Ҫ����16(Ҳ����W����16)�����ܵõ���ȷ����ά������Ϣ��
//	xyz = xyz * 16; //���׼���ʵλ��
//	imshow("disparity", disp8);
//	
//	/*���Ӳ�ͼ���浽�ļ���*/
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
//�㷨���ܣ�����һ�������ͼƬ�������ͼƬ����������ƥ��Ĳ���
//��ڲ�����
//	rgbImgLVec: �������n��rgbͼƬ��ɵ�vector
//	rgbImgRVec: �������n��rgbͼƬ��ɵ�vector
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
//	//T = (Mat_<double>(3, 1) << -61.34485, 2.89570, -4.76870);//Tƽ������,��ǰ�궨�õ�
//	//R = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);//��ת����
//	//Rodrigues(R, RMat); //Rodrigues�任
//	//cout << RMat << endl;
//	fsex["R"] >> RMat;
//	fsex["T"] >> T;
//
//	//�������������ӳ�����
//	//alpha=0(CALIB...�����)��ʾ����ǲü����ͼ��,Ȼ����resize,����ROI��������ͼ��alpha=1,���ͼ���ԭͼ�ߴ�һ�������ǿ��ܻ��к�ɫ����
//	stereoRectify(cameraMatL, distcoeffsL, cameraMatR, distcoeffsR, imgSize, RMat, T,
//		Rl, Rr, Pl, Pr, Q,
//		CALIB_ZERO_DISPARITY, 0,imgSize, &validROIL, &validROIR);
//
//	//�õ��������x��y��ӳ���
//	initUndistortRectifyMap(cameraMatL, distcoeffsL, Rl, Pl, imgSize, CV_32FC1, mapLx, mapLy);
//
//	//�õ��������x��y��ӳ���
//	initUndistortRectifyMap(cameraMatR, distcoeffsR, Rr, Pr, imgSize, CV_32FC1, mapRx, mapRy);
//
//	/*START ��ȡÿһ��ͼƬ����������ģ���Ȼ�����������ƥ��*/
//	int imgNum = rgbImgLVec.size();
//	for (int i = 0; i < imgNum; ++i) {
//		/*START �����׶�--------------------------------------------------------------------------*/
//		//�õ��������ͼƬ��·��
//		string rgbImgLPath = rgbImgLVec.at(i);
//		string rgbImgRPath = rgbImgRVec.at(i);
//
//		//�õ����������ͼƬ��Mat
//		rgbImgL = imread(rgbImgLPath, CV_LOAD_IMAGE_COLOR);
//		rgbImgR = imread(rgbImgRPath, CV_LOAD_IMAGE_COLOR);
//
//		//ת��Ϊ�Ҷ�ͼ
//		cvtColor(rgbImgL, grayImgL, CV_RGB2GRAY);
//		cvtColor(rgbImgR, grayImgR, CV_RGB2GRAY);
//
//		//��ʾ�Ҷ�ͼ
//		/*imshow("grayImgL before rectify", grayImgL);
//		imshow("grayImgR before rectify", grayImgR);*/
//
//		/*����remap�����������ͼƬ�Ѿ����沢���ж�׼��*/
//		remap(grayImgL, rectifyImgL, mapLx, mapLy, INTER_LINEAR);
//		remap(grayImgR, rectifyImgR, mapRx, mapRy, INTER_LINEAR);
//
//		//��ʾ�������α��ɫͼ
//		Mat rgbRectifyImgL, rgbRectifyImgR;
//		cvtColor(rectifyImgL, rgbRectifyImgL, CV_GRAY2RGB);
//		cvtColor(rectifyImgR, rgbRectifyImgR, CV_GRAY2RGB);
//		imshow("rgbImgL after rectify", rgbRectifyImgL);
//		//imshow("rgbImgR after rectify", rgbRectifyImgR);
//
//		/*END �����׶�------------------------------------------------------------------------------------*/
//
//		/*START ����ƥ����Խ׶�,���ڲ���------------------------------------------------------------------------------*/
//		//stereo_match��Ϊ�ص�����������ֻ�����������Ժܶ�Ҫ�õ������ݱ���ŵ�ȫ�ֱ����С�
//		namedWindow("disparity", CV_WINDOW_AUTOSIZE);
//		// ����SAD���� Trackbar
//		createTrackbar("BlockSize:\n", "disparity", &blockSize, 8, stereo_match);
//		// �����Ӳ�Ψһ�԰ٷֱȴ��� Trackbar
//		createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match);
//		// �����Ӳ�� Trackbar
//		createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);
//		stereo_match(0, 0);
//		waitKey();
//
//		/*END ����ƥ����Խ׶�----------------------------------------------------------------------------------*/	
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
1. �ٶ����Ҳ����κι�������ƥ��BM�㷨�����
2. ΪʲôstereoRectify�����
3. R��T������ʲô��Ϊʲô����д��R��T�ǣ�3��1���ģ�������˫Ŀ�궨ʱ����ǣ�3��3���ͣ�3��3����
   Ϊʲô�Ҵ�˫Ŀ�궨�õ���R����stereoMatch�л����
*/