#include "pch.h"
#include "singleCalib.h"
#include "stereoCalib.h"


void outputCameraParam(Mat cameraMatL, Mat distcoeffsL, Mat cameraMatR, Mat distcoeffsR,
	Mat R, Mat T, Mat Rl, Mat Rr, Mat Pl, Mat Pr, Mat Q)
{
	/*��������*/
	/*�������*/
	FileStorage fs("intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "cameraMatL" << cameraMatL << "distcoeffsL" << distcoeffsL << "cameraMatR" << cameraMatR << "distcoeffsR" << distcoeffsR;
		fs.release();
		cout << "cameraMatrixL=:" << cameraMatL << endl << "cameraDistcoeffL=:" << distcoeffsL << endl << "cameraMatrixR=:" << cameraMatR << endl << "cameraDistcoeffR=:" << distcoeffsR << endl;
	}
	else
	{
		cout << "Error: can not save the intrinsics!!!!" << endl;
	}

	fs.open("extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr" << Pr << "Q" << Q;
		cout << "R=" << R << endl << "T=" << T << endl << "Rl=" << Rl << endl << "Rr" << Rr << endl << "Pl" << Pl << endl << "Pr" << Pr << endl << "Q" << Q << endl;
		fs.release();
	}
	else
	{
		cout << "Error: can not save the extrinsic parameters\n";
	}

}




/*����궨�����ڽǵ�����������ϵ�µ�����*/
void calRealPoint(int boardWidth, int boardHeight, int imgNumber, int squareSize, vector<vector<Point3f>>& obj)
{
	
	for (int i = 0; i < imgNumber; ++i) {
		vector<Point3f> imgpoint;

		for (int rowIndex = 0; rowIndex < boardHeight; rowIndex++)
		{
			for (int colIndex = 0; colIndex < boardWidth; colIndex++)
			{
				imgpoint.push_back(Point3f(rowIndex * squareSize, colIndex * squareSize, 0));
			}
		}
		
		obj.push_back(imgpoint);
	}
}

/*
�㷨���ܣ�˫Ŀ�궨��
ע������ļ����б���Ҫ��ͼƬ�������õ���ͼƬ��С�������Ժ�������Ƭʱ�õ���ͼƬ��С��Ҳ�������ʵ��ͼƬ��Сһ�¡�
������
	frameNum: �궨����Ҫ��ͼƬ��֡��
	zs: (n,1)�ĵ�ͨ����������frameNum���궨������������ϵ�е����ֵ��zֵ��
*/
void stereoCalib(Size imgSiz, Size boardSiz, Size squareSiz, int frameNum) {

	/*START ˫Ŀ�궨�õ�������-------------------------------------------------------*/
	//����ͷ�ķֱ���
	int imgWidth = imgSiz.width;
	int imgHeight = imgSiz.height;
	Size imgSize = Size(imgWidth, imgHeight);

	//�ǵ�����
	int boardWidth = boardSiz.width;
	int boardHeight = boardSiz.height;
	Size boardSize = Size(boardWidth, boardHeight);

	//�ܵĽǵ���Ŀ
	int cornerNum = boardWidth * boardHeight;

	//�궨�������̸�Ĵ�С����λmm
	Size squareSize = squareSiz;

	//����궨ʱ��Ҫ���õ�ͼ��֡��
	int frameNumber = frameNum;

	//��������������Ƭ�ǵ�����꼯��
	vector<vector<Point2f>> cornersOfAllImgsL;
	//�ұ������������Ƭ�ǵ�����꼯��
	vector<vector<Point2f>> cornersOfAllImgsR;

	//��ͼ��Ľǵ��ʵ�ʵ��������꼯��
	vector<vector<Point3f>> objRealPoint;

	/*���ȱ궨�õ���������ڲξ���
	fx 0 cx
	0 fy cy
	0  0  1
	*/
	Mat cameraMatL;
	Mat distcoeffsL;
	Mat cameraMatR;
	Mat distcoeffsR;

	/*��ȡyml�ļ�*/
	FileStorage fsL;
	FileStorage fsR;

	/*END ˫Ŀ�궨�õ�������-----------------------------------------------------------------------*/

	/*START ˫Ŀ�궨Ҫ���������----------------------------------------------------------------------*/
	//R��ת���� Tƽ��ʸ�� E�������� F��������
	Mat R, T, E, F;

	//У����ת����R��ͶӰ����P����ͶӰ����Q
	Mat Rl, Rr, Pl, Pr, Q;

	//ͼ��У��֮�󣬻��ͼ����вü������У�validROI�ü�֮�������
	Rect validROIL, validROIR;

	//ӳ���
	Mat mapLx, mapLy, mapRx, mapRy;

	/*END ˫Ŀ�궨Ҫ���������*/


	vector<Point2f> cornerL; // ��������ĳһ��Ƭ�ǵ����꼯��
	vector<Point2f> cornerR; // �ұ������ĳһ��Ƭ�ǵ����꼯��

	Mat rgbImgL, grayImgL;
	Mat rgbImgR, grayImgR;


	/*������������ڲ����ͻ���ϵ��*/
	string leftBase = "img\\left\\";
	//singleCalib(boardSize, squareSize, leftBase);
	fsL = FileStorage(leftBase + "singleCalibResults.yml", FileStorage::READ);
	fsL["cameraMat"] >> cameraMatL;
	fsL["distcoeffs"] >> distcoeffsL;

	/*������������ڲ����ͻ���ϵ��*/
	string rightBase = "img\\right\\";
	//singleCalib(boardSize, squareSize, rightBase);
	fsR = FileStorage(rightBase + "singleCalibResults.yml", FileStorage::READ);
	fsR["cameraMat"] >> cameraMatR;
	fsR["distcoeffs"] >> distcoeffsR;

	Mat img;
	int frameCount = 0;

	while (frameCount < frameNumber)
	{
		char filename[100];

		/*��ȡ��ߵ�ͼ��*/
		sprintf_s(filename, "img\\left\\%d.jpg", frameCount);
		//sprintf_s(filename, "img\\left\\0.jpg");
		rgbImgL = imread(filename, CV_LOAD_IMAGE_COLOR); 
		cvtColor(rgbImgL, grayImgL, CV_BGR2GRAY);

		/*��ȡ�ұߵ�ͼ��*/
		sprintf_s(filename, "img\\right\\%d.jpg", frameCount);
		//sprintf_s(filename, "img\\right\\0.jpg");
		rgbImgR = imread(filename, CV_LOAD_IMAGE_COLOR);
		cvtColor(rgbImgR, grayImgR, CV_BGR2GRAY);

		bool isFindL, isFindR;
		isFindL = findChessboardCorners(rgbImgL, boardSize, cornerL);
		isFindR = findChessboardCorners(rgbImgR, boardSize, cornerR);
		if (isFindL == true && isFindR == true)
		{
			cornerSubPix(grayImgL, cornerL, Size(5, 5), Size(-1, 1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(rgbImgL, boardSize, cornerL, isFindL);
			imshow("chessboardL", rgbImgL);
			cornersOfAllImgsL.push_back(cornerL);

			cornerSubPix(grayImgR, cornerR, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(rgbImgR, boardSize, cornerR, isFindR);
			imshow("chessboardR", rgbImgR);
			cornersOfAllImgsR.push_back(cornerR);

			frameCount++;
			cout << "the image" << frameCount << " is good" << endl;
		}
		else
		{
			cout << "the image is bad please try again" << endl;
		}
		if (waitKey(10) == 'q')
		{
			break;
		}
	}

	//����ʵ�ʵ�У�������ά���꣬����ʵ�ʱ궨���ӵĴ�С������
	calRealPoint(boardWidth, boardHeight, frameNumber, squareSize.width, objRealPoint);
	cout << "cal real successful" << endl;

	/*�궨����ͷ*/
	double rms = stereoCalibrate(objRealPoint, cornersOfAllImgsL, cornersOfAllImgsR,
		cameraMatL, distcoeffsL,
		cameraMatR, distcoeffsR,
		imgSize, R, T, E, F, CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	cout << "Stereo Calibration done with RMS error = " << rms << endl;

	/*�����У��ӳ��*/
	stereoRectify(cameraMatL, distcoeffsL, cameraMatR, distcoeffsR, imgSize, R, T, Rl,
		Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY, -1, imgSize, &validROIL, &validROIR);

	initUndistortRectifyMap(cameraMatL, distcoeffsL, Rl, Pl, imgSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatR, distcoeffsR, Rr, Pr, imgSize, CV_32FC1, mapRx, mapRy);

	Mat rectifyImageL, rectifyImageR;
	cvtColor(grayImgL, rectifyImageL, CV_GRAY2BGR);
	cvtColor(grayImgR, rectifyImageR, CV_GRAY2BGR);

	imshow("Recitify Before", rectifyImageL);
	cout << "��Q1�˳�..." << endl;
	//����remap֮�����������ͼ���Ѿ����沢���ж�׼��
	Mat rectifyImageL2, rectifyImageR2;
	remap(rectifyImageL, rectifyImageL2, mapLx, mapLy, INTER_LINEAR);
	remap(rectifyImageR, rectifyImageR2, mapRx, mapRy, INTER_LINEAR);
	cout << "��Q2�˳�..." << endl;

	imshow("rectifyImageL", rectifyImageL2);
	imshow("rectifyImageR", rectifyImageR2);
	imwrite("elseImg\\rectifyImageL.jpg", rectifyImageL2);
	imwrite("elseImg\\rectifyImageR.jpg", rectifyImageR2);

	//����궨���
	outputCameraParam(cameraMatL, distcoeffsL, cameraMatR, distcoeffsR,
		R, T, Rl, Rr, Pl, Pr, Q);

	/*START ��ʾУ�����-----------------------------------------------------------------*/
	Mat canvas;
	double sf;
	int w, h;
	sf = 600. / MAX(imgSize.width, imgSize.height);
	w = cvRound(imgSize.width * sf);
	h = cvRound(imgSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);

	//��ͼ�񻭵�������
	Mat canvasPart = canvas(Rect(0, 0, w, h));
	resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),
		cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);

	cout << "Painted ImageL" << endl;

	//��ͼ�񻭵�������
	canvasPart = canvas(Rect(w, 0, w, h));
	resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x*sf), cvRound(validROIR.y*sf),
		cvRound(validROIR.width*sf), cvRound(validROIR.height*sf));
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	cout << "Painted ImageR" << endl;

	//���϶�Ӧ������
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

	imshow("rectified", canvas);
	imwrite("elseImg\\rectify.jpg", canvas);
	cout << "wait key" << endl;
	waitKey(100000);

	/*END ��ʾУ�����-------------------------------------------------------------------------------*/
}

//int main() {
//	FileStorage firstRunfs("firstRun.yml", FileStorage::READ);
//	int boardWidth, boardHeight, squareWidth, squareHeight, frameNumber;
//	Mat zs;
//
//	boardWidth = firstRunfs["boardWidth"];
//	boardHeight = firstRunfs["boardHeight"];
//	squareWidth = firstRunfs["squareWidth"];
//	squareHeight = firstRunfs["squareHeight"];
//	frameNumber = firstRunfs["frameNumber"];
//	firstRunfs["zs"] >> zs;
//
//	stereoCalib(Size(1280, 720), Size(boardWidth, boardHeight),
//		Size(squareWidth, squareHeight), frameNumber);
//}

/*
1. ���Ȱ�����д�úܷ��㣬���Է�����޸����ݵ�ֵ
2. Ϊʲô�ҵõ���R�ǣ�3��3���ģ�
*/