#include "pch.h"
#include "singleCalib.h"
#include "stereoCalib.h"


void outputCameraParam(Mat cameraMatL, Mat distcoeffsL, Mat cameraMatR, Mat distcoeffsR,
	Mat R, Mat T, Mat Rl, Mat Rr, Mat Pl, Mat Pr, Mat Q)
{
	/*保存数据*/
	/*输出数据*/
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




/*计算标定板上内角点在世界坐标系下的坐标*/
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
算法功能：双目标定。
注意事项：文件夹中必须要有图片；这里用到的图片大小必须是以后拍摄照片时用到的图片大小，也必须和真实的图片大小一致。
参数：
	frameNum: 标定所需要的图片的帧数
	zs: (n,1)的单通道矩阵，所有frameNum个标定板在世界坐标系中的深度值（z值）
*/
void stereoCalib(Size imgSiz, Size boardSiz, Size squareSiz, int frameNum) {

	/*START 双目标定用到的数据-------------------------------------------------------*/
	//摄像头的分辨率
	int imgWidth = imgSiz.width;
	int imgHeight = imgSiz.height;
	Size imgSize = Size(imgWidth, imgHeight);

	//角点数量
	int boardWidth = boardSiz.width;
	int boardHeight = boardSiz.height;
	Size boardSize = Size(boardWidth, boardHeight);

	//总的角点数目
	int cornerNum = boardWidth * boardHeight;

	//标定板上棋盘格的大小，单位mm
	Size squareSize = squareSiz;

	//相机标定时需要采用的图像帧数
	int frameNumber = frameNum;

	//左边摄像机所有照片角点的坐标集合
	vector<vector<Point2f>> cornersOfAllImgsL;
	//右边摄像机所有照片角点的坐标集合
	vector<vector<Point2f>> cornersOfAllImgsR;

	//各图像的角点的实际的物理坐标集合
	vector<vector<Point3f>> objRealPoint;

	/*事先标定好的左相机的内参矩阵
	fx 0 cx
	0 fy cy
	0  0  1
	*/
	Mat cameraMatL;
	Mat distcoeffsL;
	Mat cameraMatR;
	Mat distcoeffsR;

	/*读取yml文件*/
	FileStorage fsL;
	FileStorage fsR;

	/*END 双目标定用到的数据-----------------------------------------------------------------------*/

	/*START 双目标定要计算的数据----------------------------------------------------------------------*/
	//R旋转矩阵 T平移矢量 E本征矩阵 F基础矩阵
	Mat R, T, E, F;

	//校正旋转矩阵R，投影矩阵P，重投影矩阵Q
	Mat Rl, Rr, Pl, Pr, Q;

	//图像校正之后，会对图像进行裁剪，其中，validROI裁剪之后的区域
	Rect validROIL, validROIR;

	//映射表
	Mat mapLx, mapLy, mapRx, mapRy;

	/*END 双目标定要计算的数据*/


	vector<Point2f> cornerL; // 左边摄像机某一照片角点坐标集合
	vector<Point2f> cornerR; // 右边摄像机某一照片角点坐标集合

	Mat rgbImgL, grayImgL;
	Mat rgbImgR, grayImgR;


	/*计算左相机的内参数和畸变系数*/
	string leftBase = "img\\left\\";
	//singleCalib(boardSize, squareSize, leftBase);
	fsL = FileStorage(leftBase + "singleCalibResults.yml", FileStorage::READ);
	fsL["cameraMat"] >> cameraMatL;
	fsL["distcoeffs"] >> distcoeffsL;

	/*计算右相机的内参数和畸变系数*/
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

		/*读取左边的图像*/
		sprintf_s(filename, "img\\left\\%d.jpg", frameCount);
		//sprintf_s(filename, "img\\left\\0.jpg");
		rgbImgL = imread(filename, CV_LOAD_IMAGE_COLOR); 
		cvtColor(rgbImgL, grayImgL, CV_BGR2GRAY);

		/*读取右边的图像*/
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

	//计算实际的校正点的三维坐标，根据实际标定格子的大小来设置
	calRealPoint(boardWidth, boardHeight, frameNumber, squareSize.width, objRealPoint);
	cout << "cal real successful" << endl;

	/*标定摄像头*/
	double rms = stereoCalibrate(objRealPoint, cornersOfAllImgsL, cornersOfAllImgsR,
		cameraMatL, distcoeffsL,
		cameraMatR, distcoeffsR,
		imgSize, R, T, E, F, CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	cout << "Stereo Calibration done with RMS error = " << rms << endl;

	/*摄像机校正映射*/
	stereoRectify(cameraMatL, distcoeffsL, cameraMatR, distcoeffsR, imgSize, R, T, Rl,
		Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY, -1, imgSize, &validROIL, &validROIR);

	initUndistortRectifyMap(cameraMatL, distcoeffsL, Rl, Pl, imgSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatR, distcoeffsR, Rr, Pr, imgSize, CV_32FC1, mapRx, mapRy);

	Mat rectifyImageL, rectifyImageR;
	cvtColor(grayImgL, rectifyImageL, CV_GRAY2BGR);
	cvtColor(grayImgR, rectifyImageR, CV_GRAY2BGR);

	imshow("Recitify Before", rectifyImageL);
	cout << "按Q1退出..." << endl;
	//经过remap之后，左右相机的图像已经共面并且行对准了
	Mat rectifyImageL2, rectifyImageR2;
	remap(rectifyImageL, rectifyImageL2, mapLx, mapLy, INTER_LINEAR);
	remap(rectifyImageR, rectifyImageR2, mapRx, mapRy, INTER_LINEAR);
	cout << "按Q2退出..." << endl;

	imshow("rectifyImageL", rectifyImageL2);
	imshow("rectifyImageR", rectifyImageR2);
	imwrite("elseImg\\rectifyImageL.jpg", rectifyImageL2);
	imwrite("elseImg\\rectifyImageR.jpg", rectifyImageR2);

	//保存标定结果
	outputCameraParam(cameraMatL, distcoeffsL, cameraMatR, distcoeffsR,
		R, T, Rl, Rr, Pl, Pr, Q);

	/*START 显示校正结果-----------------------------------------------------------------*/
	Mat canvas;
	double sf;
	int w, h;
	sf = 600. / MAX(imgSize.width, imgSize.height);
	w = cvRound(imgSize.width * sf);
	h = cvRound(imgSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);

	//左图像画到画布上
	Mat canvasPart = canvas(Rect(0, 0, w, h));
	resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),
		cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);

	cout << "Painted ImageL" << endl;

	//右图像画到画布上
	canvasPart = canvas(Rect(w, 0, w, h));
	resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x*sf), cvRound(validROIR.y*sf),
		cvRound(validROIR.width*sf), cvRound(validROIR.height*sf));
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	cout << "Painted ImageR" << endl;

	//画上对应的线条
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

	imshow("rectified", canvas);
	imwrite("elseImg\\rectify.jpg", canvas);
	cout << "wait key" << endl;
	waitKey(100000);

	/*END 显示校正结果-------------------------------------------------------------------------------*/
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
1. 事先把数据写好很方便，可以方便的修改数据的值
2. 为什么我得到的R是（3，3）的？
*/