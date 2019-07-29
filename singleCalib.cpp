#include "pch.h"
#include "singleCalib.h"

/*输入数据默认在imgPaths.txt和img中
输出数据在calibResults.txt
*/
void singleCalib(Size boardSiz, Size square, string base)
{
	ifstream fin(base+"imgPaths.txt"); //保存所有不同角度拍摄的同一个物体的图片路径的文件
	ofstream fout(base + "assess.txt");
	FileStorage fs(base+"singleCalibResults.yml",FileStorage::WRITE); //保存标定结果

	/*START 读取每幅图像，从中提取出角点，然后对角点进行亚像素精确化------------------------------------------------------*/
	int imgCount = 0;
	Size imgSize; //暂存正在处理的图片的大小
	Size boardSize = boardSiz;
	vector<Point2f> corners; //暂存一幅图片的角点
	vector<vector<Point2f>> cornersOfAllImgs;
	string filename; //图片路径
	vector<string> filenames; 

	while (getline(fin, filename)) {
		++imgCount;
		filenames.push_back(filename);
		Mat img = imread(filename);

		/*读取图片的大小*/
		if (imgCount == 1) {
			imgSize.width = img.cols;
			imgSize.height = img.rows;
		}

		/*提取角点*/
		if (findChessboardCorners(img, 
			boardSize, 
			corners)
			== 0) 
		{
			//若找不到角点
			cout << filename << "找不到角点" << endl;
			exit(1);
		}
		else {
			Mat imgGray;
			cvtColor(img, imgGray, CV_RGB2GRAY);

			/*亚像素精确化
			corners: 初始的角点坐标向量，同时作为亚像素坐标的输出
			Size(5,5): 搜索窗口大小
			(-1,-1): 表示没有死区
			TermCriteria: 角点迭代过程的终止条件，可以为迭代次数和角点精度的组合
			*/
			cornerSubPix(
				imgGray, 
				corners, Size(5, 5), 
				Size(-1, -1), 
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			cornersOfAllImgs.push_back(corners);

			/*再图像上画角点位置*/
			drawChessboardCorners(imgGray, 
				boardSize, 
				corners, 
				false);

			imshow("Camera Calibration", imgGray);
			waitKey(1);
		}
	}
	int cornerNum = boardSize.width * boardSize.height; //标定板上角点的数量
	vector<int> pointCounts;
	for (int i = 0; i < imgCount; ++i) {
		pointCounts.push_back(cornerNum); //假定每幅图片都可以看到完整的标定板
	}
	/*END 读取每幅图像，从中提取出角点，然后对角点进行亚像素精确化---------------------------------------------*/

	/*START 摄像机单目标定-------------------------------------------------------------------------------*/
	Size squareSize = square; //标定板上每个棋盘格的大小，单位为mm
	vector<vector<Point3f>> cornerPoints; //保存标定板上角点的三维坐标

	/*定义内外参数*/
	Mat cameraMat = Mat(3, 3, CV_32FC1, Scalar::all(0)); //内参数矩阵
	Mat distcoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	vector<Mat> tvecsMat; //每幅图片的平移向量
	vector<Mat> rvecsMat; //每幅图片的旋转向量

	/*初始化标定板上角点的三维坐标*/
	for (int t = 0; t < imgCount; ++t) {
		vector<Point3f> pointSet;

		for (int i = 0; i < boardSize.height; ++i) {
			for (int j = 0; j < boardSize.width; ++j) {
				Point3f point;

				point.x = i * squareSize.width;
				point.y = j * squareSize.height;
				point.z = 0; //假定标定板放在世界坐标系中z=0的平面上

				pointSet.push_back(point);
			}
		}

		cornerPoints.push_back(pointSet);
	}

	/*开始标定
	输入参数：
		cornerPoints: 世界坐标系中角点的三维坐标
		cornersOfAllImgs: 每一个内角点对应的图像亚像素坐标点
		imgSize: 图像尺寸
	输出参数：
		cameraMat: 内参数矩阵
		discoeffs: 畸变系数
		rvecsMat: 旋转向量
		tvecsMat: 位移向量
	附加参数：
		0: 标定时选用的算法
	*/
	calibrateCamera(
		cornerPoints, 
		cornersOfAllImgs, 
		imgSize, 
		cameraMat, 
		distcoeffs, 
		rvecsMat, 
		tvecsMat, 
		0);

	/*END 摄像机单目标定-------------------------------------------------------------------------------*/

	/*START 评价标定结果----------------------------------------------------------------------------------*/
	/*输出到txt文件中*/
	double err = 0.0; //每幅图片的平均误差
	double totalErr = 0.0; //所有图片平均误差之和
	vector<Point2f> cornerPoints2; //保存标定板上角点的新的坐标
	fout << "每幅图片的标定误差：\n";

	for (int i = 0; i < imgCount; ++i) {
		vector<Point3f> tempPointSet = cornerPoints[i];

		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMat, distcoeffs, cornerPoints2);

		/* 计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = cornersOfAllImgs[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat cornerPoints2Mat = Mat(1, cornerPoints2.size(), CV_32FC2);

		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			cornerPoints2Mat.at<Vec2f>(0, j) = Vec2f(cornerPoints2[j].x, cornerPoints2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(cornerPoints2Mat, tempImagePointMat, NORM_L2);
		totalErr += err /= pointCounts[i];
		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	fout << "总体平均误差：" << totalErr / imgCount << "像素" << endl << endl;

	/*END 评价标定结果-------------------------------------------------------------------------------------*/

	/*START 保存定标结果---------------------------------------------------------------------------*/
	/*输出到txt文件中*/
	//Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  /* 保存每幅图像的旋转矩阵 */
	//fout << "相机内参数矩阵：" << endl;
	//fout << cameraMat << endl << endl;
	//fout << "畸变系数：\n";
	//fout << distcoeffs << endl << endl << endl;
	//for (int i = 0; i < imgCount; i++)
	//{
	//	fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
	//	fout << rvecsMat[i] << endl;

	//	/* 将旋转向量转换为相对应的旋转矩阵 */
	//	Rodrigues(rvecsMat[i], rotation_matrix);

	//	fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
	//	fout << rotation_matrix << endl;
	//	fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
	//	fout << tvecsMat[i] << endl << endl;
	//}
	//fout << endl;

	/*输出到yaml文件中*/
	fs << "distcoeffs" << distcoeffs << "cameraMat" << cameraMat;
	fs.release();
	
	/*END 保存标定结果------------------------------------------------------------------------*/

	/*START 将矫正后的图片导入到文件中----------------------------------------------------------*/
	Mat mapx = Mat(imgSize, CV_32FC1); //x坐标重映射参数
	Mat mapy = Mat(imgSize, CV_32FC1); //y坐标重映射参数
	Mat R = Mat::eye(3, 3, CV_32F);
	string newImgFilename; //保存矫正后的图片路径
	stringstream strStm;

	for (int i = 0; i < imgCount; ++i) {
		//用initUndistortRectifyMap函数和remap函数来矫正函数
		initUndistortRectifyMap(cameraMat, distcoeffs, R, cameraMat, imgSize, CV_32FC1, mapx, mapy);
		Mat imgSrc = imread(filenames[i]);
		Mat imgDst = imgSrc.clone();
		remap(imgSrc, imgDst, mapx, mapy, INTER_LINEAR);

		//不使用转换矩阵的方式，使用undistort函数实现
		//undistort(imgSrc, imgDst, cameraMat, distcoeffs);

		/*保存矫正后的图片*/
		strStm.clear();
		newImgFilename.clear();
		strStm << base;
		strStm << i;
		strStm >> newImgFilename;
		newImgFilename += "_d.jpg";
		imwrite(newImgFilename, imgDst);
		
	}
}
/*注意：
1. 假定标定板在世界坐标系中的z坐标为0是什么意思啊，是标定板放在摄像机镜头那里吗？
2. 我太懂根据棋盘图算出内参数矩阵和畸变参数的原理
*/


