#include "pch.h"
#include "getXyz.h"

/*
算法功能：输入n张左相机图片和右相机图片，得到矫正图片和xyz值（mm）
xyzs: 裁剪后的图片的各像素点的x,y,z值，表示实际的距离（输出项），有多少个图片就有多少个xyz
*/
void getXyz(vector<string> &rgbImgLVec, vector<string> &rgbImgRVec, vector<Mat> &xyzs, vector<Mat> &newImgs) {
	/*START 定义一些要用到的数据-----------------------------------------------------------------*/
	//图片大小
	const int imgWidth = 640;
	const int imgHeight = 480;
	Size imgSize = Size(imgWidth, imgHeight);

	FileStorage fsin("intrinsics.yml", FileStorage::READ); //内参数
	FileStorage fsex("extrinsics.yml", FileStorage::READ); //外参数

	//左右相机的内参数矩阵和畸变系数
	Mat cameraMatL,
		cameraMatR,
		distcoeffsL,
		distcoeffsR;
	fsin["cameraMatL"] >> cameraMatL;
	fsin["cameraMatR"] >> cameraMatR;
	fsin["distcoeffsL"] >> distcoeffsL;
	fsin["distcoeffsR"] >> distcoeffsR;

	//旋转向量，平移向量，旋转矩阵
	Mat R,
		T,
		RMat;
	//fsex["R"] >> R;
	fsex["T"] >> T;
	R = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);
    Rodrigues(R, RMat);


	//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
	Rect validROIL;
	Rect validROIR;

	//校正旋转矩阵R，投影矩阵P 重投影矩阵Q
	Mat Rl, Rr,
		Pl, Pr,
		Q;

	//映射表
	Mat mapLx, mapLy,
		mapRx, mapRy;

	//暂存图片的矩阵
	Mat rgbImgL, grayImgL,
		rgbImgR, grayImgR,
		rectifyImgL,
		rectifyImgR;

	Mat xyz;

	//立体匹配相关参数
	int blockSize = 5, uniquenessRatio = 8, numDisparities = 4;
	FileStorage matchfs("stereoMatchPara.yml", FileStorage::READ);
	blockSize = matchfs["blockSize"];
	uniquenessRatio = matchfs["uniquenessRatio"];
	numDisparities = matchfs["numDisparities"];

	Ptr<StereoBM> bm = StereoBM::create(16, 9);

	/*END 定义一些要用到的数据-----------------------------------------------------------------*/

	//计算立体矫正的映射矩阵，得到裁剪后的代码
	stereoRectify(cameraMatL, distcoeffsL, cameraMatR, distcoeffsR, imgSize, RMat, T,
		Rl, Rr, Pl, Pr, Q,
		CALIB_ZERO_DISPARITY, 0, imgSize, &validROIL, &validROIR);

	//得到左相机的x和y的映射表
	initUndistortRectifyMap(cameraMatL, distcoeffsL, Rl, Pl, imgSize, CV_32FC1, mapLx, mapLy);

	//得到右相机的x和y的映射表
	initUndistortRectifyMap(cameraMatR, distcoeffsR, Rr, Pr, imgSize, CV_32FC1, mapRx, mapRy);

	/*START 读取每一个图片（左右相机的），然后矫正，立体匹配*/
	int imgNum = rgbImgLVec.size();
	for (int i = 0; i < imgNum; ++i) {
		/*START 矫正阶段--------------------------------------------------------------------------*/
		//得到左右相机图片的路径
		string rgbImgLPath = rgbImgLVec.at(i);
		string rgbImgRPath = rgbImgRVec.at(i);

		//得到左右相机的图片的Mat
		rgbImgL = imread(rgbImgLPath, CV_LOAD_IMAGE_COLOR);
		rgbImgR = imread(rgbImgRPath, CV_LOAD_IMAGE_COLOR);

		//转换为灰度图
		cvtColor(rgbImgL, grayImgL, CV_RGB2GRAY);
		cvtColor(rgbImgR, grayImgR, CV_RGB2GRAY);

		/*经过remap后，左右相机的图片已经共面并且行对准了*/
		remap(grayImgL, rectifyImgL, mapLx, mapLy, INTER_LINEAR);
		remap(grayImgR, rectifyImgR, mapRx, mapRy, INTER_LINEAR);

		//得到伪彩色矫正图,用于物体检测和追踪
		Mat rgbRectifyImgL;
		cvtColor(rectifyImgL, rgbRectifyImgL, CV_GRAY2RGB);
		newImgs.push_back(rgbRectifyImgL);

		/*END 矫正阶段------------------------------------------------------------------------------------*/

		/*START 立体匹配阶段-----------------------------------------------------------------------------*/
		
		bm->setBlockSize(2 * blockSize + 5);     //SAD窗口大小，5~21之间为宜
		bm->setROI1(validROIL);
		bm->setROI2(validROIR);
		bm->setPreFilterCap(31);
		bm->setMinDisparity(0);  //最小视差，默认值为0, 可以是负值，int型
		bm->setNumDisparities(numDisparities * 16 + 16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
		bm->setTextureThreshold(10);
		bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
		bm->setSpeckleWindowSize(100);
		bm->setSpeckleRange(32);
		bm->setDisp12MaxDiff(-1);
		Mat disp, disp8;
		bm->compute(rectifyImgL, rectifyImgR, disp);//输入图像必须为灰度图
		disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式
		reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
		xyz = xyz * 16;

		xyzs.push_back(xyz);

		/*END 立体匹配阶段----------------------------------------------------------------------------------*/
	}
}

/*
1. 本来想先用原图检测，再得到矫正图，但是检测的box很难映射到矫正图中的新box，opencv中找不到实现（opencv只给出了整个图映射后的矫正图）
*/