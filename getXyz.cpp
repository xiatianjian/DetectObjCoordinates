#include "pch.h"
#include "getXyz.h"

/*
�㷨���ܣ�����n�������ͼƬ�������ͼƬ���õ�����ͼƬ��xyzֵ��mm��
xyzs: �ü����ͼƬ�ĸ����ص��x,y,zֵ����ʾʵ�ʵľ��루�������ж��ٸ�ͼƬ���ж��ٸ�xyz
*/
void getXyz(vector<string> &rgbImgLVec, vector<string> &rgbImgRVec, vector<Mat> &xyzs, vector<Mat> &newImgs) {
	/*START ����һЩҪ�õ�������-----------------------------------------------------------------*/
	//ͼƬ��С
	const int imgWidth = 640;
	const int imgHeight = 480;
	Size imgSize = Size(imgWidth, imgHeight);

	FileStorage fsin("intrinsics.yml", FileStorage::READ); //�ڲ���
	FileStorage fsex("extrinsics.yml", FileStorage::READ); //�����

	//����������ڲ�������ͻ���ϵ��
	Mat cameraMatL,
		cameraMatR,
		distcoeffsL,
		distcoeffsR;
	fsin["cameraMatL"] >> cameraMatL;
	fsin["cameraMatR"] >> cameraMatR;
	fsin["distcoeffsL"] >> distcoeffsL;
	fsin["distcoeffsR"] >> distcoeffsR;

	//��ת������ƽ����������ת����
	Mat R,
		T,
		RMat;
	//fsex["R"] >> R;
	fsex["T"] >> T;
	R = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);
    Rodrigues(R, RMat);


	//ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������  
	Rect validROIL;
	Rect validROIR;

	//У����ת����R��ͶӰ����P ��ͶӰ����Q
	Mat Rl, Rr,
		Pl, Pr,
		Q;

	//ӳ���
	Mat mapLx, mapLy,
		mapRx, mapRy;

	//�ݴ�ͼƬ�ľ���
	Mat rgbImgL, grayImgL,
		rgbImgR, grayImgR,
		rectifyImgL,
		rectifyImgR;

	Mat xyz;

	//����ƥ����ز���
	int blockSize = 5, uniquenessRatio = 8, numDisparities = 4;
	FileStorage matchfs("stereoMatchPara.yml", FileStorage::READ);
	blockSize = matchfs["blockSize"];
	uniquenessRatio = matchfs["uniquenessRatio"];
	numDisparities = matchfs["numDisparities"];

	Ptr<StereoBM> bm = StereoBM::create(16, 9);

	/*END ����һЩҪ�õ�������-----------------------------------------------------------------*/

	//�������������ӳ����󣬵õ��ü���Ĵ���
	stereoRectify(cameraMatL, distcoeffsL, cameraMatR, distcoeffsR, imgSize, RMat, T,
		Rl, Rr, Pl, Pr, Q,
		CALIB_ZERO_DISPARITY, 0, imgSize, &validROIL, &validROIR);

	//�õ��������x��y��ӳ���
	initUndistortRectifyMap(cameraMatL, distcoeffsL, Rl, Pl, imgSize, CV_32FC1, mapLx, mapLy);

	//�õ��������x��y��ӳ���
	initUndistortRectifyMap(cameraMatR, distcoeffsR, Rr, Pr, imgSize, CV_32FC1, mapRx, mapRy);

	/*START ��ȡÿһ��ͼƬ����������ģ���Ȼ�����������ƥ��*/
	int imgNum = rgbImgLVec.size();
	for (int i = 0; i < imgNum; ++i) {
		/*START �����׶�--------------------------------------------------------------------------*/
		//�õ��������ͼƬ��·��
		string rgbImgLPath = rgbImgLVec.at(i);
		string rgbImgRPath = rgbImgRVec.at(i);

		//�õ����������ͼƬ��Mat
		rgbImgL = imread(rgbImgLPath, CV_LOAD_IMAGE_COLOR);
		rgbImgR = imread(rgbImgRPath, CV_LOAD_IMAGE_COLOR);

		//ת��Ϊ�Ҷ�ͼ
		cvtColor(rgbImgL, grayImgL, CV_RGB2GRAY);
		cvtColor(rgbImgR, grayImgR, CV_RGB2GRAY);

		/*����remap�����������ͼƬ�Ѿ����沢���ж�׼��*/
		remap(grayImgL, rectifyImgL, mapLx, mapLy, INTER_LINEAR);
		remap(grayImgR, rectifyImgR, mapRx, mapRy, INTER_LINEAR);

		//�õ�α��ɫ����ͼ,�����������׷��
		Mat rgbRectifyImgL;
		cvtColor(rectifyImgL, rgbRectifyImgL, CV_GRAY2RGB);
		newImgs.push_back(rgbRectifyImgL);

		/*END �����׶�------------------------------------------------------------------------------------*/

		/*START ����ƥ��׶�-----------------------------------------------------------------------------*/
		
		bm->setBlockSize(2 * blockSize + 5);     //SAD���ڴ�С��5~21֮��Ϊ��
		bm->setROI1(validROIL);
		bm->setROI2(validROIR);
		bm->setPreFilterCap(31);
		bm->setMinDisparity(0);  //��С�ӲĬ��ֵΪ0, �����Ǹ�ֵ��int��
		bm->setNumDisparities(numDisparities * 16 + 16);//�Ӳ�ڣ�������Ӳ�ֵ����С�Ӳ�ֵ֮��,���ڴ�С������16����������int��
		bm->setTextureThreshold(10);
		bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio��Ҫ���Է�ֹ��ƥ��
		bm->setSpeckleWindowSize(100);
		bm->setSpeckleRange(32);
		bm->setDisp12MaxDiff(-1);
		Mat disp, disp8;
		bm->compute(rectifyImgL, rectifyImgR, disp);//����ͼ�����Ϊ�Ҷ�ͼ
		disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//��������Ӳ���CV_16S��ʽ
		reprojectImageTo3D(disp, xyz, Q, true); //��ʵ�������ʱ��ReprojectTo3D������X / W, Y / W, Z / W��Ҫ����16(Ҳ����W����16)�����ܵõ���ȷ����ά������Ϣ��
		xyz = xyz * 16;

		xyzs.push_back(xyz);

		/*END ����ƥ��׶�----------------------------------------------------------------------------------*/
	}
}

/*
1. ����������ԭͼ��⣬�ٵõ�����ͼ�����Ǽ���box����ӳ�䵽����ͼ�е���box��opencv���Ҳ���ʵ�֣�opencvֻ����������ͼӳ���Ľ���ͼ��
*/