#include "pch.h"
#include "singleCalib.h"

/*��������Ĭ����imgPaths.txt��img��
���������calibResults.txt
*/
void singleCalib(Size boardSiz, Size square, string base)
{
	ifstream fin(base+"imgPaths.txt"); //�������в�ͬ�Ƕ������ͬһ�������ͼƬ·�����ļ�
	ofstream fout(base + "assess.txt");
	FileStorage fs(base+"singleCalibResults.yml",FileStorage::WRITE); //����궨���

	/*START ��ȡÿ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��------------------------------------------------------*/
	int imgCount = 0;
	Size imgSize; //�ݴ����ڴ����ͼƬ�Ĵ�С
	Size boardSize = boardSiz;
	vector<Point2f> corners; //�ݴ�һ��ͼƬ�Ľǵ�
	vector<vector<Point2f>> cornersOfAllImgs;
	string filename; //ͼƬ·��
	vector<string> filenames; 

	while (getline(fin, filename)) {
		++imgCount;
		filenames.push_back(filename);
		Mat img = imread(filename);

		/*��ȡͼƬ�Ĵ�С*/
		if (imgCount == 1) {
			imgSize.width = img.cols;
			imgSize.height = img.rows;
		}

		/*��ȡ�ǵ�*/
		if (findChessboardCorners(img, 
			boardSize, 
			corners)
			== 0) 
		{
			//���Ҳ����ǵ�
			cout << filename << "�Ҳ����ǵ�" << endl;
			exit(1);
		}
		else {
			Mat imgGray;
			cvtColor(img, imgGray, CV_RGB2GRAY);

			/*�����ؾ�ȷ��
			corners: ��ʼ�Ľǵ�����������ͬʱ��Ϊ��������������
			Size(5,5): �������ڴ�С
			(-1,-1): ��ʾû������
			TermCriteria: �ǵ�������̵���ֹ����������Ϊ���������ͽǵ㾫�ȵ����
			*/
			cornerSubPix(
				imgGray, 
				corners, Size(5, 5), 
				Size(-1, -1), 
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			cornersOfAllImgs.push_back(corners);

			/*��ͼ���ϻ��ǵ�λ��*/
			drawChessboardCorners(imgGray, 
				boardSize, 
				corners, 
				false);

			imshow("Camera Calibration", imgGray);
			waitKey(1);
		}
	}
	int cornerNum = boardSize.width * boardSize.height; //�궨���Ͻǵ������
	vector<int> pointCounts;
	for (int i = 0; i < imgCount; ++i) {
		pointCounts.push_back(cornerNum); //�ٶ�ÿ��ͼƬ�����Կ��������ı궨��
	}
	/*END ��ȡÿ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��---------------------------------------------*/

	/*START �������Ŀ�궨-------------------------------------------------------------------------------*/
	Size squareSize = square; //�궨����ÿ�����̸�Ĵ�С����λΪmm
	vector<vector<Point3f>> cornerPoints; //����궨���Ͻǵ����ά����

	/*�����������*/
	Mat cameraMat = Mat(3, 3, CV_32FC1, Scalar::all(0)); //�ڲ�������
	Mat distcoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	vector<Mat> tvecsMat; //ÿ��ͼƬ��ƽ������
	vector<Mat> rvecsMat; //ÿ��ͼƬ����ת����

	/*��ʼ���궨���Ͻǵ����ά����*/
	for (int t = 0; t < imgCount; ++t) {
		vector<Point3f> pointSet;

		for (int i = 0; i < boardSize.height; ++i) {
			for (int j = 0; j < boardSize.width; ++j) {
				Point3f point;

				point.x = i * squareSize.width;
				point.y = j * squareSize.height;
				point.z = 0; //�ٶ��궨�������������ϵ��z=0��ƽ����

				pointSet.push_back(point);
			}
		}

		cornerPoints.push_back(pointSet);
	}

	/*��ʼ�궨
	���������
		cornerPoints: ��������ϵ�нǵ����ά����
		cornersOfAllImgs: ÿһ���ڽǵ��Ӧ��ͼ�������������
		imgSize: ͼ��ߴ�
	���������
		cameraMat: �ڲ�������
		discoeffs: ����ϵ��
		rvecsMat: ��ת����
		tvecsMat: λ������
	���Ӳ�����
		0: �궨ʱѡ�õ��㷨
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

	/*END �������Ŀ�궨-------------------------------------------------------------------------------*/

	/*START ���۱궨���----------------------------------------------------------------------------------*/
	/*�����txt�ļ���*/
	double err = 0.0; //ÿ��ͼƬ��ƽ�����
	double totalErr = 0.0; //����ͼƬƽ�����֮��
	vector<Point2f> cornerPoints2; //����궨���Ͻǵ���µ�����
	fout << "ÿ��ͼƬ�ı궨��\n";

	for (int i = 0; i < imgCount; ++i) {
		vector<Point3f> tempPointSet = cornerPoints[i];

		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMat, distcoeffs, cornerPoints2);

		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
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
		fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}
	fout << "����ƽ����" << totalErr / imgCount << "����" << endl << endl;

	/*END ���۱궨���-------------------------------------------------------------------------------------*/

	/*START ���涨����---------------------------------------------------------------------------*/
	/*�����txt�ļ���*/
	//Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  /* ����ÿ��ͼ�����ת���� */
	//fout << "����ڲ�������" << endl;
	//fout << cameraMat << endl << endl;
	//fout << "����ϵ����\n";
	//fout << distcoeffs << endl << endl << endl;
	//for (int i = 0; i < imgCount; i++)
	//{
	//	fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
	//	fout << rvecsMat[i] << endl;

	//	/* ����ת����ת��Ϊ���Ӧ����ת���� */
	//	Rodrigues(rvecsMat[i], rotation_matrix);

	//	fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
	//	fout << rotation_matrix << endl;
	//	fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
	//	fout << tvecsMat[i] << endl << endl;
	//}
	//fout << endl;

	/*�����yaml�ļ���*/
	fs << "distcoeffs" << distcoeffs << "cameraMat" << cameraMat;
	fs.release();
	
	/*END ����궨���------------------------------------------------------------------------*/

	/*START ���������ͼƬ���뵽�ļ���----------------------------------------------------------*/
	Mat mapx = Mat(imgSize, CV_32FC1); //x������ӳ�����
	Mat mapy = Mat(imgSize, CV_32FC1); //y������ӳ�����
	Mat R = Mat::eye(3, 3, CV_32F);
	string newImgFilename; //����������ͼƬ·��
	stringstream strStm;

	for (int i = 0; i < imgCount; ++i) {
		//��initUndistortRectifyMap������remap��������������
		initUndistortRectifyMap(cameraMat, distcoeffs, R, cameraMat, imgSize, CV_32FC1, mapx, mapy);
		Mat imgSrc = imread(filenames[i]);
		Mat imgDst = imgSrc.clone();
		remap(imgSrc, imgDst, mapx, mapy, INTER_LINEAR);

		//��ʹ��ת������ķ�ʽ��ʹ��undistort����ʵ��
		//undistort(imgSrc, imgDst, cameraMat, distcoeffs);

		/*����������ͼƬ*/
		strStm.clear();
		newImgFilename.clear();
		strStm << base;
		strStm << i;
		strStm >> newImgFilename;
		newImgFilename += "_d.jpg";
		imwrite(newImgFilename, imgDst);
		
	}
}
/*ע�⣺
1. �ٶ��궨������������ϵ�е�z����Ϊ0��ʲô��˼�����Ǳ궨������������ͷ������
2. ��̫����������ͼ����ڲ�������ͻ��������ԭ��
*/


