#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <stdlib.h>

int n_boards = 0;//可以通过输入来设置数值
const int board_dt = 10;//20帧一个棋盘视图
int board_w;
int board_h;

int main(int argc, char* argv[])
{
	/*
	if (argc != 4)
	{
		printf("Error:wrong number of input parameters\n");
		return -1;
	}
	board_w = atoi(argv[1]);
	board_h = atoi(argv[2]);
	n_boards = atoi(argv[3]);//不同位置的棋盘数,视角数
	*/
	board_w = 4;
	board_h = 7;
	n_boards = 20;//不同位置的棋盘数,视角数
	int board_n = board_w*board_h;//棋盘角点数
	CvSize board_sz = cvSize(board_w, board_h);
	CvCapture *capture = cvCreateCameraCapture(1);
	assert(capture);

	cvNamedWindow("Calibration");

	//Allocate storage 内存分配
	CvMat *image_points = cvCreateMat(n_boards*board_n, 2,CV_32FC1);
	CvMat *object_points = cvCreateMat(n_boards*board_n, 3, CV_32FC1);
	CvMat *point_counts = cvCreateMat(n_boards, 1, CV_32SC1);
	CvMat *intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat *distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);

	CvMat* rotation_vectors = cvCreateMat(n_boards, 3, CV_32FC1);
	CvMat* translation_vectors = cvCreateMat(n_boards, 3, CV_32FC1);
	
	CvPoint2D32f *corners = new CvPoint2D32f[board_n];
	int corner_count;
	int successes = 0;
	int step, frame = 0;
	IplImage *image = cvQueryFrame(capture);
	IplImage *gray_image = cvCreateImage(cvGetSize(image), 8, 1);

	//获取角点，直到所有角点都被找到
	while (successes < n_boards)
	{
		//每board_dt帧，用户获取不同角度棋盘视图
		if (frame++%board_dt == 0)
		{
			//寻找棋盘角点
			int found = cvFindChessboardCorners(image, board_sz, corners, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

			//获取这些角点的亚像素精度
			cvCvtColor(image, gray_image, CV_BGR2GRAY);
			cvFindCornerSubPix(gray_image, corners, corner_count, cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(
				CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			//绘制
			cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);
			cvShowImage("Calibration", image);

			//如果我们获得一个很好的borad将其加入我们的数据中
			if (corner_count == board_n)
			{
				step = successes*board_n;
				for (int i = step, j = 0; j < board_n; ++i, ++j)
				{
					CV_MAT_ELEM(*image_points, float, i, 0) = corners[j].x;
					CV_MAT_ELEM(*image_points, float, i, 1) = corners[j].y;
					CV_MAT_ELEM(*object_points, float, i, 0) = j / board_w;
					CV_MAT_ELEM(*object_points, float, i, 1) = j%board_w;
					CV_MAT_ELEM(*object_points, float, i, 2) = 0.0f;
				}
				CV_MAT_ELEM(*point_counts, int, successes, 0) = board_n;
				successes++;
			}
		}
		//控制开始暂停和退出
		int c = cvWaitKey(15);
		if (c == 'p')
		{
			c = 0;
			while (c != 'p' && c != 27) {
				c = cvWaitKey(30);
			}
		}
		if (c == 27)
		{
			return 0;
		}
		image = cvQueryFrame(capture);//下一帧图
	}//结束循环

	//allocate matrixes according to how many chessboards found
	CvMat * object_points2 = cvCreateMat(successes*board_n, 3, CV_32FC1);
	CvMat *image_points2 = cvCreateMat(successes*board_n, 2, CV_32FC1);
	CvMat *point_counts2 = cvCreateMat(successes, 1, CV_32SC1);

	//transfer the points into the correct size matrices
	//below,we write out the details in the next two loops. we could instead have written
	//image_points->rows=object_points->rows=successes*board_n;
	//point_counts->rows=successes;
	for (int i = 0; i < successes*board_n; ++i)
	{
		CV_MAT_ELEM(*image_points2, float, i, 0) = CV_MAT_ELEM(*image_points, float, i, 0);
		CV_MAT_ELEM(*image_points2, float, i, 1) = CV_MAT_ELEM(*image_points, float, i, 1);
		CV_MAT_ELEM(*object_points2, float, i, 0) = CV_MAT_ELEM(*object_points, float, i, 0);
		CV_MAT_ELEM(*object_points2, float, i, 1) = CV_MAT_ELEM(*object_points, float, i, 1);
		CV_MAT_ELEM(*object_points2, float, i, 2) = CV_MAT_ELEM(*object_points, float, i, 2);
	}
	for (int i = 0; i < successes; ++i)
	{
		CV_MAT_ELEM(*point_counts2, int, i, 0) = CV_MAT_ELEM(*point_counts, int, i, 0);
	}

	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);

	//at this points we have all of the chessboard corners we need
	//initialize the intrinsic matrix such that the two focal
	//lengths have a ratio of 1.0
	CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 1.0f;
	CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 1.0f;

	//calibrate the camera!
	cvCalibrateCamera2(object_points2, image_points2, point_counts2, cvGetSize(image), intrinsic_matrix, distortion_coeffs,
		rotation_vectors,
		translation_vectors, 0);
	//SAVE the intrinsics and distortions

	cvSave("Intrinsics.xml", intrinsic_matrix);
	cvSave("Distortion.xml", distortion_coeffs);
	CvMat * R = cvCreateMat(3, 3, CV_32FC1);
	CvMat *r_temp = cvCreateMat(1, 3, CV_32FC1);
	cvGetRow(rotation_vectors, r_temp, 0);
	cvRodrigues2(r_temp, R);
	cvSave("R.xml", R);
	CvMat * T = cvCreateMat(1, 3, CV_32FC1);
	cvGetRow(translation_vectors, T, 0);
	cvSave("T.xml", T);

	cvReleaseMat(&R);
	cvReleaseMat(&r_temp);
	cvReleaseMat(&T);
	//example of loading these matrices back in:

	CvMat * intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
	CvMat * distortion = (CvMat*)cvLoad("Distortion.xml");

	//build the undistort map that we will use for all subsequent frames.
	IplImage *mapx = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	IplImage *mapy = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	cvInitUndistortMap(intrinsic, distortion, mapx, mapy);

	//just run the camera to the screen ,now showing the raw and the undistorted image
	cvNamedWindow("Undistort");
	while (image)
	{
		IplImage *t = cvCloneImage(image);
		cvShowImage("Calibration", image);
		cvRemap(t, image, mapx, mapy);
		cvReleaseImage(&t);
		cvShowImage("Undistort", image);

		//handle the pause/unpause and esc
		int c = cvWaitKey(15);
		if (c == 'p')
		{
			c = 0;
			while (c != 'p' && c != 27) {
				c = cvWaitKey(250);
			}
		}
		if (c == 27)
		{
			return 0;
		}
		image = cvQueryFrame(capture);//下一帧图
	}
	
	return 0;
}

