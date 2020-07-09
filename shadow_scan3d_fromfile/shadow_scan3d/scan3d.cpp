#include "cv.h"
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define IMG_ROW_BEGIN 20 //�����������������ҶԳ�
//ƽ�淽��AX+BY+CZ+D=0
typedef struct plane {
	double A;
	double B;
	double C;
	double D;
}Plane;

//�������
typedef struct cp {
	double dx; //  [mm/pix]
	double dy; //  [mm/pix]
	double cx; //  [pix]
	double cy; //  [pix]
	double f;  //  [mm]
}CamP;

void reg2dArray(int **I_prev, int **I_max, int **I_min, int **I_contrast, double **I_shadow, double**t_x, int rows, int cols);
void free2dArray(int **I_prev, int **I_max, int **I_min, int **I_contrast, double **I_shadow, double**t_x, int rows);

bool getRefPoint(uchar **img, CvPoint2D64d *x_ref, int ref_row, int img_cols, int threshold);

void getPlaneparams(const CvPoint3D64d light, const CvPoint3D64d A, const CvPoint3D64d B, Plane *plane_params);
bool getVectorPlaneIntersect(const Plane p, const CvPoint3D64d v, CvPoint3D64d *points);
void getPixelsCameraCoord(const CvPoint2D64d pixel_points, CvPoint3D64d *pixel_camera_points, const CamP cp);

//�����ά����ռ�
void reg2dArray(int **I_prev, int **I_max, int **I_min, int **I_contrast, double **I_shadow, double**t_x, int rows, int cols)
{
	I_prev = (int **)calloc(rows, sizeof(int *)); //ǰһ֡ͼ��
	I_max = (int **)calloc(rows, sizeof(int *));//ͼ������ֵ
	I_min = (int **)calloc(rows, sizeof(int *));//ͼ�����Сֵ
	I_contrast = (int **)calloc(rows, sizeof(int *));//ͼ��Աȶ�
	I_shadow = (double **)calloc(rows, sizeof(double *));//��Ӱ
	t_x = (double **)calloc(rows, sizeof(double *));//ÿ�����ص����Ӱʱ��
	for (int row = 0; row < rows; row++)
	{
		I_prev[row] = (int *)calloc(cols, sizeof(int));
		I_max[row] = (int *)calloc(cols, sizeof(int));
		I_min[row] = (int *)calloc(cols, sizeof(int));
		I_contrast[row] = (int *)calloc(cols, sizeof(int));
		I_shadow[row] = (double *)calloc(cols, sizeof(double));
		t_x[row] = (double*)calloc(cols, sizeof(double));
	}
}

//�ͷŶ�ά����ռ�
void free2dArray(int **I_prev, int **I_max, int **I_min, int **I_contrast, double **I_shadow, double**t_x, int rows)
{
	for (int row = 0; row < rows; row++)
	{
		free(I_prev[row]);
		free(I_max[row]);
		free(I_min[row]);
		free(I_contrast[row]);
		free(I_shadow[row]);
		free(t_x[row]);
	}
	free(I_prev);
	free(I_max);
	free(I_min);
	free(I_contrast);
	free(I_shadow);
	free(t_x);
}

/*��ȡͼ������Ӱ��ο���
img:����Ҷ�ͼ��
x_ref:���ص������ؼ��ο���
ref_row:�ο���
img_cols:ͼ������
threshold:����ֵ
return ��ȡ�ɹ�Ϊtrue,����Ϊfalse
*/
bool getRefPoint(uchar **img, CvPoint2D64d *x_ref, int ref_row,int img_cols,int threshold)
{
	int x;
	int Imin, Imax, Iconstract;
	Imin = Imax = img[ref_row][IMG_ROW_BEGIN];
	Iconstract = 0;
	double Ishadow = (Imax + Imin) / 2.0;
	int ref_col=-1;
	for (int col = IMG_ROW_BEGIN + 1; col < img_cols - IMG_ROW_BEGIN; col++)
	{
		x = img[ref_row][col];
		if (x > Imax)
			Imax = x;
		else if (x < Imin)
			Imin = x;
		Iconstract = Imax - Imin;
		if (Iconstract > threshold)
		{
			Ishadow = (double)(Imax + Imin) / 2.0;
			if (x < Ishadow)
			{
				ref_col = col;
				break;
			}
		}
	}
	if (ref_col < 0)
		return false;
	x_ref->y = ref_row;
	int prev = img[ref_row][ref_col-1];
	x = img[ref_row][ref_col];
	if (ref_col != 0)
		x_ref->x = (1.0 - (Ishadow - prev) / (x - prev))*(double)(ref_col - 1) + ((Ishadow - prev) / (x - prev))*(double)ref_col;
	else
		x_ref->x = ref_col;
	return true;
}

//��ȡ��Ӱƽ�������A,B,C,D,
//AX+BY+CZ+D=0;
void getPlaneparams(const CvPoint3D64d light, const CvPoint3D64d A, const CvPoint3D64d B, Plane *plane_params)
{
	CvPoint3D64d p12;
	p12.x = A.x - light.x;
	p12.y = A.y - light.y;
	p12.z = A.z - light.z;

	CvPoint3D64d p13;
	p13.x = B.x - light.x;
	p13.y = B.y - light.y;
	p13.z = B.z - light.z;

	CvPoint3D64d n;
	n.x = p12.y*p13.z - p12.z*p13.y;
	n.y = p12.z*p13.x - p12.x*p13.z;
	n.z = p12.x*p13.y - p12.y*p13.x;

	double d1 = -(A.x*n.x + A.y*n.y + A.z*n.z);
	double d2 = -(B.x*n.x + B.y*n.y + B.z*n.z);
	double d3 = -(light.x*n.x + light.y*n.y + light.z*n.z);

	plane_params->A = n.x;
	plane_params->B = n.y;
	plane_params->C = n.z;
	plane_params->D = (d1+d2+d3)/3.0;
	if (fabs(n.x) > 100000)
	{
		printf("get parame error:\n");
		printf("p12=(%lf,%lf,%lf)\n", p12.x, p12.y, p12.z);
		printf("p13=(%lf,%lf,%lf)\n", p13.x, p13.y, p13.z);
	}
}

//��ȡ������ƽ��Ľ��㡣
bool getVectorPlaneIntersect(const Plane p, const CvPoint3D64d v, CvPoint3D64d *points)
{
	double d = p.A*v.x + p.B*v.y + p.C*v.z;
	if (fabs(d) < 1e-5)
		return false;
	double lambda = -p.D / d;
	points->x = lambda*v.x; 
	points->y = lambda*v.y;
	points->z = lambda*v.z;
	if (points->x > 1000 || points->y > 1000 || points->z>1000)
	{
		printf("out range value d=%lf\n", d);
		printf("plane params A=%lf,B=%lf,C=%lf,D=%lf\n", p.A, p.B, p.C, p.D);
		printf("vector params x=%lf,y=%lf,z=%lf\n", v.x,v.y,v.z);
	}
	return true;
}

//��ȡ�������������ϵ������
void getPixelsCameraCoord(const CvPoint2D64d pixel_points, CvPoint3D64d *pixel_camera_points, const CamP cp)
{
	pixel_camera_points->x = cp.dx*(double)(pixel_points.x -cp.cx);
	pixel_camera_points->y = cp.dy*(double)(pixel_points.y - cp.cy);
	pixel_camera_points->z = cp.f;
}

int main()
{
	FILE *fptr = fopen("./sample/pictures.txt", "r");
	if (fptr == NULL)
	{
		printf("Error:file open error\n");
		return -1;
	}
	char names[2048];
	int per_frame_time = 100;//100mmÿ֡
	cvNamedWindow("scaning");
	
	//��ȡ����
	CvMat *K = (CvMat *)cvLoad("./params/Intrinsics.xml");
	CvMat *desk_mat = (CvMat*)cvLoad("./params/deskplane.xml");
	CvMat *light_mat= (CvMat*)cvLoad("./params/light_point.xml");

	//��ʼ���������
	CamP cp;
	cp.dx = 0.01145833333 * 768 / 640 / 23;  //[mm / pix]
	cp.dy = 0.01338742394 / 23;       // [mm / pix]
	cp.cx = CV_MAT_ELEM(*K, float, 0, 2);
	cp.cy = CV_MAT_ELEM(*K, float, 1, 2);
	double fx = CV_MAT_ELEM(*K, float, 0, 0);
	double fy = CV_MAT_ELEM(*K, float, 1, 1);
	cp.f = (fx*cp.dx + fy*cp.dy) / 2.0;

	//��ʼ���������
	Plane desk_plane;
	desk_plane.A = CV_MAT_ELEM(*desk_mat, float, 0, 0);
	desk_plane.B = CV_MAT_ELEM(*desk_mat, float, 1, 0);
	desk_plane.C = CV_MAT_ELEM(*desk_mat, float, 2, 0);
	desk_plane.D = CV_MAT_ELEM(*desk_mat, float, 3, 0);

	//��ʼ����Դ����
	CvPoint3D64d light_point;
	light_point.x = CV_MAT_ELEM(*light_mat, double, 0, 0);
	light_point.y = CV_MAT_ELEM(*light_mat, double, 1, 0);
	light_point.z = CV_MAT_ELEM(*light_mat, double, 2, 0);

	printf("light_points=(%lf,%lf,%lf)\n", light_point.x, light_point.y, light_point.z);

	printf("��׼���û�����r����ʼɨ��\n");
	while (cvWaitKey(10) != 'r');

	IplImage *frame;
	int frame_numbers = 0;//�ܹ���ȡ��֡��
	int max_frame_numbers = 38;//����ȡ֡��

	rewind(fptr);
	//һЩ����,����һ֡ͼ������ȡͼ���С��Ϣ��
	fscanf(fptr, "%s ", names);
	printf(names);
	frame = cvLoadImage(names);
	int img_rows = frame->height;//ͼ������
	int img_cols = frame->width;//ͼ������
	int top_ref_row = 60;//�ϲο���
	int bottom_ref_row = img_rows - 60; //�²ο���
	int threshold = 50;//��ֵ

	//����һЩ����ռ䡣
	int **I_prev = (int **)calloc(img_rows, sizeof(int *)); //ǰһ֡ͼ��
	int **I_max = (int **)calloc(img_rows, sizeof(int *));//ͼ������ֵ
	int **I_min = (int **)calloc(img_rows, sizeof(int *));//ͼ�����Сֵ
	int **I_contrast = (int **)calloc(img_rows, sizeof(int *));//ͼ��Աȶ�
	double **I_shadow = (double **)calloc(img_rows, sizeof(double *));//��Ӱ
	double **t_x = (double **)calloc(img_rows, sizeof(double *));//ÿ�����ص����Ӱʱ��
	for (int row = 0; row < img_rows; row++)
	{
		I_prev[row] = (int *)calloc(img_cols, sizeof(int));
		I_max[row] = (int *)calloc(img_cols, sizeof(int));
		I_min[row] = (int *)calloc(img_cols, sizeof(int));
		I_contrast[row] = (int *)calloc(img_cols, sizeof(int));
		I_shadow[row] = (double *)calloc(img_cols, sizeof(double));
		t_x[row] = (double*)calloc(img_cols, sizeof(double));
	}

	// ���²ο�������
	CvPoint2D64d *x_top = (CvPoint2D64d *)calloc(max_frame_numbers+100, sizeof(CvPoint2D64d));
	CvPoint2D64d *x_bot = (CvPoint2D64d *)calloc(max_frame_numbers+100, sizeof(CvPoint2D64d));

	IplImage *img=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,1);

	cvCvtColor(frame, img, CV_BGR2GRAY);//ת�ɻҶ�ͼ
	
	uchar **img_data = (uchar**)calloc(img_rows, sizeof(uchar*));
	for (int row = 0; row < img_rows; row++)
	{
		img_data[row] = (uchar *)(img->imageData + row*img->widthStep);
	}

	//�õ�һ�ų�ʼ��
	for (int row=0; row < img_rows; row++)
	{
		uchar *ptr = (uchar*)(img->imageData + row*img->widthStep);
		for (int col=0; col < img_cols; col++)
		{
			I_prev[row][col] = I_max[row][col]= I_min[row][col] = (int)ptr[col];
			I_contrast[row][col] = 0;
			I_shadow[row][col] = I_max[row][col];
			t_x[row][col] = -1.0;
		}
	}
	if (getRefPoint(img_data, x_top, top_ref_row, img_cols, threshold) == false)
	{ 
		x_top->x = -10;
		fprintf(stderr, "unable to find x_top in %d frame\n", frame_numbers);
	}
	if (getRefPoint(img_data, x_bot, bottom_ref_row, img_cols, threshold) == false)
	{
		x_bot->x = -10;
		fprintf(stderr, "unable to find x_top in %d frame\n", frame_numbers);
	}
	frame_numbers++;

	//��ͶӰʱ������²ο���//
	while (1)
	{
		fscanf(fptr, "%s ", names);
		frame = cvLoadImage(names);
		printf("processing %d frame picture %s\n", frame_numbers,names);
		
		cvCvtColor(frame, img, CV_BGR2GRAY);//ת�ɻҶ�ͼ

		//�Ȼ�ȡ���²ο���
		if (getRefPoint(img_data, x_top + frame_numbers, top_ref_row, img_cols, threshold) == false)
		{
			(x_top + frame_numbers)->x = -10;
			fprintf(stderr, "unable to find x_top in %d frame\n", frame_numbers);
		}
		if (getRefPoint(img_data, x_bot + frame_numbers, bottom_ref_row, img_cols, threshold) == false)
		{
			(x_bot + frame_numbers)->x = -10;
			fprintf(stderr, "unable to find x_top in %d frame\n", frame_numbers);
		}
		
		cvCircle(frame, cvPoint((int)(x_top + frame_numbers)->x, (int)(x_top + frame_numbers)->y), 5,cvScalar(0,0,255));
		cvCircle(frame, cvPoint((int)(x_bot + frame_numbers)->x, (int)(x_bot + frame_numbers)->y), 5, cvScalar(0, 0, 255));
		//�ڻ�ȡÿ�����ص��ɨ��ʱ��
		int x;
		for (int row = 0; row < img_rows; row++)
		{
			for (int col = IMG_ROW_BEGIN; col < img_cols - IMG_ROW_BEGIN; col++)
			{
				if (t_x[row][col] < 0)//��û����ĵ�
				{
					x = img_data[row][col];
					if (x > I_max[row][col])
						I_max[row][col] = x;
					if (x < I_min[row][col])
						I_min[row][col] = x;
					I_contrast[row][col] = I_max[row][col] - I_min[row][col];

					if (I_contrast[row][col] > threshold) //���㼤������
					{
						I_shadow[row][col] = (double)(I_max[row][col] + I_min[row][col]) / 2.0;
						if (I_prev[row][col]<I_shadow[row][col] && x>I_shadow[row][col])
						{
							//���Բ�ֵ��ȡ��֡��
							t_x[row][col] = (1.0 - (I_shadow[row][col] - I_prev[row][col]) / (x - I_prev[row][col]))*(double)(frame_numbers - 1) +
								((I_shadow[row][col] - I_prev[row][col]) / (x - I_prev[row][col]))*(double)(frame_numbers);
						}
					}
					I_prev[row][col] = x;
				}
			}//end col loop
		}//end row loop
		frame_numbers++;
		if (frame_numbers > max_frame_numbers)
		{
			printf("��ɨ��������ܹ�ɨ��ͼ����%d\n", frame_numbers);
			break;
		}
		cvShowImage("scaning", frame);
		if (cvWaitKey(per_frame_time) == 'q')
		{
			printf("��ɨ��������ܹ�ɨ��ͼ����%d\n", frame_numbers);
			break;
		}	
	}//end while

	printf("�������ɵ��ƣ����Ժ󡣡���������\n");
	
	//��ʼ����3D����
	FILE * points_3d_file;
	points_3d_file = fopen("./params/points_3d.txt", "w");
	if (points_3d_file == NULL)
	{
		printf("open file ./params/points_3d.txt error\n ");
		return -1;
	}
	//���ٵ��ƿռ�
	CvPoint3D64f **points_3d = (CvPoint3D64f **)calloc(bottom_ref_row - top_ref_row, sizeof(CvPoint3D64f));
	for (int i = 0; i < bottom_ref_row - top_ref_row; i++)
	{
		points_3d[i]=(CvPoint3D64f *)calloc(img_cols - 2*IMG_ROW_BEGIN, sizeof(CvPoint3D64f));
	}
	//��ȡ����
	Plane shadow_plane;//��Ӱƽ��
	double time; //ͶӰʱ�䡣
	int pre_time, post_time;//ǰһ��ͶӰʱ�䣬��һ��ͶӰʱ��
	//�����ؼ��ο���x_top,x_bot����
	CvPoint2D64d x_top_temp, x_bot_temp;
	CvPoint3D64d x_top_3d_temp, x_bot_3d_temp;
	CvPoint3D64d A, B;//�ο����������ϵ�����
	CvPoint3D64d x_c;//��ǰͼ���ϵĵ�x_c;
	for (int row = top_ref_row; row < bottom_ref_row; row++)
	{
		for (int col = IMG_ROW_BEGIN; col < img_cols - IMG_ROW_BEGIN; col++)
		{
			time = t_x[row][col];
			pre_time = (int)(time);
			post_time = pre_time + 1;
			if (time < 0)//������ֵ������
			{
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].x = 0.;
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].y = 0.;
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].z = 0.;
				fprintf(points_3d_file, "%lf %lf %lf\n",0.0,0.0,0.0);
				continue;
			}
			//�����ڲο���
			if (x_top[pre_time].x < 0 || x_bot[pre_time].x < 0 || x_top[post_time].x < 0 || x_bot[post_time].x < 0)
			{
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].x = 0.;
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].y = 0.;
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].z = 0.;
				fprintf(points_3d_file, "%lf %lf %lf\n", 0.0,0.0,0.0);
				continue;
			}
			//���������ؼ��ο���
			x_top_temp.x = (1.0 - (time - (double)pre_time))*(double)(x_top[pre_time].x) +
				((time - (double)pre_time) / ((double)post_time - (double)pre_time))*
				(double)(x_top[post_time].x);
			x_top_temp.y = (1.0 - (time - (double)pre_time))*(double)(x_top[pre_time].y) +
				((time - (double)pre_time) / ((double)post_time - (double)pre_time))*
				(double)(x_top[post_time].y);

			x_bot_temp.x = (1.0 - (time - (double)pre_time))*(double)(x_bot[pre_time].x) +
				((time - (double)pre_time) / ((double)post_time - (double)pre_time))*
				(double)(x_bot[post_time].x);
			x_bot_temp.y = (1.0 - (time - (double)pre_time))*(double)(x_bot[pre_time].y) +
				((time - (double)pre_time) / ((double)post_time - (double)pre_time))*
				(double)(x_bot[post_time].y);

			getPixelsCameraCoord(x_top_temp, &x_top_3d_temp, cp);
			getPixelsCameraCoord(x_bot_temp, &x_bot_3d_temp, cp);
			CvPoint2D64d temp = cvPoint2D64f(col,row);
			getPixelsCameraCoord(temp, &x_c, cp);
			if (getVectorPlaneIntersect(desk_plane, x_top_3d_temp, &A) == false)
			{
				printf("not get vector and plane intersect\n");
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].x = 0.;
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].y = 0.;
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].z = 0.;
				fprintf(points_3d_file, "%lf %lf %lf\n", 0.0, 0.0, 0.0);
				continue;
			}
			if (getVectorPlaneIntersect(desk_plane, x_bot_3d_temp, &B)==false)
			{
				printf("not get vector and plane intersect\n");
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].x = 0.;
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].y = 0.;
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].z = 0.;
				fprintf(points_3d_file, "%lf %lf %lf\n", 0.0, 0.0, 0.0);
				continue;
			}
			getPlaneparams(light_point, A, B, &shadow_plane);
			if (fabs(shadow_plane.A) > 10000)
			{
				printf("get shadow plane error \n");
				printf("A=(%lf,%lf,%lf)\n", A.x, A.y, A.z);
				printf("B=(%lf,%lf,%lf)\n", B.x, B.y, B.z);
			}
			if (getVectorPlaneIntersect(shadow_plane, x_c, &points_3d[row - top_ref_row][col - IMG_ROW_BEGIN]) == false)
			{
				printf("not get vector and plane intersect\n");
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].x = 0.;
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].y = 0.;
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].z = 0.;
				fprintf(points_3d_file, "%lf %lf %lf\n", 0.0, 0.0, 0.0);
				continue;
			}
			fprintf(points_3d_file, "%lf %lf %lf\n", points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].x,
				points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].y, points_3d[row - top_ref_row][col - IMG_ROW_BEGIN].z);
		}//end col loop
	}//end row loop

	fclose(points_3d_file);
	printf("�������ɳɹ����ѱ��浽����./params/points_3d.txt�ļ���\n");


	cvNamedWindow("x_t");
	IplImage *x_t_image = cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,1);
	
	for (int i = 0; i < img_rows; i++)
	{
		uchar* ptr = (uchar*)(x_t_image->imageData + i*x_t_image->widthStep);
		for (int j = 0; j < img_cols; j++)
		{
			ptr[j] = 255-10*((int)t_x[i][j]+1);
		}
	}
	//printf("%d", x_t_image->imageData[2]);
	cvShowImage("x_t", x_t_image);
	cvWaitKey(0);
	//�ͷ��ڴ�ռ䡣
	cvReleaseImage(&x_t_image);
	free2dArray(I_prev, I_max, I_min, I_contrast, I_shadow, t_x, img_rows);
	I_prev = I_max = I_contrast = NULL;
	I_shadow = t_x =NULL;
	free(x_bot);
	x_bot = NULL;
	free(x_top);
	x_top = NULL;
	for (int i = 0; i < bottom_ref_row - top_ref_row; i++)
	{
		free(points_3d[i]);
		points_3d[i] = NULL;
	}
	free(points_3d);
	points_3d = NULL;
	for (int i = 0; i < img_rows; i++)
		img_data[i] = NULL;
	free(img_data);
	img_data = NULL;
	return 0;
}