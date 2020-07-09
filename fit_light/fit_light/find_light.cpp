#include "cv.h"
#include "highgui.h"

//��άƽ�淽�̲�����A,B,C,D��AX+BY+CZ+D=0
typedef struct plane{
	double A;
	double B;
	double C;
	double D;
}Plane;
typedef struct cp {
	double dx;//�����x��ÿ�����ؿ��mm  [mm/pix]
	double dy;//�����y��ÿ�����ؿ��mm  [mm/pix]
	double cx;//[pix]
	double cy;//[pix]
	double f;//[mm]����
}CamP;

//double pen_length = 76.0/23.0;
double pen_length = 135.0 / 29.6;
CvMat *pen_shadow_points = cvCreateMat(8, 2, CV_32SC1);// b,t_s
void getPointOfShadow(int e, int x, int y, int flags, void *param);

void getPlaneparams(const CvMat *dest_points, Plane *plane_params);
void getDeskparams(const CvMat *dest_points, Plane *plane_params, const CvMat *R, const CvMat *T);

bool getVectorPlaneIntersect(const Plane p, const CvPoint3D64d v, CvPoint3D64d *points);
bool getLineLineIntersect(CvPoint3D64d , CvPoint3D64d , CvPoint3D64d ,CvPoint3D64d,CvPoint3D64d*);

void getLightPoint(const CvMat *pen_image_camera_points, Plane desk_plane, CvPoint3D64d *light_point);

//�û����Ǧ�ʵ���Ӱ����͵͵㡣������¼��ص�����
void getPointOfShadow(int e ,int x, int y, int flags, void *param)
{
	static int i = 0;
	if (e == CV_EVENT_LBUTTONDOWN && i<8)
	{
		printf("%d,%d\n", x, y);
		CV_MAT_ELEM(*pen_shadow_points, int, i, 0) = x;
		CV_MAT_ELEM(*pen_shadow_points, int, i, 1) = y;
		i++;
	}
}

//����ƽ���ϵ��������ȡƽ�淽��A,B,C,D
//dest_points:ƽ��������
//plane_params�����ص�ƽ�����
void getPlaneparams(const CvMat *dest_points, Plane *plane_params)
{
	//assert(dest_points->rows <2);
	CvPoint3D64d *P = (CvPoint3D64d *)malloc(dest_points->rows * sizeof(CvPoint3D64d));
	for (int i = 0; i < dest_points->rows; i++)
	{
		(P + i)->x = CV_MAT_ELEM(*dest_points, float, i, 0);
		(P + i)->y = CV_MAT_ELEM(*dest_points, float, i, 1);
		(P + i)->z = CV_MAT_ELEM(*dest_points, float, i, 2);
	}
	CvPoint3D64d p12;
	p12.x = (P + 1)->x - P->x;
	p12.y = (P + 1)->y - P->y;
	p12.z = (P + 1)->z - P->z;

	CvPoint3D64d p13;
	p13.x = (P + 2)->x - P->x;
	p13.y = (P + 2)->y - P->y;
	p13.z = (P + 2)->z - P->z;

	CvPoint3D64d n; //ƽ�淨����
	n.x = p12.y*p13.z - p12.z*p13.y;
	n.y = p12.z*p13.x - p12.x*p13.z;
	n.z = p12.x*p13.y - p12.y*p13.x;

	float d = -(P->x*n.x + P->y*n.y + P->z*n.z);

	plane_params->A = n.x;
	plane_params->B = n.y;
	plane_params->C = n.z;
	plane_params->D = d;
	free(P);
	P = NULL;
}

//���׿��ƽ�����
//dest_points :����ĵ�
//plane_params �����ص��������ϵ�µ�׿��ƽ�����A,B,C,D��AX+BY+CZ+D=0
// R����ת����
// T��ƽ�ƾ���
//�������ķ����д�ȷ����
void getDeskparams(const CvMat *dest_points, Plane *plane_params,const CvMat *R,const CvMat *T)
{
	//���������������Ŀռ����ꡣ
	//assert(dest_points->rows <2);
	CvPoint3D64d *P = (CvPoint3D64d *)malloc(dest_points->rows * sizeof(CvPoint3D64d)); 
	CvMat* subMat1 = cvCreateMat(1, 3, CV_32FC1);
	CvMat* subMat2= cvCreateMat(1, 3, CV_32FC1);

	//����������ת��Ϊ�������ϵ�����ꡣ
	for (int i = 0; i < dest_points->rows; i++)
	{
		cvGetRow(dest_points, subMat1, i);
		cvGetRow(R, subMat2, 0);
		(P + i)->x = cvDotProduct(subMat1, subMat2) + CV_MAT_ELEM(*T, float, 0, 0);
		cvGetRow(R, subMat2, 1);
		(P + i)->y = cvDotProduct(subMat1, subMat2) + CV_MAT_ELEM(*T, float, 1, 0);
		cvGetRow(R, subMat2, 2);
		(P + i)->z = cvDotProduct(subMat1, subMat2) + CV_MAT_ELEM(*T, float, 2, 0);
	}
	cvReleaseMat(&subMat1);
	cvReleaseMat(&subMat2);
	
	CvPoint3D64d p12;
	p12.x = (P + 1)->x - P->x;
	p12.y = (P + 1)->y - P->y;
	p12.z = (P + 1)->z - P->z;

	CvPoint3D64d p13;
	p13.x = (P + 2)->x - P->x;
	p13.y = (P + 2)->y - P->y;
	p13.z = (P + 2)->z - P->z;
	
	CvPoint3D64d n; //ƽ�淨����
	n.x = p12.y*p13.z - p12.z*p13.y;
	n.y = p12.z*p13.x - p12.x*p13.z;
	n.z = p12.x*p13.y - p12.y*p13.x;

	float d = -(P->x*n.x + P->y*n.y + P->z*n.z);
	float dis = sqrt(pow(n.x, 2) + pow(n.y, 2) + pow(n.z, 2));
	//��һ��������

	plane_params->A =n.x/dis;
	plane_params->B = n.y/dis;
	plane_params->C = n.z/dis;
	plane_params->D = d/dis;
	
	/*
	///////////////
	double a = ((P + 1)->y - P->y)*((P + 2)->z - P->z) - ((P + 2)->y - P->y)*((P + 1)->z - P->z);
	double b = -(((P + 1)->x - P->x)*((P + 2)->z - P->z) - ((P + 2)->x - P->x)*((P + 1)->z - P->z));
	double c = ((P + 1)->x - P->x)*((P + 2)->y - P->y) - ((P + 2)->x - P->x)*((P + 1)->y - P->y);
	double d = P->x*((P + 1)->y*(P + 2)->z - (P + 2)->y*(P + 1)->z) - P->y*((P + 1)->x*(P + 2)->z - (P + 2)->x*(P + 1)->z) + P->z*((P + 1)->x*(P + 2)->y - (P + 2)->x*(P + 1)->y);
	double norm = sqrt(pow(a, 2) + pow(b, 2)+pow(c, 2));
	plane_params->A = a / norm;
	plane_params->B = b / norm;
	plane_params->C = c / norm;
	plane_params->D=-d / norm;
	///////////////
	*/
	
	free(P);
	P = NULL;
}

//��ͼ���ϵĵ�ת��Ϊ�������ϵ�µ����꣬������Z�ᣬ��x,y���������أ�����������룬����Ϊ�������Ϊ��ͼƬ���ġ�
void getPixelsCameraCoord(const CvMat *pixel_points, CvMat *pixel_camera_points, const CamP *cp)
{
	for (int i = 0; i < pixel_camera_points->rows; i++)
	{
		CV_MAT_ELEM(*pixel_camera_points, float, i, 0) = cp->dx*(double)(CV_MAT_ELEM(*pixel_points, int, i, 0) - cp->cx);
		CV_MAT_ELEM(*pixel_camera_points, float, i, 1) = cp->dy*(double)(CV_MAT_ELEM(*pixel_points, int, i, 1) - cp->cy);
		CV_MAT_ELEM(*pixel_camera_points, float, i, 2) = cp->f;
	}
}

//��ȡƽ�����ԭ���ֱ�ߣ��������Ľ���
// p:ƽ�����
// v������
// points :����
// �н����򷵻�true,���򷵻�false
bool getVectorPlaneIntersect(const Plane p, const CvPoint3D64d v, CvPoint3D64d *points)
{
	double d = p.A*v.x + p.B*v.y + p.C*v.z;
	if (fabs(d) < 1e-5)//dԼ����0
		return false;
	double lambda = -p.D / d;
	points->x = lambda*v.x;
	points->y = lambda*v.y;
	points->z = lambda*v.z;
	return true;
}

/*�����ֱ�ߵĽ���
// shadow_top1:��һ���ӽ�Ǧ����Ӱ�Ķ���
// pen_top1:��һ���ӽ�Ǧ�ʵĶ���
// shadow_top2:��2���ӽ�Ǧ����Ӱ�Ķ���
// pen_top2:��2���ӽ�Ǧ�ʵĶ��� ,
// light_point:������õĽ��㣬��Դ��
//�н����򷵻�true,���򷵻�false����Щ�㶼���������ϵ�µ���ά��*/
bool getLineLineIntersect
(CvPoint3D64d shadow_top1, CvPoint3D64d pen_top1, CvPoint3D64d shadow_top2, CvPoint3D64d pen_top2, CvPoint3D64d *light_point)
{
	CvPoint3D64d p21;
	p21.x = shadow_top1.x - pen_top1.x;
	p21.y = shadow_top1.y - pen_top1.y;
	p21.z = shadow_top1.z - pen_top1.z;
	CvPoint3D64d p43;
	p43.x = shadow_top2.x - pen_top2.x;
	p43.y = shadow_top2.y - pen_top2.y;
	p43.z = shadow_top2.z - pen_top2.z;
	CvPoint3D64d p13;
	p13.x = shadow_top1.x - shadow_top2.x;
	p13.y = shadow_top1.y - shadow_top2.y;
	p13.z = shadow_top1.z - shadow_top2.z;

	CvPoint3D64d p13_;
	p13_.x = p43.x - p21.x;
	p13_.y = p43.y - p21.y;
	p13_.z = p43.z - p21.z;

	double d1 = sqrt(pow(p13_.x, 2) + pow(p13_.y, 2) + pow(p13_.z, 2));
	double d2 = sqrt(pow(p13.x, 2) + pow(p13.y, 2) + pow(p13.z, 2));
	if (fabs(d1) < 1e-5)
		return false;
	double alpha = d2 / d1;

	
	CvPoint3D64d pa = cvPoint3D64f(shadow_top1.x - alpha*p21.x, shadow_top1.y - alpha*p21.y, shadow_top1.z - alpha*p21.z);

	CvPoint3D64d pb = cvPoint3D64f(shadow_top2.x - alpha*p43.x, shadow_top2.y - alpha*p43.y, shadow_top2.z - alpha*p43.z);

	light_point->x = (pa.x + pb.x) / 2.0;
	light_point->y = (pa.y + pb.y) / 2.0;
	light_point->z = (pa.z + pb.z) / 2.0;
	return true;
}

//��ù�Դλ��,ʹ��ͼƬ����������������Դ����С��
//pen_image_camera_points��ͼ��λ�õ����ص�����������µ�λ��
//desk_plane����ƽ�����
//light_point�����ع�Դ
void getLightPoint(const CvMat *pen_image_camera_points,Plane desk_plane, CvPoint3D64d *light_point)
{
	light_point->x = 0.0;
	light_point->y = 0.0;
	light_point->z = 0.0;
	FILE *f_points = fopen("./params/points.txt", "w");

	CvPoint3D64d light_points_temp; 
	int view_n = pen_image_camera_points->rows / 2;//Ǧ����ͼ�ĸ���
	CvPoint3D64d *b_points = (CvPoint3D64d*)malloc(view_n * sizeof(CvPoint3D64d));//Ǧ�ʵͶ˵�
	CvPoint3D64d *ts_points = (CvPoint3D64d*)malloc(view_n * sizeof(CvPoint3D64d));//Ǧ����Ӱ���˵�

	///////////////////////////////////����///////////////////////
	for (int i = 0,j=0; i < pen_image_camera_points->rows; i = i + 2,j++)
	{
		(b_points + j)->x = CV_MAT_ELEM(*pen_image_camera_points, float, i, 0);
		(b_points + j)->y = CV_MAT_ELEM(*pen_image_camera_points, float, i, 1);
		(b_points + j)->z = CV_MAT_ELEM(*pen_image_camera_points, float, i, 2);
		(ts_points + j)->x = CV_MAT_ELEM(*pen_image_camera_points, float, i+1, 0);
		(ts_points + j)->y = CV_MAT_ELEM(*pen_image_camera_points, float, i+1, 1);
		(ts_points + j)->z = CV_MAT_ELEM(*pen_image_camera_points, float, i+1, 2);
	}
	CvPoint3D64d K1, Ts1, K2, Ts2;
	int n = 0;
	bool s;
	for (int i = 0; i < view_n-1; i++)
	{
		s=getVectorPlaneIntersect(desk_plane, *(b_points+i), &K1);
		fprintf(f_points, "%f %f %f\n", K1.x, K1.y, K1.z);
		if (s == false) continue;
		s=getVectorPlaneIntersect(desk_plane, *(ts_points+i), &Ts1);
		fprintf(f_points, "%f %f %f\n", Ts1.x, Ts1.y, Ts1.z);
		if (s == false) continue;
		//ת��ΪǦ�ʱʼ����ꡣ
		K1.x += pen_length*desk_plane.A;
		K1.y += pen_length*desk_plane.B;
		K1.z += pen_length*desk_plane.C;

		fprintf(f_points, "%f %f %f\n", K1.x, K1.y, K1.z);
		for (int j = i+1; j < view_n; j++)
		{
			s=getVectorPlaneIntersect(desk_plane, *(b_points+j), &K2);
			fprintf(f_points, "%f %f %f\n", K2.x, K2.y, K2.z);
			if (s == false) continue;
			s=getVectorPlaneIntersect(desk_plane, *(ts_points+j), &Ts2);
			fprintf(f_points, "%f %f %f\n", Ts2.x, Ts2.y, Ts2.z);
			if (s == false) continue;
			K2.x += pen_length*desk_plane.A;
			K2.y += pen_length*desk_plane.B;
			K2.z += pen_length*desk_plane.C;
			fprintf(f_points, "%f %f %f\n", K2.x, K2.y, K2.z);
			s=getLineLineIntersect(Ts1, K1, Ts2, K2, &light_points_temp);
			if (s == false) continue;
			light_point->x += light_points_temp.x;
			light_point->y += light_points_temp.y;
			light_point->z += light_points_temp.z;
			n++;
		}
	}
	light_point->x /= n;
	light_point->y /= n;
	light_point->z /= n;
	fprintf(f_points, "%f %f %f\n", light_point->x, light_point->y, light_point->z);
	fprintf(f_points, "%f %f %f\n", desk_plane.A,desk_plane.B,desk_plane.C);
	fprintf(f_points, "%f %f %f\n", desk_plane.D, 0.0,0.0);
	fclose(f_points);
	free(b_points);
	free(ts_points);
	b_points = NULL;
	ts_points = NULL;
}

int main()
{
	cvNamedWindow("test", 1);
	//char name[4][100] = { "sample/calibration/light-calibration1.pgm" ,
	//	"sample/calibration/light-calibration2.pgm",
	//	"sample/calibration/light-calibration3.pgm",
	//	"sample/calibration/light-calibration4.pgm" };
	char name[4][100] = { "./out/img0.bmp" ,
		"./out/img1.bmp" ,
		"./out/img2.bmp" ,
		"./out/img3.bmp" , };
	//cvResizeWindow("test",img->width,img->height);
	cvSetMouseCallback("test", getPointOfShadow);
	IplImage *img;
	for (int i = 0; i < 4; i++)
	{
		img = cvLoadImage(name[i], CV_LOAD_IMAGE_COLOR);
		cvShowImage("test", img);
		cvWaitKey(0);
	}
	cvDestroyWindow("test");
	cvReleaseImage(&img);
	cvSave("./params/shadow_points.xml", pen_shadow_points);

	CvMat *K = (CvMat *)cvLoad("./params/Intrinsics.xml");//����ڲξ���
	CvMat *R = (CvMat *)cvLoad("./params/R.xml");//�����ת����R
	CvMat *T_temp= (CvMat *)cvLoad("./params/T.xml"); //���ƽ�ƾ���T
	CvMat *T = cvCreateMat(3, 1, CV_32FC1);
	cvTranspose(T_temp, T);
	//CvMat *dest_image_points = (CvMat*)cvLoad("points.xml"); //ͼ���������ϵ�������
	CvMat dest_points;//�������ϵ�µ�������
	float vals[] = { 1.,1.,0.,2.0,1.0,0.0,2.0,2.0,0.0 };
	cvInitMatHeader(&dest_points, 3, 3, CV_32FC1, vals);
	//��ȡ����ƽ�淽��
	Plane desk_plane;
	getDeskparams(&dest_points, &desk_plane, R, T);
	printf("desk parame:%f,%f,%f,%f\n", desk_plane.A, desk_plane.B, desk_plane.C, desk_plane.D);
	//��ͼ���ϵĵ�ת��Ϊ�������ϵ�µ����꣬������Z�ᣬ��x,y���������أ�����������룬����Ϊ�������Ϊ��ͼƬ���ġ�
	CamP cp;
	//cp.dx = 0.01145833333*768/640/23;  //[mm / pix]
	//cp.dy = 0.01338742394/23;       // [mm / pix]
	cp.dx = 0.0014 / 29.6;
	cp.dy = 0.0014 / 29.6;
	cp.cx = CV_MAT_ELEM(*K, float, 0, 2);
	cp.cy= CV_MAT_ELEM(*K, float, 1, 2);
	double fx = CV_MAT_ELEM(*K, float, 0, 0);
	double fy = CV_MAT_ELEM(*K, float, 1, 1);
	cp.f = (fx*cp.dx + fy*cp.dy) / 2.0;
	printf("fx*dx=%f\n", fx*cp.dx);
	printf("fy*dy=%f\n", fy*cp.dy);
	printf("f=%f\n", cp.f);
	
	CvMat *pen_image_camera_points = cvCreateMat(8, 3, CV_32FC1);
	getPixelsCameraCoord(pen_shadow_points, pen_image_camera_points, &cp);
	
	CvPoint3D64d light_points; //��Դλ��
	getLightPoint(pen_image_camera_points, desk_plane, &light_points);
	CvMat *light = cvCreateMat(3, 1, CV_64FC1);
	CV_MAT_ELEM(*light, double, 0, 0) = light_points.x;
	CV_MAT_ELEM(*light, double, 1, 0) = light_points.y;
	CV_MAT_ELEM(*light, double, 2, 0) = light_points.z;
	cvSave("./params/light_point.xml", light);
	cvReleaseMat(&light);
	printf("light points is %f,%f,%f\n", light_points.x*23, light_points.y*23, light_points.z*23);
	getchar();
	cvReleaseMat(&K);
	cvReleaseMat(&R);
	cvReleaseMat(&T);
	//cvReleaseMat(&dest_image_points);
	//cvReleaseMat(&dest_points);
	cvReleaseMat(&pen_image_camera_points);
	cvReleaseMat(&pen_shadow_points);
	cvReleaseMat(&T_temp);
	return 0;
}
 