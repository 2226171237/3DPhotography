#include "cv.h"
#include "highgui.h"

int main()
{
	char path[100] = "./out/img";
	//char path[100] = "./out/obj";
	char *str = &path[9];
	int img_index = 0;
	CvCapture *capture = cvCreateCameraCapture(1);
	IplImage *frame;
	cvNamedWindow("camera");
	printf("°´¼üpÅÄÕÕ£¬qÍË³ö\n");
	while (1)
	{
		frame = cvQueryFrame(capture);
		cvShowImage("camera", frame);
		char r = cvWaitKey(30);
		if (r == 'q')
			break;
		else if (r == 'p')
		{
			sprintf(str, "%d.bmp", img_index);
			cvSaveImage(path, frame);
			img_index++;
			printf("take a picture save in %s\n", path);
		}
	}
	cvReleaseCapture(&capture);
	cvDestroyWindow("camera");
	return 0;
}