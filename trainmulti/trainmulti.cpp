/* teachmulti.cpp
 * (c) 2016 Tuwuh Sarwoprasojo
 */

#include <opencv2/opencv.hpp>
#include <AR/ar.h>
#include <AR/video.h>
#include <iostream>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

static ARHandle* g_pARHandle = NULL;

int main(int argc, char* argv[])
{
	if (argc != 2) {
		printf("Usage: %s <image_file>\r\n", argv[0]);
		exit(0);
	}

	ostringstream os;
	os << "-device=Image -noloop -image=" << argv[1];
	arVideoOpen(os.str().c_str());

	int xsize, ysize;
	arVideoGetSize(&xsize, &ysize);
	AR_PIXEL_FORMAT pixformat = arVideoGetPixelFormat();

	ARParam cparam;
	arParamLoad("camera_para.dat", 1, &cparam);
	printf("cparam xsize: %d ysize: %d\r\n", cparam.xsize, cparam.ysize);
	if (cparam.xsize != xsize || cparam.ysize != ysize) {
		arParamChangeSize(&cparam, xsize, ysize, &cparam);
	}

	ARParamLT* pCparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET);

	g_pARHandle = arCreateHandle(pCparamLT);
	arSetPixelFormat(g_pARHandle, pixformat);
	arSetDebugMode(g_pARHandle, AR_DEBUG_DISABLE);
	arVideoCapStart();

	// Setup AR
	arSetPatternDetectionMode(g_pARHandle, AR_MATRIX_CODE_DETECTION);
	arSetLabelingMode(g_pARHandle, AR_DEFAULT_LABELING_MODE);
	arSetPattRatio(g_pARHandle, AR_PATT_RATIO);
	arSetMatrixCodeType(g_pARHandle, AR_MATRIX_CODE_TYPE_DEFAULT);

	ARUint8* pImage;
	Mat cvImage;

	pImage = arVideoGetImage();
	if (pImage) {
		arDetectMarker(g_pARHandle, pImage);

		if (pixformat == AR_PIXEL_FORMAT_RGBA) {
			cvImage = Mat(ysize, xsize, CV_8UC4);
			cvImage.data = (uchar*)pImage;
			cvtColor(cvImage, cvImage, CV_RGBA2BGRA);
		}
		else if (pixformat == AR_PIXEL_FORMAT_BGRA) {
			cvImage = Mat(ysize, xsize, CV_8UC4);
			cvImage.data = (uchar*)pImage;
		}
		else if (pixformat == AR_PIXEL_FORMAT_RGB) {
			cvImage = Mat(ysize, xsize, CV_8UC3);
			cvImage.data = (uchar*)pImage;
			cvtColor(cvImage, cvImage, CV_RGB2BGR);
		}
		else if (pixformat == AR_PIXEL_FORMAT_BGR) {
			cvImage = Mat(ysize, xsize, CV_8UC3);
			cvImage.data = (uchar*)pImage;
		}
		else {
			// Pixel format not supported!
		}

		imshow("Image", cvImage);
		waitKey();
	}

	// Clean up
	cvImage.release();
	arDeleteHandle(g_pARHandle);
}
