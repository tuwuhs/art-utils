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
	os << "-device=Image -noloop -format=RGB -image=" << argv[1];
	arVideoOpen(os.str().c_str());

	int xsize, ysize;
	arVideoGetSize(&xsize, &ysize);
	AR_PIXEL_FORMAT pixformat = arVideoGetPixelFormat();
	if (pixformat != AR_PIXEL_FORMAT_RGB) {
		printf("Pixel format mismatch: %d", pixformat);
		exit(-1);
	}

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
	Mat cvImage = Mat(ysize, xsize, CV_8UC3);

	pImage = arVideoGetImage();
	if (pImage) {
		arDetectMarker(g_pARHandle, pImage);

		cvImage.data = (uchar*)pImage;
		cvtColor(cvImage, cvImage, CV_RGB2BGR);

		cout << "Detected markers:" << endl;
		for (int i = 0; i < g_pARHandle->marker_num; i++) {
			ARMarkerInfo* pMarker = &(g_pARHandle->markerInfo[i]);
			Point2d centre = Point2d(pMarker->pos[0], pMarker->pos[1]);
			cout << pMarker->id << " - " << centre << endl;

			for (int j = 0; j < 5; j++) {
				int dir = pMarker->dir;
				line(cvImage, 
					Point2d(pMarker->vertex[(j + 4 - dir) % 4][0], 
							pMarker->vertex[(j + 4 - dir) % 4][1]),
					Point2d(pMarker->vertex[(j + 1 + 4 - dir) % 4][0],
							pMarker->vertex[(j + 1 + 4 - dir) % 4][1]),
					Scalar(0, 255, 255), 2);
			}

			putText(cvImage, format("%d", pMarker->id), centre, 
				CV_FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 255), 2);
		}

		imshow("Image", cvImage);
		waitKey();
	}

	// Clean up
	cvImage.release();
	arDeleteHandle(g_pARHandle);
}
