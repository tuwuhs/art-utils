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
static AR3DHandle* g_pAR3DHandle = NULL;

int main(int argc, char* argv[])
{
	if (argc < 3 || argc > 4) {
		printf("Usage: %s <image_file> <square_size> [<base_id>]\r\n", argv[0]);
		exit(0);
	}

	double squareSize = stod(argv[2]);
	
	int baseId = -1;
	if (argc == 4) {
		baseId = stoi(argv[3]);
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
	g_pAR3DHandle = ar3DCreateHandle(&cparam);
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

		vector< pair<int, Mat> > pattTransforms;

		cout << "Detected markers:" << endl;
		for (int markerIndex = 0; markerIndex < g_pARHandle->marker_num; markerIndex++) {
			ARMarkerInfo* pMarker = &(g_pARHandle->markerInfo[markerIndex]);
			Point2d centre = Point2d(pMarker->pos[0], pMarker->pos[1]);
			int id = pMarker->id;
			cout << "id " << id << " - " << centre << endl;
			if (id < 0) {
				cout << " invalid ID " << endl << endl;
				continue;
			} 
			else if (pMarker->cutoffPhase != AR_MARKER_INFO_CUTOFF_PHASE_NONE) {
				cout << " cutoff " << pMarker->cutoffPhase << endl << endl;
				continue;
			}
			
			if (find_if(pattTransforms.begin(), pattTransforms.end(), 
						[&id](const pair<int, Mat>& e) { return e.first == id; }) 
				!= pattTransforms.end()) {
				cout << " duplicate ID!" << endl;
			}

			ARdouble conv[3][4];
			arGetTransMatSquare(g_pAR3DHandle, pMarker, squareSize, conv);

			Mat pattTransform = Mat(4, 4, CV_64F, 0.0);
			for (int p = 0; p < 3; p++) {
				for (int q = 0; q < 4; q++) {
					pattTransform.at<double>(p, q) = conv[p][q];
				}
			}
			pattTransform.at<double>(3, 3) = 1.0;
			pattTransforms.push_back(pair<int, Mat>(id, pattTransform));

			cout << pattTransform << endl;

			// Visualize
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

			cout << endl;
		}

		auto baseIdCount = count_if(pattTransforms.begin(), pattTransforms.end(),
			[&baseId](pair<int, Mat>& e) { return e.first == baseId; });
		if (baseIdCount == 0) {
			cout << "baseId not found." << endl;
		}
		else if (baseIdCount > 1) {
			cout << "Duplicate baseId." << endl;
		}
		else {
			auto it = find_if(pattTransforms.begin(), pattTransforms.end(),
				[&baseId](pair<int, Mat>& e) { return e.first == baseId; });
			Mat& baseTransform = it->second;
			Mat baseTransformInv = baseTransform.inv();

			ofstream of;
			of.open("multi.dat");
			of << pattTransforms.size() << endl << endl;
			for (auto& kv : pattTransforms) {
				of << kv.first << endl;
				of << squareSize << endl;
				Mat T = baseTransformInv * kv.second;
				for (int p = 0; p < 3; p++) {
					for (int q = 0; q < 4; q++) {
						of << T.at<double>(p, q) << "\t";
					}
					of << endl;
				}
				of << endl;
			}
			of.close();
		}

		imshow("Image", cvImage);
		waitKey();
	}

	// Clean up
	cvImage.release();
	arVideoCapStop();
	ar3DDeleteHandle(&g_pAR3DHandle);
	arDeleteHandle(g_pARHandle);
	arParamLTFree(&pCparamLT);
}
