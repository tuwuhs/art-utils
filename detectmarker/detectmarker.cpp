/* detectmarker.cpp
 * (c) 2016 Tuwuh Sarwoprasojo
 */

#include <opencv2/opencv.hpp>
#include <AR/ar.h>
#include <AR/video.h>

#include <direct.h>
#include <io.h>

#include <fstream>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

struct Marker
{
	int id;
	Point2d center;
	vector<Point2d> vertices;
	Mat hom;
	Mat rvec;
	Mat tvec;
};

bool detectMarkerFromFile(string filename, ARParam& cparam, double squareSize, vector<Marker>& markers)
{
	ARHandle* pARHandle = NULL;
	AR3DHandle* pAR3DHandle = NULL;

	ostringstream os;
	os << "-device=Image -noloop -format=RGB -image=\"" << filename << "\"";
	arVideoOpen(os.str().c_str());

	int xsize, ysize;
	arVideoGetSize(&xsize, &ysize);
	AR_PIXEL_FORMAT pixformat = arVideoGetPixelFormat();
	if (pixformat != AR_PIXEL_FORMAT_RGB) {
		printf("Pixel format mismatch: %d", pixformat);
		return false;
	}

	printf("cparam xsize: %d ysize: %d\r\n", cparam.xsize, cparam.ysize);
	if (cparam.xsize != xsize || cparam.ysize != ysize) {
		arParamChangeSize(&cparam, xsize, ysize, &cparam);
	}

	ARParamLT* pCparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET);

	pARHandle = arCreateHandle(pCparamLT);
	arSetPixelFormat(pARHandle, pixformat);
	arSetDebugMode(pARHandle, AR_DEBUG_DISABLE);
	pAR3DHandle = ar3DCreateHandle(&cparam);
	arVideoCapStart();

	// Setup AR
	arSetPatternDetectionMode(pARHandle, AR_MATRIX_CODE_DETECTION);
	arSetLabelingMode(pARHandle, AR_DEFAULT_LABELING_MODE);
	arSetPattRatio(pARHandle, AR_PATT_RATIO);
	arSetMatrixCodeType(pARHandle, AR_MATRIX_CODE_TYPE_DEFAULT);

	ARUint8* pImage;
	Mat cvImage = Mat(ysize, xsize, CV_8UC3);

	pImage = arVideoGetImage();
	if (pImage) {
		arDetectMarker(pARHandle, pImage);

		//cvImage.data = (uchar*)pImage;
		//cvtColor(cvImage, cvImage, CV_RGB2BGR);

		//vector< pair<int, Mat> > pattTransforms;

		cout << "Detected markers:" << endl;
		for (int markerIndex = 0; markerIndex < pARHandle->marker_num; markerIndex++) {
			ARMarkerInfo& marker = pARHandle->markerInfo[markerIndex];
			Marker m;

			m.id = marker.id;
			m.center = Point2d(marker.pos[0], marker.pos[1]);
			cout << "id " << m.id << " - " << m.center << endl;
			if (m.id < 0) {
				cout << " invalid ID " << endl << endl;
				continue;
			} else if (marker.cutoffPhase != AR_MARKER_INFO_CUTOFF_PHASE_NONE) {
				cout << " cutoff " << marker.cutoffPhase << endl << endl;
				continue;
			}
			
			for (auto p : marker.vertex) {
				m.vertices.push_back(Point2d(p[0], p[1]));
			}

		//	if (find_if(pattTransforms.begin(), pattTransforms.end(),
		//		[&id](const pair<int, Mat>& e) { return e.first == id; })
		//		!= pattTransforms.end()) {
		//		cout << " duplicate ID!" << endl;
		//	}

			ARdouble conv[3][4];
			arGetTransMatSquare(pAR3DHandle, &marker, squareSize, conv);

			Mat hom = Mat(4, 4, CV_64F, 0.0);
			for (int p = 0; p < 3; p++) {
				for (int q = 0; q < 4; q++) {
					hom.at<double>(p, q) = conv[p][q];
				}
			}
			hom.at<double>(3, 3) = 1.0;

			Mat rot = hom(Range(0, 3), Range(0, 3));
			Mat tvec = hom(Range(0, 3), Range(3, 4));
			Mat rvec;
			Rodrigues(rot, rvec);

			m.hom = hom;
			m.rvec = rvec;
			m.tvec = tvec;

			cout << rvec << endl;
			cout << tvec << endl;
			cout << endl;

			markers.push_back(m);
		}
	}

	// Clean up
	arVideoCapStop();
	arVideoClose();
	ar3DDeleteHandle(&pAR3DHandle);
	arDeleteHandle(pARHandle);
	arParamLTFree(&pCparamLT);

	return true;
}

bool glob(char* path, char* patt, vector<string>& filenames)
{
	char pwdOld[256] = "";
	_getcwd(pwdOld, sizeof(pwdOld));

	_chdir(path);

	char pwd[256] = "";
	_getcwd(pwd, sizeof(pwd));

	_finddata_t filedata;
	intptr_t file = _findfirst("*.jpg", &filedata);
	if (file == -1) {
		_chdir(pwdOld); 
		return false;
	}

	do {
		ostringstream os;
		os << pwd << "\\" << filedata.name;
		filenames.push_back(os.str());
	} while (_findnext(file, &filedata) == 0);

	_chdir(pwdOld);
	return true;
}

int main(int argc, char* argv[])
{
	if (argc < 3 || argc > 4) {
		printf("Usage: %s <image_dir> <square_size> [<base_id>]\r\n", argv[0]);
		exit(0);
	}

	double squareSize = stod(argv[2]);
	
	int baseId = -1;
	if (argc == 4) {
		baseId = stoi(argv[3]);
	}

	ARParam cparam;
	arParamLoad("camera_para.dat", 1, &cparam);

	vector<string> filenames;
	glob(argv[1], "*.jpg", filenames);

	vector<vector<Marker>> markersList;
	for (auto f : filenames) {
		vector<Marker> markers;
		detectMarkerFromFile(f, cparam, squareSize, markers);
		markersList.push_back(markers);
	}

	FileStorage fs("yeah.xml", FileStorage::WRITE);
	fs << "markers_list" << "[:";
	auto itm = markersList.cbegin();
	auto itf = filenames.cbegin();
	for (; itm != markersList.end() && itf != filenames.end(); itm++, itf++) {
		fs << "{:";
		fs << "filename" << *itf;
		fs << "markers" << "[:";
		for (auto m : *itm) {
		//	fs << "{:";
			fs << m.id;
		//	fs << "rvec" << m.rvec;
		//	fs << "tvec" << m.tvec;
		//	fs << "corners" << "[";
		//	for (auto c : m.vertices) {
		//		fs << c;
		//	}
		//	fs << "]";
		//	fs << "}";
		}
		fs << "]";
		fs << "}";
	}
	fs << "]";

	return 0;
}
