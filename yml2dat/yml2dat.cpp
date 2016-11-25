/* yml2dat.cpp
 * (c) 2016 Tuwuh Sarwoprasojo
 */

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	if (argc != 2) {
		printf("Usage: %s <yml_file>\r\n", argv[0]);
		exit(0);
	}

	char* filename = argv[1];
	cout << "Reading file " << filename << "..." << endl;
	FileStorage fs(filename, FileStorage::READ);

	Mat cameraMatrix, distCoeffs;
	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> distCoeffs;

	cout << cameraMatrix << endl;
	cout << distCoeffs << endl;

	return 0;
}

