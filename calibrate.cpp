/*
 * calibrate.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: paulm
 */

// Modified from code from http://dasl.mem.drexel.edu/~noahKuntz/openCVTut10.html

#include <cassert>
#include <stdio.h>
#include <time.h>
#include <vector>

#include "cv.h"
#include "ml.h"
#include "cxcore.h"
#include "highgui.h"

// IplImage pointer which is released when object is destroyed
class StackImage
{
public:
	StackImage(IplImage* image_):
		_image(image_)
	{}

	~StackImage()
	{
		cvReleaseImage(&_image);
	}

	IplImage* operator()()
	{
		return _image;
	}

protected:
	IplImage* _image;
};

// CvMat pointer that is released when object is destroyed
class StackMat
{
public:
	StackMat(CvMat* mat_):
		_mat(mat_)
	{}

	~StackMat()
	{
		cvReleaseMat(&_mat);
	}

	CvMat* operator()()
	{
		return _mat;
	}

protected:
	CvMat* _mat;
};

class CameraCalibrator
{
public:
	// Size is measured in # of "internal corners", NOT # of squares!
	const int desired_boards; // Number of boards
	const int board_dt;
	const CvSize board_size;
	const int n_corners;
	int captured_boards;
	CvSize image_size;

protected:
	CvCapture* _capture;
	std::vector<CvPoint2D32f> _image_points;
 	CvMat* _intrinsic_matrix;
 	CvMat* _distortion_coeffs;

	std::string _intrinsicsFile;
	std::string _distortionFile;

public:
	CameraCalibrator(const CvSize& board_size_):
		desired_boards(25), board_dt(20), board_size(board_size_),
		n_corners(board_size_.height * board_size_.width),
		captured_boards(0), _intrinsic_matrix(cvCreateMat( 3, 3, CV_32FC1 )),
		_distortion_coeffs(cvCreateMat( 5, 1, CV_32FC1 ))
	{
		_capture = cvCreateCameraCapture( 300 );
		cvSetCaptureProperty(_capture, CV_CAP_PROP_FRAME_WIDTH, 640);
		cvSetCaptureProperty(_capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
		assert( _capture );
		IplImage* tempImage = cvQueryFrame(_capture);
		image_size = cvGetSize(tempImage);
		_image_points.reserve(desired_boards * n_corners);

		printf("Image Size: %d x %d\n", image_size.width, image_size.height);

		// Don't forgot the +1 for the null terminator
		char intrinsicsFile[strlen("Intrinsics.2012-03-14.143050.xml") + 1];
		char distortionFile[strlen("Distortion.2012-03-14.143050.xml") + 1];
		time_t rawtime;
		struct tm timeinfo;
		time(&rawtime);
		timeinfo = *localtime(&rawtime);
		int year = timeinfo.tm_year + 1900;
		int month = timeinfo.tm_mon + 1;
		sprintf(intrinsicsFile, "Intrinsics.%04d-%02d-%02d.%02d%02d%02d.xml",
				year, month, timeinfo.tm_mday,
				timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
		sprintf(distortionFile, "Distortion.%04d-%02d-%02d.%02d%02d%02d.xml",
				year, month, timeinfo.tm_mday,
				timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

		_intrinsicsFile = intrinsicsFile;
		_distortionFile = distortionFile;
	}

	~CameraCalibrator()
	{
		cvReleaseCapture(&_capture);
		cvReleaseMat(&_intrinsic_matrix);
		cvReleaseMat(&_distortion_coeffs);
	}

	int captureBoards()
	{
		_image_points.clear();
		captured_boards = 0;

		cvNamedWindow( "Calibration" );

		std::vector<CvPoint2D32f> corners(n_corners);
		int corner_count;
		int step, frame = 0;

		// Do NOT use StackImage for currentFrame, since docs for cvQueryFrame
		// say not to release it's return
		IplImage *currentFrame = cvQueryFrame( _capture );

		// If we need to create a gray-scale image from a color, this is where we
		// will store it
		StackImage local_gray_image(cvCreateImage( image_size, 8, 1 ));
		StackImage local_color_image(cvCloneImage(currentFrame));
		IplImage *gray_image; // points at either currentFrame or local_gray_image

		// Capture Corner views loop until we've got desired_boards
		// successful captures (all corners on the board are found)

		while( captured_boards < desired_boards ){
			currentFrame = cvQueryFrame( _capture );
			// Skip every board_dt frames to allow user to move chessboard
			if( frame++ % board_dt == 0 ){
				if (currentFrame->nChannels > 1)
				{
					cvCvtColor( currentFrame, local_gray_image(), CV_BGR2GRAY );
					gray_image = local_gray_image();
				}
				else gray_image = currentFrame;

				// Find chessboard corners:
				int found = cvFindChessboardCorners( gray_image, board_size,
						&(corners[0]), &corner_count,
						CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

				// If we got a good board...,
				if ( corner_count == n_corners ){
					// ...get subpixel accuracy on those corners...
					cvFindCornerSubPix( gray_image, &(corners[0]), corner_count,
							cvSize( 11, 11 ), cvSize( -1, -1 ),
							cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

					// ...and add it to our data
					int startI = captured_boards * n_corners;
					for( int i=0; i < n_corners; ++i) {
						_image_points.push_back(corners[i]);
					}
					captured_boards++;
					printf("Found board! (%d of %d - %.2f%%)\n", captured_boards, desired_boards, float(captured_boards)/desired_boards * 100);
				}

				// Draw it
				if ( corner_count > 0 )
				{
					// Docs say we shouldn't modify currentFrame returned by cvQueryFrame -
					// so copy to local_color_image, and draw corners over that
					cvCopy(currentFrame, local_color_image());
					cvDrawChessboardCorners( local_color_image(), board_size,
							&(corners[0]), corner_count, found );
					cvShowImage( "Calibration", local_color_image() );
				}
				else cvShowImage( "Calibration", currentFrame );
			}

			// Handle pause/unpause and ESC
			int c = cvWaitKey( 15 );
			if( c == 'p' ){
				c = 0;
				while( c != 'p' && c != 27 ){
					c = cvWaitKey( 250 );
				}
			}
			if( c == 27 ) break;
		} // End collection while loop

		return captured_boards;
	}

	void calculateCalibration()
	{
		// Allocate matrices according to how many chessboards found
		StackMat object_points_mat(cvCreateMat( captured_boards*n_corners, 3, CV_32FC1 ));
		StackMat image_points_mat(cvCreateMat( captured_boards*n_corners, 2, CV_32FC1 ));
		StackMat point_counts_mat(cvCreateMat( captured_boards, 1, CV_32SC1 ));

		int boardIndex;
		// Transfer the points into the correct size matrices
		for( int i = 0; i < captured_boards*n_corners; ++i ){
			int boardIndex = i % n_corners;
			const CvPoint2D32f& imagePt = _image_points[i];
			CV_MAT_ELEM( *image_points_mat(), float, i, 0) = imagePt.x;
			CV_MAT_ELEM( *image_points_mat(), float, i, 1) = imagePt.y;

			CV_MAT_ELEM( *object_points_mat(), float, i, 0) = boardIndex / board_size.width;
			CV_MAT_ELEM( *object_points_mat(), float, i, 1) = boardIndex % board_size.width;
			CV_MAT_ELEM( *object_points_mat(), float, i, 2) = 0.0;
		}

		for( int i=0; i < captured_boards; ++i ){
			CV_MAT_ELEM( *point_counts_mat(), int, i, 0 ) = n_corners;
		}

		// At this point we have all the chessboard corners we need
		// Initialize the intrinsic matrix such that the two focal lengths
		// have a ratio of 1.0

		CV_MAT_ELEM( *_intrinsic_matrix, float, 0, 0 ) = 1.0;
		CV_MAT_ELEM( *_intrinsic_matrix, float, 1, 1 ) = 1.0;

		// Calibrate the camera
		cvCalibrateCamera2( object_points_mat(), image_points_mat(), point_counts_mat(), image_size,
			_intrinsic_matrix, _distortion_coeffs, NULL, NULL,
			CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_ZERO_TANGENT_DIST);

		// | alphaU   0    u0 |
		// |    0   alphaV v0 |
		// |    0     0     1 |s
		assert(CV_MAT_ELEM( *_intrinsic_matrix, float, 0, 1 ) == 0);
		assert(CV_MAT_ELEM( *_intrinsic_matrix, float, 1, 0 ) == 0);
		assert(CV_MAT_ELEM( *_intrinsic_matrix, float, 2, 0 ) == 0);
		assert(CV_MAT_ELEM( *_intrinsic_matrix, float, 2, 1 ) == 0);
		assert(CV_MAT_ELEM( *_intrinsic_matrix, float, 2, 2 ) == 1);

		float rtSlamDistortion[3];
		rtSlamDistortion[0] = CV_MAT_ELEM( *_distortion_coeffs, float, 0, 0 );
		rtSlamDistortion[1] = CV_MAT_ELEM( *_distortion_coeffs, float, 1, 0 );
		rtSlamDistortion[2] = CV_MAT_ELEM( *_distortion_coeffs, float, 2, 0 );
		float rtSlamIntrinsics[4];
		rtSlamIntrinsics[0] = CV_MAT_ELEM( *_intrinsic_matrix, float, 0, 2 );  // u0
		rtSlamIntrinsics[1] = CV_MAT_ELEM( *_intrinsic_matrix, float, 1, 2 );  // v0
		rtSlamIntrinsics[2] = CV_MAT_ELEM( *_intrinsic_matrix, float, 0, 0 );  // alphaU
		rtSlamIntrinsics[3] = CV_MAT_ELEM( *_intrinsic_matrix, float, 1, 1 );  // alphaV
		printf("rt-slam distortion values: %f %f %f\n",
				rtSlamDistortion[0], rtSlamDistortion[1], rtSlamDistortion[2]);
		printf("rt-slam intrinsics values (u0, v0, alphaU, alphaV): %f %f %f %f\n",
				rtSlamIntrinsics[0], rtSlamIntrinsics[1],
				rtSlamIntrinsics[2], rtSlamIntrinsics[3]);


		// Save the intrinsics and distortions

		// Don't forgot the +1 for the null terminator
		char intrinsicsFile[strlen("Intrinsics.2012-03-14.143050.xml") + 1];
		char distortionFile[strlen("Distortion.2012-03-14.143050.xml") + 1];
		time_t rawtime;
		struct tm timeinfo;
		time(&rawtime);
		timeinfo = *localtime(&rawtime);
		int year = timeinfo.tm_year + 1900;
		int month = timeinfo.tm_mon + 1;
		sprintf(intrinsicsFile, "Intrinsics.%04d-%02d-%02d.%02d%02d%02d.xml",
				year, month, timeinfo.tm_mday,
				timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
		sprintf(distortionFile, "Distortion.%04d-%02d-%02d.%02d%02d%02d.xml",
				year, month, timeinfo.tm_mday,
				timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);


		cvSave( intrinsicsFile, _intrinsic_matrix, "Intrinsics" );
		cvSave( distortionFile, _distortion_coeffs, "Distortion" );
	}



	void loadCalibration(std::string intrinsicsFile="",
			std::string distortionFile="")
	{
		if (intrinsicsFile == "")
		{
			intrinsicsFile = _intrinsicsFile;
		}
		if (distortionFile == "")
		{
			distortionFile = _distortionFile;
		}
		// Example of loading these matrices back in
		CvMat *_intrinsic_matrix = (CvMat*)cvLoad( intrinsicsFile.c_str() );
		CvMat *_distortion_coeffs = (CvMat*)cvLoad( distortionFile.c_str() );
	}

	void showDistortion()
	{
		// Do NOT use StackImage for currentFrame, since docs for cvQueryFrame
		// say not to release it's return
		IplImage *currentFrame = cvQueryFrame( _capture );

		// Build the undistort map that we will use for all subsequent frames
		StackImage mapx(cvCreateImage( cvGetSize(currentFrame), IPL_DEPTH_32F, 1 ));
		StackImage mapy(cvCreateImage( cvGetSize(currentFrame), IPL_DEPTH_32F, 1 ));
		cvInitUndistortMap( _intrinsic_matrix, _distortion_coeffs, mapx(), mapy() );

		// Run the camera to the screen, now showing the raw and undistorted image
		cvNamedWindow( "Undistort" );

		while( currentFrame ){
			IplImage *t = cvCloneImage( currentFrame );
			cvShowImage( "Calibration", currentFrame ); // Show raw image
			cvRemap( t, currentFrame, mapx(), mapy() ); // undistort image
			cvReleaseImage( &t );
			cvShowImage( "Undistort", currentFrame ); // Show corrected image

			// Handle pause/unpause and esc
			int c = cvWaitKey( 15 );
			if( c == 'p' ){
				c = 0;
				while( c != 'p' && c != 27 ){
					c = cvWaitKey( 250 );
				}
			}
			if( c == 27 )
				break;
			currentFrame = cvQueryFrame( _capture );
		}
	}

	// Wrapper for captureBoards, calculateCalibration, showDistortion
	void calibrateAndShow()
	{
		captureBoards();
		calculateCalibration();
		showDistortion();
	}

//	CvPoint cornerNumToCornerXY(int n)
//	{
//		return cvPoint(n / board_size.width, n % board_size.width);
//	}

};

//int _tmain(int argc, _TCHAR* argv[])
int main(int argc, char* argv[])
{
	CameraCalibrator(cvSize(13,10)).calibrateAndShow();
	return 0;
}


