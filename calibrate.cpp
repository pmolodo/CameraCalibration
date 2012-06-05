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

#include "cv.h"
#include "ml.h"
#include "cxcore.h"
#include "highgui.h"

int n_boards = 0;
const int board_dt = 20;
int board_w;
int board_h;

//int _tmain(int argc, _TCHAR* argv[])
int main(int argc, char* argv[])
{
	// Width and height are # of "internal corners", NOT # of squares!
	board_w = 10; // Board width in internal corners (# of squares - 1)
	board_h = 13; // Board height in internal corners (# of squares - 1)
	n_boards = 25; // Number of boards
	int board_n = board_w * board_h;
	CvSize board_sz = cvSize( board_w, board_h );
	CvCapture* capture = cvCreateCameraCapture( 300 );
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);

	assert( capture );

	cvNamedWindow( "Calibration" );
	// Allocate Sotrage
	CvMat* image_points		= cvCreateMat( n_boards*board_n, 2, CV_32FC1 );
	CvMat* object_points		= cvCreateMat( n_boards*board_n, 3, CV_32FC1 );
	CvMat* point_counts			= cvCreateMat( n_boards, 1, CV_32SC1 );
	CvMat* intrinsic_matrix		= cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* distortion_coeffs	= cvCreateMat( 5, 1, CV_32FC1 );

	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
	int corner_count;
	int successes = 0;
	int step, frame = 0;

	IplImage *image = cvQueryFrame( capture );
	CvSize capture_size = cvGetSize( image );
	// If we need to create a gray-scale image from a color, this is where we
	// will store it
	IplImage *local_gray_storage = cvCreateImage( capture_size, 8, 1 );
	IplImage *gray_image; // points at either image or local_gray_storage

	printf("Size: %d x %d\n", capture_size.width, capture_size.height);

	// Capture Corner views loop until we've got n_boards
	// succesful captures (all corners on the board are found)

	while( successes < n_boards ){
		image = cvQueryFrame( capture );
		// Skip every board_dt frames to allow user to move chessboard
		if( frame++ % board_dt == 0 ){
			// Find chessboard corners:
			int found = cvFindChessboardCorners( image, board_sz, corners,
				&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

			// Get subpixel accuracy on those corners
			if (image->nChannels > 1)
			{
				cvCvtColor( image, local_gray_storage, CV_BGR2GRAY );
				gray_image = local_gray_storage;
			}
			else gray_image = image;
			cvFindCornerSubPix( gray_image, corners, corner_count, cvSize( 11, 11 ),
				cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			// Draw it
			cvDrawChessboardCorners( image, board_sz, corners, corner_count, found );
			cvShowImage( "Calibration", image );

			// If we got a good board, add it to our data
			if( corner_count == board_n ){
				step = successes*board_n;
				for( int i=step, j=0; j < board_n; ++i, ++j ){
					CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
					CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
					CV_MAT_ELEM( *object_points, float, i, 0 ) = j/board_w;
					CV_MAT_ELEM( *object_points, float, i, 1 ) = j%board_w;
					CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
				}
				CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
				successes++;
				printf("Found board! (%d of %d - %.2f%%)\n", successes, n_boards, float(successes)/n_boards * 100);
			}
		}

		// Handle pause/unpause and ESC
		int c = cvWaitKey( 15 );
		if( c == 'p' ){
			c = 0;
			while( c != 'p' && c != 27 ){
				c = cvWaitKey( 250 );
			}
		}
		if( c == 27 )
			return 0;
	} // End collection while loop

	// Allocate matrices according to how many chessboards found
	CvMat* object_points2 = cvCreateMat( successes*board_n, 3, CV_32FC1 );
	CvMat* image_points2 = cvCreateMat( successes*board_n, 2, CV_32FC1 );
	CvMat* point_counts2 = cvCreateMat( successes, 1, CV_32SC1 );

	// Transfer the points into the correct size matrices
	for( int i = 0; i < successes*board_n; ++i ){
		CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0 );
		CV_MAT_ELEM( *image_points2, float, i, 1) = CV_MAT_ELEM( *image_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0 );
		CV_MAT_ELEM( *object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2 );
	}

	for( int i=0; i < successes; ++i ){
		CV_MAT_ELEM( *point_counts2, int, i, 0 ) = CV_MAT_ELEM( *point_counts, int, i, 0 );
	}
	cvReleaseMat( &object_points );
	cvReleaseMat( &image_points );
	cvReleaseMat( &point_counts );

	// At this point we have all the chessboard corners we need
	// Initiliazie the intrinsic matrix such that the two focal lengths
	// have a ratio of 1.0

	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0;

	// Calibrate the camera
	cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize( image ),
		intrinsic_matrix, distortion_coeffs, NULL, NULL, CV_CALIB_FIX_ASPECT_RATIO );

	// | alphaU   0    u0 |
	// |    0   alphaV v0 |
	// |    0     0     1 |
	assert(CV_MAT_ELEM( *intrinsic_matrix, float, 0, 1 ) == 0);
	assert(CV_MAT_ELEM( *intrinsic_matrix, float, 1, 0 ) == 0);
	assert(CV_MAT_ELEM( *intrinsic_matrix, float, 2, 0 ) == 0);
	assert(CV_MAT_ELEM( *intrinsic_matrix, float, 2, 1 ) == 0);
	assert(CV_MAT_ELEM( *intrinsic_matrix, float, 2, 2 ) == 1);

	float rtSlamDistortion[3];
	rtSlamDistortion[0] = CV_MAT_ELEM( *distortion_coeffs, float, 0, 0 );
	rtSlamDistortion[1] = CV_MAT_ELEM( *distortion_coeffs, float, 1, 0 );
	rtSlamDistortion[2] = CV_MAT_ELEM( *distortion_coeffs, float, 2, 0 );
	float rtSlamIntrinsics[4];
	rtSlamIntrinsics[0] = CV_MAT_ELEM( *intrinsic_matrix, float, 0, 2 );  // u0
	rtSlamIntrinsics[1] = CV_MAT_ELEM( *intrinsic_matrix, float, 1, 2 );  // v0
	rtSlamIntrinsics[2] = CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 );  // alphaU
	rtSlamIntrinsics[3] = CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 );  // alphaV
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


	cvSave( intrinsicsFile, intrinsic_matrix, "Intrinsics" );
	cvSave( distortionFile, distortion_coeffs, "Distortion" );

	// Example of loading these matrices back in
	CvMat *intrinsic = (CvMat*)cvLoad( intrinsicsFile );
	CvMat *distortion = (CvMat*)cvLoad( distortionFile );

	// Build the undistort map that we will use for all subsequent frames
	IplImage* mapx = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	IplImage* mapy = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap( intrinsic, distortion, mapx, mapy );

	// Run the camera to the screen, now showing the raw and undistorted image
	cvNamedWindow( "Undistort" );

	while( image ){
		IplImage *t = cvCloneImage( image );
		cvShowImage( "Calibration", image ); // Show raw image
		cvRemap( t, image, mapx, mapy ); // undistort image
		cvReleaseImage( &t );
		cvShowImage( "Undistort", image ); // Show corrected image

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
		image = cvQueryFrame( capture );
	}

	return 0;
}


