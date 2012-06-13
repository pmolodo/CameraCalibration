/*
 * calibrate.cpp
 *
 *  Created on: Jun 06, 2012
 *      Author: paulm
 */

// Modified from code from http://docs.opencv.org/_downloads/camera_calibration.cpp
//   (tutorial page: http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html#cameracalibrationopencv)


#include <iostream>
#include <stdio.h>
#include <sstream>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

using namespace cv;
using namespace std;

namespace bfs = boost::filesystem;

void help()
{
    cout <<  "This is a camera calibration sample." << endl
         <<  "Usage: calibration configurationFile"  << endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}

struct tm currentTime()
{
    time_t rawtime;
    struct tm timeinfo;
    time(&rawtime);
    timeinfo = *localtime(&rawtime);
    return timeinfo;
}

struct tm startTime;

string dateString(const struct tm& timeinfo)
{
    int year = timeinfo.tm_year + 1900;
    int month = timeinfo.tm_mon + 1;
    return format("%04d-%02d-%02d", year, month, timeinfo.tm_mday);
}

string timeString(const struct tm& timeinfo)
{
    return format("%02d%02d%02d", timeinfo.tm_hour, timeinfo.tm_min,
            timeinfo.tm_sec);
}

// Replaces all occurrences of toFind in toModify with toReplace; returns number
// of replacements made
int replaceAll(string& toModify, const string& toFind, const string& toReplace)
{
    int foundIndex;
    int startPos = 0;
    int foundCount = 0;

    while(startPos < toModify.size())
    {
        foundIndex = toModify.find(toFind, startPos);
        if (foundIndex < 0) break;
        foundCount += 1;
        toModify.replace(foundIndex, toFind.size(), toReplace);
        startPos += toReplace.size();
    }
    return foundCount;
}

/*
   # When specifying paths, the following tokens are supported:
   #    %D: replaced with the date, in year-month-day format, ie: 2012-06-20
   #    %T: replaced with the time, as 6 digits, ie 161859 for 4:18 pm, and 59 seconds
   #    %d: replaced with the integer number if the output can be used several
   #        times - ie, for the filename of output images; printf-style format
   #        specifiers are allowed, ie %04d; if used when multiple outputs do
   #        not make sense, the number 0 is used
   #    %%: relaced with a single percent sign
 */


// If useStartTime, then use the time cached when main ran; otherwise, use the
// current time
void replaceTokens(string& toModify, bool useStartTime, int counter=0)
{
    struct tm timeinfo;
    if (useStartTime)
    {
        timeinfo = ::startTime;
    }
    else
    {
        timeinfo = currentTime();
    }
    replaceAll(toModify, "%D", dateString(timeinfo));
    replaceAll(toModify, "%T", timeString(timeinfo));


    // match printf-style %d tokens
    static const boost::regex intFormatRE("%[-+ #0]{0,5}[0-9]*(\\.[0-9]*)?[hl]?d");

    boost::match_results<string::iterator> match;
    //boost::smatch match;
    string::iterator start = toModify.begin();
    string::iterator end = toModify.end();
    int currentPos = 0;
    while(regex_search(start, end, match, intFormatRE))
    {
        // do the replacement in the matched substring
        string orig = match.str();
        string toReplace = format(orig.c_str(), counter);
        // match.position() will give an index relative to the start iterator
        // we give it... so to get position of match within the string, we add
        // currentPos
        currentPos += match.position();
        toModify.replace(currentPos, orig.size(), toReplace);
        currentPos += toReplace.size();
        start = toModify.begin() + currentPos;
        end = toModify.end();
    }
}

// removes trailing slashes, uses make_preferred
void standardizePath(bfs::path& inPath)
{
    string pathStr = inPath.string();

    inPath.make_preferred();
    if (inPath.filename() == ".") inPath.remove_filename();
}


class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD,
#if OPENCV_2_4
        CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
#endif
    };
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse

                  << "Input" << input
                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Input_Delay" << delay

                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                  << "Write_outputDir" << outputDir.string()
                  << "Write_outputFileName" << outputFileName.string()
                  << "Write_DetectedFeaturePoints" << bwritePoints
                  << "Write_extrinsicParameters" << bwriteExtrinsics
                  << "Write_raw_calibration_images" << bwriteRawImage
                  << "Write_raw_image_fileName" << rawImageFileName.string()
                  << "Write_overlay_calibration_images" << bwriteOverlayImage
                  << "Write_overlay_image_fileName" << overlayImageFileName.string()
                  << "Write_calibration_image_list" << bwriteImageList
                  << "Write_image_list_fileName" << imageListFileName.string()

                  << "Show_UndistortedImage" << showUndistorsed

           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_Pattern"] >> patternToUse;

        node["Input"] >> input;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Input_Delay"] >> delay;

        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;


        node["Write_outputDir"] >> outputDir;
        node["Write_outputFileName"] >> outputFileName;
        node["Write_DetectedFeaturePoints"] >> bwritePoints;
        node["Write_extrinsicParameters"] >> bwriteExtrinsics;
        node["Write_raw_calibration_images"] >> bwriteRawImage;
        node["Write_raw_image_fileName"] >> rawImageFileName;
        node["Write_overlay_calibration_images"] >> bwriteOverlayImage;
        node["Write_overlay_image_fileName"] >> overlayImageFileName;
        node["Write_calibration_image_list"] >> bwriteImageList;
        node["Write_image_list_fileName"] >> imageListFileName;

        node["Show_UndistortedImage"] >> showUndistorsed;
        interprate();
    }

    void interprate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (input.empty())      // Check for valid input
                inputType = INVALID;
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                if (readStringList(input, imageList))
                    {
                        inputType = IMAGE_LIST;
                        nrFrames = (nrFrames < imageList.size()) ? nrFrames : imageList.size();
                    }
                else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA)
                inputCapture.open(cameraID);
            if (inputType == VIDEO_FILE)
                inputCapture.open(input);
            if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                    inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Inexistent input: " << input << endl;
            goodInput = false;
        }

        flag = 0;
        if(calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;


        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
#if OPENCV_2_4
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
#endif
        if (calibrationPattern == NOT_EXISTING)
            {
                cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
                goodInput = false;
            }
        atImageList = 0;

        outputDir = bfs::absolute(outputDir);
        replacePathTokens(outputDir, true);
        makeOutputPath(outputFileName);
        replacePathTokens(outputFileName, true);
        makeOutputPath(rawImageFileName);
        makeOutputPath(overlayImageFileName);
        makeOutputPath(imageListFileName);
    }

    Mat nextImage()
    {
        Mat result;
        if( inputCapture.isOpened() )
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if( atImageList < (int)imageList.size() )
            result = imread( imageList[atImageList++].string(), CV_LOAD_IMAGE_COLOR);

        return result;
    }

    static bool readStringList( const string& filename, vector<bfs::path>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }

    void makeOutputPath( bfs::path& file)
    {
        standardizePath(file);
        if (not outputDir.empty() and file.is_relative())
        {
            file = outputDir / file;
        }
        file = absolute(file);
    }

    void outputPrep( const bfs::path& outputPath )
    {
        bfs::path parent(absolute(outputPath).parent_path());
        if (not bfs::exists(parent))
        {
            create_directories(parent);
        }
    }

    void replacePathTokens(bfs::path& path, bool useStartTime, int counter=0)
    {
        if (not path.empty())
        {
            string newPath = path.string();
            replaceTokens(newPath, useStartTime, counter);
            path = newPath;
        }
    }


public:
    Size boardSize;            // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;              // The number of frames to use from the input for calibration
    float aspectRatio;         // The aspect ratio
    int delay;                 // In case of a video input
    bool calibZeroTangentDist; // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string input;               // The input ->

    bfs::path outputDir;               // directory all other output images are placed under
    bfs::path outputFileName;          // The name of the file where to write
    bool bwritePoints;              //  Write detected feature points
    bool bwriteExtrinsics;          // Write extrinsic parameters
    bool bwriteRawImage;            // Write raw captured calibration images
    bfs::path rawImageFileName;        // The name pattern for raw images
    bool bwriteOverlayImage;        // Write board-overlayed captured calibration images
    bfs::path overlayImageFileName;    // The name pattern for overlay images
    bool bwriteImageList;           // Write out an image list when writing raw images
    bfs::path imageListFileName;       // The filename for the image list

    bool showUndistorsed;       // Show undistorted images after calibration

    int cameraID;
    vector<bfs::path> imageList;
    int atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;


};

namespace cv
{
    void write(FileStorage& fs, const std::string&, const Settings& x)
    {
        x.write(fs);
    }
    void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
    {
        if(node.empty())
            x = default_value;
        else
            x.read(node);
    }

    void read(const FileNode& node, bfs::path& p, const bfs::path& default_value = bfs::path())
    {
        if(node.empty())
        {
            p = default_value;
        }
        else
        {
            string temp;
            node >> temp;
            p = temp;
            standardizePath(p);
        }
    }
} // namespace cv

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );

int main(int argc, char* argv[])
{
    help();
    Settings s;
    const string inputSettingsFile = argc > 1 ? argv[1] : "default.yaml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }

#if OPENCV_2_4
    fs["Settings"] >> s;
#else
    read(fs["Settings"], s);
#endif
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    for(int i = 0;;++i)
    {
        Mat view;
        bool blinkOutput = false;

        view = s.nextImage();

        //-----  If no more image, or got enough, then stop calibration and show result -------------
        if( mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames )
        {
            if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
              mode = CALIBRATED;
            else
              mode = DETECTION;
        }
        if(view.empty())          // If no more images then run calibration, save and stop loop.
        {
            if( imagePoints.size() > 0 )
                runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
            break;
        }


        imageSize = view.size();  // Format input image.
        if( s.flipVertical )    flip( view, view, 0 );

        vector<Point2f> pointBuf;

        bool found;
        switch( s.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, s.boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            break;
#if OPENCV_2_4
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
#endif
        }

        Mat viewColor;
        Mat viewGray;

        // Would like to always display a color image (so that we can have color
        // text, chessboard overlays, etc)... but some processing may require
        // grayscale
        if (view.channels()==1)
        {
            viewGray = view;
            // We will always want a color copy, so copy it now
            cvtColor(view, viewColor, CV_GRAY2BGR);
        }
        else
        {
            // We may not want / need viewGray, so don't copy it now...
            // we will need to check if viewGray is empty before use!
            viewColor = view;
        }

        if ( found)                // If done with success,
        {
            // improve the found corners' coordinate accuracy for chessboard
            if( s.calibrationPattern == Settings::CHESSBOARD)
            {
                // now we want a gray image!
                if (viewGray.empty())
                {
                    cvtColor(view, viewGray, CV_BGR2GRAY);
                }
                cornerSubPix( viewGray, pointBuf, Size(11,11),
                    Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
            }

            if( mode == CAPTURING &&  // For camera only take new samples after delay time
                (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
            {
                imagePoints.push_back(pointBuf);
                prevTimestamp = clock();
                blinkOutput = s.inputCapture.isOpened();
            }

            // Draw the corners.
            drawChessboardCorners( viewColor, s.boardSize, Mat(pointBuf), found );
        }

        //------------------------- Video capture  output  undistorted ------------------------------
        if( mode == CALIBRATED && s.showUndistorsed )
        {
            Mat temp = viewColor.clone();
            undistort(temp, viewColor, cameraMatrix, distCoeffs);
        }

        //----------------------------- Output Text ------------------------------------------------
        string msg = (mode == CAPTURING) ? "100/100" :
                      mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(viewColor.cols - 2*textSize.width - 10, viewColor.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( viewColor, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if( blinkOutput )
            bitwise_not(viewColor, viewColor);

        //------------------------------ Show image and check for input commands -------------------
        imshow("Image View", viewColor);
        char key =  waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        if( key  == ESC_KEY )
            break;

        if( key == 'u' && mode == CALIBRATED )
           s.showUndistorsed = !s.showUndistorsed;

        if( s.inputCapture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
            imageSize, CV_16SC2, map1, map2);

        for(int i = 0; i < (int)s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i].string(), 1);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }


    return 0;
}

double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                  const vector<vector<Point2f> >& imagePoints,
                                  const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                  const Mat& cameraMatrix , const Mat& distCoeffs,
                                  vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                          Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
#if OPENCV_2_4
    case Settings::CIRCLES_GRID:
#endif
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
        break;

#if OPENCV_2_4
    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
        break;
#endif
    }
}

bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,  double& totalAvgErr)
{

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

// Print camera parameters to the output file
void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                       const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                       double totalAvgErr )
{
    s.outputPrep(s.outputFileName);
    FileStorage fs( s.outputFileName.string(), FileStorage::WRITE );

    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;

    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "FixAspectRatio" << s.aspectRatio;

    if( s.flag )
    {
        sprintf( buf, "flags: %s%s%s%s",
            s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );

    }

    fs << "flagValue" << s.flag;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            // Not sure if this is a difference due to OpenCV 2.2 vs 2.4...
            // but in 2.2 rvecs (and possibly tvecs?) were rows=1, col=3...
            // whereas previous code (which was built for 2.4) expected
            // rows=3, cols=1
            if (rvecs[i].rows == 3 && rvecs[i].cols == 1)
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                r = rvecs[i].t();
            }
            else if (rvecs[i].rows == 1 && rvecs[i].cols == 3)
            {
                rvecs[i].copyTo(r);
            }

            if (tvecs[i].rows == 3 && tvecs[i].cols == 1)
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                t = tvecs[i].t();
            }
            else if (tvecs[i].rows == 1 && tvecs[i].cols == 3)
            {
                tvecs[i].copyTo(t);
            }
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
}

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
        << ". avg re projection error = "  << totalAvgErr ;

    if( ok )
        saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                            imagePoints, totalAvgErr);
    return ok;
}
