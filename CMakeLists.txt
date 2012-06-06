SET(appName cameraCalibration)
SET(calibSrc calibrate.cpp)
SET(BOOST_LIBS boost_system boost_filesystem)
SET(OPENCV_LIBS
    opencv_core
    opencv_calib3d
    opencv_imgproc
    opencv_ml
    opencv_highgui)

ADD_EXECUTABLE(${appName} ${calibSrc})
TARGET_LINK_LIBRARIES( ${appName} ${BOOST_LIBS} )
TARGET_LINK_LIBRARIES( ${appName} ${OPENCV_LIBS} )