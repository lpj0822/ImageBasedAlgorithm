QT += core
QT -= gui

TARGET = ImageBasedAlgorithm
CONFIG += console
CONFIG -= app_bundle

QMAKE_CFLAGS += -D _CRT_SECURE_NO_WARNINGS
QMAKE_CFLAGS += /openmp

TEMPLATE = app

include(testing/testing.pri)
include(helpers/helpers.pri)
include(baseAlgorithm/baseAlgorithm.pri)
include(colorConverting/colorConverting.pri)
include(imageFiltering/imageFiltering.pri)
include(imageResize/imageResize.pri)
include(features2d/features2d.pri)
include(opticalFlow/opticalFlow.pri)
include(tracking/tracking.pri)
include(clusteringAlgorithm/clusteringAlgorithm.pri)

SOURCES += main.cpp

HEADERS += \

INCLUDEPATH+= D:\opencv\opencv320\MyBuild\install\include\
              D:\opencv\opencv320\MyBuild\install\include\opencv\
              D:\opencv\opencv320\MyBuild\install\include\opencv2

LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_aruco320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_bgsegm320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_bioinspired320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_calib3d320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ccalib320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_core320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_datasets320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_dnn320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_dpm320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_face320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_features2d320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_flann320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_fuzzy320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_highgui320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_imgcodecs320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_imgproc320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_line_descriptor320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ml320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_objdetect320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_optflow320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_phase_unwrapping320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_photo320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_plot320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_reg320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_rgbd320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_saliency320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_shape320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_stereo320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_stitching320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_structured_light320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_superres320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_surface_matching320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_text320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_tracking320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_video320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_videoio320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_videostab320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xfeatures2d320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ximgproc320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xobjdetect320d.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xobjdetect320d.lib

LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_aruco320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_bgsegm320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_bioinspired320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_calib3d320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ccalib320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_core320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_datasets320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_dnn320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_dpm320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_face320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_features2d320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_flann320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_fuzzy320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_highgui320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_imgcodecs320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_imgproc320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_line_descriptor320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ml320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_objdetect320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_optflow320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_phase_unwrapping320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_photo320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_plot320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_reg320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_rgbd320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_saliency320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_shape320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_stereo320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_stitching320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_structured_light320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_superres320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_surface_matching320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_text320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_tracking320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_video320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_videoio320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_videostab320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xfeatures2d320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ximgproc320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xobjdetect320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xobjdetect320.lib
