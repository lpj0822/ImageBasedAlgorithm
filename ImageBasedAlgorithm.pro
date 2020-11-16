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
include(adasAlgorithm/adasAlgorithm.pri)

SOURCES += main.cpp

HEADERS += \

INCLUDEPATH+= D:\opencv\opencv400\MyBuild\install\include

#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xphoto400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xobjdetect400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ximgproc400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xfeatures2d400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_videostab400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_videoio400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_video400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_tracking400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_text400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_surface_matching400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_superres400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_structured_light400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_stitching400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_stereo400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_shape400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_saliency400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_rgbd400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_reg400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_plot400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_photo400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_phase_unwrapping400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_optflow400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_objdetect400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ml400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_line_descriptor400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_imgproc400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_imgcodecs400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_img_hash400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_highgui400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_hfs400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_hdf400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_fuzzy400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_flann400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_features2d400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_face400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dpm400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dnn400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dnn_objdetect400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_datasets400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_core400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ccalib400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_calib3d400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_bioinspired400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_bgsegm400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_aruco400d.lib

LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xphoto400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xobjdetect400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ximgproc400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xfeatures2d400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_videostab400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_videoio400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_video400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_tracking400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_text400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_surface_matching400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_superres400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_structured_light400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_stitching400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_stereo400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_shape400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_saliency400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_rgbd400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_reg400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_plot400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_photo400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_phase_unwrapping400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_optflow400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_objdetect400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ml400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_line_descriptor400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_imgproc400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_imgcodecs400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_img_hash400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_highgui400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_hfs400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_hdf400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_fuzzy400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_flann400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_features2d400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_face400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dpm400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dnn400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dnn_objdetect400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_datasets400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_core400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ccalib400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_calib3d400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_bioinspired400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_bgsegm400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_aruco400.lib
