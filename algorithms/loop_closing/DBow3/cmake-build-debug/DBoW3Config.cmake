# ===================================================================================
#  DBoW3 CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(DBoW3 REQUIRED )
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME )
#
#    This file will define the following variables:
#      - DBoW3_LIBS          : The list of libraries to links against.
#      - DBoW3_LIB_DIR       : The directory where lib files are. Calling LINK_DIRECTORIES
#                                with this path is NOT needed.
#      - DBoW3_VERSION       : The  version of this PROJECT_NAME build. Example: "1.2.0"
#      - DBoW3_VERSION_MAJOR : Major version part of VERSION. Example: "1"
#      - DBoW3_VERSION_MINOR : Minor version part of VERSION. Example: "2"
#      - DBoW3_VERSION_PATCH : Patch version part of VERSION. Example: "0"
#
# ===================================================================================
INCLUDE_DIRECTORIES("/usr/local/include")
SET(DBoW3_INCLUDE_DIRS "/usr/local/include")

LINK_DIRECTORIES("/usr/local/lib")
SET(DBoW3_LIB_DIR "/usr/local/lib")

SET(DBoW3_LIBS opencv_calib3d;opencv_core;opencv_dnn;opencv_features2d;opencv_flann;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_shape;opencv_stitching;opencv_superres;opencv_video;opencv_videoio;opencv_videostab;opencv_viz;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_cvv;opencv_datasets;opencv_dnn_objdetect;opencv_dpm;opencv_face;opencv_freetype;opencv_fuzzy;opencv_hdf;opencv_hfs;opencv_img_hash;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_reg;opencv_rgbd;opencv_saliency;opencv_sfm;opencv_stereo;opencv_structured_light;opencv_surface_matching;opencv_text;opencv_tracking;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto;/usr/lib/x86_64-linux-gnu/libglog.so DBoW3)
SET(DBoW3_LIBRARIES opencv_calib3d;opencv_core;opencv_dnn;opencv_features2d;opencv_flann;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_shape;opencv_stitching;opencv_superres;opencv_video;opencv_videoio;opencv_videostab;opencv_viz;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_cvv;opencv_datasets;opencv_dnn_objdetect;opencv_dpm;opencv_face;opencv_freetype;opencv_fuzzy;opencv_hdf;opencv_hfs;opencv_img_hash;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_reg;opencv_rgbd;opencv_saliency;opencv_sfm;opencv_stereo;opencv_structured_light;opencv_surface_matching;opencv_text;opencv_tracking;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto;/usr/lib/x86_64-linux-gnu/libglog.so DBoW3)

SET(DBoW3_FOUND YES)
SET(DBoW3_FOUND "YES")
SET(DBoW3_VERSION        0.0.1)
SET(DBoW3_VERSION_MAJOR  0)
SET(DBoW3_VERSION_MINOR  0)
SET(DBoW3_VERSION_PATCH  1)
