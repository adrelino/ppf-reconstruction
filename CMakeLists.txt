#todo
#fix normal estimation from depth images
#reject frames if icp line lenght is too big
#global optimization

project(ppf-reconstruction)

cmake_minimum_required(VERSION 2.8)

#DEBUG
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall") # -DCMAKE_BUILD_TYPE=Debug

#C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#FREEGLUT    http://stackoverflow.com/questions/6984479/cmake-how-to-link-a-library-without-automatic-search-function-find-package
find_package(GLUT)
message(STATUS "glut inc default: ${GLUT_INCLUDE_DIRS}")
message(STATUS "glut lib default: ${GLUT_LIBRARIES}")
if(APPLE)  #find_package(GLUT) is not automatically freeglut: TODO: check
    #FIND_PATH(GLUT_INCLUDE_DIRS GL/freeglut.h /opt/X11/include)
    #FIND_LIBRARY(GLUT_LIBRARIES glut /opt/X11/lib)

    #X11/Xquartz
    #SET(GLUT_INCLUDE_DIRS /opt/X11/include)
    #SET(GLUT_LIBRARIES /opt/X11/lib/libglut.dylib)

    #brew install freeglut
    SET(GLUT_INCLUDE_DIRS /usr/local/include)
    SET(GLUT_LIBRARIES /usr/local/lib/libglut.dylib)

else()
    find_package(GLUT)  #on linux it is freeglut by default
endif()
message(STATUS "glut inc used: ${GLUT_INCLUDE_DIRS}")
message(STATUS "glut lib used: ${GLUT_LIBRARIES}")


include_directories(${GLUT_INCLUDE_DIRS})

#OPENGL
find_package(OpenGL)
include_directories(${OPENGL_INCLUDE_DIRS})

#Eigen is required by all         https://forum.kde.org/viewtopic.php?f=74&t=94166
find_package(Eigen3 REQUIRED)
IF( NOT EIGEN3_INCLUDE_DIR )
    if(APPLE)
        SET(EIGEN3_INCLUDE_DIR /usr/local/include/eigen3/) #'/usr/local is homebrews install prefix
    endif()
ENDIF()
MESSAGE("EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR}")
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR}) #http://stackoverflow.com/questions/12249140/find-package-eigen3-for-cmake


include_directories(include/)
include_directories(src/viz/)

# OpenCV dependencies
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})


file(GLOB HEADERS
    include/*.h
    include/*.hpp
)

#file(GLOB SOURCES src/*.cpp)
set(SOURCES
    src/Params.cpp
    src/PointCloud.cpp
    src/LoadingSaving.cpp
    src/PointCloudManipulation.cpp
    src/PPF.cpp
    src/PointPairFeatures.cpp
    src/CPUTimer.cpp
    #src/types_icp.cpp
    #src/g2oTypeSim3Sophus.cpp
    src/ApproachComponents.cpp
)

set(VIZ
    src/Visualize.cpp
    src/viz/cameraSimple.h
    src/viz/frustum.h
)

##opengl + glut dependencies
#add_executable(
#    modelToScene
#    src/main_modelToScene.cpp
#    ${HEADERS}
#    ${SOURCES}
#    ${VIZ}
#)
#target_link_libraries(modelToScene ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS})
#add_custom_command(TARGET modelToScene PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/samples $<TARGET_FILE_DIR:modelToScene>/samples)

#add_executable(
#    frameToModel-PPF-coarse
#    src/main_frameToModel-PPF-coarse.cpp
#    ${SOURCES}
#    ${VIZ}
#)
#target_link_libraries(frameToModel-PPF-coarse ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS})
#add_custom_command(TARGET frameToModel-PPF-coarse PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/samples $<TARGET_FILE_DIR:frameToModel-PPF-coarse>/samples)


#g2o

if(UNIX AND NOT APPLE)
    if(EXISTS /opt/ros/indigo/include/g2o)  #and ros and ros-indigo-libg2o is installed
        MESSAGE("using ROS g2o from package ros-indigo-libg2o")
        link_directories(/opt/ros/indigo/lib/)
    endif()
endif()

set(G2O_LIBRARIES
    g2o_core g2o_stuff g2o_types_slam3d g2o_types_icp g2o_solver_csparse g2o_csparse_extension
)

#add_executable(
#    globalOptimization
#    src/main_globalOptimization.cpp
#    ${HEADERS}
#    ${SOURCES}
#    ${VIZ}
#)
#target_link_libraries(globalOptimization ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBRARIES})

#add_executable(
#    frameToFrame
#    src/main_frameToFrame.cpp
#    ${HEADERS}
#    ${SOURCES}
#    ${VIZ}
#)
#target_link_libraries(frameToFrame ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBRARIES})

add_executable(
    ppf_reconstruction
    src/main_ppf_reconstruction.cpp
    ${HEADERS}
    ${SOURCES}
    ${VIZ}
)
target_link_libraries(ppf_reconstruction ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBRARIES})



#ADD_EXECUTABLE(gicp_adrian
#  src/gicp_adrian.cpp
#    ${HEADERS}
#    ${SOURCES}
#    ${VIZ}
#)
#INCLUDE_DIRECTORIES(/usr/local/include/g2o)
#INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

#TARGET_LINK_LIBRARIES(gicp_adrian ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBRARIES})



#add_executable(lm-icp
#    src/main_lm-icp.cpp
#    ${HEADERS}
#    ${SOURCES}
#    ${VIZ}
#)
#TARGET_LINK_LIBRARIES(lm-icp ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBRARIES})


#add_executable(
#    opencv-viz1
#    MACOSX_BUNDLE
#    src/viz-opencv/mainOpenCVViz.cpp
#)
#target_link_libraries(opencv-viz1 ${OpenCV_LIBS})

#add_executable(
#    opencv-viz2
#    MACOSX_BUNDLE
#    src/viz-opencv/mainTransformations.cpp
#${SOURCES}
#${VIZ}
#)
#target_link_libraries(opencv-viz2 ${OpenCV_LIBS} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

#add_executable(
#    opencv-seq
#    MACOSX_BUNDLE
#    src/viz-opencv/mainSequence.cpp
#    ${SOURCES}
#)
#target_link_libraries(opencv-seq ${OpenCV_LIBS})
#add_custom_command(TARGET opencv-seq PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/samples $<TARGET_FILE_DIR:opencv-seq>/samples)


## Gtest dependencies
#find_package(GTest)
#if (GTEST_FOUND)
#include_directories(${GTEST_INCLUDE_DIRS})
#  add_executable(
#    test_PointCloudManipulation
#    ${SOURCES}
#    tests/test_PointCloudManipulation.cpp
#    tests/testHelpers.h
#    )
#  target_link_libraries(test_PointCloudManipulation ${OpenCV_LIBS} ${GTEST_LIBRARIES} pthread)
#  add_custom_command(TARGET test_PointCloudManipulation PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/samples $<TARGET_FILE_DIR:test_PointCloudManipulation>/samples)

#  add_executable(
#    test_PointPairFeatures
#    ${SOURCES}
#    tests/test_PointPairFeatures.cpp
#    tests/testHelpers.h
#    )
#  target_link_libraries(test_PointPairFeatures ${OpenCV_LIBS} ${GTEST_LIBRARIES} pthread)
#  add_custom_command(TARGET test_PointPairFeatures PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/samples $<TARGET_FILE_DIR:test_PointPairFeatures>/samples)

#  add_executable(
#    test_PoseClustering
#    ${SOURCES}
#    tests/test_PoseClustering.cpp
#    tests/testHelpers.h
#    )
#  target_link_libraries(test_PoseClustering ${OpenCV_LIBS} ${GTEST_LIBRARIES} pthread)
#  add_custom_command(TARGET test_PoseClustering PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/samples $<TARGET_FILE_DIR:test_PoseClustering>/samples)

#else()
#  message("GTest was not found - testcases can and will not be build")
#endif()

