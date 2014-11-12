project(point-pair-features)

cmake_minimum_required(VERSION 2.8)

include(CheckCXXCompilerFlag)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall") # -DCMAKE_BUILD_TYPE=Debug

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})


#########################################################
# find freeglut    http://stackoverflow.com/questions/6984479/cmake-how-to-link-a-library-without-automatic-search-function-find-package
#########################################################
if(APPLE)  #find_package(GLUT) is not automatically freeglut: TODO: check
    FIND_PATH(GLUT_INCLUDE_DIRS GL/freeglut.h /opt/X11/include)
    FIND_LIBRARY(GLUT_LIBRARIES glut.3 /opt/X11/lib)

    IF(NOT GLUT_INCLUDE_DIRS)
      message(ERROR " freeglut not found!")
    ENDIF(GLUT_INCLUDE_DIRS)

    IF(NOT GLUT_LIBRARY_DIRS)
      message(ERROR " freeglut lib not found!")
    ENDIF(GLUT_LIBRARY_DIRS)
else()
    find_package(GLUT REQUIRED)  #on linux it is freeglut by default
endif()

include_directories(${GLUT_INCLUDE_DIRS})


#########################################################
# FIND Eigen     is required by all         https://forum.kde.org/viewtopic.php?f=74&t=94166
#########################################################
#find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)   # TODO check: is not set, but still works (because its in default location i guess)
# http://stackoverflow.com/questions/12249140/find-package-eigen3-for-cmake
#SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
#IF( NOT EIGEN3_INCLUDE_DIR )
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
#ENDIF()
#INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

include_directories(include/)

set(SOURCE_FILES
    src/Constants.h
    src/RandomN.h

    src/LoadingSaving.cpp
    src/PointCloudManipulation.cpp

    src/PPF.cpp
    src/PointPairFeatures.cpp
)


##no dependencies
add_executable(
    poseDiffTest
    src/mainPoseDiff.cpp
    ${SOURCE_FILES}
)

add_executable(
    localCoords
    src/mainLocalCoordinates.cpp
    ${SOURCE_FILES}
)


##opengl + glut dependencies
add_executable(
    main
    src/main.cpp
    ${SOURCE_FILES}
    src/Visualize.cpp
)
target_link_libraries(main ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} )
add_custom_command(TARGET main PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:main>/bunny)


add_executable(
    frameToModel
    src/main_frameToModel.cpp
    ${SOURCE_FILES}
    src/Visualize.cpp
)
target_link_libraries(frameToModel ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} )
add_custom_command(TARGET frameToModel PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:frameToModel>/bunny)

## OpenCV dependencies
find_package(OpenCV)
if (OPENCV_FOUND)
include_directories(${OPENCV_INCLUDE_DIRS})
    add_executable(
        viz
        src/mainOpenCVViz.cpp
    )
    target_link_libraries(viz ${OpenCV_LIBS})

    add_executable(
        viz2
        src/mainTransformations.cpp
    )
    target_link_libraries(viz2 ${OpenCV_LIBS})

    add_executable(
        sequence
        src/mainSequence.cpp
        ${SOURCE_FILES}
    )
    target_link_libraries(sequence ${OpenCV_LIBS})
    add_custom_command(TARGET sequence PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:sequence>/bunny)
else()
  message("OpenCV was not found - main reliing on opencv viz will not be built")
endif()

## Gtest dependencies
find_package(GTest)
if (GTEST_FOUND)
include_directories(${GTEST_INCLUDE_DIRS})
  add_executable(
    runTests
    ${SOURCE_FILES}
    tests/runTests.cpp
    tests/testHelpers.h
    )
  target_link_libraries(runTests ${GTEST_LIBRARIES} pthread)
else()
  message("GTest was not found - testcases can and will not be build")
endif()

