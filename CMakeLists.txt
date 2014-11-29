project(point-pair-features)

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


#Eigen     is required by all         https://forum.kde.org/viewtopic.php?f=74&t=94166
#find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)   # TODO check: is not set, but still works (because its in default location i guess)
# http://stackoverflow.com/questions/12249140/find-package-eigen3-for-cmake
#SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
#IF( NOT EIGEN3_INCLUDE_DIR )
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
#ENDIF()
#INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

include_directories(include/)
include_directories(src/viz/)
find_package(GTest)



set(SOURCE_FILES
    src/Constants.h
    src/RandomN.h
    src/pointcloud.cpp

    src/LoadingSaving.cpp
    src/PointCloudManipulation.cpp

    src/PPF.cpp
    src/PointPairFeatures.cpp
)

set(VIZ
    src/Visualize.cpp
    src/viz/cameraSimple.h
    src/viz/frustum.h
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
    ${VIZ}
)
target_link_libraries(main ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} )
add_custom_command(TARGET main PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:main>/bunny)

add_executable(
    frameToModel
    src/main_frameToModel.cpp
    ${SOURCE_FILES}
    ${VIZ}
)
target_link_libraries(frameToModel ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} )
add_custom_command(TARGET frameToModel PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:frameToModel>/bunny)




# OpenCV dependencies
find_package(OpenCV)
if (TRUE)#OPENCV_FOUND)
include_directories(${OPENCV_INCLUDE_DIRS})
    add_executable(
        viz
        MACOSX_BUNDLE
        src/mainOpenCVViz.cpp
    )
    target_link_libraries(viz ${OpenCV_LIBS})

    add_executable(
        viz2
        MACOSX_BUNDLE
        src/mainTransformations.cpp
    )
    target_link_libraries(viz2 ${OpenCV_LIBS})

    add_executable(
        sequence
        MACOSX_BUNDLE
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
  target_link_libraries(runTests ${OpenCV_LIBS} ${GTEST_LIBRARIES} pthread)
  add_custom_command(TARGET runTests PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:runTests>/bunny)
else()
  message("GTest was not found - testcases can and will not be build")
endif()

