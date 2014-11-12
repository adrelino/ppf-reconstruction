cmake_minimum_required(VERSION 2.8)
project(point-pair-features)

include(CheckCXXCompilerFlag)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

#  -DCMAKE_BUILD_TYPE=Debug

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
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)


#########################################################
# find freeglut
#########################################################
FIND_PATH(Freeglut_INCLUDE_DIRS GL/freeglut.h /opt/X11/include)
FIND_LIBRARY(Freeglut_LIBRARY glut.3 /opt/X11/lib)

include_directories(${Freeglut_INCLUDE_DIRS})

#link_directories(${Freeglut_LIBRARY})
link_directories("/opt/X11/lib")

IF(Freeglut_INCLUDE_DIRS)
  message("*** freeglut include is found!")
ENDIF(Freeglut_INCLUDE_DIRS)

IF(Freeglut_LIBRARY)
  message("*** freeglut lib is found!")
ENDIF(Freeglut_LIBRARY)

#########################################################
# find glew
#########################################################
FIND_PATH(Glew_INCLUDE_DIRS GL/glew.h /usr/local/include)
FIND_LIBRARY(Glew_LIBRARY GLEW /usr/local/lib)

include_directories(${Glew_INCLUDE_DIRS})
link_directories("/usr/local/lib")

IF(Glew_INCLUDE_DIRS)
  message("*** Glew inlclude is found!")
ENDIF(Glew_INCLUDE_DIRS)

IF(Glew_LIBRARY)
  message("*** Glew lib is found!")
ENDIF(Glew_LIBRARY)


add_definitions(-std=c++11)




find_package(OpenCV)
find_package(GTest)

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(EIGEN3_INCLUDE_DIR)
include_directories(${OPENCV_INCLUDE_DIRS})

LINK_DIRECTORIES()

set(SOURCE_FILES
    src/Constants.h
    src/RandomN.h

    src/LoadingSaving.cpp
    src/PointCloudManipulation.cpp

    src/PPF.cpp
    src/PointPairFeatures.cpp
)


add_executable(
    main
    src/main.cpp
    ${SOURCE_FILES}
    src/Visualize.cpp
)
target_link_libraries(main ${Glew_LIBRARY} ${Freeglut_LIBRARY} ${OPENGL_LIBRARIES} )
add_custom_command(TARGET main PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:main>/bunny)

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

add_executable(
    frameToModel
    src/main_frameToModel.cpp
    ${SOURCE_FILES}
    src/Visualize.cpp
)
target_link_libraries(frameToModel ${Glew_LIBRARY} ${Freeglut_LIBRARY} ${OPENGL_LIBRARIES} )
add_custom_command(TARGET frameToModel PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:frameToModel>/bunny)


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

if (GTEST_FOUND)
  add_executable(
    runTests
    ${SOURCE_FILES}
    tests/runTests.cpp
    tests/testHelpers.h
    )
  target_link_libraries(runTests ${OpenCV_LIBS} ${GTEST_LIBRARIES} pthread)
else()
  message("GTest was not found - testcases can and will not be build")
endif()

