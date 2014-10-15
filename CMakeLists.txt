project(point-pair-features)

cmake_minimum_required(VERSION 2.8)

include(CheckCXXCompilerFlag)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")


CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(EIGEN3_INCLUDE_DIR)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})


add_executable(
	ppfs_test
	src/main.cpp
	src/Constants.h
	src/LoadingSaving.cpp
	src/LoadingSaving.h
	src/PPF.cpp
	src/PPF.h
	src/Params.h
	src/PointCloudManipulation.cpp
	src/PointCloudManipulation.h
	src/PointPairFeatures.cpp
	src/PointPairFeatures.h
	src/Visualize.cpp
	src/Visualize.h
)

add_custom_command(TARGET ppfs_test PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:ppfs_test>)

add_executable(
        poseDiffTest
        src/mainPoseDiff.cpp
        src/Constants.h
        src/LoadingSaving.cpp
        src/LoadingSaving.h
        src/PPF.cpp
        src/PPF.h
        src/Params.h
        src/PointCloudManipulation.cpp
        src/PointCloudManipulation.h
        src/PointPairFeatures.cpp
        src/PointPairFeatures.h
        src/Visualize.cpp
        src/Visualize.h
)

target_link_libraries(ppfs_test ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES}) 
target_link_libraries(poseDiffTest ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})