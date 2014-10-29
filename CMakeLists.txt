project(point-pair-features)

cmake_minimum_required(VERSION 2.8)

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

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(EIGEN3_INCLUDE_DIR)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

set(SOURCE_FILES
    src/Constants.h
    src/RandomN.h

    src/LoadingSaving.cpp
    src/PointCloudManipulation.cpp

    src/PPF.cpp
    src/PointPairFeatures.cpp
    src/Visualize.cpp
)


add_executable(
    ppfs_test
    src/main.cpp
    ${SOURCE_FILES}
)
target_link_libraries(ppfs_test ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
add_custom_command(TARGET ppfs_test PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:ppfs_test>/bunny)

add_executable(
    sequence
    src/mainSequence.cpp
    ${SOURCE_FILES}
)
target_link_libraries(sequence ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
add_custom_command(TARGET sequence PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:sequence>/bunny)

add_executable(
    frameToModel
    src/main_frameToModel.cpp
    ${SOURCE_FILES}
)
target_link_libraries(frameToModel ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
add_custom_command(TARGET frameToModel PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/bunny $<TARGET_FILE_DIR:frameToModel>/bunny)


add_executable(
    poseDiffTest
    src/mainPoseDiff.cpp
    ${SOURCE_FILES}
)
target_link_libraries(poseDiffTest ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

add_executable(
    localCoords
    src/mainLocalCoordinates.cpp
    ${SOURCE_FILES}
)
target_link_libraries(localCoords ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

