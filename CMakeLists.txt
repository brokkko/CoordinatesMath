cmake_minimum_required(VERSION 3.23)
project(mathtest)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
include_directories(EIGEN_INCLUDE_DIR)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(PROJ CONFIG REQUIRED)
find_library(AVCODEC_LIBRARY avcodec)
find_library(AVFORMAT_LIBRARY avformat)
find_library(AVUTIL_LIBRARY avutil)
find_library(AVDEVICE_LIBRARY avdevice)
find_library(SWS_LIBRARY swscale)



add_executable(mathtest main.cpp crsPose.h crsPose.cpp rotation.h rotation.cpp cvUtils.h cvUtils.cpp)
target_link_libraries(mathtest PRIVATE ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${AVDEVICE_LIBRARY} ${SWS_LIBRARY} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} PROJ::proj)