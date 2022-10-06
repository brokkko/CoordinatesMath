cmake_minimum_required(VERSION 3.23)
project(mathtest)

set(CMAKE_CXX_STANDARD 20)

add_executable(mathtest main.cpp crsPose.h crsPose.cpp rotation.h rotation.cpp cvUtils.h cvUtils.cpp)
