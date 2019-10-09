cmake_minimum_required( VERSION 2.8 )
project( vibe )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${PROJECT_SOURCE_DIR}/lib )

add_executable( vibe vibe.cpp optical.cpp estimation.cpp )
# 链接OpenCV库
target_link_libraries( vibe ${OpenCV_LIBS} )
