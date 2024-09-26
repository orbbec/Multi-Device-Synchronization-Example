cmake_minimum_required(VERSION 3.10)
project(MultiCamearSync VERSION 1.0)

# 设置C++环境
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 判断平台
if(WIN32)
  SET(HOST_PLATFORM win_x64)
elseif(UNIX)
 SET(HOST_PLATFORM linux_x64)
endif()


# 添加include目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 查找OrbbecSDK
  find_package(OrbbecSDK QUIET)
  if(OrbbecSDK_FOUND)
    include_directories(${OrbbecSDK_INCLUDE_DIRS})
    link_directories(${OrbbecSDK_LIBRARY_DIRS})
  else()
    message( "OrbbecSDK not found")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/orbbecsdk)
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/${HOST_PLATFORM}/orbbecsdk)
    # 查找OrbbecSDK的库文件
    find_library(ORBTECSDK_LIBS NAMES OrbbecSDK PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/${HOST_PLATFORM}/orbbecsdk NO_DEFAULT_PATH)
endif()


# 查找OpenCV
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIBRARY_DIRS})
else()
  message( "OpenCV not found")
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/opencv)
  link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/${HOST_PLATFORM}/opencv)
  find_library(OpenCV_LIBS NAMES OpenCV PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/${HOST_PLATFORM}/opencv NO_DEFAULT_PATH)
endif()


#查找FFmpeg
find_package(FFmpeg QUIET)
if(FFmpeg_FOUND)
  include_directories(${FFmpeg_INCLUDE_DIRS})
  link_directories(${FFmpeg_LIBRARY_DIRS})
else()
  message( "FFmpeg not found")
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ffmpeg)
  link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/${HOST_PLATFORM}/ffmpeg)
  find_library(FFmpeg_LIBS NAMES FFmpeg PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/${HOST_PLATFORM}/ffmpeg NO_DEFAULT_PATH)
endif()


# 添加可执行文件
add_executable(${PROJECT_NAME} MultiDeviceSync.cpp cJSON.c)

# 链接库文件
target_link_libraries(${PROJECT_NAME} ${ORBTECSDK_LIBS}  ${OpenCV_LIBS})