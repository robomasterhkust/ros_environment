cmake_minimum_required(VERSION 3.8)
project(src)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        cameraDriver/mvux_camera/CameraApi.h
        cameraDriver/mvux_camera/CameraDefine.h
        cameraDriver/mvux_camera/CameraStatus.h
        cameraDriver/CamBase.cpp
        cameraDriver/CamBase.hpp
        cameraDriver/FlycapCam.cpp
        cameraDriver/FlycapCam.hpp
        cameraDriver/flycapNames.hpp
        cameraDriver/main.cpp
        cameraDriver/main.hpp
        cameraDriver/mvCamera.cpp
        cameraDriver/mvCamera.hpp
        cameraDriver/V4LCamDriver.cpp
        cameraDriver/V4LCamDriver.hpp
        cameraDriver/VideoIn.cpp
        cameraDriver/VideoIn.hpp
        detection/ArmorDetection.cpp
        detection/ArmorDetection.hpp
        detection/Camera.cpp
        detection/Camera.hpp
        detection/ConcurrentQueue.hpp
        detection/cvThreadPool.cpp
        detection/cvThreadPool.hpp
        detection/main.cpp
        detection/main.hpp
        detection/ROSCamIn.cpp
        detection/ROSCamIn.hpp
        detection/ROSInterface.cpp
        detection/ROSInterface.hpp
        detection/Settings.cpp
        detection/Settings.hpp
        detection/StopWatch.cpp
        detection/StopWatch.hpp
        tracking/tracking_node.cpp
        tracking/tracking_node.hpp
        tracking/armor_detect.cpp
        tracking/armor_detect.h
        tracking/detection_setting.cpp
        tracking/detection_setting.hpp
        defines.hpp
        helpers.hpp)

add_executable(armor_detection_node ${SOURCE_FILES})

target_link_libraries(armor_detection_node
        ${Boost_SYSTEM_LIBRARY}
        ${OpenCV_LIBS}
        )