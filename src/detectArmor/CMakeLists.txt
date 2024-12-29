cmake_minimum_required(VERSION 3.17)
project(armor_pub)

add_subdirectory(HikDriver)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#检查是否设置了构建类型 如果没有设置，默认值将设置为Release
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
#打印当前的构建类型
message("-- CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

#设置ROS2 cv_bridge库的查找路径
set(CMAKE_PREFIX_PATH /opt/ros/humble/share/cv_bridge/cmake)  
set(CMAKE_PREFIX_PATH /usr/local/runtime/cmake)
find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED) 

ament_auto_find_build_dependencies()

# 此命令只需要包含.cpp文件会自动在相应目录结构下寻找头文件
ament_auto_add_library(${PROJECT_NAME} SHARED  
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src/detectArmor/HikDriver/hikData/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src/detectArmor/HikDriver/include/HikDriver)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src/detectArmor/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenVINO_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  OpenCV
)
target_link_libraries(${PROJECT_NAME} 
    hik_driver
    ${OpenCV_LIBS}
    openvino::runtime)
    
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ArmorPubNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
INSTALL_TO_SHARE
)