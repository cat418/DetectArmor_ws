cmake_minimum_required(VERSION 3.17)
project(armor_sub)

#添加三个编译警告选项
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message("-- CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

set(CMAKE_PREFIX_PATH /opt/ros/humble/share/cv_bridge/cmake)  
set(CMAKE_PREFIX_PATH /usr/local/runtime/cmake)
find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED) 

ament_auto_find_build_dependencies()

# 此命令只需要包含.cpp文件会自动在相应目录结构下寻找头文件
ament_auto_add_library(${PROJECT_NAME} SHARED  
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenVINO_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  OpenCV
)
target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS}
    openvino::runtime)
    

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ArmorSubNode"
  EXECUTABLE ${PROJECT_NAME}_node
)


ament_auto_package(
INSTALL_TO_SHARE
)