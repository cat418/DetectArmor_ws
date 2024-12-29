find_package(Threads)
find_package(fmt REQUIRED)#c++格式化库
find_package(glog REQUIRED)#谷歌提供的日志库
find_package(OpenCV REQUIRED)

#指定这是一个静态库
add_library(hik_driver STATIC include/HikDriver/HikDriver.h src/HikDriver.cpp)
set_target_properties(hik_driver PROPERTIES
        LINKER_LANGUAGE CXX
        )#确保正确的c++编译器和链接器被使用
target_include_directories(hik_driver PUBLIC include)
target_include_directories(hik_driver PUBLIC /opt/MVS/include)

target_link_libraries(hik_driver PUBLIC
    Threads::Threads#查找并连接线程库
    glog::glog#Google提供的日志库
    fmt::fmt #c++格式化库
    ${OpenCV_LIBS}
    X11#连接x11库 这是图形界面相关的库
    MvCameraControl)

# 添加 -fPIC 编译选项 指示编译生成可以在任何内存地址执行的代码 确保hik_driver可以作为共享库进行正确的构建和连接
set_target_properties(hik_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)
