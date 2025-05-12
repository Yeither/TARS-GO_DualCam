cmake_minimum_required(VERSION 3.0.2)
project(hk_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_communication
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 设置库文件目录
set(LIB_DIR /opt/MVS/lib/64)  # 包含所有的 .so 库文件

# 搜索所有 .so 文件
file(GLOB SO_LIBS "${LIB_DIR}/*.so*")

# 调试输出
message(STATUS "LIB_DIR: ${LIB_DIR}")
message(STATUS "Found shared libraries: ${SO_LIBS}")

# 设置编译选项
add_compile_options(-std=c++11)

# 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include  # 添加相机库的头文件目录
)

# 添加源代码文件并生成可执行文件
add_executable(my_hk_driver ./src/hk_driver.cpp)

# 链接 .so 文件和其他库
target_link_libraries(my_hk_driver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${SO_LIBS}  # 链接查找到的所有 .so 文件
)

# 设置运行时库路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 设置库文件搜索路径
link_directories(${LIB_DIR})

# 安装指令
install(TARGETS my_hk_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
