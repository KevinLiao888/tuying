cmake_minimum_required(VERSION 3.12)

project(tuying)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
	find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
	find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)

if(UNIX)
        include_directories(${aris_INCLUDE_DIRS} /usr/local/include/dynamixel_sdk)
endif(UNIX)
IF(WIN32)
        include_directories(${aris_INCLUDE_DIRS} C:/Users/kevin/Desktop/Dynamixel/Dynamixel/c++/include/dynamixel_sdk)
endif(WIN32)

message(${aris_DIR})

# link_directories(/usr/local/lib)
link_directories(/home/kaanh/Desktop/Dynamixel/Dynamixel/c++/build/linux64)
# user program started...
set(SOURCE_FILES src/main.cpp src/kaanh.cpp src/kaanh.h)
add_executable(tuying ${SOURCE_FILES})


# add_library(kaanh_lib STATIC src/kaanh.cpp src/kaanh.h)
# add_executable(kaanh src/main.cpp)
# target_link_libraries(kaanh kaanh_lib)

if(UNIX)
        target_link_libraries(tuying ${aris_LIBRARIES} atomic libdxl_x64_cpp.so)
endif(UNIX)
IF(WIN32)
	target_link_libraries(tuying ${aris_LIBRARIES} C:/Users/kevin/Desktop/Dynamixel/Dynamixel/c++/build/win32/Debug/dxl_x86_cpp.lib)
endif(WIN32)
