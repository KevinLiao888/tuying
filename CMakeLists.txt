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
include_directories(${aris_INCLUDE_DIRS})

message(${aris_DIR})

# user program started...
set(SOURCE_FILES src/main.cpp src/kaanh.cpp src/kaanh.h)
add_executable(tuying ${SOURCE_FILES})


# add_library(kaanh_lib STATIC src/kaanh.cpp src/kaanh.h)
# add_executable(kaanh src/main.cpp)
# target_link_libraries(kaanh kaanh_lib)

if(UNIX)
	target_link_libraries(tuying ${aris_LIBRARIES} atomic)
    target_link_libraries(tuying ${dynamixel_LIBRARIES} /usr/local/lib/libdxl_x64_cpp.so)
endif(UNIX)
IF(WIN32)
	target_link_libraries(tuying ${aris_LIBRARIES})
	target_link_libraries(tuying ${dynamixel_LIBRARIES} C:/Users/kevin/Desktop/Dynamixel/Dynamixel/c++/build/win32/output/dxl_x86_cpp.dll)
endif(WIN32)
