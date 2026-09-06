# cmake needs this line
cmake_minimum_required(VERSION 3.10)
## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Define project name
project(calibration)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(Threads REQUIRED)
# 源文件
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_LIST)

# 公共库：避免在多个可执行文件中重复编译 src 下源码
add_library(calibration_core STATIC ${SRC_LIST})
target_include_directories(calibration_core
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include
)

add_executable(calibrateCamera calibrateCamera.cpp)
add_executable(calculateError calculateError.cpp)
add_executable(calibrateHandEye calibrateHandEye.cpp)
add_executable(validateHandEye validateHandEye.cpp)


# 链接公共库
target_link_libraries(calibrateCamera PRIVATE calibration_core)
target_link_libraries(calculateError PRIVATE calibration_core)
target_link_libraries(calibrateHandEye PRIVATE calibration_core)
target_link_libraries(validateHandEye PRIVATE calibration_core)

# 海康SDK
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(HIK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(HIK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_directories(calibration_core PUBLIC ${HIK_LIB_DIR})

# 检查是否需要链接filesystem库
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
    # GCC < 9.0 不需要显式链接
else()
    # GCC >= 9.0 和 Clang 需要链接 filesystem
    find_library(FILESYSTEM_LIB stdc++fs)
    if(FILESYSTEM_LIB)
        set(FILESYSTEM_LIBS ${FILESYSTEM_LIB})
    endif()
endif()

target_link_libraries(calibration_core
  PUBLIC
    ${OpenCV_LIBS}
    yaml-cpp
    Eigen3::Eigen
    MvCameraControl
    fmt::fmt
    Threads::Threads
    ${FILESYSTEM_LIBS}
)