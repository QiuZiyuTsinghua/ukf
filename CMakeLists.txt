cmake_minimum_required(VERSION 3.14)
project(ukf VERSION 1.0)

# 指定 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 Eigen 库
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
  )
  FetchContent_MakeAvailable(eigen)
endif()

# 添加包含目录
include_directories(include)
if(Eigen3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
else()
  include_directories(${eigen_SOURCE_DIR})
endif()

# 创建UKF源文件列表，排除main.cpp
file(GLOB UKF_SOURCES "src/*.cpp")
list(FILTER UKF_SOURCES EXCLUDE REGEX ".*main\\.cpp$")

# 创建UKF库
add_library(ukf_lib ${UKF_SOURCES})

# 创建主可执行文件
add_executable(ukf src/main.cpp)
target_link_libraries(ukf ukf_lib)

# 添加测试
enable_testing()
add_subdirectory(tests)
