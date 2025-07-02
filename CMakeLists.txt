cmake_minimum_required(VERSION 3.13)
project(slam_backend_example)

#****************************************************************#
###################################
##            cmake              ##
###################################
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE DEBUG)
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_UPPER)
if(BUILD_TYPE_UPPER STREQUAL "DEBUG")
    message("Debug mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -g -ggdb -fPIC")
    set(TYPE debug)
else()
    message("Release mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC")
    set(TYPE release)
    # add_definitions(-w)#无视警告(warning)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
endif(BUILD_TYPE_UPPER STREQUAL "DEBUG")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 禁用跳过RPATH
set(CMAKE_SKIP_RPATH FALSE)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
# 不使用安装RPATH进行构建
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
# 禁用RUNPATH
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--disable-new-dtags")
# 启用链接路径到RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 启用 ccache 加速编译
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
else()
    message(WARNING "ccache not found, compilation may be slower")
endif()

#****************************************************************#
###################################
##             build             ##
###################################
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/${TYPE})

aux_source_directory(${PROJECT_SOURCE_DIR}/src lib_sources)
add_library(${PROJECT_NAME}.lib SHARED ${lib_sources})
target_include_directories(${PROJECT_NAME}.lib PRIVATE ${PROJECT_SOURCE_DIR}/include)

include(${PROJECT_SOURCE_DIR}/cmake/Packages.cmake)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)

file(GLOB_RECURSE sources "${PROJECT_SOURCE_DIR}/examples/*.cpp")
foreach(src ${sources})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    target_include_directories(${name} PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${CERES_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIR}
        ${GFLAGS_INCLUDE_DIR}
    )
    target_link_libraries(${name} 
        ${PROJECT_NAME}.lib
        ${CERES_LIBRARIES}
        ${GTSAM_LIBRARIES}
        ${G2O_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${Sophus_LIBRARIES}
    )
endforeach()
