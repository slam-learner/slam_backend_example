list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

set(THIRDPARTY_DIR ${CMAKE_CURRENT_LIST_DIR}/../thirdparty)

include(${CMAKE_CURRENT_LIST_DIR}/Eigen.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/G2o.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/Glog.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/Gflags.cmake)

set(INSTALL_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR}/../install/lib/cmake)
set(CMAKE_PREFIX_PATH "${THIRDPARTY_DIR}/eigen3/share/eigen3/cmake;${THIRDPARTY_DIR}/gflags/lib/cmake/gflags;${THIRDPARTY_DIR}/glog/lib/cmake/glog;${INSTALL_CMAKE_DIR}/Ceres;${INSTALL_CMAKE_DIR}/GTSAM" ${CMAKE_PREFIX_PATH})
