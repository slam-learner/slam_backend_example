list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

set(THIRDPARTY_DIR ${CMAKE_CURRENT_LIST_DIR}/../thirdparty)

include(${CMAKE_CURRENT_LIST_DIR}/Ceres.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/Eigen.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/Glog.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/Gflags.cmake)
