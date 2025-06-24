set(EIGEN3_ROOT_DIR ${THIRDPARTY_DIR}/eigen3)
set(EIGEN3_INCLUDE_DIR "${EIGEN3_ROOT_DIR}/include/eigen3")

include_directories(SYSTEM ${EIGEN3_ROOT_DIR}/include/eigen3)
include_directories(SYSTEM ${EIGEN3_ROOT_DIR}/include/)
