set(Sophus_ROOT ${THIRDPARTY_DIR}/sophus)

set(Sophus_INCLUDE_DIRS ${Sophus_ROOT}/include)
set(Sophus_LIBRARY_DIRS ${Sophus_ROOT}/lib)
include_directories(SYSTEM ${Sophus_INCLUDE_DIRS})
link_directories(${Sophus_LIBRARY_DIRS})
file(GLOB Sophus_LIBRARIES ${Sophus_LIBRARY_DIRS}/lib*.so)
file(GLOB Sophus_LIBS ${Sophus_LIBRARY_DIRS}/lib*.so)
