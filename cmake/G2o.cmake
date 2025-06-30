set(INSTALL_DIR ${THIRDPARTY_DIR}/../install)

set(G2O_INCLUDE_DIRS ${INSTALL_DIR}/include)
set(G2O_LIBRARY_DIRS ${INSTALL_DIR}/lib)
include_directories(SYSTEM ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
file(GLOB G2O_LIBRARIES ${G2O_LIBRARY_DIRS}/libg2o*.so)
file(GLOB G2O_LIBS ${G2O_LIBRARY_DIRS}/libg2o*.so)
