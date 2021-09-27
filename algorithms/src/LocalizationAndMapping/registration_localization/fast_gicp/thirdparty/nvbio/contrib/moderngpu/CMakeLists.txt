project(moderngpu)
cmake_minimum_required(VERSION 2.8)

find_package(CUDA)
include_directories(${CUDA_INCLUDE_DIRS})

set(INSTALL_BIN_DIR "${CMAKE_INSTALL_PREFIX}/bin" CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")
set(INSTALL_MAN_DIR "${CMAKE_INSTALL_PREFIX}/share/man" CACHE PATH "Installation directory for manual pages")
set(INSTALL_PKGCONFIG_DIR "${CMAKE_INSTALL_PREFIX}/share/pkgconfig" CACHE PATH "Installation directory for pkgconfig (.pc) files")

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${NVBIO_SOURCE_ROOT}/include)

set(SRCS
    src/mgpucontext.cu
    src/mgpuutil.cpp
    src/mmio.cpp
    src/sparsematrix.cpp
)

cuda_add_library(moderngpu STATIC ${SRCS})

