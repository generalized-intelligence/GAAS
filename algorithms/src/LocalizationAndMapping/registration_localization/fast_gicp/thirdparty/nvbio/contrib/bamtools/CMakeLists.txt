project(bamtools)

cmake_minimum_required(VERSION 2.8)

find_package(CUDA)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${NVBIO_SOURCE_ROOT}/nvbio)

set(BAMTOOLS_SRCS
    BamAux.h
    BamReader.cpp
    BamReader.h
    BamWriter.cpp
    BamWriter.h
    BGZF.cpp
    BGZF.h
)

cuda_add_library(bamtools STATIC ${BAMTOOLS_SRCS})

