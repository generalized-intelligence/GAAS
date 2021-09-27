nvbio_module(nvBWT)

addsources(
nvBWT.cu
filelist.cpp
)

cuda_add_executable(nvBWT ${nvBWT_srcs})
target_link_libraries(nvBWT nvbio zlibstatic crcstatic moderngpu ${SYSTEM_LINK_LIBRARIES})

