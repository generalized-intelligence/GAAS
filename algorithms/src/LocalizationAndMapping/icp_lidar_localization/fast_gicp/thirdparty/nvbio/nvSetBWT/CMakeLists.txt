nvbio_module(nvSetBWT)

addsources(
input_thread.h
nvSetBWT.cu
)

cuda_add_executable(nvSetBWT ${nvSetBWT_srcs})
target_link_libraries(nvSetBWT nvbio moderngpu zlibstatic lz4 crcstatic ${SYSTEM_LINK_LIBRARIES})

