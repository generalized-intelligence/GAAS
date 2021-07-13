nvbio_module(nvLighter)

addsources(
error_correct.cu
error_correct.h
input_thread.cu
input_thread.h
output_thread.h
output_thread.cu
nvLighter.cu
sample_kmers.cu
sample_kmers.h
)

cuda_add_executable(nvLighter ${nvLighter_srcs})
target_link_libraries(nvLighter nvbio moderngpu zlibstatic lz4 crcstatic ${SYSTEM_LINK_LIBRARIES})

