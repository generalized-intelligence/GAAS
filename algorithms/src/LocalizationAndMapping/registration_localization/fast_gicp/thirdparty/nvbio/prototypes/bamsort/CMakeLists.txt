# note: this assumes htslib is installed system-wide, need to fix
nvbio_module(bamsort)

addsources(
bam_io.cu
bam_io.h
bam_sort.cu
)

cuda_add_executable(bamsort ${bamsort_srcs})
target_link_libraries(bamsort htslib nvbio moderngpu zlibstatic crcstatic ${SYSTEM_LINK_LIBRARIES})

