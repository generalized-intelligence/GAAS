nvbio_module(fmmap)

addsources(
fmmap.cu
)

cuda_add_executable(fmmap ${fmmap_srcs})
target_link_libraries(fmmap nvbio zlibstatic lz4 crcstatic ${SYSTEM_LINK_LIBRARIES})
