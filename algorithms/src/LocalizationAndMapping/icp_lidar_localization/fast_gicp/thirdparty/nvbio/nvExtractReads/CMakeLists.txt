nvbio_module(nvExtractReads)

addsources(
nvExtractReads.cu
)

cuda_add_executable(nvExtractReads ${nvExtractReads_srcs})
target_link_libraries(nvExtractReads nvbio zlibstatic lz4 crcstatic ${SYSTEM_LINK_LIBRARIES})

