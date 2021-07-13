nvbio_module(qmap)

addsources(
qmap.cu
)

cuda_add_executable(qmap ${qmap_srcs})
target_link_libraries(qmap nvbio zlibstatic lz4 crcstatic ${SYSTEM_LINK_LIBRARIES})
