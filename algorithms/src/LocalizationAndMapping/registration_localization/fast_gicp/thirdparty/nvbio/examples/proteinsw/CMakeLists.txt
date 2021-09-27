nvbio_module(proteinsw)

addsources(
proteinsw.cu
)

cuda_add_executable(proteinsw ${proteinsw_srcs})
target_link_libraries(proteinsw nvbio zlibstatic lz4 crcstatic ${SYSTEM_LINK_LIBRARIES})
