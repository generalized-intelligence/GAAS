nvbio_module("nvFM-server")

addsources(
nvFM-server.cpp
)

cuda_add_executable(nvFM-server ${nvFM-server_srcs})
target_link_libraries(nvFM-server nvbio crcstatic ${SYSTEM_LINK_LIBRARIES})

