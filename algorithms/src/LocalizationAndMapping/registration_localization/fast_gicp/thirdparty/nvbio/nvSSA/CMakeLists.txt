nvbio_module(nvSSA)

addsources(
nvSSA.cpp
)

cuda_add_executable(nvSSA ${nvSSA_srcs})
target_link_libraries(nvSSA nvbio crcstatic ${SYSTEM_LINK_LIBRARIES})

