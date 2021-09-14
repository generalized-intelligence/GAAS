nvbio_module(nvmem)

addsources(
nvmem.cu
pipeline.h
options.h
options.cpp
mem-search.h
mem-search.cu
build-chains.h
build-chains.cu
filter-chains.h
filter-chains.cu
align.h
align.cu
util.h
util.cu
)

cuda_add_executable(nvmem ${nvmem_srcs})
target_link_libraries(nvmem nvbio zlibstatic crcstatic lz4 ${SYSTEM_LINK_LIBRARIES})

