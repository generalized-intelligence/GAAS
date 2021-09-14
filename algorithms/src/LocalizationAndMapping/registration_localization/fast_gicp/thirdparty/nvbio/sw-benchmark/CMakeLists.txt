nvbio_module(sw-benchmark)

option(SSWLIB
  "Enable SSW"
  OFF)

addsources(
sw-benchmark.cu
)

if (SSWLIB)
add_definitions(-DSSWLIB)
addsources(
ssw.c
ssw.h
)
endif()

cuda_add_executable(sw-benchmark ${sw-benchmark_srcs})
target_link_libraries(sw-benchmark nvbio zlibstatic lz4 crcstatic ${SYSTEM_LINK_LIBRARIES})

