nvbio_module(sufsort-test)

addsources(
sufsort_test.cu
)

cuda_add_executable(sufsort-test ${sufsort-test_srcs})
target_link_libraries(sufsort-test nvbio moderngpu zlibstatic lz4 crcstatic ${SYSTEM_LINK_LIBRARIES})

