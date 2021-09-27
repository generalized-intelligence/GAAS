nvbio_module(nvbio-test)

addsources(
alignment_test.cu
alloc_test.cu
bloom_filter_test.cu
bwt_test.cpp
cache_test.cpp
condtion_test.cu
fasta_test.cpp
fastq_test.cpp
fmindex_test.cu
nvbio-test.cpp
packedstream_test.cpp
qgram_test.cu
rank_test.cu
string_set_test.cu
sum_tree_test.cpp
syncblocks_test.cu
utils.h
work_queue_test.cu
sequence_test.cu
wavelet_test.cu
)

cuda_add_executable(nvbio-test ${nvbio-test_srcs})
target_link_libraries(nvbio-test nvbio zlibstatic crcstatic lz4 ${SYSTEM_LINK_LIBRARIES})

