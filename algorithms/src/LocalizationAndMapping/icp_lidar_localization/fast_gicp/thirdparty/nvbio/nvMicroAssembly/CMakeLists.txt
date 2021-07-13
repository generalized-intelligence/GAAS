nvbio_module("nvMicroAssembly")

addsources(
haplotype_caller.cu
haplotype_caller.h
assembly.h
assembly_graph_inl.h
assembly_graph.h
kmers_inl.h
kmers.h
regions.h
bam_io.h
bam_io.cu
)

include_directories(${PROJECT_SOURCE_DIR}/contrib/htslib ${PROJECT_SOURCE_DIR}/contrib/zlib)

cuda_add_executable(nvMicroAssembly ${nvMicroAssembly_srcs})
target_link_libraries(nvMicroAssembly htslib nvbio zlibstatic lz4 crcstatic ${SYSTEM_LINK_LIBRARIES})
