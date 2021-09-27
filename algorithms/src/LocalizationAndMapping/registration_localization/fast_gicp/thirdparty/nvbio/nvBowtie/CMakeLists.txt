nvbio_module(nvBowtie)

addsources(
nvBowtie.cpp
)

nvbio_add_module_directory(bowtie2)
nvbio_add_module_directory(bowtie2/cuda)

cuda_add_executable(nvBowtie ${nvBowtie_srcs})
target_link_libraries(nvBowtie nvbio zlibstatic lz4 crcstatic ${SYSTEM_LINK_LIBRARIES})

