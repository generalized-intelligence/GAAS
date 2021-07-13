nvbio_module(nvbio)

# note: the order here matters as it determines link order
nvbio_add_module_directory(io)
nvbio_add_module_directory(io/fmindex)
nvbio_add_module_directory(io/sequence)
nvbio_add_module_directory(io/reads)
nvbio_add_module_directory(io/output)
nvbio_add_module_directory(basic)
nvbio_add_module_directory(basic/cuda)
nvbio_add_module_directory(fasta)
nvbio_add_module_directory(fmindex)
nvbio_add_module_directory(strings)
nvbio_add_module_directory(sufsort)
nvbio_add_module_directory(trie)

cuda_add_library(nvbio STATIC ${nvbio_srcs})

