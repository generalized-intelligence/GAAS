nvbio_module(nvbio-aln-diff)

addsources(
alignment_bam.cpp
alignment.cpp
alignment_dbg.cpp
alignment.h
filter.h
html.h
se_analyzer.h
se_analyzer.cpp
pe_analyzer.h
pe_analyzer.cpp
stats.h
utils.h
nvbio-aln-diff.cpp
)

cuda_add_executable(nvbio-aln-diff ${nvbio-aln-diff_srcs})
target_link_libraries(nvbio-aln-diff bamtools nvbio zlibstatic crcstatic ${SYSTEM_LINK_LIBRARIES})

