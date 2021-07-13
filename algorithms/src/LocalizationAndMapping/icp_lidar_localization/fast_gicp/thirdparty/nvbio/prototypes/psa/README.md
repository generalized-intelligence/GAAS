PSA: Pairwise sequence alignment benchmarking
=============================================

This is a project to benchmark Smith & Waterman implementations designed for GPU processors.
The project includes samples for input data, dataset generators, basic sequence conversion primitives and Smith & Waterman implementations. Most of them review the literature approaches or extend the NVBIO implementations.

###SW-G Prototypes

In order to give a general overview, the following implementations are included and benchmarked in the project:

   1. Register tilling using 32bit integer instructions (GPU baseline).
   2. Using video instructions (processing 4 elements per register).
   3. Combining video instructions and integer instructions (processing 4 cells per register).
   4. Using integer instructions (processing​ 4 cells per register). It uses a custom set of primitives to simulate MAX video instructions using integer instructions.
   
   * Also includes: several warp-wide GPU implementations based on CPU SIMD approaches (farrar [1] and wozniak [2]), baseline CPU implementations and experimental codes to tune the cell resolution of the DP matrix or the DP tilling process strategy.
   * Benchmarks adapted to simulate the SW-G behaviour inside a read mapper.


```
[1] Striped Smith–Waterman speeds database searches six times over other SIMD implementations. 
[2] Using video-oriented instructions to speed up sequence comparison.
```


###Performance Results

Smith & Waterman - Gotoh (Using 2bits per base) performance in GCUPS for Maxwell and Kepler platforms:

 ​​​​ Performance (GCUPS)                 |  GTX TITAN BLACK  (kepler)| GTX TITAN X (maxwell) |
 -------------------------------------|---------------------------|-----------------------|
 1) Register tilling (32b)            |         100.31            |         129.03        | 
 2) Video instructions (8b)           |         111.77            |          17.85        |
 3) Mixing video & integer inst. (8b) |         157.73            |          46.38        |
 4) Integer instructions (8b)         |          49.98            |          66.95        |
 
 
The benchmarks can also run the SW-G implementations reading the candidates from a reference instead of reading them from a packed list of candidates. The objective is to simulate a real read alignment scenario from a mapper, analyzing the performance penalty of the random-gather accesses to de reference.
 
 ​​​​ Performance (GCUPS)       |  GTX TITAN BLACK  (kepler)| GTX TITAN X (maxwell) |
 ---------------------------|---------------------------|-----------------------|
  1) Regiser tilling (32b)  |         75.86             |         128.06        | 


###Detailed executables

#####Tools
* `gregions`: Input-data generator for the benchmarks.
* `checksum`: Analyzes the divergence results and depicts the overflow result scores.

#####Benchmarks
* `benchmark_swg_2b_integer_gpu`: Baseline GPU implementation, using tilling strategies and different resolution cell scores.
* `benchmark_swg_2b_video_gpu`: GPU packs 4 cells per register and uses video instructions to process the DP cells of 8 bits.
* `benchmark_swg_2b_mixed_gpu`: GPU packs 4 cells per register and uses a mixing with video instructions and integer instructions to process the DP cells of 8 bits.
* `benchmark_swg_2b_mixedsim_gpu`: GPU packs 4 cells per register just using integer/logical instructions to process the DP cells of 8 bits.

* `benchmark_swg_ref_2b_integer_gpu`: Same baseline GPU version but reading the candidates from the reference to simulate the performance in a short mapper.

#####Miscellaneous benchmarks
* `benchmark_swg_cpu`: Baseline CPU to validate the functionality.
* `benchmark_swg_farrar_gpu`: Farrar striped implementation using warp-wide strategy GPU (needs kepler shuffle instructions).
* `benchmark_swg_wozniak_gpu`: Wozniak anti-diagonal implementation using warp-wide strategy GPU (needs kepler shuffle instructions).


###Input/Output data

This section explains the necessary data to run the SW-G benchmarks. The project includes some small samples of input data to run the code with different genomes files and sets of candidate-query. It is described how to create the datasets at the end of this document. 

####Input Data

To perform the DP alignment benchmark, an input data is necessary with the pair of query-candidate. There are two input formats: (1) using an explicit storage of the candidates that pack (in raw or ASCII mode) all the query-candidate pair or (2) using an implicit storage saving the candidate position in the reference.

The files with the next filename syntax extensions:

1. ````*.regions````: uses an explicit storage of candidates.
2. ````*.ref.regions````: uses an implicit storage of candidates.

Example:
```1K-100nt.prof.1000.100.21.regions``` stores 1000 queries of 100nt size x 21 candidates per query = 21K alignments

* These input-files are generated from *.profile files with the executable ```gregions```.

####Output data

The results are represented by a list of positions in the DP (sink) and the score of the best alignment. Each line of the output file represents one alignment.
Sample for the first output lines:

~~~~~~
==> data/1K-100nt.prof.1000.100.21.regions.sw-gotoh.32b.cpu <==
21235
0 2 119
1 34 117
2 16 116
3 25 114
4 31 79
5 35 109
6 32 49
7 39 40
8 44 118
~~~~~~

* Each line contains: alignment id, best score, column sink position.

#####Checking the outputs

The ```checksum ``` program compares the output of 2 result files of alignments and returns the differences between them and the overflows produced by the reduced resolutions of the DP cells (8 bits, 16 bits and 32 bits) 

~~~~
$bin/checksum data/1K-100nt.prof.1000.100.21.regions.nvbio-1th.32b.tc.gpu data/1K-100nt.prof.1000.100.21.ref.regions.gotoh.video.ref.8b.gpu 
[INFO] Loading results: data/1K-100nt.prof.1000.100.21.regions.nvbio-1th.32b.tc.gpu ...
[INFO] Loading results: data/1K-100nt.prof.1000.100.21.ref.regions.gotoh.video.ref.8b.gpu ...
[INFO] Checking results ...
Number of candidates benchmarked: 		 [ 21231 ]
Total 'False positives' REPORTED:		0	(0.00 %)
	 => Groups REPORTED: 				0	(0.00 %)
[INFO] Comparing results ...
	 => Total with maxScoring: 			0 	(0.00 %)
	 => Total under maxScoring: 		0 	(0.00 %)
TOTAL DIFFERENT RESULTS: 				0 	(0.00 %)
[INFO] Intersecting results ...
Intersection results:
	 => Total detected alignments: 		0 	(0.00 %)
	 => Total undetected alignments: 	0	(0.00 %)
	 => Total overhead alignments:		0 	(0.00 %)
~~~~

###Compiling process

Use ```make``` to compile each of the binaries itemized above, the executable will be generated in the bin folder.

#####Examples

Compile the input dataset generator:

~~~~~
make gregions
~~~~~

To compile the benchmarks it is necessary to indicate the size of the query and candidate in compiler time as parameters of the make command:

~~~~~
make benchmark_swg_cpu squery=100 scandidate=120
~~~~~


###Executing process

There are 2 types of benchmarks, (A) using just the 
(B) using 

#####Examples

A. Using the explicit packed query-candidate input:

~~~~~~
bin/benchmark_swg_cpu data/1K-100nt.prof.1000.100.21.regions
~~~~~~

B. Using the implicit query-reference position input, that requires the original reference:

~~~~~~
bin/bench_swg_ref_2b_integer_gpu_100 data/1K-100nt.prof.1000.100.21.ref.regions data/profiles/hsapiens_v37.fa 
~~~~~~

* In the benchmarking process it is recommendable:
	1. The use of nvprof to measure the kernel timings, with the goal of reducing the host synchronization overheads.
	2. Use input data larger than 120K alignments for a full GPU utilization.

#####Output example

~~~~~~
$bin/benchmark_swg_cpu data/1K-100nt.prof.1000.100.21.regions
SMITH & WATERMAN GOTOH 
CPU VERSION WHOLE MATRIX 
CONFIG - MATCH: 2, MISMATCH: -5, OPEN_INDEL: -2, EXTEND_INDEL: -1 
Build: Aug  5 2015 - 16:19:15 
FORMAT: 9 
FORMAT: 9 
TIME: 	 1.327312 	 GCUPS: 	 0.191982 
AVERAGE_QUERY_SIZE: 	 100, 	 AVERAGE_CANDIDATES_PER_QUERY: 	 21, 	 NUM_CANDIDATES: 	21235 
~~~~~~

The results file have the file name: ```*.gpu```, an example of this file is: _1K-100nt.prof.1000.100.21.ref.regions.gotoh.video.ref.8b.gpu_

###How to generate different input datasets

The program ```gregions``` generates the input data from the ```*.prof``` files.
The *.profile files can be generated by the ```GEM short read mapper``` from any input data, using the profile options. 

Several samples of *.profile files are included in the data folder of this project. Any of them can be used directly or replicated several times to increase artificially the size of the input data.

Syntax:

~~~~~~~
bin/gregions format_output ratio_size_candidates alignment_profile fasta_reference_genome
~~~~~~~

#####Examples

```gregions``` can be used to generate two types of input data:

A. To generate the explicit packed query-candidate input format:

~~~~~~
bin/gregions 0 1.2 data/1K-100nt.prof data/profiles/hsapiens_v37.fa
~~~~~~

B. To generate the implicit query-reference position input format, that later requires the original reference for the benchmark:

~~~~~~
bin/gregions 1 1.2 data/1K-100nt.prof data/profiles/hsapiens_v37.fa
~~~~~~

* This parameters will generate an implicit input-data with the positions of the candidates in the reference. Each of the queries have a size of 100nt and the candidates have a size of 120nt.