# COMPILATION EXAMPLES
# --------------------
# make gregions
# make checksum
# make swg_ref_2b_integer_gpu squery=100 scandidate=120
#
# EXECUTION EXAMPLES:
# ---------------------
# bin/benchmark_swg_cpu data/1K-100nt.prof.1000.100.21.regions
# bin/bench_swg_ref_2b_integer_gpu_100 data/1K-100nt.prof.1000.100.21.ref.regions data/profiles/hsapiens_v37.fa 

squery?=100
scandidate?=120

# Shell interpreter #
#####################
SHELL := /bin/bash

# Compilers  #
##############
NVCC=nvcc
CC=gcc

# CPU flags  #
##############
CFLAGS=-O3 -m64
LFLAGS=

# CUDA flags #
##############
CUDA_FLAGS=-O3 -m64 -gencode arch=compute_35,code=sm_35
CUDA_LIBRARY_FLAGS=-L/usr/local/cuda/lib64 -I/usr/local/cuda/include -lcuda -lcudart

# DEBUG FLAGS #
###############
GCC_GDB_FLAGS=-g -O0 -m64
NVCC_GDB_FLAGS=-g -G -O0 -m64 -gencode arch=compute_35,code=sm_35

all: gregions checksum \
benchmark_swg_cpu benchmark_swg_farrar_gpu benchmark_swg_wozniak_gpu benchmark_swg_2b_integer_gpu \
benchmark_swg_2b_video_gpu benchmark_swg_2b_mixed_gpu benchmark_swg_2b_mixedsim_gpu \
benchmark_swg_ref_2b_integer_gpu swg_ref_2b_host-dbg swg_ref_2b_integer_gpu-dbg \
benchmark_swg_ref_2b_integer_gpu-dbg

# MODULES:
alignments:
	$(CC) $(CFLAGS) -c src/psa_alignments.c -o build/psa_alignments.o
errors:
	$(CC) $(CFLAGS) -c src/psa_errors.c -o build/psa_errors.o
profile:
	$(CC) $(CFLAGS) -c src/psa_profile.c -o build/psa_profile.o
regions:
	$(CC) $(CFLAGS) -c src/psa_regions.c -o build/psa_regions.o
sequences:
	$(CC) $(CFLAGS) -c src/psa_sequences.c -o build/psa_sequences.o
time:
	$(CC) $(CFLAGS) -c src/psa_time.c -o build/psa_time.o -lrt

# BUILDERS:
gregions: alignments errors profile regions sequences
	$(CC) $(CFLAGS) build/psa_alignments.o build/psa_errors.o build/psa_profile.o build/psa_regions.o build/psa_sequences.o tools/genRegions.c -o bin/gregions

# CHECKERS:
checksum: 
	$(CC) $(CFLAGS) tools/checksum.c -o bin/checksum


# SWG VERSIONS:
swg_cpu:
	$(CC) $(CFLAGS) -c src/gotoh/psa_swgotoh_cpu.c -o build/psa_swg_cpu.o
swg_2b_host:
	$(CC) $(CFLAGS) -DCUDA -c src/gotoh/psa_swgotoh_2b_gpu.c -o build/psa_swg_2b_host.o $(CUDA_LIBRARY_FLAGS)
swg_ref_2b_host:
	$(CC) $(CFLAGS) -DCUDA -c src/gotoh/psa_swgotoh_ref_2b_gpu.c -o build/psa_swg_ref_2b_host.o $(CUDA_LIBRARY_FLAGS)
swfarrar_gpu:
	$(CC) $(CFLAGS) -DCUDA -c src/gotoh/misc/psa_swfarrar_gpu.c -o build/psa_swfarrar_host.o $(CUDA_LIBRARY_FLAGS)
	$(NVCC) $(CUDA_FLAGS) -DCUDA -DQUERIES_SIZE=$(squery) -DCANDIDATES_SIZE=$(scandidate) -c src/gotoh/misc/psa_swfarrar_gpu.cu -o build/psa_swfarrar_device.o
swwozniak_gpu:
	$(CC) $(CFLAGS) -DCUDA -c src/gotoh/misc/psa_swwozniak_gpu.c -o build/psa_swwozniak_host.o $(CUDA_LIBRARY_FLAGS)
	$(NVCC) $(CUDA_FLAGS) -DCUDA -DQUERIES_SIZE=$(squery) -DCANDIDATES_SIZE=$(scandidate) -c src/gotoh/misc/psa_swwozniak_gpu.cu -o build/psa_swwozniak_device.o
swg_2b_integer_gpu:
	$(NVCC) $(CUDA_FLAGS) -Xptxas -dlcm=ca -DCUDA -DQUERIES_SIZE=$(squery) -DCANDIDATES_SIZE=$(scandidate) -c src/gotoh/psa_swgotoh_2b_integer_gpu.cu -o build/psa_swg_2b_integer_device.o
swg_2b_video_gpu:
	$(NVCC) $(CUDA_FLAGS) -Xptxas -dlcm=ca -DCUDA -DQUERIES_SIZE=$(squery) -DCANDIDATES_SIZE=$(scandidate) -c src/gotoh/psa_swgotoh_2b_video_gpu.cu -o build/psa_swg_2b_video_device.o
swg_2b_mixed_gpu:
	$(NVCC) $(CUDA_FLAGS) -Xptxas -dlcm=ca -DCUDA -DQUERIES_SIZE=$(squery) -DCANDIDATES_SIZE=$(scandidate) -c src/gotoh/psa_swgotoh_2b_mixed_gpu.cu -o build/psa_swg_2b_mixed_device.o
swg_2b_mixedsim_gpu:
	$(NVCC) $(CUDA_FLAGS) -Xptxas -dlcm=ca -DCUDA -DQUERIES_SIZE=$(squery) -DCANDIDATES_SIZE=$(scandidate) -c src/gotoh/psa_swgotoh_2b_mixedsim_gpu.cu -o build/psa_swg_2b_mixedsim_device.o
swg_ref_2b_integer_gpu:
	$(NVCC) $(CUDA_FLAGS) -Xptxas -dlcm=ca -DCUDA -DQUERIES_SIZE=$(squery) -DCANDIDATES_SIZE=$(scandidate) -DREFERENCE -c src/gotoh/psa_swgotoh_ref_2b_integer_gpu.cu -o build/psa_swg_ref_2b_integer_device.o



## BENCHMARKS VERSIONS:
benchmark_swg_cpu: alignments errors profile regions sequences swg_cpu time
	$(CC) $(CFLAGS) build/psa_alignments.o build/psa_errors.o build/psa_profile.o build/psa_regions.o build/psa_sequences.o build/psa_swg_cpu.o build/psa_time.o tools/benchmark.c -o bin/benchmark_swg_cpu -lrt
benchmark_swg_farrar_gpu: alignments errors profile regions sequences swfarrar_gpu time
	$(CC) $(CFLAGS) -DCUDA build/psa_alignments.o build/psa_errors.o build/psa_profile.o build/psa_regions.o build/psa_sequences.o build/psa_swfarrar_host.o build/psa_swfarrar_device.o build/psa_time.o tools/benchmark.c -o bin/benchmark_swfarrar_gpu_$(squery) $(CUDA_LIBRARY_FLAGS)
benchmark_swg_wozniak_gpu: alignments errors profile regions sequences swwozniak_gpu time
	$(CC) $(CFLAGS) -DCUDA build/psa_alignments.o build/psa_errors.o build/psa_profile.o build/psa_regions.o build/psa_sequences.o build/psa_swwozniak_host.o build/psa_swwozniak_device.o build/psa_time.o tools/benchmark.c -o bin/benchmark_swwozniak_gpu_$(squery) $(CUDA_LIBRARY_FLAGS)
benchmark_swg_2b_integer_gpu: alignments errors profile regions sequences swg_2b_host swg_2b_integer_gpu time
	$(CC) $(CFLAGS) -DCUDA build/psa_alignments.o build/psa_errors.o build/psa_profile.o build/psa_regions.o build/psa_sequences.o build/psa_swg_2b_host.o build/psa_swg_2b_integer_device.o build/psa_time.o tools/benchmark.c -o bin/bench_swg_2b_integer_gpu_$(squery) $(CUDA_LIBRARY_FLAGS)
benchmark_swg_2b_video_gpu: alignments errors profile regions sequences swg_2b_host swg_2b_video_gpu time
	$(CC) $(CFLAGS) -DCUDA build/psa_alignments.o build/psa_errors.o build/psa_profile.o build/psa_regions.o build/psa_sequences.o build/psa_swg_2b_host.o build/psa_swg_2b_video_device.o build/psa_time.o tools/benchmark.c -o bin/bench_swg_2b_video_gpu_$(squery) $(CUDA_LIBRARY_FLAGS)
benchmark_swg_2b_mixed_gpu: alignments errors profile regions sequences swg_2b_host swg_2b_mixed_gpu time
	$(CC) $(CFLAGS) -DCUDA build/psa_alignments.o build/psa_errors.o build/psa_profile.o build/psa_regions.o build/psa_sequences.o build/psa_swg_2b_host.o build/psa_swg_2b_mixed_device.o build/psa_time.o tools/benchmark.c -o bin/bench_swg_2b_mixed_gpu_$(squery) $(CUDA_LIBRARY_FLAGS)
benchmark_swg_2b_mixedsim_gpu: alignments errors profile regions sequences swg_2b_host swg_2b_mixedsim_gpu time
	$(CC) $(CFLAGS) -DCUDA build/psa_alignments.o build/psa_errors.o build/psa_profile.o build/psa_regions.o build/psa_sequences.o build/psa_swg_2b_host.o build/psa_swg_2b_mixedsim_device.o build/psa_time.o tools/benchmark.c -o bin/bench_swg_2b_mixedsim_gpu_$(squery) $(CUDA_LIBRARY_FLAGS)
benchmark_swg_ref_2b_integer_gpu: alignments errors profile regions sequences swg_ref_2b_host swg_ref_2b_integer_gpu time
	$(CC) $(CFLAGS) -DCUDA -DREFERENCE build/psa_alignments.o build/psa_errors.o build/psa_profile.o build/psa_regions.o build/psa_sequences.o build/psa_swg_ref_2b_host.o build/psa_swg_ref_2b_integer_device.o build/psa_time.o tools/benchmark.c -o bin/bench_swg_ref_2b_integer_gpu_$(squery) $(CUDA_LIBRARY_FLAGS)



#  DEBUG             #
######################

# DEBUG SWG VERSIONS:
alignments-dbg:
	$(CC) $(GCC_GDB_FLAGS) -c src/psa_alignments.c -o build/psa_alignments-dbg.o
errors-dbg:
	$(CC) $(GCC_GDB_FLAGS) -c src/psa_errors.c -o build/psa_errors-dbg.o
profile-dbg:
	$(CC) $(GCC_GDB_FLAGS) -c src/psa_profile.c -o build/psa_profile-dbg.o
regions-dbg:
	$(CC) $(GCC_GDB_FLAGS) -c src/psa_regions.c -o build/psa_regions-dbg.o
sequences-dbg:
	$(CC) $(GCC_GDB_FLAGS) -c src/psa_sequences.c -o build/psa_sequences-dbg.o
time-dbg:
	$(CC) $(GCC_GDB_FLAGS) -c src/psa_time.c -o build/psa_time-dbg.o -lrt

## DEBUG BENCHMARKS VERSIONS:
swg_ref_2b_host-dbg:
	$(CC) $(GCC_GDB_FLAGS) -DCUDA -c src/gotoh/psa_swgotoh_ref_2b_gpu.c -o build/psa_swg_ref_2b_host-dbg.o $(CUDA_LIBRARY_FLAGS)
swg_ref_2b_integer_gpu-dbg:
	$(NVCC) $(NVCC_GDB_FLAGS) -lineinfo -Xptxas -dlcm=ca -DCUDA -DQUERIES_SIZE=$(squery) -DCANDIDATES_SIZE=$(scandidate) -c src/gotoh/psa_swgotoh_ref_2b_integer_gpu.cu -o build/psa_swg_ref_2b_integer_device-dbg.o

benchmark_swg_ref_2b_integer_gpu-dbg: alignments-dbg errors-dbg profile-dbg regions-dbg sequences-dbg swg_ref_2b_host-dbg swg_ref_2b_integer_gpu-dbg time-dbg
	$(CC) $(GCC_GDB_FLAGS) -DCUDA -DREFERENCE build/psa_alignments-dbg.o build/psa_errors-dbg.o build/psa_profile-dbg.o build/psa_regions-dbg.o build/psa_sequences-dbg.o build/psa_swg_ref_2b_host-dbg.o build/psa_swg_ref_2b_integer_device-dbg.o build/psa_time-dbg.o tools/benchmark.c -o bin/bench_swg_ref_2b_integer_gpu_$(squery)-dbg $(CUDA_LIBRARY_FLAGS)


#  INSTALL           #
######################

clean:
	rm build/*
	rm bin/*

