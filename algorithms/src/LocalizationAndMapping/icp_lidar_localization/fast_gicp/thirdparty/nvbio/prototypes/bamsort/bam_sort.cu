/*
 * Copyright (c) 2012-14, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 *
 *
 *
 *
 *
 *
 *
 *
 */

#include <nvbio/basic/types.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/dna.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/io/sequence/sequence_access.h>
#include <nvbio/io/vcf.h>
#include <nvbio/io/sequence/sequence_pac.h>

#include <cub/cub.cuh>
#include <mgpuhost.cuh>
#include <moderngpu.cuh>
#include <nvbio/strings/string_set.h>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/cuda/timer.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/basic/cuda/primitives.h>

#include <nvbio/io/output/output_types.h>

#include <moderngpu.cuh>
#include <mgpuhost.cuh>

#include "bam_io.h"
#include "bam_sort.h"

//#ifdef _OPENMP
#include <omp.h>
//#endif

using namespace nvbio;

int test_sorted(const H_KVP_batch& result);

/** --------- Sorting Modules -------- **/

// generate sort keys
void sortkey_gen(bamsort_context* context)
{
	thrust::for_each(context->active_read_ids.begin(),
			context->active_read_ids.end(),
			generate_sort_keys(*context));
}

// local sort of key-val pairs on the device
void sort(bamsort_context* context)
{
	thrust::sort_by_key(context->sort_keys.begin(), context->sort_keys.end(), context->active_read_ids.begin());
}

// used by out-of-core merge for searching a sorted array
// to find the position until which all the elements are less than the pivot
// TODO: ensure thrust uses binary search
uint32 find_split_idx(bamsort_context* context, const uint64 len, const uint64 pivot)
{
	return thrust::distance(context->patch_searched.begin(),
			thrust::partition_point(context->patch_searched.begin(), context->patch_searched.begin() + len, is_less(pivot)));
}

// out-of-core merge of two sorted batches
// for single GPU: sequentially merge the pivot elements on the CPU to determine the partitions
// since only one partition at a time can be merged on the device -- this is to avoid
// extracting and sorting the pivot elements separately;
// TODO: optimize if pivots have the same value
void merge_batches_1GPU(bamsort_context* context, const H_KVP_batch* b1, const H_KVP_batch* b2, H_KVP_batch* out,
		float& merge_time, float& data_time, float& search_time)
{
	cuda::Timer timer;

	// partition info
	uint64 b1_npivots = b1->keys.size() / PIVOT_SAMPLING_INTERVAL;
	uint64 b2_npivots = b2->keys.size() / PIVOT_SAMPLING_INTERVAL;
	uint64 b1_pivot = 1, b2_pivot = 1;
	uint64 p1L = 0, p1H = 0; // batch partition limits [L, H)
	uint64 p2L = 0, p2H = 0;
	uint64 p1_size = 0, p2_size = 0;
	uint64 out_idx = 0;

	// mgpu context
	int current_device;
	cudaGetDevice(&current_device);
	mgpu::ContextPtr mgpu_ctxt = mgpu::CreateCudaDevice(current_device);

	while(1) {
		// check if we're in the last partition
		if(b1_pivot > b1_npivots || b2_pivot > b2_npivots) {
			break; // still need to merge the batch remainders
		}
		// find the next partition
		p1L = p1H;
		p2L = p2H;

		timer.start();
		if(b1->keys[b1_pivot*PIVOT_SAMPLING_INTERVAL-1] <= b2->keys[b2_pivot*PIVOT_SAMPLING_INTERVAL-1]) {
			p1H = b1_pivot*PIVOT_SAMPLING_INTERVAL;
			// only need to search this patch since the pivots are sorted
			NVBIO_CUDA_ASSERT(context->patch_searched.size() <= PIVOT_SAMPLING_INTERVAL);
			thrust::copy(b2->keys.begin() + p2L, b2->keys.begin() + b2_pivot*PIVOT_SAMPLING_INTERVAL, context->patch_searched.begin());
			p2H = p2L + find_split_idx(context, b2_pivot*PIVOT_SAMPLING_INTERVAL - p2L, b1->keys[b1_pivot*PIVOT_SAMPLING_INTERVAL-1]);
			b1_pivot++; // advance the pivot pointer
		} else {
			p2H = b2_pivot*PIVOT_SAMPLING_INTERVAL;
			NVBIO_CUDA_ASSERT(context->patch_searched.size() <= PIVOT_SAMPLING_INTERVAL);
			thrust::copy(b1->keys.begin() + p1L, b1->keys.begin() + b1_pivot*PIVOT_SAMPLING_INTERVAL, context->patch_searched.begin());
			p1H = p1L + find_split_idx(context, b1_pivot*PIVOT_SAMPLING_INTERVAL - p1L, b2->keys[b2_pivot*PIVOT_SAMPLING_INTERVAL-1]);
			b2_pivot++;
		}
		timer.stop();
		search_time += timer.seconds();
		p1_size = p1H - p1L;
		p2_size = p2H - p2L;
		//printf("Partition sizes: %llu %llu \n", p1_size, p2_size);

		// if one of the batch partitions is empty, we are done
		if(p1_size == 0) {
			thrust::copy(b2->keys.begin() + p2L, b2->keys.begin() + p2H, out->keys.begin()+out_idx);
			thrust::copy(b2->ids.begin() + p2L, b2->ids.begin() + p2H, out->ids.begin()+out_idx);
			out_idx += p2_size;
			continue;
		} else if(p2_size == 0) {
			thrust::copy(b1->keys.begin() + p1L, b1->keys.begin() + p1H, out->keys.begin()+out_idx);
			thrust::copy(b1->ids.begin() + p1L, b1->ids.begin() + p1H, out->ids.begin()+out_idx);
			out_idx += p1_size;
			continue;
		} // TODO: if the sizes are less than a given threshold, merge on the CPU

		// transfer the partitions to the device
		NVBIO_CUDA_ASSERT(context->p1.keys.size() <= 2*PIVOT_SAMPLING_INTERVAL);
		NVBIO_CUDA_ASSERT(context->p2.keys.size() <= 2*PIVOT_SAMPLING_INTERVAL);
		NVBIO_CUDA_ASSERT(context->r.keys.size() <= 2*PIVOT_SAMPLING_INTERVAL);

		timer.start();
		thrust::copy(b1->keys.begin() + p1L, b1->keys.begin() + p1H, context->p1.keys.begin());
		thrust::copy(b2->keys.begin() + p2L, b2->keys.begin() + p2H, context->p2.keys.begin());
		thrust::copy(b1->ids.begin() + p1L, b1->ids.begin() + p1H, context->p1.ids.begin());
		thrust::copy(b2->ids.begin() + p2L, b2->ids.begin() + p2H, context->p2.ids.begin());
		timer.stop();
		data_time += timer.seconds();

		// merge
		timer.start();
		mgpu::MergePairs(context->p1.keys.begin(), context->p1.ids.begin(), p1_size,
				context->p2.keys.begin(), context->p2.ids.begin(), p2_size,
				context->r.keys.begin(), context->r.ids.begin(), *mgpu_ctxt);
		timer.stop();
		merge_time += timer.seconds();

		// transfer the results to the host
		timer.start();
		thrust::copy(context->r.keys.begin(), context->r.keys.begin() + p1_size + p2_size, out->keys.begin()+out_idx);
		thrust::copy(context->r.ids.begin(), context->r.ids.begin() + p1_size + p2_size, out->ids.begin()+out_idx);
		timer.stop();
		data_time += timer.seconds();

		out_idx += p1_size + p2_size;
	}

	// merge the final pieces
	p1_size = b1->keys.size() - p1H;
	p2_size = b2->keys.size() - p2H;
	//printf("Final partition sizes: %llu %llu \n", p1_size, p2_size);

	// if one of the batch remainders is empty, we are done
	if(p1_size == 0) {
		thrust::copy(b2->keys.begin() + p2H, b2->keys.end(), out->keys.begin()+out_idx);
		thrust::copy(b2->ids.begin() + p2H, b2->ids.end(), out->ids.begin()+out_idx);
		return;
	} else if(p2_size == 0) {
		thrust::copy(b1->keys.begin() + p1H, b1->keys.end(), out->keys.begin()+out_idx);
		thrust::copy(b1->ids.begin() + p1H, b1->ids.end(), out->ids.begin()+out_idx);
		return;
	}
	NVBIO_CUDA_ASSERT(context->p1.keys.size() <= 2*PIVOT_SAMPLING_INTERVAL);
	NVBIO_CUDA_ASSERT(context->p2.keys.size() <= 2*PIVOT_SAMPLING_INTERVAL);
	NVBIO_CUDA_ASSERT(context->r.keys.size() <= 2*PIVOT_SAMPLING_INTERVAL);

	timer.start();
	thrust::copy(b1->keys.begin() + p1H, b1->keys.end(), context->p1.keys.begin());
	thrust::copy(b2->keys.begin() + p2H, b2->keys.end(), context->p2.keys.begin());
	thrust::copy(b1->ids.begin() + p1H, b1->ids.end(), context->p1.ids.begin());
	thrust::copy(b2->ids.begin() + p2H, b2->ids.end(), context->p2.ids.begin());
	timer.stop();
	data_time += timer.seconds();

	timer.start();
	mgpu::MergePairs(context->p1.keys.begin(), context->p1.ids.begin(), p1_size,
			context->p2.keys.begin(), context->p2.ids.begin(), p2_size,
			context->r.keys.begin(), context->r.ids.begin(), *mgpu_ctxt);
	timer.stop();
	merge_time += timer.seconds();

	timer.start();
	thrust::copy(context->r.keys.begin(), context->r.keys.begin() + p1_size + p2_size, out->keys.begin()+out_idx);
	thrust::copy(context->r.ids.begin(), context->r.ids.begin() + p1_size + p2_size, out->ids.begin()+out_idx);
	timer.stop();
	data_time += timer.seconds();
}

// out-of-core merge of two sorted batches
// two GPUs
void merge_batches_2GPU(H_KVP_batch* b1, H_KVP_batch* b2, H_KVP_batch* out)
{
	// partition info
	uint64 b1_npivots = (b1->keys.size()-1) / PIVOT_SAMPLING_INTERVAL;
	uint64 b2_npivots = (b2->keys.size()-1) / PIVOT_SAMPLING_INTERVAL;

	// 1. sort the pivots
	H_VectorU64 pivots(b1_npivots + b2_npivots);
	for(uint64 i = 0; i < b1_npivots; i++) {
		pivots[i] = b1->keys[(i+1)*PIVOT_SAMPLING_INTERVAL-1];
	}
	for(uint64 i = 0; i < b2_npivots; i++) {
		pivots[b1_npivots + i] = b2->keys[(i+1)*PIVOT_SAMPLING_INTERVAL-1];
	}
	thrust::sort(pivots.begin(), pivots.end());

	printf("Merge: found and sorted pivots. Num pivots %llu \n", (uint64) pivots.size());

	std::vector<H_KVP_batch*> batches(2);
	std::vector<H_VectorU64> pivot_idx(2);
	batches[0] = b1;
	batches[1] = b2;

	// 2. search each batch for the partition delimiters
	omp_set_num_threads(2);
	#pragma omp parallel
	{
		int tid = omp_get_thread_num();
		cudaSetDevice(tid);

		H_KVP_batch* b = batches[tid];
		D_VectorU64 d_bkeys(D_BATCH_SIZE);
		pivot_idx[tid].resize(pivots.size());

		uint64 num_processed = 0;
		uint64 pid = 0;

		while(num_processed < b->keys.size() && pid < pivots.size()) {
			uint64 batch_size = D_BATCH_SIZE;
			if(b->keys.size() - num_processed < D_BATCH_SIZE) {
				batch_size = b->keys.size() - num_processed;
			}
			thrust::copy(b->keys.begin() + num_processed, b->keys.begin() + num_processed + batch_size, d_bkeys.begin());

			// find as many pivots as possible in the loaded partition
			while(1) {
				if(pid >= pivots.size() || pivots[pid] > b->keys[num_processed + batch_size - 1]) {
					break; // load the next batch
				}
				// pivot is in the loaded section
				uint64 offset = thrust::distance(d_bkeys.begin(),
						thrust::partition_point(d_bkeys.begin(), d_bkeys.begin() + batch_size, is_less(pivots[pid])));

				pivot_idx[tid][pid] = num_processed + offset;
				pid++;
			}
			num_processed += batch_size;
		}

		if(pid < pivots.size()) {
			// if pid == 0, all elements in this batch are smaller than the elements in the second batch
			for(uint64 i = pid; i < pivots.size(); i++) {
				pivot_idx[tid][i] = b->keys.size();
			}
		}

		printf("Thread %d processed %llu elements \n", tid,  num_processed);
	}

	// 3. find partition offsets into output
	// TODO: optimize out empty partitions (when pivots are equal)
	uint64 num_partitions = pivots.size() + 1;
	H_VectorU64 p_offsets(num_partitions);
	p_offsets[0] = 0;
	for(uint64 i = 1; i < num_partitions; i++) {
		p_offsets[i] = pivot_idx[0][i-1] + pivot_idx[1][i-1];
	}
	printf("Total number of partitions: %llu \n", num_partitions);

	std::vector<mgpu::ContextPtr> mgpu_ctxt(2);
	mgpu_ctxt[0] = mgpu::CreateCudaDevice(0);
	mgpu_ctxt[1] = mgpu::CreateCudaDevice(1);

	// 4. merge the partitions
	omp_set_num_threads(2);
	#pragma omp parallel
	{
		int tid = omp_get_thread_num();
		cudaSetDevice(tid);

		bamsort_context context;
		context.allocate_partition();
		uint64 p1L, p1H; // batch partition limits [L, H)
		uint64 p2L, p2H;
		uint64 p1_size, p2_size;
		uint64 part_id = tid;

		while(part_id < num_partitions) {
			uint64 out_idx = p_offsets[part_id];

			if(part_id == 0) {
				p1L = 0;
				p2L = 0;
			} else {
				p1L = pivot_idx[0][part_id-1];
				p2L = pivot_idx[1][part_id-1];
			}

			if(part_id == num_partitions - 1) {
				p1H = b1->keys.size();
				p2H = b2->keys.size();
			} else {
				p1H = pivot_idx[0][part_id];
				p2H = pivot_idx[1][part_id];
			}

			p1_size = p1H - p1L;
			p2_size = p2H - p2L;
			printf("Thread %d. Partition sizes: %llu %llu\n", tid, p1_size, p2_size);

			// if one of the batch partitions is empty, we are done
			if(p1_size == 0) {
				thrust::copy(b2->keys.begin() + p2L, b2->keys.begin() + p2H, out->keys.begin()+out_idx);
				thrust::copy(b2->ids.begin() + p2L, b2->ids.begin() + p2H, out->ids.begin()+out_idx);
				part_id += 2;
				continue;
			} else if(p2_size == 0) {
				thrust::copy(b1->keys.begin() + p1L, b1->keys.begin() + p1H, out->keys.begin()+out_idx);
				thrust::copy(b1->ids.begin() + p1L, b1->ids.begin() + p1H, out->ids.begin()+out_idx);
				part_id += 2;
				continue;
			}

			// transfer the partitions to the device
			thrust::copy(b1->keys.begin() + p1L, b1->keys.begin() + p1H, context.p1.keys.begin());
			thrust::copy(b2->keys.begin() + p2L, b2->keys.begin() + p2H, context.p2.keys.begin());
			thrust::copy(b1->ids.begin() + p1L, b1->ids.begin() + p1H, context.p1.ids.begin());
			thrust::copy(b2->ids.begin() + p2L, b2->ids.begin() + p2H, context.p2.ids.begin());

			// merge
			mgpu::MergePairs(context.p1.keys.begin(), context.p1.ids.begin(), p1_size,
					context.p2.keys.begin(), context.p2.ids.begin(), p2_size,
					context.r.keys.begin(), context.r.ids.begin(), *mgpu_ctxt[tid]);

			// transfer the results to the host
			thrust::copy(context.r.keys.begin(), context.r.keys.begin() + p1_size + p2_size, out->keys.begin()+out_idx);
			thrust::copy(context.r.ids.begin(), context.r.ids.begin() + p1_size + p2_size, out->ids.begin()+out_idx);

			part_id += 2;
		}
	}
}

/** ------ Sorting Pipelines ---------- **/

// full load -> sort -> store (no IO-compute overlapping)
// single GPU
void bamsort_pipeline_basic(const char* in_fname, const char* out_fname)
{
	cuda::Timer timer, timer_alloc, timer_all;
	float sort_time = 0, keygen_time = 0, data_time = 0, merge_time = 0, merge_data_time = 0, merge_search_time = 0;
	//timer_all.start();

	// 1. load BAM
	timer.start();
	HTSBAMReader bam_reader(in_fname);
	BAM_alignment_batch_SoA h_batch(BAM_POSITIONS | BAM_REFIDS | BAM_FLAGS);
	bam_reader.read_aln_batch(h_batch, H_BATCH_SIZE);
	timer.stop();
	printf("BAM load time: %.4fs\n", timer.seconds());
	printf("Total number of alignments: %llu \n", h_batch.num_alns);

	timer_all.start();
	// 2. split and sort
	int num_batches = 0;
	uint64 num_aln_loaded = 0;
	std::list<H_KVP_batch*> sorted_kvp_batches; // container for the sorted batches
	bamsort_context device_context; // device data for sorting

	while(num_aln_loaded < h_batch.num_alns) {
		// transfer the next batch to the device
		timer.start();
		device_context.load_batch(h_batch, num_aln_loaded, D_BATCH_SIZE);
		timer.stop();
		data_time += timer.seconds();

		// generate the sort keys
		timer.start();
		sortkey_gen(&device_context);
		timer.stop();
		keygen_time += timer.seconds();

		// sort
		timer.start();
		sort(&device_context);
		timer.stop();
		sort_time += timer.seconds();

		// save sorted batches on the host
		timer.start();
		H_KVP_batch* sorted_batch = new H_KVP_batch();
		sorted_batch->keys.resize(device_context.sort_keys.size());
		sorted_batch->ids.resize(device_context.sort_keys.size());
		thrust::copy(device_context.sort_keys.begin(), device_context.sort_keys.end(), sorted_batch->keys.begin());
		thrust::copy(device_context.active_read_ids.begin(), device_context.active_read_ids.end(), sorted_batch->ids.begin());
		sorted_kvp_batches.push_back(sorted_batch);
		timer.stop();
		data_time += timer.seconds();

		num_batches += 1;
		num_aln_loaded += sorted_batch->ids.size();
		printf("Processed %d batches and %llu reads \n", num_batches, num_aln_loaded);
	}
	printf("Local keygen-only time  : %.4fs\n", keygen_time);
	printf("Local batch sorting-only time  : %.4fs\n", sort_time);
	printf("Local device data allocation and transfer time  : %.4fs\n", data_time);

	// free device data
	device_context.free_local_sort_batch();

	// 3. merge
	H_KVP_batch* final_result = sorted_kvp_batches.front();
	if(sorted_kvp_batches.size() != 1) {
		device_context.allocate_partition();
	}
	timer.start();
	float alloc_time = 0;
	while(sorted_kvp_batches.size() > 1) {
		timer_alloc.start();
		H_KVP_batch* out = new H_KVP_batch();
		H_KVP_batch* b1 = sorted_kvp_batches.front();
		sorted_kvp_batches.pop_front();
		H_KVP_batch* b2 = sorted_kvp_batches.front();
		sorted_kvp_batches.pop_front();

		// allocate space for the merged batches
		out->keys.resize(b1->keys.size() + b2->keys.size());
		out->ids.resize(out->keys.size());
		timer_alloc.stop();
		alloc_time += timer_alloc.seconds();

		// merge
		merge_batches_1GPU(&device_context, b1, b2, out, merge_time, merge_data_time, merge_search_time);
		sorted_kvp_batches.push_back(out);

		// free batch memory
		b1->free();
		b2->free();
		free(b1);
		free(b2);

		if(sorted_kvp_batches.size() == 1) { // merged down to one sequence
			final_result = out;
			break;
		}
	}
	timer.stop();
	printf("Device merge-only time  : %.4fs\n", merge_time);
	printf("Device merge data time  : %.4fs\n", merge_data_time);
	printf("Device merge search time  : %.4fs\n", merge_search_time);
	printf("Merge out alloc time  : %.4fs\n", alloc_time);
	printf("Total merge time  : %.4fs\n", timer.seconds());

	timer_all.stop();

	test_sorted(*final_result);

	timer.start();
	//BAM_alignment_batch_SoA out_batch(h_batch.field_mask);
	//h_batch.shuffle(out_batch, final_result->ids);
	timer.stop();
	printf("Shuffle time  : %.4fs\n", timer.seconds());

	// 4. write BAM output
	timer.start();
	HTSBAMWriter bam_writer(out_fname);
	//bam_writer.write_hdr(bam_reader.header);
	//bam_writer.write_aln_batch(h_batch, final_result->ids, bam_reader.header);
	timer.stop();
	printf("BAM write time  : %.4fs\n", timer.seconds());

	//timer_all.stop();
	printf("Total BAMSORT time  : %.4fs\n", timer_all.seconds());
}

// full load -> sort -> store (no overlapping)
// multi-GPU
void bamsort_pipeline_multigpu(const char* in_fname, const char* out_fname)
{
	cuda::Timer timer, timer_all;

	// 1. load BAM
	timer.start();
	HTSBAMReader bam_reader(in_fname);
	BAM_alignment_batch_SoA h_batch(BAM_POSITIONS | BAM_REFIDS | BAM_FLAGS);
	bam_reader.read_aln_batch(h_batch, H_BATCH_SIZE);
	timer.stop();
	printf("BAM load time: %.4fs\n", timer.seconds());
	printf("Total number of alignments: %llu \n", h_batch.num_alns);

	int num_dev = 0;
	cudaGetDeviceCount(&num_dev);
	printf("Total number of CPUs: %d\n", omp_get_num_procs());
	printf("Total number of GPUs: %d\n", num_dev);
	for (int i = 0; i < num_dev; i++) {
		cudaDeviceProp dprop;
		cudaGetDeviceProperties(&dprop, i);
		printf("   %d: %s\n", i, dprop.name);
	}

	// 2. split and sort
	//TODO: figure out # devices that is appropriate to use based on load size (when num_dev > 2)
	if(h_batch.num_alns <= D_MIN_BATCH_SIZE) {
		printf("Running on a single device (load size is <= minimum single device size) \n");
		num_dev = 1;
	}

	uint64 thread_batch_size = h_batch.num_alns/num_dev;
	std::vector<H_KVP_batch*> thread_sorted_batches(num_dev);
	timer_all.start();
	omp_set_num_threads(num_dev);
	#pragma omp parallel
	{
		int tid = omp_get_thread_num();
		cudaSetDevice(tid);
		int devId;
		cudaGetDevice(&devId);
		printf("Thread %d device %d\n", tid,  devId);

		cuda::Timer ttimer;
		uint64 toffset = tid * thread_batch_size;
		uint64 tsize = thread_batch_size;
		if(tid == num_dev-1) {
			tsize = h_batch.num_alns - toffset; // remainder
		}
		printf("Thread %d offset %llu size %llu\n", tid,  toffset, tsize);

		std::list<H_KVP_batch*> sorted_kvp_batches; // host container for the sorted batches
		bamsort_context device_context;
		uint64 num_aln_loaded = 0;
		ttimer.start();
		while(num_aln_loaded < tsize) {
			// transfer the next batch to the device
			uint64 batch_size = D_BATCH_SIZE;
			if(tsize - num_aln_loaded < D_BATCH_SIZE) {
				batch_size = tsize - num_aln_loaded;
			}
			device_context.load_batch(h_batch, toffset + num_aln_loaded, batch_size);
			sortkey_gen(&device_context);
			sort(&device_context);

			// save sorted batches on the host
			H_KVP_batch* sorted_batch = new H_KVP_batch();
			sorted_batch->keys.resize(device_context.sort_keys.size());
			sorted_batch->ids.resize(device_context.sort_keys.size());
			thrust::copy(device_context.sort_keys.begin(), device_context.sort_keys.end(), sorted_batch->keys.begin());
			thrust::copy(device_context.active_read_ids.begin(), device_context.active_read_ids.end(), sorted_batch->ids.begin());
			sorted_kvp_batches.push_back(sorted_batch);
			num_aln_loaded += sorted_batch->ids.size();
			printf("Thread %d processed %llu records \n", tid, (uint64) sorted_batch->ids.size());
		}
		ttimer.stop();
		printf("Thread %d done with local sorting. Time %.4fs \n", tid, ttimer.seconds());
		device_context.free_local_sort_batch();

		// merge down to a single batch on each device
		if(sorted_kvp_batches.size() == 1) {
			thread_sorted_batches[tid] = sorted_kvp_batches.front();
		} else {
			device_context.allocate_partition();
		}
		ttimer.start();
		while(sorted_kvp_batches.size() > 1) {
			H_KVP_batch* out = new H_KVP_batch();
			H_KVP_batch* b1 = sorted_kvp_batches.front();
			sorted_kvp_batches.pop_front();
			H_KVP_batch* b2 = sorted_kvp_batches.front();
			sorted_kvp_batches.pop_front();
			// allocate space for the merged batches
			out->keys.resize(b1->keys.size() + b2->keys.size());
			out->ids.resize(out->keys.size());
			float t1, t2, t3;
			merge_batches_1GPU(&device_context, b1, b2, out, t1, t2, t3);
			sorted_kvp_batches.push_back(out);
			b1->free();
			b2->free();
			free(b1);
			free(b2);
			if(sorted_kvp_batches.size() == 1) { // merged down to one sequence
				thread_sorted_batches[tid] = out;
				break;
			}
		}
		ttimer.stop();
		printf("Thread %d done with merging. Time %.4fs \n", tid, ttimer.seconds());
	}
	H_KVP_batch* final_result = new H_KVP_batch();
	if(num_dev == 2) { //TODO: generalize to any number of devices
		final_result->keys.resize(thread_sorted_batches[0]->keys.size() + thread_sorted_batches[1]->keys.size());
		final_result->ids.resize(final_result->keys.size());
		merge_batches_2GPU(thread_sorted_batches[0], thread_sorted_batches[1], final_result);
	}
	timer_all.stop();
	printf("Total sort time  : %.4fs\n", timer_all.seconds());

	test_sorted(*final_result);

	// 4. write BAM output
	timer.start();
	HTSBAMWriter bam_writer(out_fname);
	//bam_writer.write_header(bam_reader.header);
	//bam_writer.write_aln_batch(h_batch, final_result.ids, bam_reader.header);
	timer.stop();
	printf("BAM write time  : %.4fs\n", timer.seconds());
}

// permute the sorted BAM file
void generate_unsorted_bam(const char* in_fname, const char* out_fname) {
	try {
		// load
		BAMReader bam_reader(in_fname);
		BAM_header hdr = bam_reader.header;
		BAM_alignment_batch_raw sorted_batch;
		bam_reader.read_aln_batch_raw(sorted_batch, H_BATCH_SIZE);
		uint64 num_loaded = sorted_batch.offsets.size()-1;
		printf("Total number of loaded alignments: %llu \n", num_loaded);

		// generate permutation
		std::vector<uint64> perm(num_loaded);
		for (uint64 i=0; i<num_loaded; ++i) perm.push_back(i);
		std::srand(0);
		std::random_shuffle(perm.begin(), perm.end());
		H_VectorU64 ids (num_loaded);
		for (uint64 i=0; i<num_loaded; ++i) ids.push_back(perm[i]);

		// write
		BAMWriter bam_writer(out_fname);
		bam_writer.write_header(hdr);
		bam_writer.write_aln_batch_raw(sorted_batch, ids);
	} catch (nvbio::runtime_error& e) {
		printf("%s\n", e.what());
		exit(1);
	}
}

// concatenate BAM file contents multiple times
void duplicate_unsorted_bam(const char* in_fname, const char* out_fname, int num_repeats) {
	try {
		BAMReader bam_reader(in_fname);
		BAM_header hdr = bam_reader.header;
		BAM_alignment_batch_raw sorted_batch;
		bam_reader.read_aln_batch_raw(sorted_batch, H_BATCH_SIZE);
		uint64 num_loaded = sorted_batch.offsets.size()-1;
		printf("Total number of loaded alignments: %llu \n", num_loaded);

		H_VectorU64 ids (num_loaded);
		thrust::sequence(ids.begin(), ids.end());

		BAMWriter bam_writer(out_fname);
		bam_writer.write_header(hdr);
		for(int i = 0; i < num_repeats; i++) {
			bam_writer.write_aln_batch_raw(sorted_batch, ids);
		}
	} catch (nvbio::runtime_error& e) {
		printf("%s\n", e.what());
		exit(1);
	}
}

//TODO: overlap load with device compute
int main(int argc, char **argv)
{
	if(argc < 3) {
		printf("Usage: ./bamsort <bam_file> <out_file> \n");
		exit(1);
	}

	try {
		//generate_unsorted_bam(argv[1], argv[2]);
		//duplicate_unsorted_bam(argv[1], argv[2], 1000);
		bamsort_pipeline_basic(argv[1], argv[2]);
		//bamsort_pipeline_multigpu(argv[1], argv[2]);
	} catch (nvbio::runtime_error& e) {
		printf("%s\n", e.what());
		exit(1);
	}
	return 0;
}

int test_sorted(const H_KVP_batch& result)
{
	// check that all the keys are in ascending order
	uint64 k = 0;
	for(uint64 i = 0; i < result.keys.size(); i++) {
		if(k > result.keys[i]) {
			printf("Failed test; out of order: %llu %llu %llu %llu %llu \n", (uint64) k, (uint64) result.keys[i], (uint64) result.keys[i-1], i, i-1);
			return 0;
		}
		k = result.keys[i];
	}

	printf("Passed test! \n");
	return 1;
}
