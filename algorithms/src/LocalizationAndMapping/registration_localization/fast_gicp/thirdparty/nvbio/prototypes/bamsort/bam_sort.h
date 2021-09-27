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

#pragma once

#include "bamsort_types.h"
#include "bam_io.h"

using namespace nvbio;

/** Sorting Utilities **/

#define PIVOT_SAMPLING_INTERVAL 50000000
#define D_BATCH_SIZE 100000000
#define D_MIN_BATCH_SIZE 10000000
#define H_BATCH_SIZE 100000000

// pair of key-value(id) vectors
struct H_KVP_batch
{
	H_VectorU64 keys;
	H_VectorU64 ids;

	void free()
	{
		keys = H_VectorU64();
		ids = H_VectorU64();
	}
};

struct D_KVP_batch
{
	D_VectorU64 keys;
	D_VectorU64 ids;

	struct view
	{
		D_VectorU64::const_plain_view_type keys;
		D_VectorU64::const_plain_view_type ids;
	};

	operator view() const {
		view v = {
				plain_view(keys),
				plain_view(ids),
		};
		return v;
	}

	void free()
	{
		keys = D_VectorU64();
		ids = D_VectorU64();
	}
};

/** BAM Sort Pipeline Context (sort data on the device) **/

struct bamsort_context
{
	// alignment data
	D_VectorU32 pos;
	D_VectorU32 seq_ids;
	D_VectorU16 flags;

	// local sorting
	D_VectorU64 active_read_ids;
	uint64 read_offset; // global read id offset
	D_VectorU64 sort_keys;  // sort keys

	// out-of-core merge
	D_VectorU64 patch_searched;
	D_KVP_batch p1, p2, r; // partitions and result

	struct view
	{
		// TODO: const
		D_VectorU32::plain_view_type pos;
		D_VectorU32::plain_view_type seq_ids;
		D_VectorU16::plain_view_type flags;
		D_VectorU64::plain_view_type active_read_ids;
		D_VectorU64::plain_view_type sort_keys;
		uint64 read_offset;
		D_KVP_batch::view p1;
		D_KVP_batch::view p2;
		D_KVP_batch::view r;
	};

	operator view() {
		view v = {
				plain_view(pos),
				plain_view(seq_ids),
				plain_view(flags),
				plain_view(active_read_ids),
				plain_view(sort_keys),
				read_offset,
				p1,
				p2,
				r
		};
		return v;
	}

	// allocate and load the data onto the device
	// loads the entire host batch
	void load_batch(const BAM_alignment_batch_SoA& batch, const uint64 start_read_id)
	{
		// initialize the read ids
		read_offset = start_read_id;
		active_read_ids.resize(batch.num_alns);
		thrust::sequence(active_read_ids.begin(),
				active_read_ids.begin() + batch.num_alns,
				read_offset);

		// load the data for position sorting
		pos.resize(batch.num_alns);
		seq_ids.resize(batch.num_alns);
		flags.resize(batch.num_alns);
		thrust::copy(batch.positions.begin(), batch.positions.end(), pos.begin());
		thrust::copy(batch.refIDs.begin(), batch.refIDs.end(), seq_ids.begin());
		thrust::copy(batch.flags.begin(), batch.flags.end(), flags.begin());

		// allocate space for the keys
		sort_keys.resize(batch.num_alns);
	}

	// allocate and load the data onto the device
	// loads a batch_size interval of the host batch only starting with read_offset
	void load_batch(const BAM_alignment_batch_SoA& batch, const uint64 start_read_id, const uint64 batch_size)
	{
		read_offset = start_read_id;
		uint64 n = read_offset;
		if((batch.num_alns - read_offset) >= batch_size) {
			n = batch_size;
		} else {
			n = batch.num_alns - read_offset;
		}

		// initialize the read ids
		active_read_ids.resize(n);
		thrust::sequence(active_read_ids.begin(), active_read_ids.begin() + n, read_offset);

		// allocate space
		pos.resize(n);
		seq_ids.resize(n);
		flags.resize(n);
		sort_keys.resize(n);

		// load the data for sorting
		thrust::copy(batch.positions.begin() + read_offset,
				batch.positions.begin() + read_offset + n, pos.begin());
		thrust::copy(batch.refIDs.begin() + read_offset,
				batch.refIDs.begin() + read_offset + n, seq_ids.begin());
		thrust::copy(batch.flags.begin() + read_offset,
				batch.flags.begin() + read_offset, flags.begin());
	}

	// free the data used for local sorting
	void free_local_sort_batch()
	{
		active_read_ids = D_VectorU64();
		pos = D_VectorU32();
		seq_ids = D_VectorU32();
		flags = D_VectorU16();
		sort_keys = D_VectorU64();
	}

	// allocate memory for the merge partitions
	// allocate instead of reserve to prevent dynamic allocations
	void allocate_partition()
	{
		p1.keys.resize(2*PIVOT_SAMPLING_INTERVAL);
		p1.ids.resize(2*PIVOT_SAMPLING_INTERVAL);
		p2.keys.resize(2*PIVOT_SAMPLING_INTERVAL);
		p2.ids.resize(2*PIVOT_SAMPLING_INTERVAL);
		r.keys.resize(2*PIVOT_SAMPLING_INTERVAL);
		r.ids.resize(2*PIVOT_SAMPLING_INTERVAL);
		patch_searched.resize(PIVOT_SAMPLING_INTERVAL);
	}
};

/** Sorting Pipeline Functors **/

// encapsulates common state for thrust functors
struct bamsort_lambda
{
	bamsort_context::view ctx;
	bamsort_lambda(bamsort_context::view ctx) : ctx(ctx) { }
};

// generates the sorting keys from the alignment position
// unmapped reads (pos = -1) will have the largest key
// and automatically be placed at the end of the file
// (might want to handle them separately instead)
struct generate_sort_keys : public bamsort_lambda
{
	generate_sort_keys(bamsort_context::view ctx)
	: bamsort_lambda(ctx) { }

	NVBIO_HOST_DEVICE void operator() (const uint64 idx) {
		uint64 read_index = idx - ctx.read_offset;
		uint64& key = ctx.sort_keys[read_index];
		key = (uint64) ctx.seq_ids[read_index] << 32;
		key |= (ctx.pos[read_index] + 1) << 1;
		key |= ((ctx.flags[read_index] & BAM_FLAGS_REVERSE) != 0);
	}
};

// compares the elements to a pivot value
// return true if the element is smaller than the pivot
struct is_less
{
	uint64 pivot;
	is_less (uint64 p) : pivot(p) {}

	NVBIO_HOST_DEVICE bool operator() (const uint64& x) const {
		return x <= pivot;
	}
};
