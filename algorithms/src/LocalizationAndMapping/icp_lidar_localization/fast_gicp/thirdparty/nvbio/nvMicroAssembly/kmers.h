/*
 * nvbio
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the NVIDIA CORPORATION nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "assembly_types.h"

using namespace nvbio;

#define MAX_TEMP_SPACE_BATCHES 5

// set of kmers from a set of sequences
template <typename string_set_type>
struct D_KmerSet
{
	const string_set_type string_set;	// string set from which the kmers are extracted
	D_VectorU32 active_region_ids; // active region id of each sequence in the set
	uint32 kmer_size;			// kmer size used to extract the kmers

	D_VectorSetKmerCoord coords; 		// coordinates in the sequence set
	D_VectorU64 kmers_64b;				// kmer keys: 64-bit compacted kmer sequence
	D_VectorU64 kmers_64b_distinct; 	// distinct kmer keys

	D_VectorU32 kmers_64b_unique_idxs; 	// indices to unique kmers in the sorted coordinate vector
	D_VectorU32 kmers_64b_repeat_idxs; 	// indices to unique kmers in the sorted coordinate vector
	D_VectorU32 global_to_sorted_id_map;// map[i] = sorted array index of coord with global id i
	D_VectorU32 global_to_UID_map;		// map from global id to the kmer UID
	D_VectorU8 	global_unique_flag_map;	// map[i] = 1 if kmer with global id i is unique; 0, otherwise

	D_VectorU32 kmers_counts; 			// number of occurrences of each kmer in the set

	uint32 n_kmers;		// total number of kmers in the set
	uint32 n_distinct; 	// number of distinct kmer keys in the set
	uint32 n_unique; 	// number of distinct unique kmers (unique := kmer does not occur more than once in a sequence)
	uint32 n_repeat;	// number of non-unique kmers that are extracted as graph nodes


	// super kmers (to handle non-unique kmers)
	D_VectorSetKmerCoord super_coords;
	D_VectorU32 super_prefix_uids;
	D_VectorU32 super_suffix_uids;
	uint32 n_super_coords;

	// scratch space
	D_VectorU32	scratch_u32;
	uint32 n_alloc;
	uint32 selector;

	D_KmerSet(): kmer_size(0), n_kmers(0), n_distinct(0), n_unique(0), n_repeat(0), n_super_coords(0),
			n_alloc(0), selector(0) {}
	D_KmerSet(const string_set_type _string_set, const uint32 _kmer_size):
		string_set(_string_set), kmer_size(_kmer_size),
		n_kmers(0), n_distinct(0), n_unique(0), n_repeat(0), n_super_coords(0),
		n_alloc(0), selector(0) { }
	D_KmerSet(const string_set_type _string_set, const D_VectorU32& _active_region_ids):
		string_set(_string_set), active_region_ids(_active_region_ids),
		kmer_size(0), n_kmers(0), n_distinct(0), n_unique(0), n_repeat(0), n_super_coords(0),
		n_alloc(0), selector(0) { }

	void gen_kmer_coords();
	void gen_kmer_64b_keys();
	void sort_kmers_by_64b_keys();
	void segmented_sort_kmers_by_64b_keys();
	template <typename meta_iterator_type>
	void sort_kmers_by_64b_keys_meta(const meta_iterator_type meta_data);
	void sort_kmers_by_64b_keys_seqid();
	template <typename meta_iterator_type>
	void sort_kmers_by_64b_keys_seqid_meta(const meta_iterator_type meta_data);
	void count_kmers_rle();
	void count_kmers();
	void count_distinct_by_prefix(D_VectorU32& prefix_unique_id_map);
	void partition_kmers_by_uniqueness();
	void gen_prefix_map();
	void gen_global_unique_map();
	void gen_global_UID_map();
	void gen_global_to_sorted_id_map();
	void mark_unique_kmers();
	void filter_coords_by_prefix_uniqueness(const D_VectorU8& unique_map);
	void extract_super_kmers();
	void collapse_and_extract_non_overlaps(D_VectorSetKmerCoord& kmers_out, D_VectorU32& prefix_ids_out,
			D_VectorU32& suffix_ids_out, D_VectorU32& counts_out);
	void count_distinct_by_prefix();


	void init_alloc_temp_space()
	{
		n_alloc = MAX_TEMP_SPACE_BATCHES;
		scratch_u32.resize(n_alloc*n_kmers);
	}

	D_VectorU32::iterator get_scratch_space(uint32 size)
	{
		if((size > n_kmers) || ((selector + 1) > MAX_TEMP_SPACE_BATCHES)) {
			printf("Requested more memory than batch size \n");
			exit(-1);
		}
		selector++;
		return scratch_u32.begin() + (selector-1)*n_kmers;
	}

	void reset_scratch_space()
	{
		selector = 0;
	}
};

#include "kmers_inl.h"
