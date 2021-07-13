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

#include <cstdlib>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/basic/atomics.h>
#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/cuda/primitives.h>
#include <nvbio/strings/seeds.h>

#include "assembly_types.h"

/* K-mer Extraction/Counting/Uniqueness Functionality */

// extract the string set sequence id from the kmer coordinates
struct get_kmer_seq_id : public thrust::unary_function<SequenceSetKmerCoord,uint32>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const SequenceSetKmerCoord kmer_coord) const
    {
        return kmer_coord.x;
    }
};

struct get_kmer_reg_id
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const SequenceSetKmerCoord kmer_coord) const
    {
        return kmer_coord.w;
    }
};

// extract the kmer size from the kmer coordinates
struct get_kmer_size : public thrust::unary_function<SequenceSetKmerCoord,uint32>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const SequenceSetKmerCoord kmer_coord) const
    {
        return kmer_coord.z - kmer_coord.y;
    }
};

struct get_global_id : public thrust::unary_function<SequenceSetKmerCoord,uint32>
{
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const SequenceSetKmerCoord coord) const
    {
        return coord.z;
    }
};

// store the global kmer id in its coordinate
struct set_global_id
{
	SequenceSetKmerCoord* coords;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	set_global_id(SequenceSetKmerCoord* coords) : coords(coords) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 idx) const
    {
        coords[idx].z = idx;
    }
};

// store the global kmer id and the active region id in its coordinate
struct set_global_id_region_id
{
	SequenceSetKmerCoord* coords;
	const uint32* active_region_ids;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	set_global_id_region_id(SequenceSetKmerCoord* coords, const uint32* _active_region_ids) :
		coords(coords), active_region_ids(_active_region_ids) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 idx) const
    {
		SequenceSetKmerCoord& coord = coords[idx];
		coord.z = idx; // since the kmer size is fixed, can reuse this field
        coord.w = active_region_ids[coord.x];
    }
};

// maps from the coordinate of a kmer
// to the global id of its prefix
struct get_prefix_global_id : public thrust::unary_function<uint32,uint32>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const SequenceSetKmerCoord coord) const
    {
    	return coord.z + coord.x;
    }
};

// maps from the id of a kmer
// to the global id of its prefix
struct get_prefix_global_id_by_idx : public thrust::unary_function<uint32,uint32>
{
	const SequenceSetKmerCoord* coords;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	get_prefix_global_id_by_idx(const SequenceSetKmerCoord* _coords):
	coords(_coords) { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint32 idx) const
    {
    	const SequenceSetKmerCoord coord = coords[idx];
    	return coord.z + coord.x;
    }
};

// maps from the id of a kmer
// to the global id of its suffix
struct get_suffix_global_id_by_idx : public thrust::unary_function<uint32,uint32>
{
	const SequenceSetKmerCoord* coords;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	get_suffix_global_id_by_idx(const SequenceSetKmerCoord* _coords):
	coords(_coords) { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint32 idx) const
    {
    	const SequenceSetKmerCoord coord = coords[idx];
    	return coord.z + coord.x + 1;
    }
};

// creates a map from the id of a kmer
// to the global id of its prefix
struct compute_prefix_global_id
{
	const SequenceSetKmerCoord* coords;
	uint32* prefix_ids;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	compute_prefix_global_id(const SequenceSetKmerCoord* _coords, uint32* _prefix_ids):
	coords(_coords), prefix_ids(_prefix_ids) { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 id) const
    {
    	const SequenceSetKmerCoord coord = coords[id];
    	prefix_ids[id] = coord.z + coord.x;
    }
};

// creates a map from the id of a kmer
// to the global id of its suffix
struct compute_suffix_global_id
{
	const SequenceSetKmerCoord* coords;
	uint32* suffix_ids;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	compute_suffix_global_id(const SequenceSetKmerCoord* _coords, uint32* _suffix_ids):
	coords(_coords), suffix_ids(_suffix_ids) { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 id) const
    {
    	const SequenceSetKmerCoord coord = coords[id];
    	suffix_ids[id] = coord.z + coord.x + 1;
    }
};

// maps kmer global id to its UID
struct global_to_uid : public thrust::unary_function<uint64,uint64>
{
	const uint32* uid_map;
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	global_to_uid(const uint32* _uid_map) : uid_map(_uid_map) { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint32 global_id) const
    {
    	return uid_map[global_id];
    }
};

// creates a map from the global id to the id in the sorted array
struct global_to_sorted_id
{
	uint32* global_to_sorted_id_map;
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	global_to_sorted_id(uint32* _map) : global_to_sorted_id_map(_map) { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const thrust::tuple<uint32, SequenceSetKmerCoord> idx_coord) const
    {
    	const uint32 idx = thrust::get<0>(idx_coord);
    	const SequenceSetKmerCoord coord = thrust::get<1>(idx_coord);
    	global_to_sorted_id_map[coord.z] = idx; // TODO: not coalesced
    }
};

// compute the kmer keys: 64-bit compacted kmer sequence
template <typename string_set_type>
struct kmers_to_64b_functor
{
    typedef typename string_set_type::string_type sequence;
    const string_set_type   string_set;
    const uint32            kmer_size;
	const uint32            dna_symbol_size;
	const uint32            symbol_mask;

	NVBIO_HOST_DEVICE
    kmers_to_64b_functor(const uint32 _kmer_size, const uint32 _dna_symbol_size, const string_set_type _string_set) :
        kmer_size(_kmer_size), dna_symbol_size(_dna_symbol_size), symbol_mask((1u << dna_symbol_size) - 1u),
        string_set(_string_set) {}

    NVBIO_HOST_DEVICE
    uint64 operator() (const SequenceSetKmerCoord kmer_coord) const
    {
    	const sequence seq = string_set[kmer_coord.x];
        const uint32 seq_pos = kmer_coord.y;
        const uint32 seq_len = seq.length();

        uint64 kmer_key = 0u;
        for (uint32 i = 0; i < kmer_size; i++) {
        	kmer_key |= uint64(seq_pos + i < seq_len ? (seq[seq_pos + i] & symbol_mask) : 0u) << ((kmer_size-1-i)*dna_symbol_size); //(i*dna_symbol_size);
        }
        return kmer_key;
    }
};

// equality based on kmer key and sequence id coordinate
struct kmer_key_sid_eq
{
	const uint64* keys;
	const SequenceSetKmerCoord* coords;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	kmer_key_sid_eq(const uint64* _keys, const SequenceSetKmerCoord* _coords):
		keys(_keys), coords(_coords) { }

	NVBIO_HOST_DEVICE bool operator()(const uint32 i, const uint32 j)
	{
		return keys[i] == keys[j] && coords[i].x == coords[j].x;
	}
};

// equality based on kmer key and region id coordinate
struct kmer_key_rid_eq
{
	const uint64* keys;
	const SequenceSetKmerCoord* coords;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	kmer_key_rid_eq(const uint64* _keys, const SequenceSetKmerCoord* _coords):
		keys(_keys), coords(_coords) { }

	NVBIO_HOST_DEVICE bool operator()(const uint32 i, const uint32 j)
	{
		return keys[i] == keys[j] && coords[i].w == coords[j].w;
	}
};

// equality based on sequence id and position
struct kmer_pos_sid_eq
{
	const SequenceSetKmerCoord* coords;
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	kmer_pos_sid_eq(const SequenceSetKmerCoord* _coords): coords(_coords) { }

	NVBIO_HOST_DEVICE bool operator()(const uint32 i, const uint32 j)
	{
		const SequenceSetKmerCoord ci = coords[i];
		const SequenceSetKmerCoord cj = coords[j];
		return ci.y == cj.y && ci.x == cj.x;
	}
};

// equality based on kmer key only
struct kmer_uid_eq
{
	const uint32* keys;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	kmer_uid_eq(const uint32* _keys) : keys(_keys) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	bool operator()(const uint32 i, const uint32 j)
	{
		return keys[i] == keys[j];
	}
};

// extract i-th word from the kmer in a string set
template <typename string_set_type>
struct kmer_word_extractor_functor
{
	typedef typename string_set_type::string_type sequence;
	const string_set_type   string_set;
	const uint32            dna_symbol_size;
	const uint32            symbol_offset;
	const uint32            symbols_per_word;
	const SequenceSetKmerCoord* coords;
	const uint32 			i;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    kmer_word_extractor_functor(const string_set_type _string_set, const uint32 _dna_symbol_size,
    		const SequenceSetKmerCoord* _coords, const uint32 _i) :
    	dna_symbol_size(_dna_symbol_size),
    	symbol_offset(uint32(8u*sizeof(uint32)) - dna_symbol_size),
    	symbols_per_word(uint32(8u * sizeof(uint32))/dna_symbol_size),
    	coords(_coords), string_set(_string_set), i(_i) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint32 kmer_idx) const
    {
    	SequenceSetKmerCoord kmer_coord = coords[kmer_idx];
    	const uint32 kmer_pos = kmer_coord.y;
    	const uint32 kmer_size = kmer_coord.z - kmer_coord.y; // TODO: z-field is currently used for the global id, use kmer_size if fixed
    	const sequence seq = string_set[kmer_coord.x];
    	const uint32 seq_len = seq.length();

		uint32 word = 0u;
		for (uint32 j = 0; j < kmer_size; j++) {
			const uint32 jj = kmer_pos + i*symbols_per_word + j;
			const uint32 c = jj < seq_len ? char_to_iupac16(dna_to_char(seq[jj])) : 0u;
			word |= (c << (symbol_offset - j*dna_symbol_size));
		}
    	return word;
    }
};

// add tuples of kmer and sequence counts
struct kmer_count_tuple_sum
{
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	thrust::tuple<uint32, uint32> operator() (const thrust::tuple<uint32, uint32>& x, const thrust::tuple<uint32, uint32>& y) const
    {
        return thrust::tuple<uint32, uint32>(thrust::get<0>(x) + thrust::get<0>(y), thrust::get<1>(x) + thrust::get<1>(y));
    }
};

// check if a kmer occurs more than once in at least one sequence
// given a tuple of the number of sequences that contain it and
// the number of times it occurs in all the sequences
struct is_unique_kmer
{
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator() (const thrust::tuple<uint32, uint32>& nSeq_nOcc) const
    {
        return thrust::get<0>(nSeq_nOcc) >= thrust::get<1>(nSeq_nOcc);
    }
};

// given the last id of a kmer in the group of consecutive equal kmers
// mark all identical consecutive kmers as unique in the map
struct mark_kmer_uniqueness
{
	uint8* is_unique_map;
	const uint64* sorted_kmer_keys;
	const SequenceSetKmerCoord* sorted_coords;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	mark_kmer_uniqueness(uint8* _is_unique_map, const uint64* _sorted_kmer_keys, const SequenceSetKmerCoord* _sorted_coords) :
	is_unique_map(_is_unique_map), sorted_kmer_keys(_sorted_kmer_keys), sorted_coords(_sorted_coords) { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 kmer_id) const
    {
    	const uint64 kmer_key = sorted_kmer_keys[kmer_id];
    	const uint32 rid = sorted_coords[kmer_id].w;

    	int32 idx = kmer_id;
    	while(idx >= 0) {
    		if(sorted_coords[idx].w != rid || sorted_kmer_keys[idx] != kmer_key) break;
    		is_unique_map[idx] = 1; //TODO: thrust documentation states first, but last in experiments
    		idx--;
    	}
    }
};

struct store_kmer_unique_ids
{
	uint32* unique_id_map;
	const uint64* sorted_kmer_keys;
	const SequenceSetKmerCoord* sorted_coords;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	store_kmer_unique_ids(uint32* _unique_id_map, const uint64* _sorted_kmer_keys, const SequenceSetKmerCoord* _sorted_coords) :
	unique_id_map(_unique_id_map), sorted_kmer_keys(_sorted_kmer_keys), sorted_coords(_sorted_coords) { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const thrust::tuple<uint32, uint32>& uid_kid) const
    {
    	const uint32 uid = thrust::get<0>(uid_kid);
    	const uint32 kid = thrust::get<1>(uid_kid);
    	const uint64 kmer_key = sorted_kmer_keys[kid];
    	const uint32 rid = sorted_coords[kid].w;

    	int32 idx = kid;
    	while(idx >= 0) {
    		if(sorted_coords[idx].w != rid || sorted_kmer_keys[idx] != kmer_key) break;
    		unique_id_map[idx] = uid;
    		idx--; //TODO: thrust documentation states first, but last in experiments
    	}
    }
};

//struct populate_unique_kmer_data
//{
//	const uint64* sorted_kmer_keys;
//	const SequenceSetKmerCoord* sorted_coords;
//	const uint32* unique_kmer_idxs;
//
//	// fill in unique kmer annotations
//	uint8* unique_flag_map;
//	uint32* unique_UID_map;
//
//	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
//	populate_unique_kmer_data(const uint64* _sorted_kmer_keys, const SequenceSetKmerCoord* _coords,
//			const uint32* _unique_kmer_idxs, uint8* _unique_flag_map, uint32* _unique_UID_map) :
//	sorted_kmer_keys(_sorted_kmer_keys), sorted_coords(_coords), unique_kmer_idxs(_unique_kmer_idxs),
//	unique_flag_map(_unique_flag_map), unique_UID_map (_unique_UID_map) { }
//
//    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
//    void operator() (const uint32 uid) const
//    {
//    	const uint32 unique_kmer_idx = unique_kmer_idxs[uid];
//    	const uint64 key = sorted_kmer_keys[unique_kmer_idx]; // find the group of coordinates with this key and region
//    	const uint32 region = sorted_coords[unique_kmer_idx].w;
//    	uint32 global_kmer_id = sorted_coords[unique_kmer_idx].z;
//    	unique_flag_map[global_kmer_id] = 1;
//    	unique_UID_map[global_kmer_id] = uid;
//
//    	int32 idx = unique_kmer_idx - 1;
//    	while(idx >= 0) {
//    		if(sorted_kmer_keys[idx] != key || sorted_coords[idx].w != region) break;
//    		global_kmer_id = sorted_coords[idx].z;
//    		unique_flag_map[global_kmer_id] = 1; //TODO: thrust documentation states first, but last in experiments
//    		unique_UID_map[global_kmer_id] = uid;
//    		idx--;
//    	}
//    }
//};

struct populate_unique_kmer_data
{
	const uint32* kmer_counts;
	const SequenceSetKmerCoord* sorted_coords;
	const uint32* unique_kmer_idxs;

	// fill in unique kmer annotations
	uint8* unique_flag_map;
	uint32* unique_UID_map;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	populate_unique_kmer_data(const uint32* _kmer_counts, const SequenceSetKmerCoord* _coords,
			const uint32* _unique_kmer_idxs, uint8* _unique_flag_map, uint32* _unique_UID_map) :
	kmer_counts(_kmer_counts), sorted_coords(_coords), unique_kmer_idxs(_unique_kmer_idxs),
	unique_flag_map(_unique_flag_map), unique_UID_map (_unique_UID_map) { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 uid) const
    {
    	const uint32 unique_kmer_idx = unique_kmer_idxs[uid];
    	const uint32 count = kmer_counts[uid];
    	uint32 global_kmer_id = sorted_coords[unique_kmer_idx].z;
    	unique_flag_map[global_kmer_id] = 1;
    	unique_UID_map[global_kmer_id] = uid;

    	for(uint32 i = 1; i < count; i++) {
    		global_kmer_id = sorted_coords[unique_kmer_idx - i].z; //TODO: thrust bug: pointer to last instead of first
    		unique_flag_map[global_kmer_id] = 1; // TODO: not coalesced
    		unique_UID_map[global_kmer_id] = uid;
    	}
    }
};

// marks kmers that have a prefix satisfying the following:
// the prefix is unique and has a matching kmer
// (i.e. it is not the last (k-1)mer in the sequence)
struct mark_unique_prefix_kmer
{
	uint8* unique_prefix_map;
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	mark_unique_prefix_kmer(uint8* _unique_prefix_map) : unique_prefix_map(_unique_prefix_map) { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const thrust::tuple<uint8, uint64>& uflag_prefix) const
    {
    	const uint8 is_unique_prefix = thrust::get<0>(uflag_prefix);
    	const uint32 kmer_id = thrust::get<1>(uflag_prefix);

    	if((is_unique_prefix == 1) && (kmer_id != (uint32) -1)) {
    		unique_prefix_map[kmer_id] = 1; // TODO: writes not coalesced
    	}
    }
};

// ---------- Super-kmers: chains of consecutive kmers

// given a non-unique kmer N, finds the closest unique predecessor and successor kmers in the sequence, P and S
// returns the new coordinate for the chain starting with the P and ending with S [P...N...S]
// returns a dummy coordinate (N.x, -1, -1, 0) if no such previous unique kmer exists
// TODO: could use shared memory here
template <typename string_set_type>
struct extract_super_kmers_functor
{
	typedef typename string_set_type::string_type sequence;
	const string_set_type   string_set;
	const uint32* repeat_global_ids; // global ids of the repeat kmers
	const uint8* global_unique_flag_map; // unsorted
	const uint32* global_UID_map;
	const uint32* global_sorted_idx_map;
	const SequenceSetKmerCoord* coords; // sorted
	const uint32 kmer_size;

	// structures to populate
	SequenceSetKmerCoord* super_coords;
	uint32* prefix_uids;
	uint32* suffix_uids;

	NVBIO_HOST_DEVICE
	extract_super_kmers_functor(const uint32* _repeat_global_ids,
			const uint8* _is_unique_map, const uint32* _global_UID_map, const uint32* _sorted_idx_map,
			const SequenceSetKmerCoord* _coords, const string_set_type _string_set, const uint32 _kmer_size,
			SequenceSetKmerCoord* _super_coords, uint32* _prefix_uids, uint32* _suffix_uids):
				repeat_global_ids(_repeat_global_ids), global_unique_flag_map(_is_unique_map), global_UID_map(_global_UID_map),
				global_sorted_idx_map(_sorted_idx_map),
				coords(_coords), string_set(_string_set), kmer_size(_kmer_size),
				super_coords(_super_coords), prefix_uids(_prefix_uids), suffix_uids(_suffix_uids){ }

	NVBIO_HOST_DEVICE
	void operator()(const uint32 idx) const {
		const uint32 global_id = repeat_global_ids[idx];
		const SequenceSetKmerCoord coord = coords[global_sorted_idx_map[global_id]];
		const sequence seq = string_set[coord.x];
		const uint32 seq_len = seq.length();
		SequenceSetKmerCoord super_coord = make_uint4(coord.x, (uint32)-1, (uint32)-1, coord.w);
		uint32 prefix_id = (uint32) -1;
		uint32 suffix_id = (uint32) -1;

		// find the unique predecessor
		uint32 i = 1u;
		while(i <= coord.y) { // search only this sequence
			if(global_unique_flag_map[global_id-i] == 1) {
				super_coord.y = coord.y - i;
				super_coord.z = coord.y + kmer_size;
				prefix_id = global_UID_map[global_id-i];
				break;
			}
			i++;
		}

		if(super_coord.y != (uint32) -1) { // otherwise this chain will be ignored
			// find the unique successor
			i = 1u;
			while(i < seq_len - coord.y - kmer_size + 1) { // search only until the end of the sequence
				if(global_unique_flag_map[global_id+i] == 1) {
					super_coord.z = coord.y + i + kmer_size;
					suffix_id = global_UID_map[global_id+i];
					break;
				}
				i++;
			}
		}

		super_coords[idx] = super_coord;
		prefix_uids[idx] = prefix_id;
		suffix_uids[idx] = suffix_id;
	}
};

template <typename string_set_type>
struct print_super_kmer_functor
{
	typedef typename string_set_type::string_type sequence;
	const string_set_type   string_set;
	const SequenceSetKmerCoord* coords;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    print_super_kmer_functor(const string_set_type _string_set, const SequenceSetKmerCoord* _coords) :
    	coords(_coords), string_set(_string_set) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 kmer_idx) const
    {
    	SequenceSetKmerCoord kmer_coord = coords[kmer_idx];
    	const uint32 kmer_pos = kmer_coord.y;
    	const uint32 kmer_size = kmer_coord.z - kmer_coord.y;
    	const sequence seq = string_set[kmer_coord.x];
    	const uint32 seq_len = seq.length();

    	uint8 kmer_seq[14];
    	for (uint32 k = 0; k < kmer_size; k++) {
			uint8 c = iupac16_to_char(char_to_iupac16(dna_to_char(seq[kmer_pos + k])));
			kmer_seq[k] = c;
		}
    	kmer_seq[kmer_size] = '\0';

    	printf("id: %llu, reg %u, seq %s \n", kmer_idx, kmer_coord.w, kmer_seq);
    }
};

// returns true if the super kmer does not represent a valid chain of kmers
// based on the value of its unique predecessor id
struct is_invalid_super_kmer
{
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	bool operator()(const uint32 chain_prefix_id)
	{
		return chain_prefix_id == (uint32) -1;
	}
};

struct is_unique_suffix_id : public thrust::unary_function<uint32,bool>
{
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	bool operator()(const uint32 suffix_id)
	{
		return suffix_id != (uint32) -1;
	}
};

struct is_unique_uid : public thrust::unary_function<uint32,uint8>
{
	const uint32 max_unique_uid;
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	is_unique_uid(const uint32 _max_unique_uid) : max_unique_uid(_max_unique_uid) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	uint8 operator()(const uint32 uid)
	{
		return uid < max_unique_uid;
	}
};

struct is_repeat_kmer
{
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	bool operator()(const uint8 flag)
	{
		return (flag != 1);
	}
};

// collapse kmers that start at the same position and belong to the same sequence
// by just returning the one with max length
struct collapse_same_start_kmers
{
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	thrust::tuple<SequenceSetKmerCoord, uint32, uint32> operator() (
			const thrust::tuple<SequenceSetKmerCoord, uint32, uint32>& x,
			const thrust::tuple<SequenceSetKmerCoord, uint32, uint32>& y) const
    {
		const SequenceSetKmerCoord i = thrust::get<0>(x);
		const SequenceSetKmerCoord j = thrust::get<0>(y);

		if(i.z > j.z) {
			return x;
		}
		return y;
    }
};

// computes the max prefix overlap with the closest lexicographically smaller super-kmer
// assumes that the function is not called for idx == 0
// returns the number of kmers by which the chains overlap
// no overlap if the previous kmer is from a different region
template <typename string_set_type>
struct find_max_kmer_overlaps
{
	typedef typename string_set_type::string_type sequence;
	const string_set_type   string_set;
	const SequenceSetKmerCoord* coords; // sorted in lexicographic order
	const uint32 kmer_size;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	find_max_kmer_overlaps(const string_set_type _string_set, const SequenceSetKmerCoord* _coords,
			const uint32 _kmer_size): string_set(_string_set), coords(_coords), kmer_size(_kmer_size) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	uint32 operator() (const uint32 idx) const
	{
		const SequenceSetKmerCoord coord_curr = coords[idx];
		const SequenceSetKmerCoord coord_prev = coords[idx-1];

		if(coord_curr.w != coord_prev.w) return 0;

		const sequence seq_curr = string_set[coord_curr.x];
		const sequence seq_prev = string_set[coord_prev.x];
		const uint32 max_overlap =
				(coord_curr.z - coord_curr.y) > (coord_prev.z - coord_prev.y) ?
				(coord_prev.z - coord_prev.y) : (coord_curr.z - coord_curr.y);

		uint32 overlap = 0u;
		for (uint32 j = 0; j < max_overlap; j++)
		{
			if(seq_curr[coord_curr.y + j] == seq_prev[coord_prev.y +j]) {
				overlap++;
			} else {
				break;
			}
		}
		uint32 n_ovp = (overlap >= kmer_size) ? overlap - kmer_size + 1 : 0;
		return n_ovp;
	}
};

// returns the number of kmers that did not overlap with the lexicographic
// super-kmer predecessor and are not unique
// i.e. total number of kmers in the chain - number of overlapping kmers
// does not count the unique first and last kmers
struct num_non_overlapped_repeats : public thrust::unary_function<uint32,uint32>
{
	const SequenceSetKmerCoord* coords;
	const uint32* suffix_kmer_ids;
	const uint32* overlaps;
	const uint32 kmer_size;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	num_non_overlapped_repeats(const SequenceSetKmerCoord* _coords,
			const uint32* _suffix_kmer_ids,
			const uint32* _overlaps,
			const uint32 _kmer_size):
		coords(_coords), suffix_kmer_ids(_suffix_kmer_ids), overlaps(_overlaps), kmer_size(_kmer_size) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	uint32 operator() (const uint32 idx) const
	{
		const SequenceSetKmerCoord coord = coords[idx];
		uint32 len = coord.z - coord.y;
		const uint32 n_kmers = (len >= kmer_size) ? len - kmer_size + 1 : 0;

		uint32 n_unique = 0;
		if(overlaps[idx] == 0) {
			n_unique++; // skip the header unique kmer if this is the first chain starting with this kmer
						// otherwise, it will be included in the overlap
		}
		if(overlaps[idx] != n_kmers && suffix_kmer_ids[idx] != (uint32) -1) { // if not all the kmers overlapped
			n_unique++; // skip the last unique kmer
		}
		return n_kmers - overlaps[idx] - n_unique;
	}
};

struct num_adj_repeats : public thrust::unary_function<uint32,uint32>
{
	const SequenceSetKmerCoord* coords;
	const uint32* suffix_kmer_ids;
	const uint32* overlaps;
	const uint32 kmer_size;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	num_adj_repeats(const SequenceSetKmerCoord* _coords,
			const uint32* _suffix_kmer_ids,
			const uint32* _overlaps,
			const uint32 _kmer_size):
		coords(_coords), suffix_kmer_ids(_suffix_kmer_ids), overlaps(_overlaps), kmer_size(_kmer_size) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	uint32 operator() (const uint32 idx) const
	{
		const SequenceSetKmerCoord coord = coords[idx];
		uint32 len = coord.z - coord.y;
		const uint32 n_kmers = (len >= kmer_size) ? len - kmer_size + 1 : 0; // TODO: remove len check

		uint32 n_repeat_kmers_to_extract = n_kmers - overlaps[idx];
		if(overlaps[idx] == 0) {
			n_repeat_kmers_to_extract--; // skip the first unique kmer
		}
		if(overlaps[idx] != n_kmers && suffix_kmer_ids[idx] != (uint32) -1) { // if not all the kmers overlapped
			n_repeat_kmers_to_extract--; // skip the last unique kmer
		}

		// number of kmers to extract will be the same as number of edges, plus RU edge if exists
		uint32 n_adj = n_repeat_kmers_to_extract;
		if(overlaps[idx] != n_kmers && suffix_kmer_ids[idx] != (uint32) -1) { // if not all the kmers overlapped
			n_adj++; // add the last R-U edge
		}

		return n_adj;
	}
};

template <typename string_set_type>
struct extract_non_overlapped_repeats
{
	typedef typename string_set_type::string_type sequence;
	const string_set_type   string_set;
	const SequenceSetKmerCoord* super_coords;
	const uint32* super_prefix_global_ids;
	const uint32* super_suffix_global_ids;
	const uint32* chain_offsets;
	const uint32* adj_extraction_offsets;
	const uint32* chain_overlaps;
	const uint32 kmer_size;
	const uint32 uid_offset;
	const uniform_seeds_functor<> seeder;

	SequenceSetKmerCoord* kmers_out;
	uint32* prefix_uids;
	uint32* suffix_uids;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	extract_non_overlapped_repeats(const string_set_type _string_set,
			const SequenceSetKmerCoord* _chain_coords,
			const uint32* _chain_pref_gids, const uint32* _chain_suffix_gids,
			const uint32* _chain_offsets,
			const uint32* _adj_extraction_offsets,
			const uint32* _chain_overlaps,
			const uint32 _kmer_size, const uint32 _uid_offset,
			SequenceSetKmerCoord* _kmers_out, uint32* _prefix_uids, uint32* _suffix_uids):
		string_set(_string_set),
		super_coords(_chain_coords),
		super_prefix_global_ids(_chain_pref_gids),
		super_suffix_global_ids(_chain_suffix_gids),
		chain_offsets(_chain_offsets),
		adj_extraction_offsets(_adj_extraction_offsets),
		chain_overlaps(_chain_overlaps),
		kmer_size(_kmer_size), uid_offset(_uid_offset),
		kmers_out(_kmers_out),
		prefix_uids(_prefix_uids), suffix_uids(_suffix_uids),
		seeder(_kmer_size, 1u) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	void operator() (const uint32 idx) const
	{
		const SequenceSetKmerCoord chain_coord = super_coords[idx];
		const sequence seq = string_set[chain_coord.x];
		const uint32 seq_len = seq.length();
		const uint32 offset = chain_offsets[idx] + uid_offset;
		const uint32 adj_offset = adj_extraction_offsets[idx];

		const uint8 unique_suffix = (super_suffix_global_ids[idx] != (uint32) -1);
		const uint32 n_kmers_to_extract = chain_offsets[idx+1] - chain_offsets[idx];

		uint32 len = chain_coord.z - chain_coord.y;
		const uint32 n_kmers = (len >= kmer_size) ? len - kmer_size + 1 : 0; // TODO: remove len check

		// if everything overlapped or nothing to extract and no possible R-U edge
		if(chain_overlaps[idx] == n_kmers || (n_kmers_to_extract + unique_suffix == 0)) return;

		const uint32 kmer_id = (chain_coord.z - unique_suffix) - n_kmers_to_extract - kmer_size + 1;
		for (uint32 j = 0; j < n_kmers_to_extract; j++) {
			const uint2 kmer = seeder.seed(seq_len, kmer_id + j);
			kmers_out[offset + j] = make_uint4(chain_coord.x, kmer.x, kmer.y, chain_coord.w);

			suffix_uids[adj_offset + j] = offset + j;
			if(j < n_kmers_to_extract - 1) {
				prefix_uids[adj_offset + j + 1] = offset + j;
			} else {
				if(unique_suffix == 1) {
					prefix_uids[adj_offset + j + 1] = offset + j; // the last kmer is a prefix only to the RU (k+1)mer
				}
			}
		}
		if(unique_suffix == 1) {
			suffix_uids[adj_offset + n_kmers_to_extract] = super_suffix_global_ids[idx];
		}

		// find the first prefix
		if(chain_overlaps[idx] == 0 || chain_overlaps[idx] == 1) {
			// overlap is just the unique header kmer or none
			// use the unique id of the kmer
			prefix_uids[adj_offset] = super_prefix_global_ids[idx];
		} else {
			uint32 pred_chain_idx = idx - 1;
			while(1) {
				if(chain_overlaps[pred_chain_idx] >= chain_overlaps[idx]) { // not stored by the previous chain // >=2
					pred_chain_idx--;
				} else {
					// ignores the unique predecessor that overlapped and was not extracted
					prefix_uids[adj_offset] = uid_offset + chain_offsets[pred_chain_idx] + (chain_overlaps[idx] - chain_overlaps[pred_chain_idx]) - 2; // last overlapped kmer
					break;
				}
			}
		}
	}
};

struct count_overlapped_adjacencies
{
	const uint32* adj_extraction_offsets;
	const uint32* overlaps;
	uint32* counts;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	count_overlapped_adjacencies(const uint32* _adj_extraction_offsets, const uint32* _overlaps, uint32* _counts):
		adj_extraction_offsets(_adj_extraction_offsets),
		overlaps(_overlaps),
		counts(_counts) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	void operator() (const uint32 idx) const
	{
		int32 n_to_update = overlaps[idx] - 1;
		if(n_to_update <= 0) return;

		uint32 pred_chain_idx = idx - 1;
		while(n_to_update > 0) {
			const int32 pred_overlap = overlaps[pred_chain_idx] - 1;
			if(pred_overlap >= n_to_update) { // everything is shared with the pred
				pred_chain_idx--;
			} else {
				// update the counts in this pred chain
				const uint32 n_update_pred = n_to_update - (pred_overlap > 0 ? pred_overlap : 0);
				const uint32 pred_offset = adj_extraction_offsets[pred_chain_idx];
				for(uint32 i = 0; i < n_update_pred; i++) {
					nvbio::atomic_add(&counts[pred_offset + i], 1);
				}
				n_to_update -= n_update_pred;
			}
		}
	}
};

struct extract_repeat_adjacencies
{
	const uint32* node_offsets;
	const uint32* prefix_uids;
	const uint32* suffix_uids;
	const uint32* edge_counts;
	uint32* node_adj_map;
	uint32* edge_counts_shuffled;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	extract_repeat_adjacencies(const uint32* _node_offsets, const uint32* _prefix_uids,
			const uint32* _suffix_uids, const uint32* _edge_counts,
			uint32* _node_adj_map, uint32* _edge_counts_shuffled):
		node_offsets(_node_offsets),
		prefix_uids(_prefix_uids),
		suffix_uids(_suffix_uids),
		edge_counts(_edge_counts),
		node_adj_map(_node_adj_map),
		edge_counts_shuffled(_edge_counts_shuffled){ }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	void operator() (const uint32 prefix_idx) const
	{
		const uint32 from_uid = prefix_uids[prefix_idx];
		const uint32 offset = node_offsets[from_uid];
		const uint32 num_edges = node_offsets[from_uid+1] - node_offsets[from_uid];

		uint32 i = 0;
		while(i < num_edges) { // at most this many prefixes to handle
			if(prefix_idx < i) break;
			const uint32 uid = prefix_uids[prefix_idx - i];
			if(uid != from_uid) break; // handled all the prefixes with the given UID
			node_adj_map[offset + i] = suffix_uids[prefix_idx - i];
			edge_counts_shuffled[offset + i] = edge_counts[prefix_idx - i];
			i++;
		}
	}
};

struct extract_unique_adjacencies
{
	const uint32* node_offsets;
	const uint32* prefix_uids;
	const uint32* suffix_uids;
	const uint32* edge_counts;
	uint32* node_adj_map;
	uint32* edge_counts_shuffled;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	extract_unique_adjacencies(const uint32* _node_offsets, const uint32* _prefix_uids,
			const uint32* _suffix_uids, const uint32* _edge_counts,
			uint32* _node_adj_map, uint32* _edge_counts_shuffled):
		node_offsets(_node_offsets),
		prefix_uids(_prefix_uids),
		suffix_uids(_suffix_uids),
		edge_counts(_edge_counts),
		node_adj_map(_node_adj_map),
		edge_counts_shuffled(_edge_counts_shuffled){ }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	void operator() (const uint32 prefix_idx) const
	{
		const uint32 from_uid = prefix_uids[prefix_idx];
		const uint32 offset_last = node_offsets[from_uid+1] - 1;
		const uint32 num_edges = node_offsets[from_uid+1] - node_offsets[from_uid];

		uint32 i = 0;
		uint32 n_unique = 0;
		while(i < num_edges) { // at most this many prefixes to handle
			if(prefix_idx < i) break;
			const uint32 uid = prefix_uids[prefix_idx - i];
			if(uid != from_uid) break; // handled all the prefixes with the given UID
			if(suffix_uids[prefix_idx - i] == (uint32) -1) { // repeat (TODO: fill with -1)
				i++;
				continue;
			}
			node_adj_map[offset_last - n_unique] = suffix_uids[prefix_idx - i]; // fill out from the end
			edge_counts_shuffled[offset_last - n_unique] = edge_counts[prefix_idx - i];
			i++;
			n_unique++;
		}
	}
};

// converts a coordinate to the corresponding sequence -- used for debugging
template <typename string_set_type>
struct super_coord_to_seq
{
	typedef typename string_set_type::string_type sequence;
	const string_set_type   string_set;
	const SequenceSetKmerCoord* super_coords;
	const uint32* offsets;
	uint8* sequences;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	super_coord_to_seq(const string_set_type _string_set,
			const SequenceSetKmerCoord* _super_coords, const uint32* _offsets,
			uint8* _sequences) :
		string_set(_string_set), super_coords(_super_coords), offsets(_offsets),
		sequences(_sequences) {}

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	void operator() (const uint64 idx) const
	{
		const SequenceSetKmerCoord coord = super_coords[idx];
		const sequence seq = string_set[coord.x];
		const uint32 seq_pos = coord.y;
		const uint32 offset = offsets[idx];

		for (uint32 i = 0; i < coord.z - coord.y; i++) {
			uint8 c = dna_to_char(seq[seq_pos + i]);
			sequences[offset + i] = c;
		}
	}
};

// generate the coordinates of each kmer in the given set
template <typename string_set_type>
void D_KmerSet<string_set_type>::gen_kmer_coords()
{
	n_kmers = enumerate_string_set_seeds(
			string_set,
			uniform_seeds_functor<>(kmer_size, 1u),
			coords);

	// record the index into the kmer array in the coordinates vector
	thrust::for_each(
			thrust::make_counting_iterator<uint32>(0u),
			thrust::make_counting_iterator<uint32>(0u) + n_kmers,
			set_global_id_region_id(plain_view(coords), plain_view(active_region_ids)));
}

// generate the 64-bit compacted kmer sequence keys
// assumes the coordinates have already been generated
template <typename string_set_type>
void D_KmerSet<string_set_type>::gen_kmer_64b_keys()
{
	kmers_64b.resize(n_kmers);
	thrust::transform(
			coords.begin(),
			coords.begin() + n_kmers,
			kmers_64b.begin(),
			kmers_to_64b_functor<string_set_type>(kmer_size, 2, string_set));
}

// sort the kmers based on the 64-bit kmer sequence key only
// assumes the keys have already been generated
template <typename string_set_type>
void D_KmerSet<string_set_type>::sort_kmers_by_64b_keys()
{
	// identical keys from the same sequence will be consecutive
	// identical keys from the same region will also be consecutive
	thrust::stable_sort_by_key(
				kmers_64b.begin(),
				kmers_64b.begin() + n_kmers,
				coords.begin());
}

// stable sort the kmers based on the 64-bit kmer sequence key
// assumes the kmers are sorted by seq id only already
// assumes the keys have already been generated
template <typename string_set_type>
template <typename meta_iterator_type>
void D_KmerSet<string_set_type>::sort_kmers_by_64b_keys_meta(const meta_iterator_type meta_data)
{
	thrust::stable_sort_by_key(
			kmers_64b.begin(),
			kmers_64b.begin() + n_kmers,
			thrust::make_zip_iterator(thrust::make_tuple(coords.begin(), meta_data)));
}

// sort the kmers based on the 64-bit kmer sequence key by region
// assumes the keys have already been generated
template <typename string_set_type>
void D_KmerSet<string_set_type>::segmented_sort_kmers_by_64b_keys()
{
	// identical keys from the same sequence will be consecutive
	// identical keys from the same region will also be consecutive
	thrust::stable_sort_by_key(
				kmers_64b.begin(),
				kmers_64b.begin() + n_kmers,
				coords.begin());

	D_VectorU32::iterator region_ids = get_scratch_space(n_kmers);
	thrust::transform(
			coords.begin(),
			coords.begin() + n_kmers,
			region_ids,
			get_kmer_reg_id());

	thrust::stable_sort_by_key(
			region_ids,
			region_ids + n_kmers,
			thrust::make_zip_iterator(thrust::make_tuple(coords.begin(), kmers_64b.begin())));

	reset_scratch_space();
}

// sort kmers by key and set sequence id
// doing 2-pass sort to use the much faster thrust radix-sort
// instead of merge-sort on custom keys
template <typename string_set_type>
void D_KmerSet<string_set_type>::sort_kmers_by_64b_keys_seqid()
{
	thrust::stable_sort_by_key(
			kmers_64b.begin(),
			kmers_64b.begin() + n_kmers,
			coords.begin());
	D_VectorU32 seq_ids(n_kmers);
	thrust::transform(
			coords.begin(),
			coords.begin() + n_kmers,
			seq_ids.begin(),
			get_kmer_seq_id());
	thrust::stable_sort_by_key(
			seq_ids.begin(),
			seq_ids.begin() + n_kmers,
			thrust::make_zip_iterator(thrust::make_tuple(kmers_64b.begin(), coords.begin())));
}

// sort kmers by key and set sequence id
// sort the additional meta-data along with the kmers
// doing 2-pass sort to use the much faster thrust radix-sort
// instead of merge-sort on custom keys
template <typename string_set_type>
template <typename meta_iterator_type>
void D_KmerSet<string_set_type>::sort_kmers_by_64b_keys_seqid_meta(const meta_iterator_type meta_data)
{
	thrust::stable_sort_by_key(
			kmers_64b.begin(),
			kmers_64b.begin() + n_kmers,
			thrust::make_zip_iterator(thrust::make_tuple(coords.begin(), meta_data)));
	D_VectorU32 seq_ids(n_kmers);
	thrust::transform(
			coords.begin(),
			coords.begin() + n_kmers,
			seq_ids.begin(),
			get_kmer_seq_id());
	thrust::stable_sort_by_key(
			seq_ids.begin(),
			seq_ids.begin() + n_kmers,
			thrust::make_zip_iterator(thrust::make_tuple(kmers_64b.begin(), coords.begin(), meta_data)));
}

// sort kmers in lexicographic order - sorts the indices and returns the iterator
// kmer length can vary
// if kmer_size is set, all kmers are assumed to have this size
template <typename string_set_type>
D_VectorU32::iterator sort_kmers_lexicographic(const string_set_type string_set,
		D_VectorSetKmerCoord& coords,
		const uint32 n_kmers, const uint32 fixed_kmer_size,
		D_VectorU32& indices)
{
	const uint32 WORD_BITS = uint32(8u * sizeof(uint32));
	const uint32 SYMBOL_SIZE = 4u;
	const uint32 SYMBOLS_PER_WORD = WORD_BITS/SYMBOL_SIZE;

	// maximum size of a kmer in the set
	uint32 max_length = fixed_kmer_size;
	if(fixed_kmer_size == 0u) {
		max_length = thrust::reduce(
	            thrust::make_transform_iterator(coords.begin(), get_kmer_size()),
	            thrust::make_transform_iterator(coords.begin() + n_kmers, get_kmer_size()),
	            0u,
	            thrust::maximum<uint32>());
	}

	// maximum number of words needed to represent a kmer
	const uint32 max_words = (max_length + SYMBOLS_PER_WORD-1)/SYMBOLS_PER_WORD;

	// ----- LSD radix-sort (word by word) -----
	//uint32 n_kmers = coords.size();
	D_VectorU32 radices(2*n_kmers); // kmer words (sort keys)
	indices.resize(2*n_kmers); // kmer ids (values to be sorted)
	thrust::copy(
			thrust::make_counting_iterator<uint32>(0u),
			thrust::make_counting_iterator<uint32>(n_kmers),
	        indices.begin());

	// ping-pong buffers
	cuda::SortBuffers<uint32*,uint32*> sort_buffers;
	cuda::SortEnactor sort_enactor;
	sort_buffers.selector = 0;
	sort_buffers.keys[0] = nvbio::device_view(radices);
	sort_buffers.keys[1] = nvbio::device_view(radices) + n_kmers;
	sort_buffers.values[0] = nvbio::device_view(indices);
	sort_buffers.values[1] = nvbio::device_view(indices) + n_kmers;

	// sort in LSD order
	for (int32 word_idx = max_words-1; word_idx >= 0; --word_idx) {
		// extract the given radix word from each kmer
		thrust::transform(
				indices.begin() + sort_buffers.selector * n_kmers,
				indices.begin() + sort_buffers.selector * n_kmers + n_kmers,
				radices.begin() + sort_buffers.selector * n_kmers,
				kmer_word_extractor_functor<string_set_type>(
		                string_set,
		                SYMBOL_SIZE,
		                plain_view(coords),
		                word_idx));
		// sort the words
		sort_enactor.sort(n_kmers, sort_buffers);
	}

	return indices.begin() + sort_buffers.selector * n_kmers;
}

template <typename string_set_type>
void segmented_sort_super_kmers_lexicographic(const string_set_type string_set, uint32 n_coords,
		D_VectorSetKmerCoord& super_coords, D_VectorU32& super_prefix_ids, D_VectorU32& super_suffix_ids)
{
	D_VectorU32 ids(n_coords);
	D_VectorSetKmerCoord temp(super_coords);
	D_VectorU32 temp_pref(super_prefix_ids);
	D_VectorU32 temp_suf(super_suffix_ids);

	// first pass: sort by coord key
	D_VectorU32::iterator sorted_ids = sort_kmers_lexicographic(string_set, super_coords, n_coords, 0u, ids);
	thrust::gather(
			sorted_ids,
			sorted_ids + n_coords,
			temp.begin(),
			super_coords.begin());
	thrust::gather(
			sorted_ids,
			sorted_ids + n_coords,
			thrust::make_zip_iterator(thrust::make_tuple(temp_pref.begin(), temp_suf.begin())),
			thrust::make_zip_iterator(thrust::make_tuple(super_prefix_ids.begin(), super_suffix_ids.begin())));

	// second pass: stable sort by region id => as a result coord keys will be sorted by region
	//D_VectorU32 reg_ids(n_coords);
	thrust::transform(
			super_coords.begin(),
			super_coords.begin() + n_coords,
			ids.begin(),
			get_kmer_reg_id());

	thrust::stable_sort_by_key(
			ids.begin(),
			ids.begin() + n_coords,
			thrust::make_zip_iterator(thrust::make_tuple(super_coords.begin(), super_prefix_ids.begin(), super_suffix_ids.begin())));
}

// count the kmer occurrences across all the sequences in the set
// assumes that the keys have been sorted
template <typename string_set_type>
void D_KmerSet<string_set_type>::count_kmers_rle()
{
	// count the number of distinct keys and extract them
	kmers_64b_distinct.resize(n_kmers);
	kmers_counts.resize(n_kmers);
	D_VectorU8 d_temp_storage;
	n_distinct = cuda::runlength_encode(
			n_kmers,
			kmers_64b.begin(),
			kmers_64b_distinct.begin(),
			kmers_counts.begin(),
			d_temp_storage);
}

// count the kmer occurrences across all the sequences in the set
// assumes that the keys have been sorted
template <typename string_set_type>
void D_KmerSet<string_set_type>::count_kmers()
{
	// count the number of distinct keys and extract them
	kmers_64b_unique_idxs.resize(n_kmers);
	kmers_counts.resize(n_kmers);
	n_unique = thrust::reduce_by_key(
			thrust::counting_iterator<uint32>(0),
			thrust::counting_iterator<uint32>(0) + n_kmers,
			thrust::constant_iterator<uint32>(1),
			kmers_64b_unique_idxs.begin(),
			kmers_counts.begin(),
			kmer_key_rid_eq(plain_view(kmers_64b), plain_view(coords))).first - kmers_64b_unique_idxs.begin();
}

// partition kmers by index into the sorted kmer key set
// based on whether they are unique or not
// a kmer is not unique if it occurs more than once in at least one sequence
template <typename string_set_type>
void D_KmerSet<string_set_type>::partition_kmers_by_uniqueness()
{
	// count the number of distinct kmers per sequence
	D_VectorU32::iterator distinct_idxs_per_seq = get_scratch_space(n_kmers);
	D_VectorU32::iterator count_per_seq = get_scratch_space(n_kmers);
	thrust::counting_iterator<uint64> ids(0);
	uint64 n_distinct_per_seq = thrust::reduce_by_key(
			ids,
			ids + n_kmers,
			thrust::constant_iterator<uint32>(1),
			distinct_idxs_per_seq,
			count_per_seq,
			kmer_key_sid_eq(plain_view(kmers_64b), plain_view(coords))).first - distinct_idxs_per_seq;

	D_VectorU32::iterator distinct_idxs = get_scratch_space(n_distinct_per_seq);
	D_VectorU32::iterator seq_count = get_scratch_space(n_distinct_per_seq); // number of sequences containing the same kmer
	D_VectorU32::iterator kmer_counts = get_scratch_space(n_distinct_per_seq); // number of kmer occurrences across all sequences
	n_distinct = thrust::reduce_by_key(
			distinct_idxs_per_seq,
			distinct_idxs_per_seq + n_distinct_per_seq,
			thrust::make_zip_iterator(thrust::make_tuple(thrust::constant_iterator<uint32>(1), count_per_seq)),
			distinct_idxs,
			thrust::make_zip_iterator(thrust::make_tuple(seq_count, kmer_counts)),
			kmer_key_rid_eq(plain_view(kmers_64b), plain_view(coords)),
			kmer_count_tuple_sum()).first - distinct_idxs;

	// partition the distinct indices into unique and non-unique kmers
	kmers_64b_unique_idxs.resize(n_distinct);
	kmers_counts.resize(n_distinct);
	n_unique =	thrust::copy_if(
			thrust::make_zip_iterator(thrust::make_tuple(distinct_idxs, kmer_counts)),
			thrust::make_zip_iterator(thrust::make_tuple(distinct_idxs + n_distinct, kmer_counts + n_distinct)),
			thrust::make_zip_iterator(thrust::make_tuple(seq_count, kmer_counts)),
			thrust::make_zip_iterator(thrust::make_tuple(kmers_64b_unique_idxs.begin(), kmers_counts.begin())),
			is_unique_kmer()) - thrust::make_zip_iterator(thrust::make_tuple(kmers_64b_unique_idxs.begin(), kmers_counts.begin()));

	reset_scratch_space();
}

template <typename string_set_type>
void D_KmerSet<string_set_type>::gen_global_unique_map()
{
	D_VectorU8 unique_map_sorted(n_kmers);
	thrust::for_each(
				kmers_64b_unique_idxs.begin(),
				kmers_64b_unique_idxs.begin() + n_unique,
				mark_kmer_uniqueness(plain_view(unique_map_sorted), plain_view(kmers_64b), plain_view(coords)));

	global_unique_flag_map.resize(n_kmers);
	thrust::gather(
				global_to_sorted_id_map.begin(),
				global_to_sorted_id_map.begin() + n_kmers,
				unique_map_sorted.begin(),
				global_unique_flag_map.begin());
}

template <typename string_set_type>
void D_KmerSet<string_set_type>::gen_global_UID_map()
{
	D_VectorU32 id_map_sorted(n_kmers);
	thrust::fill(id_map_sorted.begin(), id_map_sorted.begin() + n_kmers, (uint32) -1);
	thrust::for_each(
				thrust::make_zip_iterator(thrust::make_tuple(thrust::make_counting_iterator<uint32>(0u), kmers_64b_unique_idxs.begin())),
				thrust::make_zip_iterator(thrust::make_tuple(thrust::make_counting_iterator<uint32>(0u) + n_unique, kmers_64b_unique_idxs.begin() + n_unique)),
				store_kmer_unique_ids(plain_view(id_map_sorted), plain_view(kmers_64b), plain_view(coords)));

	global_to_UID_map.resize(n_kmers);
	thrust::gather(
				global_to_sorted_id_map.begin(),
				global_to_sorted_id_map.begin() + n_kmers,
				id_map_sorted.begin(),
				global_to_UID_map.begin());
}

template <typename string_set_type>
void D_KmerSet<string_set_type>::gen_global_to_sorted_id_map()
{
	global_to_sorted_id_map.resize(n_kmers);
	thrust::for_each(
			thrust::make_zip_iterator(thrust::make_tuple(thrust::make_counting_iterator<uint32>(0u), coords.begin())),
			thrust::make_zip_iterator(thrust::make_tuple(thrust::make_counting_iterator<uint32>(0u) + n_kmers, coords.begin() + n_kmers)),
			global_to_sorted_id(plain_view(global_to_sorted_id_map)));
}

template <typename string_set_type>
void D_KmerSet<string_set_type>::mark_unique_kmers()
{
	global_unique_flag_map.resize(n_kmers);
	global_to_UID_map.resize(n_kmers);
	thrust::fill(global_to_UID_map.begin(), global_to_UID_map.begin() + n_kmers, (uint32) -1);
	thrust::for_each(
			thrust::make_counting_iterator<uint32>(0u),
			thrust::make_counting_iterator<uint32>(0u) + n_unique,
			populate_unique_kmer_data(plain_view(kmers_counts), plain_view(coords), plain_view(kmers_64b_unique_idxs),
					plain_view(global_unique_flag_map), plain_view(global_to_UID_map)));
}


struct test_funct : public thrust::unary_function<uint32,uint32>
{
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	uint32 operator()(const uint32 uid)
	{
		return uid;
	}
};

// only keep the coordinates that have a unique prefix
template <typename string_set_type>
void D_KmerSet<string_set_type>::filter_coords_by_prefix_uniqueness(const D_VectorU8& unique_map)
{
	D_VectorSetKmerCoord unique_pref_coords(n_kmers);
	uint32 n_unique_coords = thrust::copy_if( // remove_if
			coords.begin(),
			coords.begin() + n_kmers,
			thrust::make_permutation_iterator(
					unique_map.begin(),
					thrust::make_transform_iterator(coords.begin(), get_prefix_global_id())),
			unique_pref_coords.begin(),
			thrust::identity<uint8>()) - unique_pref_coords.begin();

	thrust::copy(unique_pref_coords.begin(), unique_pref_coords.begin() + n_unique_coords, coords.begin());
	n_kmers = n_unique_coords;
}

// for every kmer that is not unique N
// find its closest unique predecessor P and successor S kmer in the same sequence
// and output a kmer chain (P ->... N ... -> S)
// as a new 'super' kmer coordinate
template <typename string_set_type>
void D_KmerSet<string_set_type>::extract_super_kmers()
{
	// collect the ids of the repeat kmers
	kmers_64b_repeat_idxs.resize(n_kmers);
	uint32 n_chains = thrust::copy_if(
			thrust::make_counting_iterator<uint32>(0u),
			thrust::make_counting_iterator<uint32>(0u) + n_kmers,
			global_unique_flag_map.begin(),
			kmers_64b_repeat_idxs.begin(),
			thrust::logical_not<uint8>()) - kmers_64b_repeat_idxs.begin();

	D_VectorSetKmerCoord super_kmers(n_chains);
	D_VectorU32 prefix_uids(n_chains);
	D_VectorU32 suffix_uids(n_chains);
	thrust::for_each(
			thrust::make_counting_iterator(0),
			thrust::make_counting_iterator(0) + n_chains,
			extract_super_kmers_functor<string_set_type>(
					plain_view(kmers_64b_repeat_idxs),
					plain_view(global_unique_flag_map),
					plain_view(global_to_UID_map),
					plain_view(global_to_sorted_id_map),
					plain_view(coords),
					string_set,
					kmer_size,
					plain_view(super_kmers),
					plain_view(prefix_uids),
					plain_view(suffix_uids)));

	// collapse chains that belong to the same sequence and same unique kmer predecessor
	// (these will be consecutive by construction)
	// keep the max chain length coord per same kmer and seq id
	// -- will keep one copy of the chains that is terminated with a unique kmer
	// and the longest chain amongst the ones that are not
	super_coords.resize(n_chains);
	super_prefix_uids.resize(n_chains);
	super_suffix_uids.resize(n_chains);
	uint32 n_collapsed_chains = thrust::reduce_by_key(
				thrust::make_counting_iterator(0u),
				thrust::make_counting_iterator(0u) + n_chains,
				thrust::make_zip_iterator(thrust::make_tuple(super_kmers.begin(), prefix_uids.begin(), suffix_uids.begin())),
				thrust::make_discard_iterator(),
				thrust::make_zip_iterator(thrust::make_tuple(super_coords.begin(), super_prefix_uids.begin(), super_suffix_uids.begin())),
				kmer_pos_sid_eq(plain_view(super_kmers)),
				collapse_same_start_kmers()).second - thrust::make_zip_iterator(thrust::make_tuple(super_coords.begin(), super_prefix_uids.begin(), super_suffix_uids.begin()));

	// eliminate non-unique kmers that are not preceded by a unique kmer
	D_VectorSetKmerCoord t1(n_chains);
	D_VectorU32 t2(n_chains);
	D_VectorU32 t3(n_chains);
	n_super_coords = thrust::remove_copy_if(
			thrust::make_zip_iterator(thrust::make_tuple(
					super_coords.begin(),
					super_prefix_uids.begin(),
					super_suffix_uids.begin())),
			thrust::make_zip_iterator(thrust::make_tuple(
					super_coords.begin() + n_collapsed_chains,
					super_prefix_uids.begin() + n_collapsed_chains,
					super_suffix_uids.begin() + n_collapsed_chains)),
			super_prefix_uids.begin(),
			thrust::make_zip_iterator(thrust::make_tuple(
					t1.begin(),
					t2.begin(),
					t3.begin())),
			is_invalid_super_kmer()) - thrust::make_zip_iterator(thrust::make_tuple(
					t1.begin(),
					t2.begin(),
					t3.begin()));


	thrust::copy(t1.begin(), t1.begin() + n_chains, super_coords.begin());
	thrust::copy(t2.begin(), t2.begin() + n_chains, super_prefix_uids.begin());
	thrust::copy(t3.begin(), t3.begin() + n_chains, super_suffix_uids.begin());

	//super_coords.erase(super_coords.begin() + n_super_coords, super_coords.end());
	//super_prefix_uids.erase(super_prefix_uids.begin() + n_super_coords, super_prefix_uids.end());
	//super_suffix_uids.erase(super_suffix_uids.begin() + n_super_coords, super_suffix_uids.end());

//	printf("Collapsed Filtered Super Kmer Coords: \n");
//	for(uint64 i = 0; i <  n_collapsed_chains_filtered; i++) {
//		SequenceSetKmerCoord c = (SequenceSetKmerCoord) super_coords[i];
//		printf("coord [%u %u %u %u] pref %llu suf %llu\n", c.x, c.y, c.z, c.w, (uint64) super_prefix_global_ids[i],
//				(uint64) super_suffix_global_ids[i]);
//	}

	reset_scratch_space();
}

// collapses lexicographically sorted chains of kmers by computing their prefix overlaps
// and extracting kmers not included in the overlapping region
// along with kmer adjacencies and adjacency count information
template <typename string_set_type>
void D_KmerSet<string_set_type>::collapse_and_extract_non_overlaps(D_VectorSetKmerCoord& kmers_out,
		D_VectorU32& prefix_ids_out, D_VectorU32& suffix_ids_out, D_VectorU32& adj_counts)
{
	uint32 n_super_kmers = n_super_coords;

	// find overlap with the previous chain (in sorted order)
	// overlap is measured in # of shared prefix kmers
	D_VectorU32 max_predecessor_overlaps(n_super_kmers);
	thrust::transform(
			thrust::make_counting_iterator<uint32>(1u), // skip the first chain
			thrust::make_counting_iterator<uint32>(0u) + n_super_kmers,
			max_predecessor_overlaps.begin() + 1,
			find_max_kmer_overlaps<string_set_type>(string_set, plain_view(super_coords), kmer_size));

	// find the number of kmer nodes that need to be created
	// the first kmer in a chain is a unique kmer and should be ignored
	// the last kmer in the chain might be unique, in which case it is also ignored
	thrust::device_vector<uint8> temp_storage;
	D_VectorU32 chain_extraction_offsets(n_super_kmers + 1);
	cuda::inclusive_scan(
			n_super_kmers,
			thrust::make_transform_iterator(
					thrust::make_counting_iterator<uint32>(0u),
					num_non_overlapped_repeats(
							plain_view(super_coords),
							plain_view(super_suffix_uids),
							plain_view(max_predecessor_overlaps), kmer_size)),
			chain_extraction_offsets.begin() + 1,
			thrust::plus<uint32>(),
			temp_storage);

	D_VectorU32 adj_extraction_offsets(n_super_kmers + 1);
	cuda::inclusive_scan(
			n_super_kmers,
			thrust::make_transform_iterator(
					thrust::make_counting_iterator<uint32>(0u),
					num_adj_repeats(
							plain_view(super_coords),
							plain_view(super_suffix_uids),
							plain_view(max_predecessor_overlaps), kmer_size)),
			adj_extraction_offsets.begin() + 1,
			thrust::plus<uint32>(),
			temp_storage);

	// extract non-unique kmers and the (k+1)mer prefix and suffix kmer ids from the collapsed chains
	n_repeat = chain_extraction_offsets[n_super_kmers];
	uint32 n_prefsuf_to_extract = adj_extraction_offsets[n_super_kmers];

	//printf("n_super_coords %u \n", n_super_coords);
	//printf("n_repeat %u \n", n_repeat);
	//printf("Num X-R-X edges %u \n", n_prefsuf_to_extract);

	// allocate the necessary storage and populate
	kmers_out.resize(n_unique + n_repeat);
	prefix_ids_out.resize(n_prefsuf_to_extract);
	suffix_ids_out.resize(n_prefsuf_to_extract);
	thrust::for_each(
			thrust::make_counting_iterator<uint32>(0u),
			thrust::make_counting_iterator<uint32>(0u) + n_super_kmers,
			extract_non_overlapped_repeats<string_set_type>(
					string_set,
					plain_view(super_coords),
					plain_view(super_prefix_uids),
					plain_view(super_suffix_uids),
					plain_view(chain_extraction_offsets),
					plain_view(adj_extraction_offsets),
					plain_view(max_predecessor_overlaps),
					kmer_size, n_unique,
					plain_view(kmers_out),
					plain_view(prefix_ids_out),
					plain_view(suffix_ids_out)));


	// count the number of repeated kmer adjacencies
    adj_counts.resize(n_prefsuf_to_extract);
	thrust::fill(adj_counts.begin(), adj_counts.begin() + n_prefsuf_to_extract, 1);
	thrust::for_each(
			thrust::make_counting_iterator<uint32>(1u),
			thrust::make_counting_iterator<uint32>(0u) + n_super_kmers,
			count_overlapped_adjacencies(
					plain_view(adj_extraction_offsets),
					plain_view(max_predecessor_overlaps),
					plain_view(adj_counts)));

}
