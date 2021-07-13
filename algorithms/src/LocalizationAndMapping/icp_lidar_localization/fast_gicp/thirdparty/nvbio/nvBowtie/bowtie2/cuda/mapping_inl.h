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

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/utils.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvBowtie/bowtie2/cuda/seed_hit.h>
#include <nvBowtie/bowtie2/cuda/seed_hit_deque_array.h>
#include <nvbio/io/utils.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/pingpong_queues.h>
#include <nvbio/basic/cached_iterator.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/priority_deque.h>
#include <nvbio/basic/strided_iterator.h>
#include <nvbio/basic/transform_iterator.h>
#include <nvbio/basic/index_transform_iterator.h>
#include <nvbio/basic/algorithms.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

namespace detail {

///@addtogroup nvBowtie
///@{

///@addtogroup Mapping
///@{

///@addtogroup MappingDetail
///@{

// Checks if the seed's Ns are compatible with the search, i.e.
// if there are 0 Ns in the exactly-matched region, and at most 1 N
// in the inexact matching region.
template <typename ReadStream>
NVBIO_DEVICE NVBIO_FORCEINLINE
bool check_N(const ReadStream& seed, const uint32 exact_len, const uint32 seed_len)
{
    for (uint32 i = 0; i < exact_len; ++i)
        if (seed[i] >= 4)
            return false;

    uint32 N_count = 0;
    for (uint32 i = exact_len; i < seed_len; ++i)
    {
        if (seed[i] >= 4 && ++N_count > 1)
            return false;
    }
    return true;
}

// This function is a lot like match_reverse,
// except that it accepts lower and upper bounds on the sequence stream
template< typename FMType, typename StreamType > NVBIO_DEVICE NVBIO_FORCEINLINE
uint2 match_range(uint2 range, const FMType index, StreamType query,
                  uint32 begin, uint32 end)
{
    for (uint32 i = begin; i < end && range.x <= range.y; ++i)
    {
        const uint8 c = query[i];
        if (c > 3)  return make_uint2(1, 0);

        const uint2 bwt_cnts = rank(index, make_uint2(range.x-1, range.y), c);
        range.x = index.L2(c) + bwt_cnts.x + 1;
        range.y = index.L2(c) + bwt_cnts.y;
    }
    return range;
}

// Store an array of hits into a SeedHitDequeArray
//
NVBIO_DEVICE NVBIO_FORCEINLINE
void store_deque(
    SeedHitDequeArrayDeviceView hit_deques,
    const uint32                read_id,
    const uint32                n_hits,
    const SeedHit*              hitstorage)
{
    SeedHit* hit_data = hit_deques.alloc_deque( read_id, n_hits );
    if (hit_data != NULL)
    {
        hit_deques.resize_deque( read_id, n_hits );
        for (uint32 i = 0; i < n_hits; ++i)
            hit_data[i] = hitstorage[i];
    }
}

enum { CHECK_EXACT=1u, IGNORE_EXACT=0u };
enum MappingAlgorithm
{
    EXACT_MAPPING        = 0u,
    APPROX_MAPPING       = 1u,
    CASE_PRUNING_MAPPING = 2u,
};

/// This is the guts of our mapper. We look for hits that match exactly 
/// in the range [0, len1) and have one mismatch in [len1, len2). If 
/// find_exact is set, we also check for a perfect match.
template< bool find_exact,
          typename Stream,
          typename FMType,
          typename HitType> NVBIO_DEVICE NVBIO_FORCEINLINE
void map(const Stream query, 
         /*const*/ uint32 len1,
         const uint32 len2,
         const FMType index,
         const SeedHit::Flags hit_flags,
         HitType& hitheap, uint32 max_hits,
         uint32& range_sum,
         uint32& range_count)
{
    // TODO:
    // if we know there is one (and only one) N in the second part of the seed,
    // we could avoid doing the mismatch search at all other positions.
    // However, due to the control divergence it would generate, it might be
    // better to filter these seeds away and leave them for a second kernel
    // call.
    //

    // if there is an N, we can do exact matching till the N
    uint32 N_pos = 0;
    uint32 N_cnt = 0;
    for (uint32 i = 0; i < len2; ++i)
    {
        const uint8 c = query[i];
        if (c > 3)
        {
            // if the N is in our exact-matching region, or it is the second one, we cannot align this
            if (i < len1 || N_cnt) return;

            N_pos = i;
            N_cnt++;
        }
    }
    // do exact-matching till the N, if there is one
    len1 = N_cnt ? N_pos : len1;

    // Exact Match first half of read
    uint2 base_range = make_uint2(0, index.length());
          base_range = match_range(base_range, index, query, 0, len1);

    // Allow an error in the next batch
    for(uint32 i = len1; i < len2 && base_range.x <= base_range.y; ++i)
    {
        const uint8 c = query[i];

        // Get ranges for all chars at pos i
        uint4 occ_lo, occ_hi;
        rank4(index, make_uint2(base_range.x-1, base_range.y), &occ_lo, &occ_hi);

        // Try each character
        for (uint8 sub = 0; sub < 4; ++sub)
        {
            if (sub != c && comp(occ_hi, sub) > comp(occ_lo, sub))
            {
                // Manually insert sub
                uint2 range = make_uint2(index.L2(sub) + comp(occ_lo, sub) + 1,
                                         index.L2(sub) + comp(occ_hi, sub) );

                // Extend match to end of read
                range = match_range(range, index, query, i+1, len2);

                // Store Result
                if (range.x <= range.y) {
                    if (hitheap.size() == max_hits)
                        hitheap.pop_bottom();
                    hitheap.push(SeedHit(hit_flags, inclusive_to_exclusive(range)));

                    range_sum += range.y - range.x + 1u;
                    range_count++;
                }
            }
        }

        // Extend base_range
        if (c < 4) {
            base_range.x = index.L2(c) + comp(occ_lo, c) + 1;
            base_range.y = index.L2(c) + comp(occ_hi, c);
        } else { base_range = make_uint2(1, 0); break; }
    }

    if (find_exact && base_range.x <= base_range.y)
    {
        if (hitheap.size() == max_hits)
            hitheap.pop_bottom();
        hitheap.push(SeedHit(hit_flags, inclusive_to_exclusive(base_range)));

        range_sum += base_range.y - base_range.x + 1u;
        range_count++;
    }
}

/// abstract seed-mapper interface
template <MappingAlgorithm ALG>
struct seed_mapper {};

///
/// perform exact seed-mapping
///
template <>
struct seed_mapper<EXACT_MAPPING>
{
    template<typename SeedIterator, typename FMType, typename rFMType, typename HitType>
    NVBIO_FORCEINLINE NVBIO_DEVICE
    static void enact(
        const FMType fmi, const rFMType rfmi,
        const SeedIterator  seed,
        const uint2         read_range,
        const uint32        pos,
        const uint32        seed_len,
        HitType&            hitheap,
        uint32&             range_sum,
        uint32&             range_count,
        const ParamsPOD&    params,
        const bool          fw,
        const bool          rc)
    {
        const OffsetXform <typename SeedIterator::index_type> forward_offset(0);
        const ReverseXform<typename SeedIterator::index_type> reverse_offset(seed_len);

        typedef index_transform_iterator< SeedIterator, OffsetXform <typename SeedIterator::index_type> > fSeedReader;
        typedef index_transform_iterator< SeedIterator, ReverseXform<typename SeedIterator::index_type> > rSeedReader;

        SeedHit::Flags flags;
        uint2          range;

        // forward scan, forward index=0
        const fSeedReader f_reader(seed, forward_offset);
        if (util::count_occurrences( f_reader, seed_len, 4u, 1u ))
            return;

        if (fw)
        {
            range = make_uint2(0, fmi.length());
            range = match_range(range, fmi, f_reader, 0, seed_len);
            if (range.x <= range.y)
            {
                flags = SeedHit::build_flags(STANDARD, FORWARD, read_range.y-pos-seed_len);
                if (hitheap.size() == params.max_hits)
                    hitheap.pop_bottom();
                hitheap.push(SeedHit(flags, inclusive_to_exclusive(range)));

                range_sum += range.y - range.x + 1u;
                range_count++;
            }
        }

        if (rc)
        {
        #if USE_REVERSE_INDEX
            // Complement seed=1, forward scan, reverse index=1
            const transform_iterator<fSeedReader, complement_functor<4> > cf_reader(f_reader, complement_functor<4>());

            range = make_uint2(0, rfmi.length());
            range = match_range(range, rfmi, cf_reader, 0, seed_len);
            if (range.x <= range.y)
            {
                flags = SeedHit::build_flags(COMPLEMENT, REVERSE, pos-read_range.x+seed_len-1);
                if (hitheap.size() == params.max_hits)
                    hitheap.pop_bottom();
                hitheap.push(SeedHit(flags, inclusive_to_exclusive(range)));
            }
        #else
            // Complement seed=1, reverse scan, forward index=0
            const rSeedReader r_reader(seed, reverse_offset);                        
            const transform_iterator<rSeedReader, complement_functor<4> > cr_reader(r_reader, complement_functor<4>());

            range = make_uint2(0, fmi.length());
            range = match_range(range, fmi, cr_reader, 0, seed_len);
            if (range.x <= range.y)
            {
                flags = SeedHit::build_flags(COMPLEMENT, FORWARD, pos-read_range.x);
                if (hitheap.size() == params.max_hits)
                    hitheap.pop_bottom();
                hitheap.push(SeedHit(flags, inclusive_to_exclusive(range)));

                range_sum += range.y - range.x + 1u;
                range_count++;
            }
        #endif
        }
    }
};

///
/// perform approximate seed-mapping allowing 0 mismatches in the subseed and 1 mismatch in
/// the remainder of the seed
///
template <>
struct seed_mapper<APPROX_MAPPING>
{
    template<typename SeedIterator, typename FMType, typename rFMType, typename HitType>
    NVBIO_FORCEINLINE NVBIO_DEVICE
    static void enact(
        const FMType fmi, const rFMType rfmi,
        const SeedIterator  seed,
        const uint2         read_range,
        const uint32        pos,
        const uint32        seed_len,
        HitType&            hitheap,
        uint32&             range_sum,
        uint32&             range_count,
        const ParamsPOD&    params,
        const bool          fw,
        const bool          rc)
    {
        const OffsetXform <typename SeedIterator::index_type> forward_offset(0);
        const ReverseXform<typename SeedIterator::index_type> reverse_offset(seed_len);

        typedef index_transform_iterator< SeedIterator, OffsetXform <typename SeedIterator::index_type> > fSeedReader;
        typedef index_transform_iterator< SeedIterator, ReverseXform<typename SeedIterator::index_type> > rSeedReader;

        SeedHit::Flags flags;

        // Standard seed=0, forward scan, forward index=0
        const fSeedReader f_reader(seed, forward_offset);
        if (util::count_occurrences( f_reader, seed_len, 4u, 2u ))
            return;

        if (fw)
        {
            flags = SeedHit::build_flags(STANDARD, FORWARD, read_range.y-pos-seed_len);
            map<CHECK_EXACT> (f_reader,  params.subseed_len, seed_len,  fmi, flags, hitheap, params.max_hits, range_sum, range_count);
        }

        // Complement seed=1, reverse scan, forward  index=0
        if (rc)
        {
            const rSeedReader r_reader(seed, reverse_offset);                        
            const transform_iterator<rSeedReader, complement_functor<4> > cr_reader(r_reader, complement_functor<4>());
            flags = SeedHit::build_flags(COMPLEMENT, FORWARD, pos-read_range.x);
            map<IGNORE_EXACT>(cr_reader, params.subseed_len, seed_len,  fmi, flags, hitheap, params.max_hits, range_sum, range_count);
        }
    }
};

///
/// perform approximate seed-mapping allowing 1 mismatch using case-pruning, i.e:
///   * split the seed in two,
///   * perform exact mapping of the first half and allow 1 mismatch in the second half
///   * perform exact mapping of the second half and allow 1 mismatch in the first half
///
template <>
struct seed_mapper<CASE_PRUNING_MAPPING>
{
    template<typename SeedIterator, typename FMType, typename rFMType, typename HitType>
    NVBIO_FORCEINLINE NVBIO_DEVICE
    static void enact(
        const FMType fmi, const rFMType rfmi,
        const SeedIterator  seed,
        const uint2         read_range,
        const uint32        pos,
        const uint32        seed_len,
        HitType&            hitheap,
        uint32&             range_sum,
        uint32&             range_count,
        const ParamsPOD&    params,
        const bool          fw,
        const bool          rc)
    {
        const OffsetXform <typename SeedIterator::index_type> forward_offset(0);
        const ReverseXform<typename SeedIterator::index_type> reverse_offset(seed_len);

        typedef index_transform_iterator< SeedIterator, OffsetXform <typename SeedIterator::index_type> > fSeedReader;
        typedef index_transform_iterator< SeedIterator, ReverseXform<typename SeedIterator::index_type> > rSeedReader;

        SeedHit::Flags flags;

        // Standard seed=0, forward scan, forward index=0
        const fSeedReader f_reader(seed, forward_offset);
        if (fw)
        {
            flags = SeedHit::build_flags(STANDARD, FORWARD, read_range.y-pos-seed_len);
            map<CHECK_EXACT> (f_reader,  (seed_len  )/2, seed_len,  fmi, flags, hitheap, params.max_hits, range_sum, range_count);
        }
        // Standard seed=0, reverse scan, reverse index=1
        const rSeedReader r_reader(seed, reverse_offset);
        if (fw)
        {
            flags = SeedHit::build_flags(STANDARD, REVERSE, read_range.y-pos-1);
            map<IGNORE_EXACT>(r_reader,  (seed_len+1)/2, seed_len, rfmi, flags, hitheap, params.max_hits, range_sum, range_count);
        }

        // Complement seed=1, forward scan, reverse index=1
        if (rc)
        {
            const transform_iterator<fSeedReader, complement_functor<4> > cf_reader(f_reader, complement_functor<4>());
            flags = SeedHit::build_flags(COMPLEMENT, REVERSE, pos-read_range.x+seed_len-1);
            map<CHECK_EXACT> (cf_reader, (seed_len  )/2, seed_len, rfmi, flags, hitheap, params.max_hits, range_sum, range_count);
        }

        // Complement seed=1, reverse scan, forward  index=0
        if (rc)
        {
            const transform_iterator<rSeedReader, complement_functor<4> > cr_reader(r_reader, complement_functor<4>());
            flags = SeedHit::build_flags(COMPLEMENT, FORWARD, pos-read_range.x);
            map<IGNORE_EXACT>(cr_reader, (seed_len+1)/2, seed_len,  fmi, flags, hitheap, params.max_hits, range_sum, range_count);
        }
    }
};

///
/// Exact whole read filtering kernel. Maps the entire read exactly
///
template<typename BatchType, typename FMType, typename rFMType> __global__ 
void map_whole_read_kernel(
    const BatchType                                 read_batch, const FMType fmi, const rFMType rfmi,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    typedef PackedStringLoader<
        typename BatchType::sequence_storage_iterator,
        BatchType::SEQUENCE_BITS,
        BatchType::SEQUENCE_BIG_ENDIAN,
        uncached_tag >                              packed_loader_type;
    typedef typename packed_loader_type::iterator   seed_iterator;

    // instantiate a packed loader to cache the seed in local memory
    packed_loader_type packer_loader;

    // instantiate storage for a local memory hits queue
    SeedHit local_hits[512];

    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    const uint32 grid_size = BLOCKDIM * gridDim.x;

    for (uint32 id = thread_id; id < queues.in_size; id += grid_size)
    {
        const uint32 read_id = queues.in_queue[ id ];
        NVBIO_CUDA_ASSERT( read_id < read_batch.size() );
        const uint2  read_range = read_batch.get_range( read_id );

        // filter away short strings
        if (read_range.y - read_range.x < params.min_read_len)
        {
            hits.resize_deque( read_id, 0 );
            return;
        }

        typedef SeedHitDequeArrayDeviceView::hit_vector_type hit_storage_type;
        typedef SeedHitDequeArrayDeviceView::hit_deque_type  hit_deque_type;
        hit_deque_type hitheap( hit_storage_type( 0, local_hits ), true );

        uint32 range_sum   = 0;
        uint32 range_count = 0;

        // load the whole read
        const seed_iterator read = packer_loader.load( read_batch.sequence_stream() + read_range.x, read_range.y - read_range.x );

        // map the read
        seed_mapper<EXACT_MAPPING>::enact(
            fmi, rfmi,
            read,
            read_range,
            read_range.x,
            read_range.y - read_range.x,
            hitheap,
            range_sum,
            range_count,
            params,
            fw,
            rc );

        // save the hits
        store_deque( hits, read_id, hitheap.size(), local_hits );

        // enqueue for re-seeding if there are no hits
        if (reseed)
            reseed[ id ] = (range_count == 0);
    }
}

///
/// Seed filtering kernel. Maps the set of seeds for each read in the input
/// queue, filling the corresponding hit priority-deque, and fills the output queue with
/// the list of reads that need reseeding.
///
template<MappingAlgorithm ALGO, typename BatchType, typename FMType, typename rFMType> __global__ 
void map_queues_kernel(
    const BatchType                                 read_batch, const FMType fmi, const rFMType rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    typedef PackedStringLoader<
        typename BatchType::sequence_storage_iterator,
        BatchType::SEQUENCE_BITS,
        BatchType::SEQUENCE_BIG_ENDIAN,
        uncached_tag >                              packed_loader_type;
        //lmem_cache_tag<128> >                       packed_loader_type; // TODO: this produces a crashing kernel, for no good reason
    typedef typename packed_loader_type::iterator   seed_iterator;

    // instantiate a packed loader to cache the seed in local memory
    packed_loader_type packer_loader;

    // instantiate storage for a local memory hits queue
    SeedHit local_hits[512];

    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    const uint32 grid_size = BLOCKDIM * gridDim.x;

    for (uint32 id = thread_id; id < queues.in_size; id += grid_size)
    {
        const uint32 read_id = queues.in_queue[ id ];
        NVBIO_CUDA_ASSERT( read_id < read_batch.size() );
        const uint2  read_range = read_batch.get_range( read_id );
        const uint32 read_len   = read_range.y - read_range.x;

        // filter away short strings
        if (read_len < params.min_read_len)
        {
            hits.resize_deque( read_id, 0 );
            continue;
        }

        const uint32 seed_len     = nvbio::min( params.seed_len, read_len );
        const uint32 seed_freq    = (uint32)params.seed_freq( read_len );
        const uint32 retry_stride = seed_freq/(params.max_reseed+1);

        typedef SeedHitDequeArrayDeviceView::hit_vector_type hit_storage_type;
        typedef SeedHitDequeArrayDeviceView::hit_deque_type  hit_deque_type;
        hit_deque_type hitheap( hit_storage_type( 0, local_hits ), true );

        uint32 range_sum   = 0;
        uint32 range_count = 0;

        // loop over seeds
        for (uint32 pos = read_range.x + retry * retry_stride; pos + seed_len <= read_range.y; pos += seed_freq)
        {
            const seed_iterator seed = packer_loader.load( read_batch.sequence_stream() + pos, seed_len );

            seed_mapper<ALGO>::enact(
                fmi, rfmi,
                seed,
                read_range,
                pos,
                seed_len,
                hitheap,
                range_sum,
                range_count,
                params,
                fw,
                rc );
        }

        // save the hits
        store_deque( hits, read_id, hitheap.size(), local_hits );

        // enqueue for re-seeding if there are too many hits
        if (reseed)
            reseed[ id ] = (range_count == 0 || range_sum >= params.rep_seeds * range_count);

        NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.read_id == read_id, "map:  ranges[%u], rows[%u]\n", read_id, range_count, range_sum );
    }
}

///
/// Seed filtering kernel. Maps the set of seeds for each read in the input
/// queue, filling the corresponding hit priority-deque.
/// Internally performs re-seeding to find a good seeding scheme.
///
template<MappingAlgorithm ALGO, typename BatchType, typename FMType, typename rFMType> __global__ 
void map_kernel(
    const BatchType             read_batch, const FMType fmi, const rFMType rfmi,
    SeedHitDequeArrayDeviceView hits,
    const uint2                 seed_range,
    const ParamsPOD             params,
    const bool                  fw,
    const bool                  rc)
{
    typedef PackedStringLoader<
        typename BatchType::sequence_storage_iterator,
        BatchType::SEQUENCE_BITS,
        BatchType::SEQUENCE_BIG_ENDIAN,
        uncached_tag >                              packed_loader_type;
        //lmem_cache_tag<128> >                       packed_loader_type; // TODO: this produces a crashing kernel, for no good reason
    typedef typename packed_loader_type::iterator   seed_iterator;

    // instantiate a packed loader to cache the seed in local memory
    packed_loader_type packer_loader;

    // instantiate storage for a local memory hits queue
    SeedHit local_hits[512];

    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    const uint32 grid_size = BLOCKDIM * gridDim.x;

    for (uint32 read_id = thread_id; read_id < read_batch.size(); read_id += grid_size)
    {
        const uint2  read_range = read_batch.get_range( read_id );
        const uint32 read_len   = read_range.y - read_range.x;

        // filter away short strings
        if (read_len < params.min_read_len)
        {
            hits.resize_deque( read_id, 0 );
            return;
        }

        const uint32 seed_len     = nvbio::min( params.seed_len, read_len );
        const uint32 seed_freq    = (uint32)params.seed_freq( read_len );
        const uint32 retry_stride = seed_freq/(params.max_reseed+1);

        typedef SeedHitDequeArrayDeviceView::hit_vector_type hit_storage_type;
        typedef SeedHitDequeArrayDeviceView::hit_deque_type  hit_deque_type;
        hit_deque_type hitheap( hit_storage_type( 0, local_hits ), true );

        bool active = true;

        //for (uint32 retry = 0; retry <= params.max_reseed; ++retry) // TODO: this makes the kernel crash even when
                                                                      // max_reseed is 0, for apparently no good reason...
        for (uint32 retry = 0; retry <= 0; ++retry)
        {
            if (active)
            {
                // restore the result heap
                hitheap.clear();

                uint32 range_sum   = 0;
                uint32 range_count = 0;

                for (uint32 i = seed_range.x; i < seed_range.y; ++i)
                {
                    const uint32 pos = read_range.x + retry * retry_stride + i * seed_freq;
                    if (pos + seed_len <= read_range.y)
                    {
                        const seed_iterator seed = packer_loader.load( read_batch.sequence_stream() + pos, seed_len );

                        seed_mapper<ALGO>::enact(
                            fmi, rfmi,
                            seed,
                            read_range,
                            pos,
                            seed_len,
                            hitheap,
                            range_sum,
                            range_count,
                            params,
                            fw,
                            rc );
                    }
                }

                // check whether we could stop here or if we need re-seeding
                if (retry == params.max_reseed || range_count == 0 && range_sum < params.rep_seeds * range_count)
                    active = false;
            }
        }

        // save the hits
        store_deque( hits, read_id, hitheap.size(), local_hits );
    }
}

///@}  // group MappingDetail
///@}  // group Mapping
///@}  // group nvBowtie

} // namespace detail

//
// call the map-sub kernel
//
template <typename BatchType, typename FMType, typename rFMType>
void map_case_pruning_t(
    const BatchType&                                read_batch, const FMType fmi, const rFMType rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    const int blocks = nvbio::cuda::max_active_blocks( detail::map_queues_kernel<detail::CASE_PRUNING_MAPPING,BatchType,FMType,rFMType>, BLOCKDIM, 0 );
    detail::map_queues_kernel<detail::CASE_PRUNING_MAPPING> <<<blocks, BLOCKDIM>>>(
        read_batch, fmi, rfmi, retry, queues, reseed, hits, params, fw, rc );
}

//
// call the map-hybrid kernel
//
template <typename BatchType, typename FMType, typename rFMType>
void map_approx_t(
    const BatchType&                                read_batch, const FMType fmi, const rFMType rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    const int blocks = nvbio::cuda::max_active_blocks( detail::map_queues_kernel<detail::APPROX_MAPPING,BatchType,FMType,rFMType>, BLOCKDIM, 0 );
    detail::map_queues_kernel<detail::APPROX_MAPPING> <<<blocks, BLOCKDIM>>>(
        read_batch, fmi, rfmi, retry, queues, reseed, hits, params, fw, rc );
}

//
// call the map-hybrid kernel
//
template <typename BatchType, typename FMType, typename rFMType>
void map_approx_t(
    const BatchType&            read_batch, const FMType fmi, const rFMType rfmi,
    SeedHitDequeArrayDeviceView hits,
    const uint2                 seed_range,
    const ParamsPOD             params,
    const bool                  fw,
    const bool                  rc)
{
    const int blocks = nvbio::cuda::max_active_blocks( detail::map_kernel<detail::APPROX_MAPPING,BatchType,FMType,rFMType>, BLOCKDIM, 0 );
    detail::map_kernel<detail::APPROX_MAPPING> <<<blocks, BLOCKDIM>>>(
        read_batch, fmi, rfmi, hits, seed_range, params, fw, rc );
}

//
// call the map-exact kernel
//
template <typename BatchType, typename FMType, typename rFMType>
void map_whole_read_t(
    const BatchType&                                read_batch, const FMType fmi, const rFMType rfmi,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    const uint32 blocks = nvbio::cuda::max_active_blocks( detail::map_whole_read_kernel<BatchType,FMType,rFMType>, BLOCKDIM, 0 );
    detail::map_whole_read_kernel<<<blocks, BLOCKDIM>>> (
        read_batch, fmi, rfmi, queues, reseed, hits, params, fw, rc );
}

//
// call the map-exact kernel
//
template <typename BatchType, typename FMType, typename rFMType>
void map_exact_t(
    const BatchType&                                read_batch, const FMType fmi, const rFMType rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    const uint32 blocks = nvbio::cuda::max_active_blocks( detail::map_queues_kernel<detail::EXACT_MAPPING,BatchType,FMType,rFMType>, BLOCKDIM, 0 );
    detail::map_queues_kernel<detail::EXACT_MAPPING> <<<blocks, BLOCKDIM>>>(
        read_batch, fmi, rfmi, retry, queues, reseed, hits, params, fw, rc );
}

//
// call the map-exact kernel
//
template <typename BatchType, typename FMType, typename rFMType>
void map_exact_t(
    const BatchType&            read_batch, const FMType fmi, const rFMType rfmi,
    SeedHitDequeArrayDeviceView hits,
    const uint2                 seed_range,
    const ParamsPOD             params,
    const bool                  fw,
    const bool                  rc)
{
    const uint32 blocks = nvbio::cuda::max_active_blocks( detail::map_kernel<detail::EXACT_MAPPING,BatchType,FMType,rFMType>, BLOCKDIM, 0 );
    detail::map_kernel<detail::EXACT_MAPPING> <<<blocks, BLOCKDIM>>>(
        read_batch, fmi, rfmi, hits, seed_range, params, fw, rc );
}

//
// call the appropriate mapping kernel
//
template <typename BatchType, typename FMType, typename rFMType>
void map_t(
    const BatchType&                                read_batch, const FMType fmi, const rFMType rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    // check whether we allow substitutions
    if (params.allow_sub)
    {
        // check whether we have to perform exact matching on a subseed
        if (params.subseed_len == 0)
        {
            // call the hybrid fuzzy mapping kernel
            map_case_pruning_t(
                read_batch, fmi, rfmi, retry, queues, reseed, hits, params, fw, rc );
        }
        else
        {
            // call the hybrid exact-fuzzy mapping kernel
            map_approx_t(
                read_batch, fmi, rfmi, retry, queues, reseed, hits, params, fw, rc );
        }
    }
    else
    {
        // call the hybrid exact mapping kernel
        map_exact_t(
            read_batch, fmi, rfmi, retry, queues, reseed, hits, params, fw, rc );
    }
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
