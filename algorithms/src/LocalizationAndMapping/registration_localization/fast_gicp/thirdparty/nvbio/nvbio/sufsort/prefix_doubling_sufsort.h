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

#include <nvbio/sufsort/sufsort_priv.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/cuda/ldg.h>
#include <thrust/device_vector.h>
#include <thrust/transform_scan.h>
#include <thrust/binary_search.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/counting_iterator.h>
#include <mgpuhost.cuh>
#include <moderngpu.cuh>
#include <cub/cub.cuh>

namespace nvbio {
namespace cuda {

///
/// A sorting enactor for sorting all suffixes of a given string using a modified parallel version
/// of "Faster Suffix Sorting" by Larsson and Sadanake.
///
struct PrefixDoublingSufSort
{
    /// constructor
    ///
    PrefixDoublingSufSort() :
        extract_time(0.0f),
        gather_time(0.0f),
        radixsort_time(0.0f),
        segment_time(0.0f),
        compact_time(0.0f),
        inverse_time(0.0f) { m_mgpu = mgpu::CreateCudaDevice(0); }

    /// Sort all the suffixes of a given string, using a modified parallel version
    /// of "Faster Suffix Sorting" by Larsson and Sadanake.
    ///
    /// \param string_len           string length
    /// \param string               string iterator
    /// \param d_suffixes           device vector of output suffixes
    ///
    /// All the other parameters are temporary device buffers
    ///
    template <typename string_type, typename output_iterator>
    void sort(
        const typename stream_traits<string_type>::index_type   string_len,
        const string_type                                       string,
        output_iterator                                         d_suffixes);

    /// reserve enough storage for sorting n strings/suffixes
    ///
    void reserve(const uint32 n)
    {
        if (n > d_active_slots.size())
        {
            clear();

            d_inv_keys.resize( n );
            d_active_slots.resize( n + 4 );
            d_sort_keys.resize( n + 4 );
            d_sort_indices.resize( n + 4 );
            d_partial_flags.resize( n + 16 );
            d_temp_flags.resize( n + 16 );
            d_segment_flags.resize( n + 16 );
            d_segment_keys.resize( n + 4 );
            d_segment_heads.resize( n + 4 );
            d_segment_blocks.resize( ((n+4)+127/128) );
        }
    }
    /// free all temporary storage
    ///
    void clear()
    {
        d_inv_keys.clear();
        d_sort_keys.clear();
        d_sort_indices.clear();
        d_partial_flags.clear();
        d_temp_flags.clear();
        d_segment_flags.clear();
        d_segment_keys.clear();
        d_segment_heads.clear();
        d_active_slots.clear();
        d_segment_blocks.clear();
    }

    float extract_time;             ///< timing stats
    float gather_time;              ///< timing stats
    float radixsort_time;           ///< timing stats
    float segment_time;            ///< timing stats
    float compact_time;             ///< timing stats
    float inverse_time;             ///< timing stats

private:
    thrust::device_vector<uint32>   d_inv_keys;         ///< compressed prefix keys
    thrust::device_vector<uint32>   d_sort_keys;        ///< radix-sorting keys
    thrust::device_vector<uint32>   d_sort_indices;     ///< radix-sorting indices
    thrust::device_vector<uint32>   d_active_slots;     ///< active slots vector
    thrust::device_vector<uint8>    d_segment_flags;    ///< segment head flags
    thrust::device_vector<uint8>    d_partial_flags;    ///< segment head flags
    thrust::device_vector<uint8>    d_temp_flags;       ///< segment head flags
    thrust::device_vector<uint32>   d_segment_keys;     ///< segment keys
    thrust::device_vector<uint32>   d_segment_heads;    ///< segment heads
    thrust::device_vector<uint32>   d_segment_blocks;
    mgpu::ContextPtr                m_mgpu;
};

/// perform prefix-doubling on the selected suffixes
///
#define VECTORIZED_PREFIX_DOUBLING
#if defined(VECTORIZED_PREFIX_DOUBLING)
#define PREFIX_DOUBLING_VECTOR 4
#else
#define PREFIX_DOUBLING_VECTOR 1
#endif
__global__ void prefix_doubling_kernel(
    const uint32            n_slots,
    const uint32            n_suffixes,
    const uint32            j,
    const uint32*           suffixes,
    const uint32*           inv_keys,
          uint32*           out_keys)
{
    //
    // perform prefix-doubling:
    //
    //   - suffixes[i] encodes the suffix at index i;
    //
    //   - in order to perform prefix-doubling, we want to set K[i] as
    //     the position of suffix SA[i] + j, i.e. K[i] = invSA[ SA[i] + j ]
    //
    const cuda::ldg_pointer<uint32> inv_keys_ldg( inv_keys );

  #if !defined(VECTORIZED_PREFIX_DOUBLING) // reference scalar implementation
    const uint32 idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= n_slots)
       return;

    const uint32 suf_nj = suffixes[idx] + j;
    const uint32 K_nj   = suf_nj < n_suffixes ? inv_keys_ldg[ suf_nj ] : 0u;
          out_keys[idx] = K_nj;
  #else // vectorized implementation
    const uint32 idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx*4u >= n_slots)
        return;

    uint4 suf_nj = ((const uint4*)suffixes)[idx];
          suf_nj = make_uint4( suf_nj.x + j, suf_nj.y + j, suf_nj.z + j, suf_nj.w + j );

    const uint4 K_nj = make_uint4(
        suf_nj.x < n_suffixes ? inv_keys_ldg[ suf_nj.x ] : 0u,
        suf_nj.y < n_suffixes ? inv_keys_ldg[ suf_nj.y ] : 0u,
        suf_nj.z < n_suffixes ? inv_keys_ldg[ suf_nj.z ] : 0u,
        suf_nj.w < n_suffixes ? inv_keys_ldg[ suf_nj.w ] : 0u );

    ((uint4*)out_keys)[idx] = K_nj;
  #endif
}
/// perform prefix-doubling on the selected suffixes
///
inline void prefix_doubling(
    const uint32            n_slots,
    const uint32            n_suffixes,
    const uint32            j,
    const uint32*           suffixes,
    const uint32*           inv_keys,
          uint32*           out_keys)
{
    const uint32 blockdim = 256;
    const uint32 n_blocks = ((n_slots+PREFIX_DOUBLING_VECTOR-1)/PREFIX_DOUBLING_VECTOR + blockdim-1) / blockdim;
    prefix_doubling_kernel<<<n_blocks,blockdim>>>(
        n_slots,
        n_suffixes,
        j,
        suffixes,
        inv_keys,
        out_keys );
}

/// build a set of head flags looking at adjacent keys and compute their block sums
///
template <uint32 BLOCKDIM>
__global__ void build_head_flags_kernel(
    const uint32            n_flags,
    const uint32*           keys,
          uint8*            flags,
          uint32*           blocks)
{
    const uint32 thread_id = (threadIdx.x + blockIdx.x * blockDim.x);
    const uint32 idx = thread_id * 4;

    // load the previous key
    const uint32 key_p = idx && idx < n_flags ? keys[idx-1] : 0xFFFFFFFF;

    // load the next 4 keys
    const uint4  key  = idx < n_flags ?   ((const uint4*)keys)[thread_id] : make_uint4( 0, 0, 0, 0 );

    // fetch 4 flags in one go
    uchar4       flag = idx < n_flags ? ((const uchar4*)flags)[thread_id] : make_uchar4( 0, 0, 0, 0 );
        
    // merge them with the new flags
    flag = make_uchar4(
        (key.x != key_p) ? 1u : flag.x,
        (key.y != key.x) ? 1u : flag.y,
        (key.z != key.y) ? 1u : flag.z,
        (key.w != key.z) ? 1u : flag.w );

    // clamp them
    flag = make_uchar4(
            idx + 0u < n_flags ? flag.x : 0u,
            idx + 1u < n_flags ? flag.y : 0u,
            idx + 2u < n_flags ? flag.z : 0u,
            idx + 3u < n_flags ? flag.w : 0u );

    // write them out
    if (idx < n_flags)
        ((uchar4*)flags)[thread_id] = flag;

    // and compute their block sum
    const uint32 n = flag.x + flag.y + flag.z + flag.w;
    typedef cub::BlockReduce<uint32,BLOCKDIM> BlockReduce;

    __shared__ typename BlockReduce::TempStorage  reduce_smem_storage;

    const uint32 block_aggregate = BlockReduce( reduce_smem_storage ).Sum( n );

    if (threadIdx.x == 0)
        blocks[blockIdx.x] = block_aggregate;
}

/// extract the slots corresponding to the first key in each segment
///
template <uint32 BLOCKDIM>
__global__ void extract_segments_kernel(
    const uint32            n_flags,
    const uint8*            flags,
    const uint32*           blocks,
    const uint32*           slots,
          uint32*           keys,
          uint32*           segments)
{
    const uint32 thread_id = (threadIdx.x + blockIdx.x * blockDim.x);
    const uint32 idx = thread_id * 4;

    // fetch 4 flags in one go
    const uchar4 in_flag = idx < n_flags ? ((const uchar4*)flags)[thread_id] : make_uchar4( 0, 0, 0, 0 );

    const uchar4 flag = make_uchar4(
            idx + 0u < n_flags ? in_flag.x : 0u,
            idx + 1u < n_flags ? in_flag.y : 0u,
            idx + 2u < n_flags ? in_flag.z : 0u,
            idx + 3u < n_flags ? in_flag.w : 0u );

    // compute their sum
    const uint32 n = flag.x + flag.y + flag.z + flag.w;

    uint32 partial_scan;
    uint32 partial_aggregate;

    typedef cub::BlockScan<uint32,BLOCKDIM> BlockScan;

    __shared__ typename BlockScan::TempStorage scan_smem_storage;

    BlockScan( scan_smem_storage ).ExclusiveSum( n, partial_scan, partial_aggregate );

    // add the block partial
    partial_scan += blocks[ blockIdx.x ];

    // perform the serial scan to get the segment key
    const uint4 key = make_uint4(
        partial_scan + uint32(flag.x),
        partial_scan + uint32(flag.x + flag.y),
        partial_scan + uint32(flag.x + flag.y + flag.z),
        partial_scan + uint32(flag.x + flag.y + flag.z + flag.w) );

    // write the output key
    if (idx < n_flags)
        ((uint4*)keys)[thread_id] = key;

    if (n)
    {
        // write the output segment head
        const uint4 slot = ((const uint4*)slots)[thread_id];

        if (flag.x) segments[key.x] = slot.x + 1u;
        if (flag.y) segments[key.y] = slot.y + 1u;
        if (flag.z) segments[key.z] = slot.z + 1u;
        if (flag.w) segments[key.w] = slot.w + 1u;
    }
}

/// given a sorted set of n input keys, flags and slots, this function:
///
///  - updates the set of flags or'ing them with the head flags of the segments
///    of equal keys
///
///  - computes a new "compact" set of keys in the range [1,#distinct-keys]
///
///  - computes a set of segment heads segments[1,...,#distinct-keys]:
///    segments[i] := slot[m] + 1, where m is the smallest integer such that
///    keys[m] = i.
///
inline uint32 extract_segments(
    const uint32            n_flags,
    const uint32*           in_keys,
          uint8*            flags,
          uint32*           blocks,
    const uint32*           slots,
          uint32*           keys,
          uint32*           segments)
{
    const uint32 blockdim = 128;
    const uint32 n_blocks = ((n_flags+3)/4 + blockdim-1) / blockdim;

    // update the head flags and compute their block sums
    build_head_flags_kernel<blockdim> <<<n_blocks,blockdim>>>(
        n_flags,
        in_keys,
        flags,
        blocks );

    *thrust::device_ptr<uint8>( flags + n_flags ) = 1u; // add a sentinel

    // scan the block partials
    *thrust::device_ptr<uint32>( blocks + n_blocks ) = 0u;
    thrust::exclusive_scan(
        thrust::device_ptr<uint32>( blocks ),
        thrust::device_ptr<uint32>( blocks ) + n_blocks + 1u,
        thrust::device_ptr<uint32>( blocks ),
        0u,
        thrust::plus<uint32>() );

    // and extract the new keys and segments
    extract_segments_kernel<blockdim> <<<n_blocks,blockdim>>>(
        n_flags,
        flags,
        blocks,
        slots,
        keys,
        segments );

    return *thrust::device_ptr<uint32>( blocks + n_blocks );
}


/// scatter flags/slots/indices to the position specified by keys (-1) only if the given stencil is true
///
__global__ void compact_kernel(
    const uint32            n,
    const uint8*            stencil,
    const uint32*           keys,
    const uint8*            flags,
    const uint32*           slots,
    const uint32*           indices,
          uint8*            out_flags,
          uint32*           out_slots,
          uint32*           out_indices)
{
    const uint32 thread_id = (threadIdx.x + blockIdx.x * blockDim.x);
    const uint32 idx = 4 * thread_id;
    if (idx >= n)
        return;

    const uchar4 s = ((const uchar4*)stencil)[thread_id];

    if (s.x + s.y + s.z + s.w == 0)
        return;

    const uint4 key   =    ((const uint4*)keys)[thread_id];
    const uchar4 flag =  ((const uchar4*)flags)[thread_id];
    const uint4 slot  =   ((const uint4*)slots)[thread_id];
    const uint4 index = ((const uint4*)indices)[thread_id];

    if (s.x)
    {
          out_flags[ key.x - 1 ] = flag.x;
          out_slots[ key.x - 1 ] = slot.x;
        out_indices[ key.x - 1 ] = index.x;
    }
    if (s.y)
    {
          out_flags[ key.y - 1 ] = flag.y;
          out_slots[ key.y - 1 ] = slot.y;
        out_indices[ key.y - 1 ] = index.y;
    }
    if (s.z)
    {
          out_flags[ key.z - 1 ] = flag.z;
          out_slots[ key.z - 1 ] = slot.z;
        out_indices[ key.z - 1 ] = index.z;
    }
    if (s.w)
    {
          out_flags[ key.w - 1 ] = flag.w;
          out_slots[ key.w - 1 ] = slot.w;
        out_indices[ key.w - 1 ] = index.w;
    }
}

/// scatter flags/slots/indices to the position specified by keys (-1) only if the given stencil is true
///
inline void compact(
    const uint32            n,
    const uint8*            stencil,
    const uint32*           keys,
    const uint8*            flags,
    const uint32*           slots,
    const uint32*           indices,
          uint8*            out_flags,
          uint32*           out_slots,
          uint32*           out_indices)
{
    const uint32 blockdim = 128;
    const uint32 n_words  = (n + 3) / 4;
    const uint32 n_blocks = (n_words + blockdim-1) / blockdim;

    compact_kernel<<<n_blocks,blockdim>>>(
        n,
        stencil,
        keys,
        flags,
        slots,
        indices,
        out_flags,
        out_slots,
        out_indices );
}

/// Sort all the suffixes of a given string, using a modified parallel version of "Faster Suffix Sorting"
/// by Larsson and Sadanake.
///
/// \param string_len           string length
/// \param string               string iterator
/// \param d_suffixes           device vector of output suffixes
///
template <typename string_type, typename output_iterator>
void PrefixDoublingSufSort::sort(
    const typename stream_traits<string_type>::index_type   string_len,
    const string_type                                       string,
    output_iterator                                         d_suffixes)
{
    typedef typename stream_traits<string_type>::index_type index_type;
    const uint32 SYMBOL_SIZE = stream_traits<string_type>::SYMBOL_SIZE;

    typedef uint32 word_type;
    NVBIO_VAR_UNUSED const uint32   WORD_BITS = uint32( 8u * sizeof(word_type) );
    NVBIO_VAR_UNUSED const uint32 DOLLAR_BITS =
        SYMBOL_SIZE*4 <= WORD_BITS-4 ? 4 :          // more than three symbols per word
        SYMBOL_SIZE*3 <= WORD_BITS-2 ? 2 :          // up to three symbols per word
        SYMBOL_SIZE*2 <= WORD_BITS-2 ? 2 :          // up to two symbols per word
                                       0;           // only one symbol per word

    const uint32 SYMBOLS_PER_WORD = priv::symbols_per_word<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS>();

    const uint32 n_suffixes = string_len;

    // reserve temporary storage
    reserve( n_suffixes );

    // initialize the segment flags
    thrust::fill(
        d_segment_flags.begin(),
        d_segment_flags.begin() + n_suffixes,
        uint8(0u) );

    // initialize the device sorting indices
    thrust::copy(
        thrust::make_counting_iterator<uint32>(0u),
        thrust::make_counting_iterator<uint32>(0u) + n_suffixes,
        d_sort_indices.begin() );

    // initialize the active slots
    thrust::copy(
        thrust::make_counting_iterator<uint32>(0u),
        thrust::make_counting_iterator<uint32>(0u) + n_suffixes,
        d_active_slots.begin() );

    // first pass: sort all suffixes by the first radix
    NVBIO_CUDA_DEBUG_STATEMENT( fprintf(stderr,"    primary key sorting\n") );
    {
        Timer timer;
        timer.start();

        // extract the given radix word from each of the partially sorted suffixes and merge it with the existing keys
        priv::string_suffix_word_functor<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS,string_type,word_type> word_functor( string_len, string, 0u );

        thrust::copy(
            thrust::make_transform_iterator( d_sort_indices.begin(),              word_functor ),
            thrust::make_transform_iterator( d_sort_indices.begin() + n_suffixes, word_functor ),
            d_sort_keys.begin() );

        NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
        timer.stop();
        extract_time += timer.seconds();

        timer.start();

        cuda::SortBuffers<uint32*,uint32*> sort_buffers;
        cuda::SortEnactor                  sort_enactor;

        sort_buffers.selector  = 0;
        sort_buffers.keys[0]   = nvbio::raw_pointer( d_sort_keys );
        sort_buffers.keys[1]   = nvbio::raw_pointer( d_segment_keys );
        sort_buffers.values[0] = nvbio::raw_pointer( d_sort_indices );
        sort_buffers.values[1] = nvbio::raw_pointer( d_segment_heads );

        // sort the keys together with the indices
        sort_enactor.sort( n_suffixes, sort_buffers );

        if (sort_buffers.selector)
        {
            // swap the buffers
            d_sort_keys.swap( d_segment_keys );
            d_sort_indices.swap( d_segment_heads );
        }

        NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
        timer.stop();
        radixsort_time += timer.seconds();
    }

    NVBIO_CUDA_DEBUG_STATEMENT( fprintf(stderr,"    primary key compression\n") );
    {
        Timer timer;
        timer.start();

        // find out consecutive items with equal keys
        // and take the minimum of their positions over the newly defined key sets, i.e:
        //   pos  :  1  2  3  4  5  6  7  8  9
        //   keys : (1, 1, 2, 3, 3, 3, 4, 4, 4) ->
        //   out  : (1, 1, 3, 4, 4, 4, 7, 7, 7)
        NVBIO_VAR_UNUSED const uint32 n_segments = extract_segments(
            n_suffixes,
            nvbio::raw_pointer( d_sort_keys ),
            nvbio::raw_pointer( d_segment_flags ),
            nvbio::raw_pointer( d_segment_blocks ),
            nvbio::raw_pointer( d_active_slots ),
            nvbio::raw_pointer( d_segment_keys ),
            nvbio::raw_pointer( d_segment_heads ) );

        NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
        timer.stop();
        segment_time += timer.seconds();

        timer.start();

        // now scatter the new keys to obtain the inverse keys
        thrust::scatter(
            thrust::make_permutation_iterator( d_segment_heads.begin(), d_segment_keys.begin() ),
            thrust::make_permutation_iterator( d_segment_heads.begin(), d_segment_keys.begin() ) + n_suffixes,
            d_sort_indices.begin(),
            d_inv_keys.begin() );

        NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
        timer.stop();
        inverse_time += timer.seconds();
    }

    // keep track of the number of active suffixes
    uint32 n_active_suffixes = n_suffixes;

    for (uint32 j = SYMBOLS_PER_WORD; j < string_len; j *= 2u)
    {
    #define CSS_KEY_PRUNING
    #if defined(CSS_KEY_PRUNING)
        //
        // extract all the strings from segments with more than 1 element: in order to do this we want to mark
        // all the elements that need to be copied. These are exactly all the 1's followed by a 0, and all the
        // 0's. Conversely, all the 1's followed by a 1 don't need to copied.
        // i.e. we want to transform the flags:
        //  (1,1,0,0,0,1,1,1,0,1) in the vector:
        //  (0,1,1,1,1,0,0,1,1,0)
        // This can be done again looking at neighbours, this time with a custom binary-predicate implementing
        // the above logic.
        // As thrust::adjacent_difference is right-aligned, we'll obtain the following situation instead:
        //  segment-flags: (1,1,0,0,0,1,1,1,0,1,[1])     (where [1] is a manually inserted sentinel value)
        //  segment-keys:(1,0,1,1,1,1,0,0,1,1,0,[.])
        // where we have added a sentinel value to the first vector, and will find our results starting
        // at position one in the output
        //
        thrust::adjacent_difference(
            d_segment_flags.begin(),
            d_segment_flags.begin() + n_active_suffixes+1u,
            d_partial_flags.begin() + 3u,
            priv::remove_singletons() );

        // and scan them to get their candidate output positions
        thrust::inclusive_scan(
            d_partial_flags.begin() + 4u,
            d_partial_flags.begin() + 4u + n_active_suffixes,
            d_segment_keys.begin(),
            thrust::plus<uint32>() );
            
        const uint32 n_partials = d_segment_keys[ n_active_suffixes-1u ];

        //NVBIO_CUDA_DEBUG_STATEMENT( fprintf(stderr,"    partial: %.3f%%\n", 100.0f*float(n_partials)/float(n_suffixes) ) );

        // check if the number of "unique" keys is small enough to justify reducing the active set
        //if (2u*n_segments >= n_active_suffixes)
        if (3u*n_partials <= n_active_suffixes*2u)
        {
            //NVBIO_CUDA_DEBUG_STATEMENT( fprintf(stderr,"\n    segments: %.3f%%, at pass %u\n", 100.0f*float(n_segments)/float(n_suffixes), word_idx) );

            Timer timer;
            timer.start();

            // before shrinking the set of active suffixes scatter the already sorted indices to the output
            // in their proper place
            thrust::scatter_if(
                d_sort_indices.begin(),                                                                     // input begin
                d_sort_indices.begin() + n_active_suffixes,                                                 // input end
                d_active_slots.begin(),                                                                     // map
                thrust::make_transform_iterator( d_partial_flags.begin() + 4u, is_false_functor<uint32>() ),// stencil
                d_suffixes );                                                                               // output

            // now keep only the slots we are interested in
            compact(
                n_active_suffixes,
                nvbio::raw_pointer( d_partial_flags ) + 4u,
                nvbio::raw_pointer( d_segment_keys ),
                nvbio::raw_pointer( d_segment_flags ),
                nvbio::raw_pointer( d_active_slots ),
                nvbio::raw_pointer( d_sort_indices ),
                nvbio::raw_pointer( d_temp_flags ),
                nvbio::raw_pointer( d_sort_keys ),
                nvbio::raw_pointer( d_segment_heads ) );

            // and swap the buffers
            d_segment_flags.swap( d_temp_flags );
            d_active_slots.swap( d_sort_keys );
            d_sort_indices.swap( d_segment_heads );

            // overwrite the number of active suffixes
            n_active_suffixes = n_partials;

            NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
            timer.stop();
            compact_time += timer.seconds();

            NVBIO_CUDA_DEBUG_STATEMENT( fprintf(stderr,"      active: %.3f%%\n", 100.0f*float(n_active_suffixes)/float(n_suffixes) ) );
        }
      #endif // if defined(CSS_KEY_PRUNING)

        NVBIO_CUDA_DEBUG_STATEMENT( fprintf(stderr,"    secondary key sorting [%u]\n", j) );
        Timer timer;
        timer.start();

        // build the new sorting keys
        //
        //   - d_keys encodes an array K*, such that K*[i] = K[SA[i]] is the key currently bound to suffix i;
        //
        //   - in order to perform prefix-doubling, we want to replace K[i] with K[i+j],
        //     but we have to modify K* instead, because that's all we have got;
        //
        //   - so we have to replace K*[n] with:
        //         K[SA[n]+j] = K*[invSA[SA[n]+j]]
        //
        prefix_doubling(
            n_active_suffixes,
            n_suffixes,
            j,
            nvbio::raw_pointer( d_sort_indices ),
            nvbio::raw_pointer( d_inv_keys ),
            nvbio::raw_pointer( d_sort_keys ) );

        NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
        timer.stop();
        gather_time += timer.seconds();

        timer.start();

        uint32* d_comp_flags = (uint32*)nvbio::raw_pointer( d_temp_flags );

        // build the compressed flags
        priv::pack_flags(
            n_active_suffixes,
            nvbio::raw_pointer( d_segment_flags ),
            d_comp_flags );

        // sort within segments
        mgpu::SegSortPairsFromFlags(
            nvbio::raw_pointer( d_sort_keys ),
            nvbio::raw_pointer( d_sort_indices ),
            n_active_suffixes,
            d_comp_flags,
            *m_mgpu );

        NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
        timer.stop();
        radixsort_time += timer.seconds();

        timer.start();

        // find out consecutive items with equal keys, merging them with the parent's
        // and take the minimum of their positions over the newly defined key sets, i.e:
        //   pos  :  1  2  3  4  5  6  7  8  9
        //   keys : (1, 1, 2, 3, 3, 3, 4, 4, 4) ->
        //   out  : (1, 1, 3, 4, 4, 4, 7, 7, 7)
        const uint32 n_segments = extract_segments(
            n_active_suffixes,
            nvbio::raw_pointer( d_sort_keys ),
            nvbio::raw_pointer( d_segment_flags ),
            nvbio::raw_pointer( d_segment_blocks ),
            nvbio::raw_pointer( d_active_slots ),
            nvbio::raw_pointer( d_segment_keys ),
            nvbio::raw_pointer( d_segment_heads ) );

        if (n_segments == n_active_suffixes) // we are done!
        {
            // scatter the sorted indices to the output in their proper place
            thrust::scatter(
                d_sort_indices.begin(),
                d_sort_indices.begin() + n_active_suffixes,
                d_active_slots.begin(),
                d_suffixes );

            break; // bail out of the sorting loop
        }

        NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
        timer.stop();
        segment_time += timer.seconds();

        timer.start();

        // now scatter the new keys to obtain the inverse keys
        thrust::scatter(
            thrust::make_permutation_iterator( d_segment_heads.begin(), d_segment_keys.begin() ),
            thrust::make_permutation_iterator( d_segment_heads.begin(), d_segment_keys.begin() ) + n_active_suffixes,
            d_sort_indices.begin(),
            d_inv_keys.begin() );

        NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
        timer.stop();
        inverse_time += timer.seconds();
    }
}

} // namespace cuda
} // namespace nvbio
