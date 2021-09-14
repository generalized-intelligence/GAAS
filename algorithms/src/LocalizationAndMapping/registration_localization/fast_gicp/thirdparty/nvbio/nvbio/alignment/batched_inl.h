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

#include <nvbio/alignment/utils.h>
#include <nvbio/alignment/batched_stream.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/basic/cuda/work_queue.h>
#include <nvbio/basic/strided_iterator.h>
#include <nvbio/basic/vector.h>
#include <nvbio/strings/prefetcher.h>
#if defined(_OPENMP)
#include <omp.h>
#endif

namespace nvbio {
namespace aln {

///@addtogroup private
///@{

template <typename stream_type, typename column_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void batched_alignment_score(stream_type& stream, column_type column, const uint32 work_id, const uint32 thread_id)
{
    typedef typename stream_type::aligner_type  aligner_type;
    typedef typename stream_type::context_type  context_type;
    typedef typename stream_type::strings_type  strings_type;

    // load the alignment context
    context_type context;
    if (stream.init_context( work_id, &context ) == false)
    {
        // handle the output
        stream.output( work_id, &context );
        return;
    }

    // compute the end of the current DP matrix window
    const uint32 len = equal<typename aligner_type::algorithm_tag,PatternBlockingTag>() ?
        stream.pattern_length( work_id, &context ) :
        stream.text_length( work_id, &context );

    // load the strings to be aligned
    strings_type strings;
    stream.load_strings( work_id, 0, len, &context, &strings );

    // score the current DP matrix window
    alignment_score(
        stream.aligner(),
        strings.pattern,
        strings.quals,
        strings.text,
        context.min_score,
        context.sink,
        column );

    // handle the output
    stream.output( work_id, &context );
}

template <uint32 BLOCKDIM, uint32 MINBLOCKS, uint32 COLUMN_SIZE, typename stream_type, typename cell_type>
__global__ void
__launch_bounds__(BLOCKDIM,MINBLOCKS)
lmem_batched_alignment_score_kernel(stream_type stream, cell_type* columns, const uint32 stride)
{
    const uint32 tid = blockIdx.x * BLOCKDIM + threadIdx.x;

    if (tid >= stream.size())
        return;

    // fetch the proper column storage
    cell_type column[COLUMN_SIZE];

    batched_alignment_score( stream, column, tid, tid );
}

template <uint32 BLOCKDIM, uint32 MINBLOCKS, typename stream_type, typename cell_type>
__global__ void
__launch_bounds__(BLOCKDIM,MINBLOCKS)
batched_alignment_score_kernel(stream_type stream, cell_type* columns, const uint32 stride)
{
    const uint32 tid = blockIdx.x * BLOCKDIM + threadIdx.x;

    if (tid >= stream.size())
        return;

    // fetch the proper column storage
    typedef strided_iterator<cell_type*> column_type;
    column_type column = column_type( columns + tid, stride );

    batched_alignment_score( stream, column, tid, tid );
}

template <uint32 BLOCKDIM, uint32 MINBLOCKS, typename stream_type, typename cell_type>
__global__ void
__launch_bounds__(BLOCKDIM,MINBLOCKS)
persistent_batched_alignment_score_kernel(stream_type stream, cell_type* columns, const uint32 stride)
{
    const uint32 grid_threads = gridDim.x * BLOCKDIM;
    const uint32 thread_id    = threadIdx.x + blockIdx.x*BLOCKDIM;

    const uint32 stream_end = stream.size();

    // fetch the proper column storage
    typedef strided_iterator<cell_type*> column_type;
    column_type column = column_type( columns + thread_id, stride );

    // let this CTA fetch all tiles at a grid-threads stride, starting from blockIdx.x*BLOCKDIM
    for (uint32 stream_begin = 0; stream_begin < stream_end; stream_begin += grid_threads)
    {
        const uint32 work_id = thread_id + stream_begin;

        if (work_id < stream_end)
            batched_alignment_score( stream, column, work_id, thread_id );
    }
}

template <uint32 BLOCKDIM, typename stream_type, typename cell_type>
NVBIO_FORCEINLINE NVBIO_DEVICE
void warp_batched_alignment_score(stream_type& stream, cell_type* columns, const uint32 stride, const uint32 work_id, const uint32 warp_id)
{
    typedef typename stream_type::aligner_type  aligner_type;
    typedef typename stream_type::context_type  context_type;
    typedef typename stream_type::strings_type  strings_type;

    // load the alignment context
    context_type context;
    if (stream.init_context( work_id, &context ) == false)
    {
        if (warp_tid() == 0)
        {
            // handle the output
            stream.output( work_id, &context );
        }
        return;
    }

    // compute the end of the current DP matrix window
    const uint32 len = equal<typename aligner_type::algorithm_tag,PatternBlockingTag>() ?
        stream.pattern_length( work_id, &context ) :
        stream.text_length( work_id, &context );

    // load the strings to be aligned
    strings_type strings;
    stream.load_strings( work_id, 0, len, &context, &strings );

    // fetch the proper column storage
    typedef block_strided_iterator<cuda::Arch::WARP_SIZE,cell_type*> column_type;
    column_type column = column_type( columns + warp_id * cuda::Arch::WARP_SIZE, stride );

    // score the current DP matrix window
    uint2 sink;
    const int32 score = warp::alignment_score<BLOCKDIM>(
        stream.aligner(),
        strings.pattern,
        strings.quals,
        strings.text,
        context.min_score,
        &sink,
        column );

    if (warp_tid() == 0)
    {
        context.sink.report( score, sink );

        // handle the output
        stream.output( work_id, &context );
    }
}

template <uint32 BLOCKDIM, typename stream_type, typename cell_type>
__global__ void warp_batched_alignment_score_kernel(stream_type stream, cell_type* columns, const uint32 stride)
{
    const uint32 wid = (blockIdx.x * BLOCKDIM + threadIdx.x) >> cuda::Arch::LOG_WARP_SIZE;

    if (wid >= stream.size())
        return;

    warp_batched_alignment_score<BLOCKDIM>( stream, columns, stride, wid, wid );
}

template <uint32 BLOCKDIM, typename stream_type, typename cell_type>
__global__ void warp_persistent_batched_alignment_score_kernel(stream_type stream, cell_type* columns, const uint32 stride)
{
    const uint32 grid_warps = (gridDim.x * BLOCKDIM) >> cuda::Arch::LOG_WARP_SIZE;
    const uint32 wid        = (threadIdx.x + blockIdx.x*BLOCKDIM) >> cuda::Arch::LOG_WARP_SIZE;

    const uint32 stream_end = stream.size();

    // let this CTA fetch all tiles at a grid-warps stride
    for (uint32 work_id = wid; work_id < stream_end; work_id += grid_warps)
        warp_batched_alignment_score<BLOCKDIM>( stream, columns, stride, work_id, wid );
}

///@} // end of private group

///@addtogroup Alignment
///@{

///
///@addtogroup BatchAlignment
///@{

///
/// HostThreadScheduler specialization of BatchedAlignmentScore.
///
/// \tparam stream_type     the stream of alignment jobs
///
template <typename stream_type>
struct BatchedAlignmentScore<stream_type,HostThreadScheduler>
{
    static const uint32 MAX_THREADS = 128; // whatever CPU we have, we assume we are never going to have more than this number of threads

    typedef typename stream_type::aligner_type                  aligner_type;
    typedef typename column_storage_type<aligner_type>::type    cell_type;

    /// return the per-element column storage size
    ///
    static uint32 column_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        const uint32 column_size = equal<typename aligner_type::algorithm_tag,PatternBlockingTag>() ?
            uint32( max_text_len    * sizeof(cell_type) ) :
            uint32( max_pattern_len * sizeof(cell_type) );

        return align<4>( column_size );
    }

    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL);
};

// return the minimum number of bytes required by the algorithm
//
template <typename stream_type>
uint64 BatchedAlignmentScore<stream_type,HostThreadScheduler>::min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
{
    return column_storage( max_pattern_len, max_text_len ) * MAX_THREADS;
}

// return the maximum number of bytes required by the algorithm
//
template <typename stream_type>
uint64 BatchedAlignmentScore<stream_type,HostThreadScheduler>::max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
{
    return column_storage( max_pattern_len, max_text_len ) * MAX_THREADS;
}

// enact the batch execution
//
template <typename stream_type>
void BatchedAlignmentScore<stream_type,HostThreadScheduler>::enact(stream_type stream, uint64 temp_size, uint8* temp)
{
    const uint32 column_size = equal<typename aligner_type::algorithm_tag,PatternBlockingTag>() ?
        uint32( stream.max_pattern_length() ) :
        uint32( stream.max_text_length() );

    const uint64 min_temp_size = min_temp_storage(
        stream.max_pattern_length(),
        stream.max_text_length(),
        stream.size() );

    nvbio::vector<host_tag,uint8> temp_vec( min_temp_size );
    cell_type* columns = (cell_type*)nvbio::raw_pointer( temp_vec );

    #if defined(_OPENMP)
    #pragma omp parallel for
    #endif
    for (int work_id = 0; work_id < int( stream.size() ); ++work_id)
    {
      #if defined(_OPENMP)
        const uint32 thread_id = omp_get_thread_num();
      #else
        const uint32 thread_id = 0;
      #endif

        // fetch the proper column storage
        //typedef strided_iterator<cell_type*> column_type;
        //column_type column = column_type( columns + thread_id, queue_capacity );

        // for the CPU it might be better to keep column storage contiguous
        cell_type* column = columns + thread_id * column_size;

        // and solve the actual alignment problem
        batched_alignment_score( stream, column, work_id, thread_id );
    }
}

///
/// DeviceThreadScheduler specialization of BatchedAlignmentScore.
///
/// \tparam stream_type     the stream of alignment jobs
///
template <uint32 BLOCKDIM, uint32 MINBLOCKS, typename stream_type>
struct BatchedAlignmentScore<stream_type,DeviceThreadBlockScheduler<BLOCKDIM,MINBLOCKS> >
{
    typedef typename stream_type::aligner_type                  aligner_type;
    typedef typename column_storage_type<aligner_type>::type    cell_type;

    /// return the per-element column storage size
    ///
    static uint32 column_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        const uint32 column_size = equal<typename aligner_type::algorithm_tag,PatternBlockingTag>() ?
            uint32( max_text_len    * sizeof(cell_type) ) :
            uint32( max_pattern_len * sizeof(cell_type) );

        return align<4>( column_size );
    }

    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL);
};

// return the minimum number of bytes required by the algorithm
//
template <uint32 BLOCKDIM, uint32 MINBLOCKS, typename stream_type>
uint64 BatchedAlignmentScore<stream_type,DeviceThreadBlockScheduler<BLOCKDIM,MINBLOCKS> >::min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
{
    return column_storage( max_pattern_len, max_text_len ) * 1024;
}

// return the maximum number of bytes required by the algorithm
//
template <uint32 BLOCKDIM, uint32 MINBLOCKS, typename stream_type>
uint64 BatchedAlignmentScore<stream_type,DeviceThreadBlockScheduler<BLOCKDIM,MINBLOCKS> >::max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
{
    return align<32>( column_storage( max_pattern_len, max_text_len ) * stream_size );
}

// enact the batch execution
//
template <uint32 BLOCKDIM, uint32 MINBLOCKS, typename stream_type>
void BatchedAlignmentScore<stream_type,DeviceThreadBlockScheduler<BLOCKDIM,MINBLOCKS> >::enact(stream_type stream, uint64 temp_size, uint8* temp)
{
    const uint32 column_size = equal<typename aligner_type::algorithm_tag,PatternBlockingTag>() ?
        uint32( stream.max_text_length() ) :
        uint32( stream.max_pattern_length() );

    // if the column is small, let's just use statically allocated local memory
    if (column_size <= 1024)
    {
        const uint32 n_blocks = (stream.size() + BLOCKDIM-1) / BLOCKDIM;

        lmem_batched_alignment_score_kernel<BLOCKDIM,MINBLOCKS,1024> <<<n_blocks, BLOCKDIM>>>(
            stream,
            (cell_type*)temp,
            stream.size() );
    }
    else
    {
        // the column is large to be allocated in local memory, let's use global memory
        const uint64 min_temp_size = min_temp_storage(
            stream.max_pattern_length(),
            stream.max_text_length(),
            stream.size() );

        nvbio::vector<device_tag,uint8> temp_vec;
        if (temp == NULL)
        {
            temp_size = nvbio::max( min_temp_size, temp_size );
            temp_vec.resize( temp_size );
            temp = nvbio::plain_view( temp_vec );
        }

        // set the queue capacity based on available memory
        const uint32 queue_capacity = uint32( temp_size / column_storage( stream.max_pattern_length(), stream.max_text_length() ) );

        if (queue_capacity >= stream.size())
        {
            const uint32 n_blocks = (stream.size() + BLOCKDIM-1) / BLOCKDIM;

            batched_alignment_score_kernel<BLOCKDIM,MINBLOCKS> <<<n_blocks, BLOCKDIM>>>(
                stream,
                (cell_type*)temp,
                stream.size() );
        }
        else
        {
            // compute the number of blocks we are going to launch
            const uint32 n_blocks = nvbio::max( nvbio::min(
                (uint32)cuda::max_active_blocks( persistent_batched_alignment_score_kernel<BLOCKDIM,MINBLOCKS,stream_type,cell_type>, BLOCKDIM, 0u ),
                queue_capacity / BLOCKDIM ), 1u );

            persistent_batched_alignment_score_kernel<BLOCKDIM,MINBLOCKS> <<<n_blocks, BLOCKDIM>>>(
                stream,
                (cell_type*)temp,
                queue_capacity );
        }
    }
}

///
/// DeviceWarpScheduler specialization of BatchedAlignmentScore.
///
/// \tparam stream_type     the stream of alignment jobs
///
template <typename stream_type>
struct BatchedAlignmentScore<stream_type,DeviceWarpScheduler>
{
    static const uint32 BLOCKDIM = 128;

    typedef typename stream_type::aligner_type                  aligner_type;
    typedef typename column_storage_type<aligner_type>::type    cell_type;

    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL);
};

// return the minimum number of bytes required by the algorithm
//
template <typename stream_type>
uint64 BatchedAlignmentScore<stream_type,DeviceWarpScheduler>::min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
{
    return max_text_len * sizeof(cell_type) * 1024;
}

// return the maximum number of bytes required by the algorithm
//
template <typename stream_type>
uint64 BatchedAlignmentScore<stream_type,DeviceWarpScheduler>::max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
{
    return max_text_len * sizeof(cell_type) * stream_size;
}

// enact the batch execution
//
template <typename stream_type>
void BatchedAlignmentScore<stream_type,DeviceWarpScheduler>::enact(stream_type stream, uint64 temp_size, uint8* temp)
{
    const uint64 min_temp_size = min_temp_storage(
        stream.max_pattern_length(),
        stream.max_text_length(),
        stream.size() );

    nvbio::vector<device_tag,uint8> temp_vec;
    if (temp == NULL)
    {
        temp_size = nvbio::max( min_temp_size, temp_size );
        temp_vec.resize( temp_size );
        temp = nvbio::plain_view( temp_vec );
    }

    NVBIO_VAR_UNUSED static const uint32 WARP_SIZE = cuda::Arch::WARP_SIZE;

    // set the queue capacity based on available memory
    const uint32 queue_capacity = align_down<WARP_SIZE>( uint32( temp_size / (align<WARP_SIZE>( stream.max_text_length() ) * sizeof(cell_type)) ) );

    const uint32 BLOCKWARPS = BLOCKDIM >> cuda::Arch::LOG_WARP_SIZE;
    if (queue_capacity >= stream.size())
    {
        const uint32 n_warps  = stream.size();
        const uint32 n_blocks = (n_warps + BLOCKWARPS-1) / BLOCKWARPS;

        warp_batched_alignment_score_kernel<BLOCKDIM> <<<n_blocks, BLOCKDIM>>>(
            stream,
            (cell_type*)temp,
            align<WARP_SIZE>( stream.size() ) ); // make sure everything is properly aligned
    }
    else
    {
        // compute the number of blocks we are going to launch
        const uint32 n_blocks = nvbio::max( nvbio::min(
            (uint32)cuda::max_active_blocks( warp_persistent_batched_alignment_score_kernel<BLOCKDIM,stream_type,cell_type>, BLOCKDIM, 0u ),
            queue_capacity / BLOCKDIM ), 1u );

        warp_persistent_batched_alignment_score_kernel<BLOCKDIM> <<<n_blocks, BLOCKDIM>>>(
            stream,
            (cell_type*)temp,
            queue_capacity );
    }
}

///
/// DeviceStagedThreadScheduler specialization of BatchedAlignmentScore.
///
/// \tparam stream_type     the stream of alignment jobs
///
template <typename stream_type>
struct BatchedAlignmentScore<stream_type,DeviceStagedThreadScheduler>
{
    static const uint32 BLOCKDIM = 128;

    typedef typename stream_type::aligner_type                      aligner_type;
    typedef typename checkpoint_storage_type<aligner_type>::type    cell_type;

    /// return the per-element column storage size
    ///
    static uint32 column_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        const uint32 column_size = equal<typename aligner_type::algorithm_tag,PatternBlockingTag>() ?
            uint32( max_text_len    * sizeof(cell_type) ) :
            uint32( max_pattern_len * sizeof(cell_type) );

        return align<4>( column_size );
    }

    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
    {
        return column_storage( max_pattern_len, max_text_len ) * 1024;
    }

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
    {
        return column_storage( max_pattern_len, max_text_len ) * stream_size;
    }

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL)
    {
        const uint64 min_temp_size = min_temp_storage(
            stream.max_pattern_length(),
            stream.max_text_length(),
            stream.size() );

        nvbio::vector<device_tag,uint8> temp_vec;
        if (temp == NULL)
        {
            temp_size = nvbio::max( min_temp_size, temp_size );
            temp_vec.resize( temp_size );
            temp = nvbio::plain_view( temp_vec );
        }

        // set the queue capacity based on available memory
        const uint32 max_pattern_len = stream.max_pattern_length();
        const uint32 max_text_len    = stream.max_text_length();
        const uint32 queue_capacity  = uint32( temp_size / column_storage( max_pattern_len, max_text_len ) );

        m_work_queue.set_capacity( queue_capacity );

        // prepare the work stream
        ScoreStream<stream_type> score_stream(
            stream,
            temp,
            NULL,
            queue_capacity );

        // consume the work stream
        m_work_queue.consume( score_stream );
    }

private:
    cuda::WorkQueue<
        cuda::PersistentThreadsQueueTag,
        StagedScoreUnit<stream_type>,
        BLOCKDIM>                               m_work_queue;
};

// --- Traceback --------------------------------------------------------------------------------------------------------- //

///@addtogroup private
///@{

template <uint32 CHECKPOINTS, typename stream_type, typename cell_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void batched_alignment_traceback(stream_type& stream, cell_type* checkpoints, uint32* submatrices, cell_type* columns, const uint32 stride, const uint32 work_id, const uint32 thread_id)
{
    typedef typename stream_type::aligner_type  aligner_type;
    typedef typename stream_type::context_type  context_type;
    typedef typename stream_type::strings_type  strings_type;

    // load the alignment context
    context_type context;
    if (stream.init_context( work_id, &context ) == false)
    {
        // handle the output
        stream.output( work_id, &context );
        return;
    }

    // compute the end of the current DP matrix window
    const uint32 pattern_len = stream.pattern_length( work_id, &context );

    // load the strings to be aligned
    strings_type strings;
    stream.load_strings( work_id, 0, pattern_len, &context, &strings );

    // fetch the proper checkpoint storage
    typedef strided_iterator<cell_type*> checkpoint_type;
    checkpoint_type checkpoint = checkpoint_type( checkpoints + thread_id, stride );

    // fetch the proper submatrix storage
    typedef strided_iterator<uint32*> submatrix_storage_type;
    submatrix_storage_type submatrix_storage = submatrix_storage_type( submatrices + thread_id, stride );
    const uint32 BITS = direction_vector_traits<aligner_type>::BITS;
    PackedStream<submatrix_storage_type,uint8,BITS,false> submatrix( submatrix_storage );

    // fetch the proper column storage
    typedef strided_iterator<cell_type*> column_type;
    column_type column = column_type( columns + thread_id, stride );

    // score the current DP matrix window
    context.alignment = alignment_traceback<CHECKPOINTS>(
        stream.aligner(),
        strings.pattern,
        strings.quals,
        strings.text,
        context.min_score,
        context.backtracer,
        checkpoint,
        submatrix,
        column );

    // handle the output
    stream.output( work_id, &context );
}

template <uint32 BLOCKDIM, uint32 MINBLOCKS, uint32 CHECKPOINTS, typename stream_type, typename cell_type>
__global__ void
__launch_bounds__(BLOCKDIM,MINBLOCKS)
batched_alignment_traceback_kernel(stream_type stream, cell_type* checkpoints, uint32* submatrices, cell_type* columns, const uint32 stride)
{
    const uint32 tid = blockIdx.x * BLOCKDIM + threadIdx.x;

    if (tid >= stream.size())
        return;

    batched_alignment_traceback<CHECKPOINTS>( stream, checkpoints, submatrices, columns, stride, tid, tid );
}

template <uint32 BLOCKDIM, uint32 MINBLOCKS, uint32 CHECKPOINTS, typename stream_type, typename cell_type>
__global__ void
__launch_bounds__(BLOCKDIM,MINBLOCKS)
persistent_batched_alignment_traceback_kernel(stream_type stream, cell_type* checkpoints, uint32* submatrices, cell_type* columns, const uint32 stride)
{
    const uint32 grid_threads = gridDim.x * BLOCKDIM;
    const uint32 thread_id    = threadIdx.x + blockIdx.x*BLOCKDIM;

    const uint32 stream_end = stream.size();

    // let this CTA fetch all tiles at a grid-threads stride, starting from blockIdx.x*BLOCKDIM
    for (uint32 stream_begin = 0; stream_begin < stream_end; stream_begin += grid_threads)
    {
        const uint32 work_id = thread_id + stream_begin;

        if (work_id < stream_end)
            batched_alignment_traceback<CHECKPOINTS>( stream, checkpoints, submatrices, columns, stride, work_id, thread_id );
    }
}

///@} // end of private group

///
/// DeviceThreadScheduler specialization of BatchedAlignmentTraceback.
///
/// \tparam stream_type     the stream of alignment jobs
///
template <uint32 BLOCKDIM, uint32 MINBLOCKS, uint32 CHECKPOINTS, typename stream_type>
struct BatchedAlignmentTraceback<CHECKPOINTS, stream_type,DeviceThreadBlockScheduler<BLOCKDIM,MINBLOCKS> >
{
    typedef typename stream_type::aligner_type                  aligner_type;
    typedef typename column_storage_type<aligner_type>::type    cell_type;

    /// return the per-element column storage size
    ///
    static uint32 column_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        const uint32 column_size = equal<typename aligner_type::algorithm_tag,PatternBlockingTag>() ?
            uint32( max_text_len    * sizeof(cell_type) ) :
            uint32( max_pattern_len * sizeof(cell_type) );

        return align<4>( column_size );
    }

    /// return the per-element checkpoint storage size
    ///
    static uint32 checkpoint_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        if (equal<typename aligner_type::algorithm_tag,PatternBlockingTag>())
            return align<4>( uint32( max_text_len * ((max_pattern_len + CHECKPOINTS-1) / CHECKPOINTS) * sizeof(cell_type) ) );
        else
            return align<4>( uint32( max_pattern_len * ((max_text_len + CHECKPOINTS-1) / CHECKPOINTS) * sizeof(cell_type) ) );
    }

    /// return the per-element storage size
    ///
    static uint32 submatrix_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        if (equal<typename aligner_type::algorithm_tag,PatternBlockingTag>())
        {
            typedef typename stream_type::aligner_type  aligner_type;
            const uint32 BITS = direction_vector_traits<aligner_type>::BITS;
            const uint32 ELEMENTS_PER_WORD = 32 / BITS;
            return ((max_text_len * CHECKPOINTS + ELEMENTS_PER_WORD-1) / ELEMENTS_PER_WORD) * sizeof(uint32);
        }
        else
        {
            typedef typename stream_type::aligner_type  aligner_type;
            const uint32 BITS = direction_vector_traits<aligner_type>::BITS;
            const uint32 ELEMENTS_PER_WORD = 32 / BITS;
            return ((max_pattern_len * CHECKPOINTS + ELEMENTS_PER_WORD-1) / ELEMENTS_PER_WORD) * sizeof(uint32);
        }
    }

    /// return the per-element storage size
    ///
    static uint32 element_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        return     column_storage( max_pattern_len, max_text_len ) +
               checkpoint_storage( max_pattern_len, max_text_len ) +
                submatrix_storage( max_pattern_len, max_text_len );
    }

    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL);
};

// return the minimum number of bytes required by the algorithm
//
template <uint32 BLOCKDIM, uint32 MINBLOCKS, uint32 CHECKPOINTS, typename stream_type>
uint64 BatchedAlignmentTraceback<CHECKPOINTS, stream_type,DeviceThreadBlockScheduler<BLOCKDIM,MINBLOCKS> >::min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
{
    return element_storage( max_pattern_len, max_text_len ) * 1024;
}

// return the maximum number of bytes required by the algorithm
//
template <uint32 BLOCKDIM, uint32 MINBLOCKS, uint32 CHECKPOINTS, typename stream_type>
uint64 BatchedAlignmentTraceback<CHECKPOINTS,stream_type,DeviceThreadBlockScheduler<BLOCKDIM,MINBLOCKS> >::max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
{
    return element_storage( max_pattern_len, max_text_len ) * stream_size;
}

// enact the batch execution
//
template <uint32 BLOCKDIM, uint32 MINBLOCKS, uint32 CHECKPOINTS, typename stream_type>
void BatchedAlignmentTraceback<CHECKPOINTS,stream_type,DeviceThreadBlockScheduler<BLOCKDIM,MINBLOCKS> >::enact(stream_type stream, uint64 temp_size, uint8* temp)
{
    const uint64 min_temp_size = min_temp_storage(
        stream.max_pattern_length(),
        stream.max_text_length(),
        stream.size() );

    nvbio::vector<device_tag,uint8> temp_vec;
    if (temp == NULL)
    {
        temp_size = nvbio::max( min_temp_size, temp_size );
        temp_vec.resize( temp_size );
        temp = nvbio::plain_view( temp_vec );
    }

    // set the queue capacity based on available memory
    const uint32 max_pattern_len = stream.max_pattern_length();
    const uint32 max_text_len    = stream.max_text_length();
    const uint32 queue_capacity  = uint32( temp_size / element_storage( max_pattern_len, max_text_len ) );

    const uint64 column_size      =     column_storage( max_pattern_len, max_text_len );
    const uint64 checkpoints_size = checkpoint_storage( max_pattern_len, max_text_len );

    if (queue_capacity >= stream.size())
    {
        const uint32 n_blocks = (stream.size() + BLOCKDIM-1) / BLOCKDIM;

        cell_type* checkpoints = (cell_type*)(temp);
        cell_type* columns     = (cell_type*)(temp + (checkpoints_size)               * stream.size());
        uint32*    submatrices = (uint32*)   (temp + (checkpoints_size + column_size) * stream.size());

        batched_alignment_traceback_kernel<BLOCKDIM,MINBLOCKS,CHECKPOINTS> <<<n_blocks, BLOCKDIM>>>(
            stream,
            checkpoints,
            submatrices,
            columns,
            stream.size() );
    }
    else
    {
        // compute the number of blocks we are going to launch
        const uint32 n_blocks = nvbio::max( nvbio::min(
            (uint32)cuda::max_active_blocks( persistent_batched_alignment_traceback_kernel<BLOCKDIM,MINBLOCKS,CHECKPOINTS,stream_type,cell_type>, BLOCKDIM, 0u ),
            queue_capacity / BLOCKDIM ), 1u );

        cell_type* checkpoints = (cell_type*)(temp);
        cell_type* columns     = (cell_type*)(temp + (checkpoints_size)               * queue_capacity);
        uint32*    submatrices = (uint32*)   (temp + (checkpoints_size + column_size) * queue_capacity);

        persistent_batched_alignment_traceback_kernel<BLOCKDIM,MINBLOCKS,CHECKPOINTS> <<<n_blocks, BLOCKDIM>>>(
            stream,
            checkpoints,
            submatrices,
            columns,
            queue_capacity );
    }
}

namespace priv {

//
// An alignment stream class to be used in conjunction with the BatchAlignmentScore class
//
template <
    typename t_aligner_type,
    typename pattern_set_type,
    typename qualities_set_type,
    typename text_set_type,
    typename sink_iterator>
struct AlignmentStream
{
    typedef t_aligner_type                              aligner_type;

    typedef typename pattern_set_type::string_type                          input_pattern_string;
    typedef typename text_set_type::string_type                             input_text_string;
    typedef StringPrefetcher<input_pattern_string, lmem_cache_tag<128> >    pattern_prefetcher_type;
    typedef typename pattern_prefetcher_type::string_type                   pattern_string;
    typedef StringPrefetcher<input_text_string, lmem_cache_tag<128> >       text_prefetcher_type;
    typedef typename text_prefetcher_type::string_type                      text_string;
    typedef typename qualities_set_type::string_type                        quals_string;
    typedef typename std::iterator_traits<sink_iterator>::value_type        sink_type;

    // an alignment context
    struct context_type
    {
        int32                   min_score;
        sink_type               sink;
    };
    // a container for the strings to be aligned
    struct strings_type
    {
        pattern_prefetcher_type pattern_prefetcher;
        text_prefetcher_type    text_prefetcher;

        pattern_string          pattern;
        quals_string            quals;
        text_string             text;
    };

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    AlignmentStream(
        aligner_type                _aligner,
        const uint32                _count,
        const pattern_set_type      _patterns,
        const qualities_set_type    _quals,
        const text_set_type         _texts,
              sink_iterator         _sinks,
        const uint32                _max_pattern_length,
        const uint32                _max_text_length) :
        m_aligner               ( _aligner ),
        m_count                 (_count),
        m_patterns              (_patterns),
        m_quals                 (_quals),
        m_texts                 (_texts),
        m_sinks                 (_sinks),
        m_max_pattern_length    ( _max_pattern_length ),
        m_max_text_length       ( _max_text_length ) {}

    // get the aligner
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const aligner_type& aligner() const { return m_aligner; };

    // return the maximum pattern length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_pattern_length() const { return m_max_pattern_length; }

    // return the maximum text length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_text_length() const { return m_max_text_length; }

    // return the stream size
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_count; }

    // return the i-th pattern's length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 pattern_length(const uint32 i, context_type* context) const { return m_patterns[i].length(); }

    // return the i-th text's length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 text_length(const uint32 i, context_type* context) const { return m_texts[i].length(); }

    // initialize the i-th context
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool init_context(
        const uint32    i,
        context_type*   context) const
    {
        // initialize the sink
        context->sink = sink_type();

        context->min_score = Field_traits<int32>::min();
        return true;
    }

    // initialize the i-th context
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void load_strings(
        const uint32        i,
        const uint32        window_begin,
        const uint32        window_end,
        const context_type* context,
              strings_type* strings) const
    {
        strings->pattern = strings->pattern_prefetcher.load( m_patterns[i] );
        strings->quals   = m_quals[i];
        strings->text    = strings->text_prefetcher.load( m_texts[i] );
    }

    // handle the output
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void output(
        const uint32        i,
        const context_type* context) const
    {
        // copy the sink
        m_sinks[i] = context->sink;
    }

    aligner_type        m_aligner;
    uint32              m_count;
    pattern_set_type    m_patterns;
    qualities_set_type  m_quals;
    text_set_type       m_texts;
    sink_iterator       m_sinks;
    const uint32        m_max_pattern_length;
    const uint32        m_max_text_length;
};

} // namespace priv

//
// A convenience function for aligning a batch of patterns to a corresponding batch of texts.
//
template <
    typename aligner_type,
    typename pattern_set_type,
    typename text_set_type,
    typename sink_iterator,
    typename scheduler_type>
void batch_alignment_score(
    const aligner_type      aligner,
    const pattern_set_type  patterns,
    const text_set_type     texts,
          sink_iterator     sinks,
    const scheduler_type    scheduler,
    const uint32            max_pattern_length,
    const uint32            max_text_length)
{
    typedef priv::AlignmentStream<aligner_type,pattern_set_type,trivial_quality_string_set,text_set_type,sink_iterator> stream_type;

    typedef aln::BatchedAlignmentScore<stream_type, scheduler_type> batch_type;  // our batch type

    // create the stream
    stream_type stream(
        aligner,
        patterns.size(),
        patterns,
        trivial_quality_string_set(),
        texts,
        sinks,
        max_pattern_length,
        max_text_length );

    // enact the batch
    batch_type batch;
    batch.enact( stream );
}

//
// A convenience function for aligning a batch of patterns to a corresponding batch of texts.
//
template <
    typename aligner_type,
    typename pattern_set_type,
    typename qualities_set_type,
    typename text_set_type,
    typename sink_iterator,
    typename scheduler_type>
void batch_alignment_score(
    const aligner_type          aligner,
    const pattern_set_type      patterns,
    const qualities_set_type    quals,
    const text_set_type         texts,
          sink_iterator         sinks,
    const scheduler_type        scheduler,
    const uint32                max_pattern_length,
    const uint32                max_text_length)
{
    typedef priv::AlignmentStream<aligner_type,pattern_set_type,qualities_set_type,text_set_type,sink_iterator> stream_type;

    typedef aln::BatchedAlignmentScore<stream_type, scheduler_type> batch_type;  // our batch type

    // create the stream
    stream_type stream(
        aligner,
        patterns.size(),
        patterns,
        quals,
        texts,
        sinks,
        max_pattern_length,
        max_text_length );

    // enact the batch
    batch_type batch;
    batch.enact( stream );
}

//
// A convenience function for aligning a batch of patterns to a corresponding batch of texts.
//
template <
    uint32   BAND_LEN,
    typename aligner_type,
    typename pattern_set_type,
    typename text_set_type,
    typename sink_iterator,
    typename scheduler_type>
void batch_banded_alignment_score(
    const aligner_type      aligner,
    const pattern_set_type  patterns,
    const text_set_type     texts,
          sink_iterator     sinks,
    const scheduler_type    scheduler,
    const uint32            max_pattern_length,
    const uint32            max_text_length)
{
    typedef priv::AlignmentStream<aligner_type,pattern_set_type,trivial_quality_string_set,text_set_type,sink_iterator> stream_type;

    typedef aln::BatchedBandedAlignmentScore<BAND_LEN, stream_type, scheduler_type> batch_type;  // our batch type

    // create the stream
    stream_type stream(
        aligner,
        patterns.size(),
        patterns,
        trivial_quality_string_set(),
        texts,
        sinks,
        max_pattern_length,
        max_text_length );

    // enact the batch
    batch_type batch;
    batch.enact( stream );
}

///@} // end of BatchAlignment group

///@} // end of the Alignment group

} // namespace aln
} // namespace nvbio
