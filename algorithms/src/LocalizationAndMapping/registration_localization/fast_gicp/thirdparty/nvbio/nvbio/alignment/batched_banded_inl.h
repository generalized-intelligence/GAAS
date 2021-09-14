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

#include <nvbio/basic/types.h>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/alignment/utils.h>
#include <nvbio/basic/cuda/work_queue.h>
#include <nvbio/basic/strided_iterator.h>
#include <nvbio/alignment/batched_stream.h>

namespace nvbio {
namespace aln {

///@addtogroup private
///@{

template <uint32 BAND_LEN, typename stream_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void batched_banded_alignment_score(const stream_type& stream, const uint32 work_id)
{
    typedef typename stream_type::aligner_type  aligner_type;
    typedef typename stream_type::context_type  context_type;
    typedef typename stream_type::strings_type  strings_type;

    // load the alignment context
    context_type context;
    if (stream.init_context( work_id, &context ) == true)
    {
        // compute the end of the current DP matrix window
        const uint32 len = equal<typename aligner_type::algorithm_tag,TextBlockingTag>() ?
            stream.text_length( work_id, &context ) :
            stream.pattern_length( work_id, &context );

        // load the strings to be aligned
        strings_type strings;
        stream.load_strings( work_id, 0, len, &context, &strings );

        // score the current DP matrix window
        banded_alignment_score<BAND_LEN>(
            stream.aligner(),
            strings.pattern,
            strings.quals,
            strings.text,
            context.min_score,
            context.sink );
    }

    // handle the output
    stream.output( work_id, &context );
}

template <uint32 BLOCKDIM, uint32 MINBLOCKS, uint32 BAND_LEN, typename stream_type>
__global__ void
__launch_bounds__(BLOCKDIM,MINBLOCKS)
batched_banded_alignment_score_kernel(const stream_type stream)
{
    const uint32 tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= stream.size())
        return;

    batched_banded_alignment_score<BAND_LEN>( stream, tid );
}

///@} // end of private group

///
/// HostThreadScheduler specialization of BatchedBandedAlignmentScore.
///
/// \tparam stream_type     the stream of alignment jobs
///
template <uint32 BAND_LEN, typename stream_type>
struct BatchedBandedAlignmentScore<BAND_LEN,stream_type,HostThreadScheduler>
{
    static const uint32 BLOCKDIM = 128;

    typedef typename stream_type::aligner_type                  aligner_type;
    typedef typename column_storage_type<aligner_type>::type    cell_type;

    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size) { return 0u; }

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size) { return 0u; }

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL);
};

// enact the batch execution
//
template <uint32 BAND_LEN, typename stream_type>
void BatchedBandedAlignmentScore<BAND_LEN,stream_type,HostThreadScheduler>::enact(stream_type stream, uint64 temp_size, uint8* temp)
{
  #if defined(_OPENMP)
    #pragma omp parallel for
  #endif
    for (int tid = 0; tid < int( stream.size() ); ++tid)
        batched_banded_alignment_score<BAND_LEN>( stream, tid );
}

///
/// DeviceThreadScheduler specialization of BatchedBandedAlignmentScore.
///
/// \tparam stream_type     the stream of alignment jobs
///
template <uint32 BLOCKDIM, uint32 MINBLOCKS, uint32 BAND_LEN, typename stream_type>
struct BatchedBandedAlignmentScore<BAND_LEN,stream_type,DeviceThreadBlockScheduler<BLOCKDIM,MINBLOCKS> >
{
    typedef typename stream_type::aligner_type                  aligner_type;
    typedef typename column_storage_type<aligner_type>::type    cell_type;

    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size) { return 0u; }

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size) { return 0u; }

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL);
};

// enact the batch execution
//
template <uint32 BLOCKDIM, uint32 MINBLOCKS, uint32 BAND_LEN, typename stream_type>
void BatchedBandedAlignmentScore<BAND_LEN,stream_type,DeviceThreadBlockScheduler<BLOCKDIM,MINBLOCKS> >::enact(stream_type stream, uint64 temp_size, uint8* temp)
{
    const uint32 n_blocks = (stream.size() + BLOCKDIM-1) / BLOCKDIM;

    batched_banded_alignment_score_kernel<BLOCKDIM,MINBLOCKS,BAND_LEN> <<<n_blocks, BLOCKDIM>>>( stream );
}


///
/// DeviceStagedThreadScheduler specialization of BatchedBandedAlignmentScore.
///
/// \tparam stream_type     the stream of alignment jobs
///
template <uint32 BAND_LEN, typename stream_type>
struct BatchedBandedAlignmentScore<BAND_LEN,stream_type,DeviceStagedThreadScheduler>
{
    static const uint32 BLOCKDIM = 128;

    typedef typename stream_type::aligner_type                      aligner_type;
    typedef typename checkpoint_storage_type<aligner_type>::type    cell_type;

    /// return the per-element column storage size
    ///
    static uint32 column_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        const uint32 column_size = uint32( BAND_LEN * sizeof(cell_type) );

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

        thrust::device_vector<uint8> temp_dvec;
        if (temp == NULL)
        {
            temp_size = nvbio::max( min_temp_size, temp_size );
            temp_dvec.resize( temp_size );
            temp = nvbio::device_view( temp_dvec );
        }

        // set the queue capacity based on available memory
        const uint32 max_pattern_len = stream.max_pattern_length();
        const uint32 max_text_len    = stream.max_text_length();
        const uint32 queue_capacity  = uint32( temp_size / column_storage( max_pattern_len, max_text_len ) );

        m_work_queue.set_capacity( queue_capacity );

        // prepare the work stream
        ScoreStream<stream_type> score_stream(
            stream,                 // the alignments stream
            temp,                   // band storage
            NULL,                   // no need for checkpoints
            queue_capacity );       // the queue capacity, used for the memory striding

        // consume the work stream
        m_work_queue.consume( score_stream );
    }

private:
    cuda::WorkQueue<
        cuda::PersistentThreadsQueueTag,
        BandedScoreUnit<BAND_LEN, stream_type>,
        BLOCKDIM>                               m_work_queue;
};

// --- Banded Traceback --------------------------------------------------------------------------------------------------------- //

///@addtogroup private
///@{

template <uint32 BAND_LEN, uint32 CHECKPOINTS, typename stream_type, typename cell_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void batched_banded_alignment_traceback(stream_type& stream, cell_type* checkpoints, uint32* submatrices, const uint32 stride, const uint32 work_id, const uint32 thread_id)
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

    // fetch the proper checkpoint storage
    typedef strided_iterator<cell_type*> checkpoint_type;
    checkpoint_type checkpoint = checkpoint_type( checkpoints + thread_id, stride );

    // fetch the proper submatrix storage
    typedef strided_iterator<uint32*> submatrix_storage_type;
    submatrix_storage_type submatrix_storage = submatrix_storage_type( submatrices + thread_id, stride );
    const uint32 BITS = direction_vector_traits<aligner_type>::BITS;
    PackedStream<submatrix_storage_type,uint8,BITS,false> submatrix( submatrix_storage );

    // score the current DP matrix window
    context.alignment = banded_alignment_traceback<BAND_LEN, CHECKPOINTS>(
        stream.aligner(),
        strings.pattern,
        strings.quals,
        strings.text,
        context.min_score,
        context.backtracer,
        checkpoint,
        submatrix );

    // handle the output
    stream.output( work_id, &context );
}

template <uint32 BLOCKDIM, uint32 BAND_LEN, uint32 CHECKPOINTS, typename stream_type, typename cell_type>
__global__ void batched_banded_alignment_traceback_kernel(stream_type stream, cell_type* checkpoints, uint32* submatrices, const uint32 stride)
{
    const uint32 tid = blockIdx.x * BLOCKDIM + threadIdx.x;

    if (tid >= stream.size())
        return;

    batched_banded_alignment_traceback<BAND_LEN, CHECKPOINTS>( stream, checkpoints, submatrices, stride, tid, tid );
}

template <uint32 BLOCKDIM, uint32 BAND_LEN, uint32 CHECKPOINTS, typename stream_type, typename cell_type>
__global__ void persistent_banded_batched_alignment_traceback_kernel(stream_type stream, cell_type* checkpoints, uint32* submatrices, const uint32 stride)
{
    const uint32 grid_threads = gridDim.x * BLOCKDIM;
    const uint32 thread_id    = threadIdx.x + blockIdx.x*BLOCKDIM;

    const uint32 stream_end = stream.size();

    // let this CTA fetch all tiles at a grid-threads stride, starting from blockIdx.x*BLOCKDIM
    for (uint32 stream_begin = 0; stream_begin < stream_end; stream_begin += grid_threads)
    {
        const uint32 work_id = thread_id + stream_begin;

        if (work_id < stream_end)
            batched_banded_alignment_traceback<BAND_LEN, CHECKPOINTS>( stream, checkpoints, submatrices, stride, work_id, thread_id );
    }
}

///@} // end of private group

///
/// DeviceThreadScheduler specialization of BatchedAlignmentTraceback.
///
/// \tparam stream_type     the stream of alignment jobs
///
template <uint32 BAND_LEN, uint32 CHECKPOINTS, typename stream_type>
struct BatchedBandedAlignmentTraceback<BAND_LEN,CHECKPOINTS, stream_type,DeviceThreadScheduler>
{
    static const uint32 BLOCKDIM = 128;

    typedef typename stream_type::aligner_type                  aligner_type;
    typedef typename column_storage_type<aligner_type>::type    cell_type;

    /// return the per-element checkpoint storage size
    ///
    static uint32 checkpoint_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        return align<4>( uint32( BAND_LEN * ((max_pattern_len + CHECKPOINTS-1) / CHECKPOINTS) * sizeof(cell_type) ) );
    }

    /// return the per-element submatrix storage size
    ///
    static uint32 submatrix_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        typedef typename stream_type::aligner_type  aligner_type;
        const uint32 BITS = direction_vector_traits<aligner_type>::BITS;
        const uint32 ELEMENTS_PER_WORD = 32 / BITS;
        return ((BAND_LEN * CHECKPOINTS + ELEMENTS_PER_WORD-1) / ELEMENTS_PER_WORD) * sizeof(uint32);
    }

    /// return the per-element storage size
    ///
    static uint32 element_storage(const uint32 max_pattern_len, const uint32 max_text_len)
    {
        return checkpoint_storage( max_pattern_len, max_text_len ) +
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
template <uint32 BAND_LEN, uint32 CHECKPOINTS, typename stream_type>
uint64 BatchedBandedAlignmentTraceback<BAND_LEN,CHECKPOINTS, stream_type,DeviceThreadScheduler>::min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
{
    return element_storage( max_pattern_len, max_text_len ) * 1024;
}

// return the maximum number of bytes required by the algorithm
//
template <uint32 BAND_LEN, uint32 CHECKPOINTS, typename stream_type>
uint64 BatchedBandedAlignmentTraceback<BAND_LEN,CHECKPOINTS,stream_type,DeviceThreadScheduler>::max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size)
{
    return element_storage( max_pattern_len, max_text_len ) * stream_size;
}

// enact the batch execution
//
template <uint32 BAND_LEN, uint32 CHECKPOINTS, typename stream_type>
void BatchedBandedAlignmentTraceback<BAND_LEN,CHECKPOINTS,stream_type,DeviceThreadScheduler>::enact(stream_type stream, uint64 temp_size, uint8* temp)
{
    const uint64 min_temp_size = min_temp_storage(
        stream.max_pattern_length(),
        stream.max_text_length(),
        stream.size() );

    thrust::device_vector<uint8> temp_dvec;
    if (temp_size == 0u)
    {
        temp_dvec.resize( min_temp_size );
        temp = nvbio::device_view( temp_dvec );
        temp_size = min_temp_size;
    }

    // set the queue capacity based on available memory
    const uint32 max_pattern_len = stream.max_pattern_length();
    const uint32 max_text_len    = stream.max_text_length();
    const uint32 queue_capacity  = uint32( temp_size / element_storage( max_pattern_len, max_text_len ) );

    const uint64 checkpoints_size = checkpoint_storage( max_pattern_len, max_text_len );

    if (queue_capacity >= stream.size())
    {
        const uint32 n_blocks = (stream.size() + BLOCKDIM-1) / BLOCKDIM;

        cell_type* checkpoints = (cell_type*)(temp);
        uint32*    submatrices = (uint32*)   (temp + checkpoints_size * stream.size());

        batched_banded_alignment_traceback_kernel<BLOCKDIM,BAND_LEN,CHECKPOINTS> <<<n_blocks, BLOCKDIM>>>(
            stream,
            checkpoints,
            submatrices,
            stream.size() );
    }
    else
    {
        // compute the number of blocks we are going to launch
        const uint32 n_blocks = nvbio::max( nvbio::min(
            (uint32)cuda::max_active_blocks( persistent_banded_batched_alignment_traceback_kernel<BLOCKDIM,BAND_LEN,CHECKPOINTS,stream_type,cell_type>, BLOCKDIM, 0u ),
            queue_capacity / BLOCKDIM ), 1u );

        cell_type* checkpoints = (cell_type*)(temp);
        uint32*    submatrices = (uint32*)   (temp + checkpoints_size * queue_capacity);

        persistent_banded_batched_alignment_traceback_kernel<BLOCKDIM,BAND_LEN,CHECKPOINTS> <<<n_blocks, BLOCKDIM>>>(
            stream,
            checkpoints,
            submatrices,
            queue_capacity );
    }
}

///@} // end of BatchAlignment group

///@} // end of the Alignment group

} // namespace alignment
} // namespace nvbio
