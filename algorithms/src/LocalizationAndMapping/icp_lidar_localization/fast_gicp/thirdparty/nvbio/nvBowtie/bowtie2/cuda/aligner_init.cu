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

#include <nvBowtie/bowtie2/cuda/aligner.h>
#include <nvbio/basic/primitives.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

template <typename T>
T* resize(bool do_alloc, thrust::device_vector<T>& vec, const uint32 size, uint64& bytes)
{
    bytes += size * sizeof(T);
    if (do_alloc)
    {
        vec.resize( size );
        return thrust::raw_pointer_cast(&vec.front());
    }
    return NULL;
}

template <typename T>
T* resize(bool do_alloc, thrust::host_vector<T>& vec, const uint32 size, uint64& bytes)
{
    bytes += size * sizeof(T);
    if (do_alloc)
    {
        vec.resize( size );
        return thrust::raw_pointer_cast(&vec.front());
    }
    return NULL;
}

template <typename T>
T* resize(bool do_alloc, std::vector<T>& vec, const uint32 size, uint64& bytes)
{
    bytes += size * sizeof(T);
    if (do_alloc)
    {
        vec.resize( size );
        return &vec[0];
    }
    return NULL;
}

bool Aligner::init_alloc(const uint32 BATCH_SIZE, const Params& params, const EndType type, bool do_alloc, std::pair<uint64,uint64>* mem_stats)
{
    size_t free, total;
    cudaMemGetInfo(&free, &total);

    //const uint32 band_len = (type == kPairedEnds) ? MAXIMUM_BAND_LENGTH : band_length( params.max_dist );

    uint64 d_allocated_bytes = 0;
    uint64 h_allocated_bytes = 0;

    // alloc the seeding queues
    d_allocated_bytes += seed_queues.resize_arena( BATCH_SIZE, do_alloc );

    // alloc the hit deques
    d_allocated_bytes += hit_deques.resize( BATCH_SIZE, params.max_hits, do_alloc );
    if (params.randomized)
        rseeds_dptr = resize( do_alloc, rseeds_dvec, BATCH_SIZE, d_allocated_bytes );

    // alloc the scoring queues
    d_allocated_bytes += scoring_queues.resize( BATCH_SIZE, BATCH_SIZE, do_alloc ); // TODO: pass read-end 'type' field

    idx_queue_dptr = resize( do_alloc, idx_queue_dvec,      BATCH_SIZE*2,   d_allocated_bytes ); // 2x for ping-pong buffers
    if (params.mode != AllMapping)
                     resize( do_alloc, sorting_queue_dvec,  BATCH_SIZE*2,   d_allocated_bytes ); // 2x for ping-pong buffers
    else
                     resize( do_alloc, sorting_queue_dvec,  BATCH_SIZE*2*4, d_allocated_bytes ); // 4x for 64-bit sorting

    nvbio::cuda::check_error("allocating queues");

    mapq_dptr = resize( do_alloc, mapq_dvec, BATCH_SIZE, d_allocated_bytes );

    if (params.mode != AllMapping)
    {
        reseed_dptr    = resize( do_alloc, reseed_dvec,     BATCH_SIZE,     d_allocated_bytes );
        trys_dptr      = resize( do_alloc, trys_dvec,       BATCH_SIZE,     d_allocated_bytes );
        best_data_dptr = resize( do_alloc, best_data_dvec,  BATCH_SIZE*2,   d_allocated_bytes );

        if (type == kPairedEnds)
            best_data_dptr_o = resize( do_alloc, best_data_dvec_o,  BATCH_SIZE*2, d_allocated_bytes );
    }
    else
    {
        // in all-mapping mode we store the temporary output in a double-buffered ring-buffer
        buffer_alignments_dptr = resize( do_alloc, buffer_alignments_dvec,  BATCH_SIZE*2,   d_allocated_bytes );
        buffer_read_info_dptr  = resize( do_alloc, buffer_read_info_dvec,   BATCH_SIZE*2,   d_allocated_bytes );

        output_alignments_dptr = resize( do_alloc, output_alignments_dvec,  BATCH_SIZE,     d_allocated_bytes );
        output_read_info_dptr  = resize( do_alloc, output_read_info_dvec,   BATCH_SIZE,     d_allocated_bytes );

        flags_dptr = resize( do_alloc, flags_dvec, BATCH_SIZE, d_allocated_bytes );
    }
    nvbio::cuda::check_error("allocating output buffers");

    hits_stats_dptr = resize( do_alloc, hits_stats_dvec, 128, d_allocated_bytes );
                      resize( do_alloc, hits_stats_hvec, 128, d_allocated_bytes );

    if (params.mode == AllMapping)
    {
        hits_count_scan_dptr = resize( do_alloc, hits_count_scan_dvec,     BATCH_SIZE+1,                       d_allocated_bytes );
        hits_range_scan_dptr = resize( do_alloc, hits_range_scan_dvec,     params.max_hits * BATCH_SIZE+1,     d_allocated_bytes );
    }

    //const uint32 n_cigar_entries = BATCH_SIZE*(MAXIMUM_BAND_LEN_MULT*band_len+1);
    //const uint32 n_mds_entries   = BATCH_SIZE*MAX_READ_LEN;
    const uint32 n_cigar_entries = (128 * BATCH_SIZE)/sizeof(io::Cigar);    // 256MB
    const uint32 n_mds_entries   = (256 * BATCH_SIZE)/sizeof(uint8);        // 256MB
    if (do_alloc)
    {
        log_verbose(stderr, "[%u]     allocating %u MB of string storage\n[%u]       CIGARs : %u MB\n[%u]       MDs    : %u MB\n",
            ID, uint32(n_cigar_entries * sizeof(io::Cigar) + n_mds_entries)/(1024*1024),
            ID, uint32(n_cigar_entries * sizeof(io::Cigar))/(1024*1024),
            ID, n_mds_entries/(1024*1024) );
    }

    // allocate CIGARs & MDs
    d_allocated_bytes += cigar.resize( BATCH_SIZE, n_cigar_entries, do_alloc );
    d_allocated_bytes += mds.resize(   BATCH_SIZE, n_mds_entries,   do_alloc );

    // allocate CIGAR coords
    cigar_coords_dptr = resize( do_alloc, cigar_coords_dvec, BATCH_SIZE, d_allocated_bytes );
    nvbio::cuda::check_error("allocating CIGARs");

    if (type == kPairedEnds)
    {
        // allocate the device queue
        opposite_queue_dptr = resize( do_alloc, opposite_queue_dvec, BATCH_SIZE, d_allocated_bytes );
    }

    // allocate DP storage
    uint32 dp_storage = 0;

    {
        //
        // allocate two thirds of available device memory for scoring / traceback
        //

        const uint32 bytes_per_read = uint32( float( params.avg_read_length ) * 1.5f );

        const uint64 read_mem = bytes_per_read * BATCH_SIZE * (type == kPairedEnds ? 2u : 1u);
                                                  // assume 250B per read

        const uint64 guard_band = 512*1024*1024 + // we want to leave 512MB free,
                                  read_mem;       // needed for kernels using lmem

        const uint64 min_dp_storage = 64*1024*1024; // minimum amount of DP storage

        if (do_alloc)
            cudaMemGetInfo(&free, &total);
        else if (free >= d_allocated_bytes + guard_band + min_dp_storage)
            free -= d_allocated_bytes;
        else
            return false;

        const uint32 free_words    = uint32( free / 4u );
        const uint32 min_free_words = uint32( guard_band / 4u );

        uint32 target_words  = (free_words * 2u) / 3u;
               target_words  = nvbio::min( target_words, free_words - min_free_words );

        const uint32 buffer_words = target_words;
        if (do_alloc)
            log_verbose(stderr, "[%u]     allocating %u MB of DP storage\n", ID, (buffer_words*4)/(1024*1024) );

        dp_storage = buffer_words * sizeof(uint32);
    }

    // allocate a large temporary buffer to for scoring and traceback
    dp_buffer_dptr = resize( do_alloc, dp_buffer_dvec, dp_storage, d_allocated_bytes );

    nvbio::cuda::check_error("allocating alignment buffers");

    if (mem_stats)
    {
        mem_stats->first  = h_allocated_bytes;
        mem_stats->second = d_allocated_bytes;
    }
    return true;
}

bool Aligner::init(const uint32 id, const uint32 batch_size, const Params& params, const EndType type)
{
    ID         = id;
    BATCH_SIZE = batch_size;

    // initialize the batch number
    batch_number = 0;

    try {
        std::pair<uint64,uint64> mem_stats;

        init_alloc( batch_size, params, type, false, &mem_stats );

        log_stats(stderr, "[%u]   allocating alignment buffers... started\n[%u]     estimated: HOST %lu MB, DEVICE %lu MB)\n",
            ID,
            ID,
            mem_stats.first / (1024*1024),
            mem_stats.second / (1024*1024) );

        init_alloc( batch_size, params, type, true, &mem_stats );

        log_stats(stderr, "[%u]   allocating alignment buffers... done\n[%u]     allocated: HOST %lu MB, DEVICE %lu MB)\n",
            ID,
            ID,
            mem_stats.first / (1024*1024),
            mem_stats.second / (1024*1024) );
    }
    catch (...) {
        log_error(stderr, "[%u]   allocating alignment buffers failed!\n", ID);
        return false;
    }
    return true;
}

// Compute the total number of matches found
__global__ 
void hits_stats_kernel(
    const uint32 batch_size,
    const SeedHit* hit_data,
    const uint32*  hit_counts,
          uint64*  hit_stats)
{
    const uint32 read_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (read_id >= batch_size) return;

    const uint32 hit_ranges = hit_counts[ read_id ];

    strided_iterator<const SeedHit*> hits( hit_data+read_id, batch_size );

    typedef vector_view< strided_iterator<const SeedHit*> > Storage;
    typedef priority_deque< SeedHit, Storage, hit_compare > HitQueue;
    Storage qStore( hit_ranges, hits );
    HitQueue hitheap( qStore, HitQueue::CONSTRUCTED );

    __shared__ uint32 shared_max_range;
    __shared__ uint32 shared_max_hits;
    __shared__ uint32 shared_top_max_hits;

    shared_max_range   = 0;
    shared_max_hits       = 0;
    shared_top_max_hits   = 0;

    __syncthreads();

    uint32 hits_cnt  = 0;
    uint32 max_range = 0;
    for (uint32 i = 0; i < hit_ranges; ++i)
    {
        hits_cnt += hits[i].get_range_size();
        max_range = nvbio::max( max_range, hits[i].get_range_size() );
    }

    const SeedHit top     = hit_ranges ? hitheap.top()                         : SeedHit();
    const uint32  top_cnt = hit_ranges ? top.get_range().y - top.get_range().x : 0u;

    // update the number of ranges and number of total hits
    atomicAdd( hit_stats + HIT_STATS_RANGES, uint64(hit_ranges) );
    atomicAdd( hit_stats + HIT_STATS_TOTAL, uint64(hits_cnt) );
    atomicAdd( hit_stats + HIT_STATS_TOP,   uint64(top_cnt) );

    // bin the number of hits, and update the bin counter
    const uint32 log_hits = hits_cnt == 0 ? 0u : nvbio::log2( hits_cnt )+1u;
    atomicAdd( hit_stats + HIT_STATS_BINS + log_hits, uint64(1u) );

    // bin the number of top hits, and update the bin counter
    const uint32 log_top_hits = top_cnt == 0 ? 0u : nvbio::log2( top_cnt )+1u;
    atomicAdd( hit_stats + HIT_STATS_TOP_BINS + log_top_hits, uint64(1u) );

    // update the maximum
    if (shared_max_range < max_range)
        atomicMax( &shared_max_range, max_range );

    // update the maximum
    if (shared_max_hits < hits_cnt)
        atomicMax( &shared_max_hits, hits_cnt );

    // update the maximum
    if (shared_top_max_hits < top_cnt)
        atomicMax( &shared_top_max_hits, top_cnt );

    __syncthreads();

    // update the maximum
    if (threadIdx.x == 0)
    {
        if (hit_stats[ HIT_STATS_MAX_RANGE ] < shared_max_range)
            atomicMax( (uint32*)(hit_stats + HIT_STATS_MAX_RANGE), shared_max_range );
        if (hit_stats[ HIT_STATS_MAX ] < shared_max_hits)
            atomicMax( (uint32*)(hit_stats + HIT_STATS_MAX), shared_max_hits );
        if (hit_stats[ HIT_STATS_TOP_MAX ] < shared_top_max_hits)
            atomicMax( (uint32*)(hit_stats + HIT_STATS_TOP_MAX), shared_top_max_hits );
    }
}

void Aligner::keep_stats(const uint32 count, Stats& stats)
{
    thrust::fill( hits_stats_dvec.begin(), hits_stats_dvec.end(), 0u );
    hits_stats(
        count,
        nvbio::device_view( hit_deques.hits() ),
        nvbio::device_view( hit_deques.counts() ),
        hits_stats_dptr );

    cudaThreadSynchronize();
    nvbio::cuda::check_error("hit stats kernel");

    nvbio::cuda::thrust_copy_vector(hits_stats_hvec, hits_stats_dvec);

    // poll until previous stats have been consumed
    //while (output_thread.stats.stats_ready) {}

    stats.hits_ranges    += hits_stats_hvec[ HIT_STATS_RANGES ];
    stats.hits_total     += hits_stats_hvec[ HIT_STATS_TOTAL ];
    stats.hits_max        = std::max( stats.hits_max, uint32( hits_stats_hvec[ HIT_STATS_MAX ] ) );
    stats.hits_max_range  = std::max( stats.hits_max_range, uint32( hits_stats_hvec[ HIT_STATS_MAX_RANGE ] ) );
    stats.hits_top_total += hits_stats_hvec[ HIT_STATS_TOP ];
    stats.hits_top_max    = std::max( stats.hits_top_max, uint32( hits_stats_hvec[ HIT_STATS_TOP_MAX ] ) );
    for (uint32 i = 0; i < 28; ++i)
    {
        stats.hits_bins[i]     += hits_stats_hvec[ HIT_STATS_BINS + i ];
        stats.hits_top_bins[i] += hits_stats_hvec[ HIT_STATS_TOP_BINS + i ];
    }
    stats.hits_stats++;

    // mark stats as ready to be consumed
    //output_thread.stats.stats_ready = true;
}

// Compute the total number of matches found
void hits_stats(
    const uint32    batch_size,
    const SeedHit*  hit_data,
    const uint32*   hit_counts,
          uint64*   hit_stats)
{
    const uint32 blocks = (batch_size + BLOCKDIM-1) / BLOCKDIM;

    hits_stats_kernel<<<blocks, BLOCKDIM>>>( batch_size, hit_data, hit_counts, hit_stats );
}

// copy the contents of a section of a ring buffer into a plain array
struct ring_buffer_to_plain_array_functor
{
    // constructor
    ring_buffer_to_plain_array_functor(
        const uint32* _buffer,
        const uint32  _buffer_size,
        const uint32  _begin,
        const uint32  _end,
              uint32* _output)
    : buffer( _buffer ),
      buffer_size( _buffer_size ),
      begin( _begin ),
      end( _end ),
      output( _output ) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 i) const
    {
        if (begin + i < end)
            output[i] = buffer[ (begin + i) % buffer_size ];
    }

    const uint32* buffer;
    const uint32  buffer_size;
    const uint32  begin;
    const uint32  end;
          uint32* output;
};

void ring_buffer_to_plain_array(
    const uint32* buffer,
    const uint32  buffer_size,
    const uint32  begin,
    const uint32  end,
          uint32* output)
{
    nvbio::for_each<device_tag>(
        end - begin,
        thrust::make_counting_iterator<uint32>(0),
        ring_buffer_to_plain_array_functor(
            buffer,
            buffer_size,
            begin,
            end,
            output ) );
}

// initialize a set of alignments
//
__global__
void mark_unaligned_kernel(
    const uint32            n_active_reads,
    const uint32*           active_reads,
    const io::Alignment*    best_data,
    uint8*                  reseed)
{
    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= n_active_reads) return;

    // fetch the read
    const uint32 read_id = active_reads[ thread_id ];

    // mark if unaligned
    if (!best_data[ read_id ].is_aligned())
        reseed[ thread_id ] = 1u;
}

// mark unaligned reads that need reseeding
//
void mark_unaligned(
    const uint32            n_active_reads,
    const uint32*           active_reads,
    const io::Alignment*    best_data,
    uint8*                  reseed)
{
    const int blocks = (n_active_reads + BLOCKDIM-1) / BLOCKDIM;

    mark_unaligned_kernel<<<blocks, BLOCKDIM>>>(
        n_active_reads,
        active_reads,
        best_data,
        reseed );
}

// initialize a set of alignments
//
__global__
void mark_discordant_kernel(
    const uint32            n_reads,
          io::Alignment*    anchor_data,
          io::Alignment*    opposite_data,
    const uint32            stride)
{
    const uint32 read_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (read_id >= n_reads) return;

    // A pair of alignments is considered "discordant" if they are unique and
    // they are not concordant.
    if (anchor_data[ read_id            ].is_concordant() == false && // anchor is   : unpaired
        anchor_data[ read_id            ].is_aligned()    == true  && //               aligned
        anchor_data[ read_id + stride   ].is_aligned()    == false && //               unique
        opposite_data[ read_id          ].is_aligned()    == true  && // opposite is : aligned
        opposite_data[ read_id + stride ].is_aligned()    == false)   //               unique
    {
        // mark as paired and discordant
        anchor_data[ read_id ].m_paired       = 1u;
        anchor_data[ read_id ].m_discordant   = 1u;
        opposite_data[ read_id ].m_paired     = 1u;
        opposite_data[ read_id ].m_discordant = 1u;
    }
}

// mark unique unaligned read pairs as discordant
//
void mark_discordant(
    const uint32            n_reads,
          io::Alignment*    anchor_data,
          io::Alignment*    opposite_data,
    const uint32            stride)
{
    const int blocks = (n_reads + BLOCKDIM-1) / BLOCKDIM;

    mark_discordant_kernel<<<blocks, BLOCKDIM>>>(
        n_reads,
        anchor_data,
        opposite_data,
        stride );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
