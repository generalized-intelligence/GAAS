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

#include <nvBowtie/bowtie2/cuda/persist.h>
#include <nvBowtie/bowtie2/cuda/seed_hit.h>
#include <nvBowtie/bowtie2/cuda/seed_hit_deque_array.h>
#include <crc/crc.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

namespace {

std::string mate_file_name(const std::string& file_name, const uint32 anchor)
{
    const size_t dot = file_name.find('.');
    if (dot == std::string::npos)
        return file_name + (anchor ? ".2" : ".1");
    else
    {
        std::string output = file_name;
        output.insert( dot, (anchor ? ".2" : ".1") );
        return output;
    }
}

} // anonymous namespace

// clear the persisting files
//
void persist_clear(const std::string& file_name)
{
    // clear mate 0
    {
        const std::string mate_file = mate_file_name( file_name, 0u );
        FILE* file = fopen( mate_file.c_str(), "w" ); fclose( file );
    }
    // clear mate 1
    {
        const std::string mate_file = mate_file_name( file_name, 0u );
        FILE* file = fopen( mate_file.c_str(), "w" ); fclose( file );
    }
}

// compute a set of hits
//
void persist_hits(
    const std::string&                              file_name,
    const char*                                     name,
    const uint32                                    anchor,
    const uint32                                    count,
    const SeedHitDequeArray&                        hit_deques)
{
    // check whether we need to persist anything
    if (file_name == "")
        return;

    const std::string mate_file = mate_file_name( file_name, anchor );

    FILE* file = fopen( mate_file.c_str(), "a+" );

    SeedHitDequeArrayHostStorage hit_deques_h = static_cast<const SeedHitDequeArrayDeviceStorage&>( hit_deques );

    fprintf( file, "\n---------------------------------------------------------------------------------\n" );
    fprintf( file, "%s\n", name );
    fprintf( file, "---------------------------------------------------------------------------------\n" );
    for (uint32 i = 0; i < count; ++i)
        fprintf( file, "cnt[%u] = %u\n", i, uint32(hit_deques_h.m_counts[i]) );
    fprintf( file, "\n" );

    for (uint32 i = 0; i < count; ++i)
    {
        const uint32 n_ranges = hit_deques_h.m_counts[i];
        const SeedHit* hits   = &hit_deques_h.m_hits[0] + hit_deques_h.m_index[i];

        fprintf( file, "hits[%u] = [%u]{\n", i, n_ranges );
        for (uint32 j = 0; j < n_ranges; ++j)
        {
            const SeedHit hit = hits[j];
            fprintf( file, "  range[%u] = { (%u, %u - %u), dir[%u], pos[%u], rc[%u] }\n",
                j, hit.get_range().x, hit.get_range().y, hit.get_range().y - hit.get_range().x,
                hit.get_indexdir(),
                hit.get_posinread(),
                hit.get_readtype() );
        }
        fprintf( file, "}\n" );
    }
    fprintf( file, "\n" );

    fclose( file );
}

// persist a set of reads
//
void persist_reads(
    const std::string&                              file_name,
    const char*                                     name,
    const uint32                                    anchor,
    const uint32                                    count,
    const thrust::device_vector<uint32>::iterator   iterator)
{
    // check whether we need to persist anything
    if (file_name == "")
        return;

    const std::string mate_file = mate_file_name( file_name, anchor );

    FILE* file = fopen( mate_file.c_str(), "a+" );

    thrust::host_vector<int32> hvec( count );
    thrust::copy(
        iterator,
        iterator + count,
        hvec.begin() );

    fprintf( file, "\n---------------------------------------------------------------------------------\n" );
    fprintf( file, "%s\n", name );
    fprintf( file, "---------------------------------------------------------------------------------\n" );
    for (uint32 i = 0; i < count; ++i)
        fprintf( file, "read[%u] = %u\n", i, hvec[i] );
    fprintf( file, "\n" );

    fclose( file );
}

// persist a set of selected hits
//
void persist_selection(
    const std::string&                              file_name,
    const char*                                     name,
    const uint32                                    anchor,
    const uint32                                    read_count,
    const packed_read*                              read_infos_dptr,
    const uint32                                    n_multi,
    const uint32                                    hits_queue_size,
    const ReadHitsIndex&                            hits_index,
    const HitQueues&                                hits_queue)
{
    // check whether we need to persist anything
    if (file_name == "")
        return;

    const std::string mate_file = mate_file_name( file_name, anchor );

    FILE* file = fopen( mate_file.c_str(), "a+" );

    fprintf( file, "\n---------------------------------------------------------------------------------\n" );
    fprintf( file, "%s\n", name );
    fprintf( file, "---------------------------------------------------------------------------------\n" );

    thrust::host_vector<uint32>      loc_vec( hits_queue.loc );
    thrust::host_vector<packed_seed> seed_vec( hits_queue.seed );

    if (n_multi > 1)
    {
        const uint32 link_stride = hits_index.m_stride;
        thrust::host_vector<uint32>     link_hvec( hits_index.m_links );
        thrust::host_vector<uint32>     read_infos_hvec( read_count );
        thrust::host_vector<uint32>     idx_hvec( read_count );

        uint32* link_hptr       =          thrust::raw_pointer_cast( &link_hvec.front() );
        uint32* read_infos_hptr = (uint32*)thrust::raw_pointer_cast( &read_infos_hvec.front() );

        cudaDeviceSynchronize();
        cudaMemcpy( read_infos_hptr,   read_infos_dptr,    read_count  * sizeof(uint32),    cudaMemcpyDeviceToHost );
        cudaDeviceSynchronize();

        // sort the reads so as to show everything in the same order all the times
        thrust::copy(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + read_count,
            idx_hvec.begin() );

        thrust::sort_by_key(
            read_infos_hvec.begin(),
            read_infos_hvec.end(),
            idx_hvec.begin() );

        fprintf( file, "selection = [reads: %u - batch-size: %u - link-stride: %u]\n", read_count, n_multi, link_stride );
        for (uint32 i = 0; i < read_count; ++i)
        {
            const packed_read *read_info = (packed_read *)&read_infos_hvec[i];

            const uint32 idx = idx_hvec[i];
            const uint32 n   = link_hvec[idx];

            fprintf( file, "read[%06u:%06u:%u] = [%02u]{", i, read_info->read_id, read_info->top_flag, n );
            if (n)
            {
                strided_iterator<const uint32*> link_vec( link_hptr + idx + link_stride, link_stride );

                for (uint32 j = 0; j < n; ++j)
                {
                    const packed_seed seed = seed_vec[ link_vec[j] ];
                    const uint32      loc  = loc_vec[ link_vec[j] ];
                    fprintf( file, "  seed[pos:%u,dir:%u,rc:%u,top:%u,loc:%u]\n", (uint32)seed.pos_in_read, (uint32)seed.index_dir, (uint32)seed.rc, (uint32)seed.top_flag, loc );
                }
            }
            fprintf( file, "}\n" );
        }
    }
    else
    {
        thrust::host_vector<uint32>     read_infos_hvec( read_count );
        thrust::host_vector<uint32>     idx_hvec( read_count );

        uint32* read_infos_hptr = (uint32*)thrust::raw_pointer_cast( &read_infos_hvec.front() );

        cudaDeviceSynchronize();
        cudaMemcpy( read_infos_hptr,   read_infos_dptr,    read_count  * sizeof(uint32),    cudaMemcpyDeviceToHost );
        cudaDeviceSynchronize();

        // sort the reads so as to show everything in the same order all the times
        thrust::copy(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + read_count,
            idx_hvec.begin() );

        thrust::sort_by_key(
            read_infos_hvec.begin(),
            read_infos_hvec.end(),
            idx_hvec.begin() );

        fprintf( file, "selection = [reads: %u - batch-size: 1]\n", read_count );
        for (uint32 i = 0; i < read_count; ++i)
        {
            const packed_read *read_info = (packed_read *)&read_infos_hvec[i];

            const uint32 idx = idx_hvec[i];
            fprintf( file, "read[%06u:%06u:%u] = [%02u]{", i, read_info->read_id, read_info->top_flag, 1u );

            const packed_seed seed = seed_vec[ idx ];
            const uint32      loc  = loc_vec[ idx ];
            fprintf( file, "  seed[pos:%u,dir:%u,rc:%u,top:%u,loc:%u]\n", (uint32)seed.pos_in_read, (uint32)seed.index_dir, (uint32)seed.rc, (uint32)seed.top_flag, loc );

            fprintf( file, "}\n" );
        }
    }
    fclose( file );
}

// persist a set of scores
//
void persist_scores(
    const std::string&                              file_name,
    const char*                                     name,
    const uint32                                    anchor,
    const uint32                                    read_count,
    const uint32                                    n_multi,
    const uint32                                    hits_queue_size,
    const ScoringQueues&                            scoring_queues)
{
    // check whether we need to persist anything
    if (file_name == "")
        return;

    const std::string mate_file = mate_file_name( file_name, anchor );

    FILE* file = fopen( mate_file.c_str(), "a+" );

    fprintf( file, "\n---------------------------------------------------------------------------------\n" );
    fprintf( file, "%s\n", name );
    fprintf( file, "---------------------------------------------------------------------------------\n" );

    thrust::host_vector<packed_read> read_infos_hvec( scoring_queues.active_reads.in_queue );
    thrust::host_vector<int32>       score_hvec( scoring_queues.hits.score );

    uint32* read_infos_hptr = (uint32*)thrust::raw_pointer_cast( &read_infos_hvec.front() );
     int32* score_hptr      =          thrust::raw_pointer_cast( &score_hvec.front() );

    if (n_multi > 1)
    {
        const uint32 link_stride = scoring_queues.hits_index.m_stride;
        thrust::host_vector<uint32> link_hvec( scoring_queues.hits_index.m_links );
        thrust::host_vector<uint32> idx_hvec( read_count );

        uint32* link_hptr = thrust::raw_pointer_cast( &link_hvec.front() );

        // sort the reads so as to show everything in the same order all the times
        thrust::copy(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + read_count,
            idx_hvec.begin() );

        thrust::sort_by_key(
            read_infos_hptr,
            read_infos_hptr + read_count,
            idx_hvec.begin() );

        fprintf( file, "scores = [reads: %u - batch-size: %u - link-stride: %u]\n", read_count, n_multi, link_stride );
        for (uint32 i = 0; i < read_count; ++i)
        {
            const packed_read *read_info = (packed_read *)&read_infos_hvec[i];

            const uint32 idx = idx_hvec[i];
            const uint32 n   = link_hvec[idx];
            if (n)
            {
                strided_iterator<const uint32*> link_vec( link_hptr + idx + link_stride, link_stride );

                fprintf( file, "read[%06u:%06u:%u] = {\n", i, read_info->read_id, read_info->top_flag );
                for (uint32 j = 0; j < n; ++j)
                {
                    const uint32 link = link_vec[j];
                    fprintf( file, "  score[%04u] = %d\n", j, score_hvec[link] );
                }
                fprintf( file, "}\n" );
            }
        }
        fprintf( file, "\n" );

        // sort
        thrust::sort(
            score_hvec.begin(),
            score_hvec.end() );

        // write sorted score list
        fprintf( file, "\n" );
        for (uint32 i = 0; i < hits_queue_size; ++i)
            fprintf( file, "score[%u] = %d\n", i, score_hvec[i] );
        fprintf( file, "\n" );

        int64 sum = 0;
        for (uint32 i = 0; i < hits_queue_size; ++i)
            sum += score_hvec[i];
        fprintf( file, "sum = %lld\n", sum );

        // compute a crc
        const char* ptr = (const char*)score_hptr;
        const uint64 crc = crcCalc( ptr, sizeof(int32)*hits_queue_size );
        fprintf( file, "crc = %llu\n", crc );
    }
    else
    {
        thrust::host_vector<uint32> idx_hvec( read_count );

        // sort the reads so as to show everything in the same order all the times
        thrust::copy(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + read_count,
            idx_hvec.begin() );

        thrust::sort_by_key(
            read_infos_hptr,
            read_infos_hptr + read_count,
            idx_hvec.begin() );

        for (uint32 i = 0; i < read_count; ++i)
        {
            const packed_read *read_info = (packed_read *)&read_infos_hvec[i];

            const uint32 idx   = idx_hvec[i];
            const uint32 score = score_hvec[idx];
            fprintf( file, "read[%06u:%06u:%u] : score[%d]\n", i, read_info->read_id, read_info->top_flag, score );
        }
        fprintf( file, "\n" );
    }
    fclose( file );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
